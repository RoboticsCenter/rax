"""MCP backend for the real UFACTORY xArm 6 (xArm SDK + RealSense cameras).

Safety layering, outermost first:
  1. The physical emergency stop and the operator beside the robot.
  2. Operator arming (services.xarm.arming): motion tools refuse unless armed, and a
     running motion issues hold once the operator disarms.
  3. This backend: a TCP workspace box, per-call step limits, joint limits and FK checks
     through the vendor SDK.
  4. The xArm controller's own limits, planner and watchdogs.
"""
from __future__ import annotations

import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import uuid

import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation

from . import arming
from .calibration import CameraModel, backproject, camera_specs, project as project_to_pixel
from .sdk import MotionStopped, TargetRejected, XArmDriver
from . import end_effectors

PROJECT = Path(__file__).resolve().parents[2]
WORKSPACE_NAME = 'agent-workspace'
HOME_Q = (0.0, -0.70, -1.20, 0.0, 1.90, 0.0)
MAX_POSE_TIMEOUT_S = 600.0
LOWER = np.array([-2*np.pi, -2.059, -3.927, -2*np.pi, -1.692, -2*np.pi])
UPPER = np.array([2*np.pi, 2.094, 0.192, 2*np.pi, np.pi, 2*np.pi])
DEFAULTS = {
    'xarm_ip': '192.168.1.194',
    'workspace_min': [0.20, -0.55, 0.08],
    'workspace_max': [0.85, 0.55, 0.70],
    'workspace_guard_margin_m': 0.02,
    'link_min_z': 0.08,
    'max_translation_m': 0.35,
    'max_joint_step_rad': 1.2,
    'joint_speed_radps': 0.35,
    'joint_acc_radps2': 1.0,
    'joint_limit_margin_rad': 0.05,
    'cartesian_speed_mps': 0.05,
    'cartesian_acc_mps2': 0.20,
    'move_timeout_s': 300.0,
    'settle_s': 0.3,
    'poll_period_s': 0.05,
    'joint_tolerance_rad': 0.02,
    'cameras': None,
    'camera_cpus': '0-15',
    'camera_depth_mode': 'Z16',
    'stream_enabled': True,
    'stream_fps': 30,
    'stream_window_s': 600,
    'stream_depth_hz': 1.0,
    'stream_jpeg_quality': 80,
    'robot_log_hz': 10.0,
    'stream_archive': True,
    'motion_archive_enabled': True,
    'motion_archive_padding_s': 0.75,
    'motion_archive_codec': None,
    'agent_image_width': 640,
    'gripper_type': 'xarm',
    'gripper_max_width_m': 0.085,
    'gripper_open_position': 850,
    'gripper_closed_position': 0,
    'gripper_speed': 5000,
    'gripper_force': 60,
    'end_effector': None,
}
LAUNCHER_OPTIONS = ('agent_python', 'protected_paths')


def _schema(properties=None, required=()):
    return {'type': 'object', 'properties': properties or {}, 'required': list(required),
            'additionalProperties': False}


NUMBER = {'type': 'number'}
VEC3 = {'type': 'array', 'items': NUMBER, 'minItems': 3, 'maxItems': 3}
QUAT = {'type': 'array', 'items': NUMBER, 'minItems': 4, 'maxItems': 4,
        'description': 'TCP orientation quaternion [x, y, z, w] in the xArm base frame.'}


def _validate_arguments(value, schema, path='arguments'):
    """Enforce the JSON types and bounds declared by the xArm tool schemas."""
    kind = schema['type']
    valid = {'object': isinstance(value, dict), 'array': isinstance(value, (list, tuple)),
             'number': type(value) in (int, float), 'integer': type(value) is int,
             'boolean': type(value) is bool, 'string': isinstance(value, str)}
    if not valid[kind]:
        raise ValueError(f'{path} must be {kind}')
    if 'enum' in schema and value not in schema['enum']:
        raise ValueError(f"{path} must be one of {schema['enum']}")
    if kind == 'object':
        properties = schema['properties']
        missing, unknown = set(schema.get('required', ())) - value.keys(), value.keys() - properties.keys()
        if missing or unknown:
            raise ValueError(f'{path}: missing {sorted(missing)}, unknown {sorted(unknown)}')
        for name, item in value.items():
            _validate_arguments(item, properties[name], f'{path}.{name}')
    elif kind == 'array':
        if not schema.get('minItems', 0) <= len(value) <= schema.get('maxItems', float('inf')):
            raise ValueError(f'{path} has an invalid number of items')
        for index, item in enumerate(value):
            _validate_arguments(item, schema['items'], f'{path}[{index}]')
    elif kind in ('number', 'integer'):
        if not np.isfinite(value) or not schema.get('minimum', -np.inf) <= value <= schema.get('maximum', np.inf):
            raise ValueError(f'{path} must be finite and within its declared bounds')


def matrix(position, quaternion):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(np.asarray(quaternion, dtype=np.float64)).as_matrix()
    T[:3, 3] = position
    return T


def rotation_error(target_quat, measured_quat):
    return (Rotation.from_quat(target_quat) * Rotation.from_quat(measured_quat).inv()).as_rotvec()


class CameraService:
    """Client for services.xarm.camera_server, started lazily under taskset."""

    def __init__(self, options, frames_root, log_path, depth_mode, stream=None):
        self.options, self.frames_root, self.log_path = options, Path(frames_root), Path(log_path)
        self.stream = stream
        self.depth_mode = depth_mode
        directory = Path(f'/tmp/rax-xarm-cam-{os.getuid()}')
        directory.mkdir(mode=0o700, exist_ok=True)
        self.socket_path = directory/(hashlib.sha256(str(frames_root).encode()).hexdigest()[:16] + '.sock')
        self.process = None
        self.lock = threading.Lock()

    def _start(self):
        specs = [{k: v for k, v in spec.items() if k != 'extrinsics'} for spec in self.options['cameras']]
        caps = {name: '8' for name in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS',
                                       'NUMEXPR_NUM_THREADS')}
        env = {**os.environ, **caps, 'OMP_WAIT_POLICY': 'PASSIVE', 'PYTHONPATH': str(PROJECT)}
        command = ['taskset', '-c', str(self.options['camera_cpus']), sys.executable, '-m',
                   'services.xarm.camera_server', '--socket', str(self.socket_path),
                   '--cameras', json.dumps(specs), '--depth-mode', self.depth_mode,
                   '--frames-root', str(self.frames_root), '--fps', str(self.options['stream_fps'])]
        if self.stream is not None:
            command += ['--stream', str(self.stream), '--window-seconds', str(self.options['stream_window_s']),
                        '--depth-hz', str(self.options['stream_depth_hz']),
                        '--stream-quality', str(self.options['stream_jpeg_quality'])]
            if self.options['stream_archive']:
                command += ['--archive', str(self.stream.parent/'archive')]
        log = open(self.log_path, 'ab')
        self.socket_path.unlink(missing_ok=True)
        self.process = subprocess.Popen(command, cwd=PROJECT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        deadline = time.monotonic() + 60
        while not self.socket_path.exists():
            if self.process.poll() is not None:
                raise RuntimeError(f'camera server exited ({self.process.returncode}); see camera-server.log')
            if time.monotonic() > deadline:
                raise RuntimeError('camera server did not start')
            time.sleep(0.1)

    def start(self):
        with self.lock:
            if self.process is None or self.process.poll() is not None:
                self._start()

    def request(self, payload, timeout=90.0):
        with self.lock:
            if self.process is None or self.process.poll() is not None:
                self._start()
            with socket.socket(socket.AF_UNIX) as connection:
                connection.settimeout(timeout)
                connection.connect(str(self.socket_path))
                connection.sendall((json.dumps(payload) + '\n').encode())
                line = connection.makefile('r').readline()
        if not line:
            raise RuntimeError('camera server closed the connection')
        value = json.loads(line)
        if not value.pop('ok', False):
            raise RuntimeError(value.get('error', 'camera server error'))
        return value

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(25)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None


class XArmBackend:
    mutating = frozenset({'xarm_move_pose', 'xarm_move_joints', 'xarm_gripper'})

    def __init__(self, config, *, arm=None, cameras=None, hand=None, camera_models=None):
        self.config = config
        unknown = set(config.backend_options) - set(DEFAULTS) - set(LAUNCHER_OPTIONS)
        if unknown:
            raise ValueError(f'unknown xarm backend options: {sorted(unknown)}')
        self.options = {**DEFAULTS, **config.backend_options}
        self.end_effector_kind = end_effectors.kind(self.options)
        self.has_hand = self.end_effector_kind == 'wuji_hand2'
        self.hand_config = None
        if self.has_hand:
            from services.wuji.config import validate_config, MUTATING_TOOLS
            self.hand_config = validate_config(self.options['end_effector'])
            self.mutating = frozenset({'xarm_move_pose', 'xarm_move_joints', *MUTATING_TOOLS})
        elif self.options['end_effector']:
            self.options['gripper_type'] = 'g2' if self.end_effector_kind == 'g2' else 'xarm'
        self.box_min = np.asarray(self.options['workspace_min'], dtype=np.float64)
        self.box_max = np.asarray(self.options['workspace_max'], dtype=np.float64)
        if not np.all(self.box_min < self.box_max):
            raise ValueError('workspace_min must be below workspace_max on every axis')
        # Operator calibration can inject an empty model map and capture raw
        # camera frames directly, without pretending old extrinsics are valid.
        self.models = camera_models if camera_models is not None else {
            spec['name']: CameraModel.from_spec(spec) for spec in camera_specs(self.options)}
        self.arm = arm or XArmDriver(self.options)
        try:
            self.end_effector = hand if hand is not None else end_effectors.make(config, self.arm, on_fault=self.arm.hold)
        except BaseException:
            close = getattr(self.arm, 'close', None)
            if close:
                close()
            raise
        self._closed = False
        self.workspace = config.root/WORKSPACE_NAME
        self.frames_root = self.workspace/'runtime'/'frames'
        self.depth = config.observation_profile == 'rgbd'
        self.stream = config.root/'stream' if self.options['stream_enabled'] else None
        self.motion_archive = config.root/'motion-archive' if self.stream is not None \
            and self.options['motion_archive_enabled'] else None
        self.last_motion = None
        self.motion_clip_threads = []
        self._stop = threading.Event()
        self.cameras = cameras if cameras is not None else CameraService(
            self.options, self.frames_root, config.root/'camera-server.log',
            self.options['camera_depth_mode'] if self.depth else 'NONE', stream=self.stream)
        try:
            if cameras is None and self.stream is not None:
                self.stream.mkdir(mode=0o755, exist_ok=True)
                self.cameras.start()
                if self.motion_archive is not None:
                    self.motion_archive.mkdir(mode=0o755, exist_ok=True)
                threading.Thread(target=self._log_robot, name='robot-log', daemon=True).start()
        except BaseException:
            self.close()
            raise

    def _log_robot(self):
        from .camera_server import minute_name
        folder = self.stream/'robot'
        folder.mkdir(exist_ok=True)
        period = 1.0 / float(self.options['robot_log_hz'])
        width = None
        tick = 0
        while not self._stop.wait(period):
            now = time.time()
            try:
                state = self.arm.state()
                if not self.has_hand and tick % max(1, int(self.options['robot_log_hz'] / 2)) == 0:
                    width = self.end_effector.status().get('width')
                tick += 1
                record = {'t_ms': int(now * 1000),
                          'tcp_position': np.round(state['tcp_position'], 5).tolist(),
                          'tcp_quaternion': np.round(state['tcp_quaternion'], 5).tolist(),
                          'joints': np.round(np.asarray(state['joints']), 4).tolist(),
                          'gripper_width': None if width is None else round(float(width), 4)}
                if self.has_hand:
                    record.pop('gripper_width')
                    record['hand'] = self.end_effector.status()
            except Exception as exc:
                record = {'t_ms': int(now * 1000), 'error': str(exc)[:200]}
            with open(folder/f'{minute_name(now)}.jsonl', 'a') as log:
                log.write(json.dumps(record) + '\n')

    def tools(self):
        names = sorted(self.models)
        lo, hi = self.box_min.round(3).tolist(), self.box_max.round(3).tolist()
        definitions = [
            {'name': 'xarm_status', 'inputSchema': _schema(),
             'description': 'Robot state without moving: TCP pose (xArm base frame), joints, gripper width, '
                            'whether the operator has armed motion, the workspace box and limits.'},
            {'name': 'xarm_observe', 'description':
                f'Capture the RealSense cameras ({", ".join(names)}) now. Returns downscaled images plus the '
                'TCP pose; full-resolution RGB (and depth as uint16 millimetre PNG in rgbd sessions) and a '
                'meta.json with intrinsics and camera poses are saved under runtime/frames/<frame_id>/. Use '
                'frame_id with xarm_pixel_to_position. project: base-frame points to mark on the images.',
             'inputSchema': _schema({'cameras': {'type': 'array', 'items': {'type': 'string', 'enum': names}},
                                     'project': {'type': 'array', 'items': VEC3, 'maxItems': 16}})},
            {'name': 'xarm_pixel_to_position', 'description':
                'Base-frame 3-D points for pixels of a saved frame, from its depth. Pixels are in the '
                'full-resolution image (width x height in meta.json), not the downscaled preview.',
             'inputSchema': _schema({'frame_id': {'type': 'string'}, 'camera': {'type': 'string', 'enum': names},
                                     'pixels': {'type': 'array', 'minItems': 1, 'maxItems': 64,
                                                'items': {'type': 'array', 'items': NUMBER, 'minItems': 2,
                                                          'maxItems': 2}},
                                     'radius': {'type': 'integer', 'minimum': 0, 'maximum': 15}},
                                    ['frame_id', 'camera', 'pixels'])},
            {'name': 'xarm_move_pose', 'description':
                'Move the TCP to a base-frame pose with the xArm controller Cartesian line mode and wait until '
                f'it settles. Workspace box {lo} .. {hi} m; at most {self.options["max_translation_m"]} m per '
                f'call; speed is capped at {self.options["cartesian_speed_mps"] * 1000:g} mm/s. '
                'Optional speed_mps selects a lower speed for this move; omitted uses the configured cap. Omit quaternion '
                'to keep the current orientation. relative=true adds position to the current TCP position. '
                f'timeout_s allows 1..{MAX_POSE_TIMEOUT_S:g} seconds; omitted uses '
                f'{min(self.options["move_timeout_s"], MAX_POSE_TIMEOUT_S):g} seconds. '
                'Request more time for large rotations or slow moves. '
                'Requires operator arming. Returns the measured pose and error.',
             'inputSchema': _schema({'position': VEC3, 'quaternion': QUAT, 'relative': {'type': 'boolean'},
                                     'speed_mps': {'type': 'number', 'minimum': 0.001,
                                                   'maximum': self.options['cartesian_speed_mps']},
                                     'tolerance_mm': {'type': 'number', 'minimum': 0.5, 'maximum': 20},
                                     'timeout_s': {'type': 'number', 'minimum': 1, 'maximum': MAX_POSE_TIMEOUT_S,
                                                   'default': min(self.options['move_timeout_s'], MAX_POSE_TIMEOUT_S)}},
                                    ['position'])},
            {'name': 'xarm_move_joints', 'description':
                'Joint-space move with xArm set_servo_angle. positions: 6 joint angles (rad); or preset "home". '
                'The path is sampled through the controller FK and checked against workspace/table clearance '
                'before moving. Longer moves execute bounded SDK segments within this call; '
                'the caller need not subdivide joint targets. Uses the configured move timeout '
                f'({self.options["move_timeout_s"]:g} seconds) for the whole call. Requires operator arming.',
             'inputSchema': _schema({'positions': {'type': 'array', 'items': NUMBER, 'minItems': 6,
                                                   'maxItems': 6},
                                     'preset': {'type': 'string', 'enum': ['home']}})},
            {'name': 'xarm_gripper', 'description':
                'xArm gripper. open; move to width (m); grasp closes to width (force is used only by gripper_g2). '
                'For a G2 gripper, width is metres but the SDK position is millimetres, 0..84; speed is mm/s. '
                'Position feedback is reported; verify a grasp visually because the standard xArm gripper does '
                'not report a reliable is_grasped signal. Requires operator arming.',
             'inputSchema': _schema({'action': {'type': 'string', 'enum': ['open', 'move', 'grasp']},
                                     'width': {'type': 'number', 'minimum': 0,
                                               'maximum': self.options['gripper_max_width_m']},
                                     'force': {'type': 'number', 'minimum': 1, 'maximum': 100},
                                     'speed': {'type': 'integer', 'minimum': 15,
                                               'maximum': 225 if self.options['gripper_type'] == 'g2' else 10000}},
                                    ['action'])},
            {'name': 'xarm_stream', 'description':
                'Look back in the continuous recording without moving anything. seconds_ago: frames from that '
                'many seconds ago (0 = newest), labelled with their age, plus the arm state then. sheet: one '
                'contact sheet per camera of count evenly spaced frames over the last seconds.',
             'inputSchema': _schema({'cameras': {'type': 'array', 'items': {'type': 'string', 'enum': names}},
                                     'seconds_ago': {'type': 'array', 'items': {'type': 'number', 'minimum': 0},
                                                     'maxItems': 8},
                                     'sheet': _schema({'seconds': {'type': 'number', 'minimum': 1},
                                                       'count': {'type': 'integer', 'minimum': 2, 'maximum': 16}},
                                                      ['seconds'])})},
            {'name': 'xarm_hold', 'inputSchema': _schema(),
             'description': 'Stop/hold the current motion. Always allowed.'},
        ]
        if self.has_hand:
            from services.wuji.config import tools
            definitions = [t for t in definitions if t['name'] != 'xarm_gripper']
            definitions.extend(tools(self.hand_config))
            definitions[0]['description'] = ('xArm pose and joints, Wuji hand feedback, controller errors, '
                                             'operator arming, cameras and configured workspace limits.')
            next(t for t in definitions if t['name'] == 'xarm_hold')['description'] = (
                'Stop the arm and request Wuji emergency stop independently. Always allowed. '
                'Reports each outcome; an SDK return alone does not confirm physical stopping.')
        return definitions

    def _require_armed(self):
        state = arming.read(self.config.root)
        if not state['armed']:
            raise PermissionError(f"motion is not armed ({state['reason']}); the operator arms motion")

    def _check_box(self, point, margin=0.0, what='target'):
        point = np.asarray(point, dtype=np.float64)
        if np.any(point < self.box_min - margin) or np.any(point > self.box_max + margin):
            raise ValueError(f'{what} {np.round(point, 3).tolist()} is outside the workspace box '
                             f'{self.box_min.round(3).tolist()} .. {self.box_max.round(3).tolist()}')

    def _pose_goal(self, arguments, state):
        tcp_p, tcp_q = state['tcp_position'], state['tcp_quaternion']
        position = np.asarray(arguments['position'], dtype=np.float64)
        if not np.all(np.isfinite(position)):
            raise ValueError('position must be finite')
        if arguments.get('relative'):
            position = tcp_p + position
        quaternion = arguments.get('quaternion')
        quaternion = tcp_q if quaternion is None else np.asarray(quaternion, dtype=np.float64)
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(norm) or norm < 0.5:
            raise ValueError('quaternion must be a unit quaternion [x, y, z, w]')
        quaternion = quaternion/norm
        self._check_box(position)
        step = float(np.linalg.norm(position - tcp_p))
        if step > self.options['max_translation_m']:
            raise ValueError(f"move of {step:.3f} m exceeds {self.options['max_translation_m']} m per call; "
                             'split it into shorter moves')
        problem = self.arm.check_tcp_limit(position, quaternion)
        if problem:
            raise ValueError(f'unreachable for the controller: {problem}')
        return position, quaternion

    def _joint_goal(self, arguments):
        if ('positions' in arguments) == ('preset' in arguments):
            raise ValueError('give exactly one of positions or preset')
        goal = np.asarray(HOME_Q if arguments.get('preset') == 'home' else arguments['positions'], dtype=np.float64)
        margin = self.options['joint_limit_margin_rad']
        if not np.all(np.isfinite(goal)) or np.any(goal < LOWER + margin) or np.any(goal > UPPER - margin):
            raise ValueError(f'joint target outside the joint limits minus {margin} rad')
        problem = self.arm.check_joint_limit(goal)
        if problem:
            raise ValueError(problem)
        return goal

    def _check_joint_path(self, start, goal):
        step = float(np.max(np.abs(goal - start)))
        if step > self.options['max_joint_step_rad']:
            raise ValueError(f"joint segment of {step:.2f} rad exceeds {self.options['max_joint_step_rad']} rad "
                             'per SDK command')
        for fraction in np.linspace(0.0, 1.0, 31):
            p, _ = self.arm.forward_kinematics(start + fraction * (goal - start))
            self._check_box(p, margin=self.options['workspace_guard_margin_m'],
                            what=f'TCP at {fraction:.0%} of the joint path')
            if p[2] < self.options['link_min_z']:
                raise ValueError(f"joint path brings the TCP to z={p[2]:.3f} m at {fraction:.0%} "
                                 f"(minimum {self.options['link_min_z']} m)")
        return step

    def _joint_plan(self, start, goal):
        """Validate the complete path before dispatching any bounded segment."""
        limit = float(self.options['max_joint_step_rad'])
        tolerance = float(self.options['joint_tolerance_rad'])
        if not np.isfinite([limit, tolerance]).all() or not 0 < tolerance < limit:
            raise ValueError('joint_tolerance_rad must be positive and smaller than max_joint_step_rad')
        step = float(np.max(np.abs(goal - start)))
        # Reserve the endpoint tolerance: the next SDK command is bounded from
        # measured joints even when the preceding segment stops short of target.
        count = 1 if step <= limit else int(np.ceil(step / (limit - tolerance)))
        waypoints = [start + (goal - start) * (i / count) for i in range(1, count + 1)]
        previous = start
        for waypoint in waypoints:
            self._check_joint_path(previous, waypoint)
            previous = waypoint
        return waypoints

    def validate(self, name, arguments):
        if self.has_hand and name.startswith('wuji_'):
            from services.wuji.config import validate_tool, MOTION_TOOLS, MUTATING_TOOLS, effort
            from services.wuji.controller import Trajectory, EffortRamp
            goal = validate_tool(name, arguments, self.hand_config)
            if name in MUTATING_TOOLS:
                self._require_armed()
                state = self.end_effector.require_ready()
                if name in MOTION_TOOLS:
                    Trajectory(state['target_positions'], goal['positions'], self.hand_config, goal['duration_s'])
                elif name == 'wuji_set_effort':
                    effort(goal['effort_a'], state['control_parameters']['current_limit_a'])
                    EffortRamp(state['command_effort_a'], goal['effort_a'], self.hand_config, goal['duration_s'])
                elif name == 'wuji_set_impedance':
                    if any(abs(v) > 1e-10 for v in state['command_effort_a']):
                        raise ValueError('ramp feed-forward effort to zero before changing impedance/current limits')
                    if max(map(abs, state['velocity_rad_s'])) > self.hand_config['stopped_velocity_rad_s']:
                        raise ValueError('impedance changes require a measured stationary hand')
            return
        if name == 'xarm_gripper' and self.has_hand:
            raise ValueError('xarm_gripper is unavailable with a Wuji hand')
        definition = next((tool for tool in self.tools() if tool['name'] == name), None)
        if definition is None:
            raise ValueError(f'unknown tool {name}')
        _validate_arguments(arguments, definition['inputSchema'])
        if name == 'xarm_move_pose':
            self._pose_goal(arguments, self.arm.state())
        elif name == 'xarm_move_joints':
            self._joint_plan(np.asarray(self.arm.state()['joints']), self._joint_goal(arguments))
        elif name == 'xarm_gripper':
            action = arguments['action']
            if action in ('move', 'grasp') and 'width' not in arguments:
                raise ValueError(f'{action} needs width')
            if action != 'grasp' and 'force' in arguments:
                raise ValueError('force applies to grasp only')
        elif name == 'xarm_pixel_to_position':
            self._frame_dir(arguments['frame_id'])
        elif name == 'xarm_stream':
            if 'sheet' in arguments and 'seconds_ago' in arguments:
                raise ValueError('give either sheet or seconds_ago')
            ages = arguments.get('seconds_ago', [0])
            if any(age > self.options['stream_window_s'] for age in ages):
                raise ValueError(f"only the last {self.options['stream_window_s']} s are kept")
            if len(ages) * len(arguments.get('cameras') or self.models) > 12:
                raise ValueError('at most 12 images per call (seconds_ago x cameras)')
        if name in self.mutating:
            self._require_armed()
            if self.has_hand:
                self.end_effector.require_ready()

    def call(self, name, arguments):
        if self.has_hand and name.startswith('wuji_'):
            self.validate(name, arguments)
            return self._hand_call(name, arguments)
        handler = getattr(self, '_' + name.removeprefix('xarm_'), None)
        if handler is None or name not in {tool['name'] for tool in self.tools()}:
            raise ValueError(f'unknown tool {name}')
        if self.has_hand and name in self.mutating:
            self.end_effector.require_ready()
            try:
                return handler(**arguments)
            except Exception as exc:
                # Deadline expiry is not itself lost control. Confirm the stop from
                # feedback before telling the frontend whether control is known.
                timed_out = isinstance(exc, TimeoutError)
                outcome = self._stop_assembly(verify_arm=timed_out)
                exc.control_certain = timed_out and all(
                    result.get('confirmed') is True for result in outcome.values())
                exc.robot_state = {**getattr(exc, 'robot_state', {}), 'assembly_stop': outcome,
                                   'hand': self.end_effector.status()}
                raise
        return handler(**arguments)

    def _hand_call(self, name, arguments):
        if name == 'wuji_status':
            return self.end_effector.status(), []
        if name == 'wuji_hold':
            return self.end_effector.hold(), []
        from services.wuji.config import validate_tool
        goal = validate_tool(name, arguments, self.hand_config)
        self._require_armed()
        started = time.time()
        try:
            method = {'wuji_set_effort': 'set_effort', 'wuji_set_impedance': 'set_impedance'}.get(name, 'move')
            result = getattr(self.end_effector, method)(**goal)
        except Exception as exc:
            outcome = self._stop_assembly()
            self._schedule_motion_clip(name, started, time.time(), 'failed', exc)
            exc.control_certain = False
            exc.robot_state = {**getattr(exc, 'robot_state', {}), 'hand': self.end_effector.status(),
                               'assembly_stop': outcome}
            raise
        clip = self._schedule_motion_clip(name, started, time.time(), 'completed')
        if clip:
            result['motion_clip'] = clip
        self.last_motion = {'tool': name, 'at': time.time(), **result}
        return result, []

    def _stop_arm_verified(self):
        """A stop ACK alone is insufficient: require stable, healthy state-4 feedback."""
        self.arm.hold()
        outcome = {'requested': True, 'sdk_returned': True, 'confirmed': False}
        deadline = time.monotonic() + 2.0
        anchor, stable_since, samples = None, None, 0
        while time.monotonic() < deadline:
            state = self.arm.state()
            q = np.asarray(state['joints'], dtype=float)
            p = np.asarray(state['tcp_position'], dtype=float)
            quat = np.asarray(state['tcp_quaternion'], dtype=float)
            healthy = (state.get('state') == 4 and state.get('error_code') == 0
                       and state.get('warn_code') == 0 and q.shape == (6,) and p.shape == (3,)
                       and quat.shape == (4,) and all(np.all(np.isfinite(v)) for v in (q, p, quat)))
            if not healthy:
                anchor, stable_since, samples = None, None, 0
            else:
                stable = (anchor is not None and np.max(np.abs(q-anchor[0])) <= .001
                          and np.linalg.norm(p-anchor[1]) <= .0005
                          and np.linalg.norm(rotation_error(quat, anchor[2])) <= np.radians(.1))
                if not stable:
                    anchor, stable_since, samples = (q.copy(), p.copy(), quat.copy()), time.monotonic(), 0
                samples += 1
                elapsed = time.monotonic() - stable_since
                if samples >= 5 and elapsed >= .4:
                    return {**outcome, 'confirmed': True, 'stationary_s': round(elapsed, 3),
                            'samples': samples, 'measured': self._pose_report(state)}
            time.sleep(.05)
        return {**outcome, 'reason': 'Healthy stationary stop feedback was not confirmed within 2 seconds'}

    def _stop_assembly(self, *, verify_arm=False):
        """Independent stop dispatch; one blocked SDK cannot suppress the other stop."""
        outcomes = {}
        lock = threading.Lock()
        def attempt(name, function):
            try:
                result = function()
                value = result if isinstance(result, dict) else {
                    'requested': True, 'sdk_returned': True, 'confirmed': False}
            except Exception as exc:
                value = {'requested': True, 'sdk_returned': False, 'confirmed': False, 'error': str(exc)}
            with lock:
                outcomes[name] = value
        actions = [('arm', self._stop_arm_verified if verify_arm else self.arm.hold)]
        if self.has_hand:
            actions.append(('hand', self.end_effector.stop))
        threads = [threading.Thread(target=attempt, args=action, daemon=True) for action in actions]
        for thread in threads:
            thread.start()
        deadline = time.monotonic() + (self.hand_config['stop_timeout_s']+2 if self.has_hand else 2)
        for thread in threads:
            thread.join(max(0, deadline-time.monotonic()))
        with lock:
            return {name: outcomes.get(name, {'requested': True, 'sdk_returned': False,
                                              'confirmed': False, 'pending': True}) for name, _ in actions}

    def _status(self):
        return self.status(), []

    def _hold(self):
        if self.has_hand:
            return {'assembly_stop': self._stop_assembly()}, []
        self.arm.hold()
        return {'held': True, **self._pose_report(self.arm.state())}, []

    def _guard(self, state):
        if not arming.read(self.config.root)['armed']:
            self.arm.hold()
            raise MotionStopped('the operator disarmed motion; the arm holds')
        if self.has_hand:
            self.end_effector.require_ready()
        self._check_box(state['tcp_position'], margin=self.options['workspace_guard_margin_m'], what='measured TCP')

    def _monitor_pose(self, goal_p, goal_q, *, tolerance_m, tolerance_rad, timeout_s):
        start = time.monotonic()
        inside_since = None
        while True:
            state = self.arm.state()
            self._guard(state)
            err_p = float(np.linalg.norm(goal_p - state['tcp_position']))
            err_r = float(np.linalg.norm(rotation_error(goal_q, state['tcp_quaternion'])))
            now = time.monotonic()
            if err_p <= tolerance_m and err_r <= tolerance_rad:
                inside_since = inside_since or now
                if now - inside_since >= self.options['settle_s']:
                    return {'reached': True, 'seconds': round(now - start, 2),
                            'position_error_mm': round(err_p * 1000, 2),
                            'rotation_error_deg': round(np.degrees(err_r), 2),
                            'measured_position': np.round(state['tcp_position'], 5).tolist(),
                            'measured_quaternion': np.round(state['tcp_quaternion'], 5).tolist()}
            else:
                inside_since = None
            if now - start > timeout_s:
                self.arm.hold()
                raise self._held(TimeoutError(f"pose not reached in {timeout_s:.1f}s: position error "
                                               f"{err_p * 1000:.1f} mm, rotation error "
                                               f"{np.degrees(err_r):.2f} deg; the arm holds where it is"),
                                 goal_p)
            time.sleep(self.options['poll_period_s'])

    def _schedule_motion_clip(self, tool, start_s, end_s, status, detail=None):
        if self.motion_archive is None:
            return None
        padding = max(0.0, float(self.options['motion_archive_padding_s']))
        clip_id = f"{time.strftime('%Y%m%dT%H%M%S', time.gmtime(start_s))}_{tool}_{uuid.uuid4().hex[:6]}"
        out_dir = self.motion_archive/clip_id
        start_ms, end_ms = int((start_s - padding) * 1000), int((end_s + padding) * 1000)
        metadata = {'tool': tool, 'status': status, 'motion_start_ms': int(start_s * 1000),
                    'motion_end_ms': int(end_s * 1000), 'padding_s': padding}
        if detail:
            metadata['detail'] = str(detail)[:500]

        def encode():
            # Wait until the post-motion padding is in the stream before extracting it.
            time.sleep(max(0.0, end_s + padding + 0.25 - time.time()))
            try:
                from .archive import encode_interval
                encode_interval(self.stream, out_dir, sorted(self.models), start_ms=start_ms, end_ms=end_ms,
                                fps=self.options['stream_fps'], codec=self.options['motion_archive_codec'],
                                metadata=metadata)
            except Exception as exc:
                log = self.config.root/'motion-archive.log'
                with open(log, 'a') as stream:
                    stream.write(f'{clip_id}: {type(exc).__name__}: {exc}\n')

        thread = threading.Thread(target=encode, name=f'motion-clip-{clip_id}', daemon=True)
        thread.start()
        self.motion_clip_threads.append(thread)
        return {'directory': f'motion-archive/{clip_id}', 'start_ms': start_ms, 'end_ms': end_ms,
                'status': status, 'cameras': sorted(self.models)}

    def _move_pose(self, position, quaternion=None, relative=False, tolerance_mm=1.5, timeout_s=None,
                   speed_mps=None):
        speed = self.options['cartesian_speed_mps'] if speed_mps is None else speed_mps
        # Direct operator calls and the MCP frontend must obey the same ceiling.
        _validate_arguments(speed, {'type': 'number', 'minimum': 0.001,
                                   'maximum': self.options['cartesian_speed_mps']}, 'speed_mps')
        timeout = min(self.options['move_timeout_s'], MAX_POSE_TIMEOUT_S) if timeout_s is None else timeout_s
        _validate_arguments(timeout, {'type': 'number', 'minimum': 1,
                                     'maximum': MAX_POSE_TIMEOUT_S}, 'timeout_s')
        timeout = float(timeout)
        goal_p, goal_q = self._pose_goal({'position': position, 'quaternion': quaternion, 'relative': relative},
                                         self.arm.state())
        self._require_armed()
        motion_start = time.time()
        try:
            self.arm.move_pose(goal_p, goal_q, speed_mps=speed,
                               acc_mps2=self.options['cartesian_acc_mps2'], timeout_s=timeout)
            result = self._monitor_pose(goal_p, goal_q, tolerance_m=tolerance_mm/1000.0,
                                        tolerance_rad=np.radians(max(0.5, tolerance_mm/3.0)),
                                        timeout_s=timeout)
        except ValueError as exc:
            self.arm.hold()
            self._schedule_motion_clip('xarm_move_pose', motion_start, time.time(), 'failed', exc)
            raise self._held(MotionStopped(f'{exc}; the arm holds'), goal_p) from exc
        except TargetRejected as exc:
            self.arm.hold()
            self._schedule_motion_clip('xarm_move_pose', motion_start, time.time(), 'failed', exc)
            raise self._held(TargetRejected(f'the controller refused this target: {exc}; the arm holds'),
                             goal_p) from exc
        except (MotionStopped, TimeoutError) as exc:
            self._schedule_motion_clip('xarm_move_pose', motion_start, time.time(), 'failed', exc)
            raise self._held(exc, goal_p)
        clip = self._schedule_motion_clip('xarm_move_pose', motion_start, time.time(), 'completed')
        if clip:
            result['motion_clip'] = clip
        self.last_motion = {'tool': 'xarm_move_pose', 'at': time.time(), **result}
        return {**result, 'speed_mps': speed, 'timeout_s': timeout, 'target_position': np.round(goal_p, 5).tolist(),
                'target_quaternion': np.round(goal_q, 5).tolist()}, []

    def _held(self, exc, goal_p):
        report = self._pose_report(self.arm.state())
        report['distance_to_target_mm'] = round(float(np.linalg.norm(np.asarray(report['tcp_position']) - goal_p))
                                                * 1000, 1)
        exc.control_certain = True
        exc.robot_state = report
        return exc

    def _move_joints(self, positions=None, preset=None):
        arguments = {'positions': positions} if positions is not None else {'preset': preset}
        goal = self._joint_goal(arguments)
        start = np.asarray(self.arm.state()['joints'])
        waypoints = self._joint_plan(start, goal)
        self._require_armed()
        timeout = float(self.options['move_timeout_s'])
        started, motion_start = time.monotonic(), time.time()
        try:
            previous = start
            for waypoint in waypoints:
                state = self.arm.state()
                self._guard(state)
                measured = np.asarray(state['joints'])
                # The nominal segment was already checked. Recheck geometry only
                # when actual feedback differs from its preflighted start.
                if not np.array_equal(measured, previous):
                    self._check_joint_path(measured, waypoint)
                remaining = timeout - (time.monotonic() - started)
                if remaining <= 0:
                    raise TimeoutError(f'joint move exceeded its {timeout:.1f}s timeout before the next segment')
                self.arm.move_joints(waypoint, speed_radps=self.options['joint_speed_radps'],
                                     acc_radps2=self.options['joint_acc_radps2'], timeout_s=remaining)
                while True:
                    state = self.arm.state()
                    self._guard(state)
                    error = np.abs(np.asarray(state['joints']) - waypoint)
                    if float(error.max()) <= self.options['joint_tolerance_rad']:
                        break
                    if time.monotonic() - started > timeout:
                        self.arm.hold()
                        raise TimeoutError(f"joint target not reached in {timeout:.1f}s: largest error "
                                           f"{float(error.max()):.3f} rad; the arm holds where it is")
                    time.sleep(self.options['poll_period_s'])
                previous = waypoint
        except (MotionStopped, TimeoutError, TargetRejected, ValueError) as exc:
            self.arm.hold()
            self._schedule_motion_clip('xarm_move_joints', motion_start, time.time(), 'failed', exc)
            report = self._pose_report(self.arm.state())
            exc.control_certain = True
            exc.robot_state = report
            raise exc
        state = self.arm.state()
        error = np.abs(np.asarray(state['joints']) - goal)
        result = {'seconds': round(time.monotonic() - started, 2),
                  'segments': len(waypoints), 'max_joint_error_rad': round(float(error.max()), 4),
                  **self._pose_report(state)}
        clip = self._schedule_motion_clip('xarm_move_joints', motion_start, time.time(), 'completed')
        if clip:
            result['motion_clip'] = clip
        self.last_motion = {'tool': 'xarm_move_joints', 'at': time.time(), **result}
        return result, []

    def _gripper(self, action, width=None, force=None, speed=None):
        self._require_armed()
        if action == 'open':
            width = self.options['gripper_max_width_m']
        state = self.end_effector.status()
        if state.get('stale'):
            raise RuntimeError('gripper state is stale; tell the operator')
        motion_start = time.time()
        self.end_effector.move(width, speed=int(round(float(speed or self.options['gripper_speed']))), force=force)
        time.sleep(0.2)
        value = self.end_effector.status()
        result = {'action': action, 'width': round(float(value['width']), 4),
                  'is_grasped': bool(value.get('is_grasped', False)),
                  'raw_position': value.get('raw_position')}
        clip = self._schedule_motion_clip('xarm_gripper', motion_start, time.time(), 'completed')
        if clip:
            result['motion_clip'] = clip
        return result, []

    def _frame_dir(self, frame_id):
        if not isinstance(frame_id, str) or not frame_id.isalnum():
            raise ValueError('invalid frame_id')
        path = self.frames_root/frame_id
        if not (path/'meta.json').is_file():
            raise ValueError(f'unknown frame_id {frame_id}')
        return path

    def _observe(self, cameras=None, project=None):
        if not self.models:
            raise ValueError('camera geometry is unavailable; operator calibration must capture raw frames')
        names = list(cameras or sorted(self.models))
        frame_id = time.strftime('%Y%m%d%H%M%S') + uuid.uuid4().hex[:6]
        out_dir = self.frames_root/frame_id
        before = self.arm.state()
        captured = self.cameras.request({'op': 'grab', 'cameras': names, 'depth': self.depth,
                                         'out_dir': str(out_dir)})['cameras']
        after = self.arm.state()
        tcp_p, tcp_q = after['tcp_position'], after['tcp_quaternion']
        moved = float(np.linalg.norm(after['tcp_position'] - before['tcp_position']))
        T_base_tcp = matrix(tcp_p, tcp_q)
        meta = {'frame_id': frame_id, 'time': time.time(), 'tcp_position': tcp_p.tolist(),
                'tcp_quaternion': tcp_q.tolist(), 'joints': np.asarray(after['joints']).tolist(),
                'arm_moved_during_capture_mm': round(moved * 1000, 2), 'depth_units': 'mm (uint16, 0 = invalid)',
                'cameras': {}}
        if self.has_hand:
            meta['hand'] = self.end_effector.status()
        images, report = [], {}
        for name in names:
            entry = captured[name]
            T_base_cam = self.models[name].base_from_camera(T_base_tcp)
            meta['cameras'][name] = {'rgb': Path(entry['rgb']).name,
                                     'depth': Path(entry['depth']).name if entry['depth'] else None,
                                     'K': entry['K'], 'width': entry['width'], 'height': entry['height'],
                                     'T_base_cam': T_base_cam.tolist(), 'timestamp_ns': entry['timestamp_ns'],
                                     **{key: entry[key] for key in ('distortion_model', 'distortion_coeffs',
                                        'serial', 'fps', 'depth_scale_m', 'depth_scale_factor', 'depth_alignment',
                                        'intrinsics_source', 'depth_registration_source') if key in entry}}
            image = cv.imread(entry['rgb'])
            marks = []
            for point in project or []:
                uv = project_to_pixel(point, T_base_cam, entry['K'])
                marks.append(None if uv is None else [round(uv[0], 1), round(uv[1], 1)])
                if uv is not None:
                    cv.drawMarker(image, (int(round(uv[0])), int(round(uv[1]))), (0, 0, 255),
                                  cv.MARKER_CROSS, 24, 2)
            scale = self.options['agent_image_width'] / image.shape[1]
            small = cv.resize(image, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA)
            ok, jpeg = cv.imencode('.jpg', small, [cv.IMWRITE_JPEG_QUALITY, 85])
            images.append({'type': 'image', 'mimeType': 'image/jpeg', 'data': base64.b64encode(jpeg).decode()})
            report[name] = {'preview_scale': round(scale, 4), 'width': entry['width'], 'height': entry['height'],
                            'captured_ms': entry['timestamp_ns'] // 1_000_000,
                            **({'projected_pixels': marks} if project else {})}
        (out_dir/'meta.json').write_text(json.dumps(meta, indent=1))
        return {'frame_id': frame_id, 'directory': f'runtime/frames/{frame_id}', 'images_in_order': names,
                'cameras': report, 'arm_moved_during_capture_mm': meta['arm_moved_during_capture_mm'],
                **({'hand': meta['hand']} if self.has_hand else {}),
                **self._pose_report(after)}, images

    def _pixel_to_position(self, frame_id, camera, pixels, radius=3):
        directory = self._frame_dir(frame_id)
        meta = json.loads((directory/'meta.json').read_text())
        entry = meta['cameras'].get(camera)
        if entry is None or not entry['depth']:
            raise ValueError(f'frame {frame_id} has no depth for camera {camera}')
        depth = cv.imread(str(directory/entry['depth']), cv.IMREAD_UNCHANGED)
        T_base_cam = np.asarray(entry['T_base_cam'])
        points = []
        for u, v in pixels:
            if not (0 <= u < entry['width'] and 0 <= v < entry['height']):
                points.append({'pixel': [u, v], 'error': 'outside the image'})
                continue
            r = int(radius)
            ui, vi = int(round(u)), int(round(v))
            window = depth[max(0, vi - r):vi + r + 1, max(0, ui - r):ui + r + 1]
            valid = window[window > 0]
            if valid.size == 0:
                points.append({'pixel': [u, v], 'error': 'no valid depth here'})
                continue
            z = float(np.median(valid)) / 1000.0
            p_cam = backproject(u, v, z, entry['K'])
            p_base = (T_base_cam @ np.append(p_cam, 1.0))[:3]
            points.append({'pixel': [u, v], 'position': np.round(p_base, 4).tolist(), 'depth_m': round(z, 4),
                           'valid_fraction': round(valid.size / window.size, 2)})
        return {'frame_id': frame_id, 'camera': camera, 'points': points}, []

    def _stream(self, cameras=None, seconds_ago=None, sheet=None):
        from . import stream as recording
        if self.stream is None:
            raise ValueError('the continuous recording is disabled in this session')
        if seconds_ago is not None and sheet is not None:
            raise ValueError('give seconds_ago or sheet, not both')
        names = list(cameras or sorted(self.models))
        window_ms = int(self.options['stream_window_s'] * 1000)
        now_ms = int(time.time() * 1000)
        images, report = [], {'now_ms': now_ms, 'cameras': names}

        def encode(image, width):
            scale = width / image.shape[1]
            small = cv.resize(image, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA) if scale < 1 else image
            ok, jpeg = cv.imencode('.jpg', small, [cv.IMWRITE_JPEG_QUALITY, 85])
            images.append({'type': 'image', 'mimeType': 'image/jpeg', 'data': base64.b64encode(jpeg).decode()})

        if sheet is not None:
            seconds = min(float(sheet['seconds']), window_ms / 1000)
            report['sheets'] = []
            for name in names:
                grid, times = recording.contact_sheet(self.stream, name, now_ms - int(seconds * 1000), now_ms,
                                                      int(sheet.get('count', 12)), now_ms)
                if grid is None:
                    report['sheets'].append({'camera': name, 'error': 'no frames in that span'})
                    continue
                encode(grid, 1280)
                report['sheets'].append({'camera': name, 'frame_ages_s': [round((now_ms - t) / 1000, 1)
                                                                          for t in times]})
            report['robot'] = {'start': recording.robot_at(self.stream, now_ms - int(seconds * 1000)),
                               'end': recording.robot_at(self.stream, now_ms)}
            return report, images
        ages = list(seconds_ago if seconds_ago is not None else [0])
        if len(ages) * len(names) > 12:
            raise ValueError('at most 12 images per call (seconds_ago x cameras)')
        report['frames'] = []
        for age in ages:
            target = now_ms - int(float(age) * 1000)
            if age * 1000 > window_ms:
                raise ValueError(f"only the last {self.options['stream_window_s']} s are kept")
            entry = {'seconds_ago': age, 'robot': recording.robot_at(self.stream, target), 'images': []}
            for name in names:
                found = recording.nearest(self.stream, name, target)
                if found is None:
                    entry['images'].append({'camera': name, 'error': 'no frame near that time'})
                    continue
                t, path = found
                image = recording.label(cv.imread(str(path)), f'{name} -{(now_ms - t) / 1000:.1f}s')
                encode(image, self.options['agent_image_width'])
                entry['images'].append({'camera': name, 't_ms': t, 'age_s': round((now_ms - t) / 1000, 2),
                                        'file': str(Path('runtime/stream')/path.relative_to(self.stream))})
            report['frames'].append(entry)
        return report, images

    def _pose_report(self, state):
        tcp_q = state['tcp_quaternion']
        return {'tcp_position': np.round(state['tcp_position'], 5).tolist(),
                'tcp_quaternion': np.round(tcp_q, 5).tolist(),
                'tcp_rpy_deg': np.round(Rotation.from_quat(tcp_q).as_euler('xyz', degrees=True), 2).tolist()}

    def status(self):
        state = self.arm.state()
        value = {'robot': 'ufactory_xarm6', 'motion': arming.read(self.config.root), **self._pose_report(state),
                 'joints': np.round(np.asarray(state['joints']), 4).tolist(),
                 'controller_state': state.get('state'), 'controller_mode': state.get('mode'),
                 'error_code': state.get('error_code'), 'warn_code': state.get('warn_code'),
                 'workspace_box': {'min': self.box_min.tolist(), 'max': self.box_max.tolist()},
                 'cameras': sorted(self.models), 'depth': self.depth,
                 'stream': None if self.stream is None else {
                     'directory': 'runtime/stream', 'window_s': self.options['stream_window_s'],
                     'fps': self.options['stream_fps'], 'depth_hz': self.options['stream_depth_hz'],
                     'now_ms': int(time.time() * 1000)},
                 'motion_archive': None if self.motion_archive is None else {
                     'directory': 'motion-archive', 'padding_s': self.options['motion_archive_padding_s']},
                 'last_motion': self.last_motion}
        value['end_effector'] = self.end_effector_kind
        if self.has_hand:
            value['hand'] = self.end_effector.status()
        else:
            try:
                grip = self.end_effector.status()
                value['gripper'] = {k: grip.get(k) for k in
                                    ('width', 'max_width', 'raw_position', 'is_grasped', 'is_moving', 'stale')}
            except Exception as exc:
                value['gripper'] = {'error': str(exc)}
        return value

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        try:
            self.end_effector.close()
        finally:
            self._close_arm_and_cameras()

    def _close_arm_and_cameras(self):
        try:
            self.arm.hold()
        except Exception:
            pass
        for thread in list(self.motion_clip_threads):
            thread.join(timeout=5)
        try:
            self.cameras.close()
        finally:
            close = getattr(self.arm, 'close', None)
            if close is not None:
                close()
