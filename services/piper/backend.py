"""Single-arm RAX backend, inspired by real/xarmwuji's ownership and interlocks."""
import json
from pathlib import Path
import time
import uuid

import cv2 as cv
import numpy as np
from scipy.spatial.transform import Rotation

from services.franka import arming
from services.franka import stream as recording
from .camera import Camera, image_content
from .config import options
from .kinematics import LIMITS, fk, frames, ik, vector
from .transport import PairLease
from .sdk import PiperArm
from .hand import RevoHand


def schema(properties=None, required=()):
    return {'type': 'object', 'properties': properties or {}, 'required': list(required),
            'additionalProperties': False}


def array(size, kind='number'):
    return {'type': 'array', 'items': {'type': kind}, 'minItems': size, 'maxItems': size}


def validate_schema(value, definition):
    kind = definition['type']
    valid = {'object': isinstance(value, dict), 'array': isinstance(value, (list, tuple)),
             'number': type(value) in (int, float), 'integer': type(value) is int}
    if not valid.get(kind, False):
        raise ValueError(f'Expected {kind}')
    if kind == 'object':
        properties = definition['properties']
        if set(value) - properties.keys() or set(definition['required']) - value.keys():
            raise ValueError('Unknown or missing tool arguments')
        for name, item in value.items():
            validate_schema(item, properties[name])
    elif kind == 'array':
        if not definition['minItems'] <= len(value) <= definition['maxItems']:
            raise ValueError('Invalid vector size')
        for item in value:
            validate_schema(item, definition['items'])
    elif not np.isfinite(value):
        raise ValueError('Numbers must be finite')


class PiperBackend:
    mutating = frozenset({'piper_move_joints', 'piper_move_pose', 'revo_move', 'piper_hold'})

    def __init__(self, config, *, arm=None, hand=None, camera=None):
        if config.observation_profile != 'rgb-only':
            raise ValueError('Piper currently requires rgb-only; depth/TCP camera calibration is not commissioned')
        self.config, self.options = config, options(config.backend_options)
        self._closed = False
        self._lease = None
        self.arm, self.hand, self.camera = arm, hand, camera
        self._arm_dispatched = self._hand_dispatched = False
        self.tool = np.eye(4)
        self.tool[:3, :3] = Rotation.from_quat(self.options['tool_quaternion_xyzw']).as_matrix()
        self.tool[:3, 3] = self.options['tool_translation_m']
        self.box_min = np.asarray(self.options['workspace_min'])
        self.box_max = np.asarray(self.options['workspace_max'])
        self.last_hand = None
        try:
            if arm is None or hand is None:
                self._lease = PairLease(self.options['arm_usb_serial'])
            self.arm = arm if arm is not None else PiperArm(self.options, permit=lambda: (
                self.options['motion_commissioned'] and arming.read(self.config.root)['armed']),
                guard=self._check_configuration)
            self.arm.state()
            self.hand = hand if hand is not None else RevoHand(self.options, config.root)
            self.last_hand = self.hand.state()
            self.camera = camera if camera is not None else Camera(self.options, config.root, self._robot_record)
        except BaseException:
            self.close()
            raise

    def tools(self):
        return [
            {'name': 'piper_status', 'description': 'Measured right-arm joints, flange/TCP pose, Revo feedback and interlocks.',
             'inputSchema': schema()},
            {'name': 'piper_observe', 'description': 'Fresh overview RGB and measured state; saves evidence. No metric depth or extrinsics.',
             'inputSchema': schema()},
            {'name': 'piper_move_joints', 'description': 'Six absolute joints in radians. Small, fully checked joint-space path; operator arming required.',
             'inputSchema': schema({'positions': array(6)}, ['positions'])},
            {'name': 'piper_move_pose', 'description': 'Absolute calibrated hand TCP pose in Piper base: metres, quaternion xyzw. Joint-interpolated path, not a straight Cartesian path.',
             'inputSchema': schema({'position': array(3), 'quaternion': array(4)}, ['position', 'quaternion'])},
            {'name': 'revo_move', 'description': 'Six normalized motor targets, NOT radians or gripper width: thumb flex, thumb opposition, index, middle, ring, little. Small bounded steps; no current mode or stored actions.',
             'inputSchema': schema({'positions': array(6, 'integer')}, ['positions'])},
            {'name': 'piper_hold', 'description': 'Best-effort cancel of this assembly motion; never disables motors or opens fingers.',
             'inputSchema': schema()},
            {'name': 'piper_stream', 'description': 'Review RGB history: seconds_ago up to the configured retention window; default latest. Robot records are read-only.',
             'inputSchema': schema({'seconds_ago': {'type': 'array', 'items': {'type': 'number'},
                                                   'minItems': 1, 'maxItems': 12}})},
        ]

    def _require_armed(self):
        if not self.options['motion_commissioned']:
            raise ValueError('Motion is not commissioned by the operator')
        value = arming.read(self.config.root)
        if not value['armed']:
            raise ValueError(value['reason'])

    def _ready(self, state):
        feedback = state.get('independent_feedback')
        if feedback and (not feedback['sdk_agrees'] or feedback['arm_status'] or feedback['error']
                         or feedback['enabled'] != [True] * 6 or any(feedback['motor_faults'])):
            raise RuntimeError('Independent CAN feedback is not healthy, enabled and consistent with the SDK')
        if (state.get('serial') != self.options['arm_usb_serial'] or state.get('side') != 'right'
                or state.get('motion_fault') or not state.get('firmware_verified', True)
                or state.get('ctrl_mode', 1) != 1
                or state.get('arm_status') != 0 or state.get('error') != 0
                or state.get('enabled') != [True] * 6):
            raise RuntimeError('Right arm must already be healthy and enabled; RAX never enables it automatically')

    def _box(self, position):
        if np.any(position < self.box_min) or np.any(position > self.box_max):
            raise ValueError('TCP path leaves the operator workspace')

    def _check_joints(self, joints):
        if np.any(joints < LIMITS[:, 0]) or np.any(joints > LIMITS[:, 1]):
            raise ValueError('Piper joint limits exceeded')

    def _check_configuration(self, joints):
        self._check_joints(joints)
        chain = frames(joints, self.tool)
        self._box(chain[-1][:3, 3])
        if any(frame[2, 3] < self.options['link_min_z'] for frame in chain[2:]):
            raise ValueError('Arm or tool origin crosses the configured table clearance')
        if chain[-2][2, 3] - self.options['hand_extent_m'] < self.options['link_min_z']:
            raise ValueError('Conservative hand envelope crosses the configured table clearance')

    def _joint_goal(self, name, arguments, state):
        current = vector(state['joints'], 6, 'joint feedback')
        if name == 'piper_move_joints':
            goal = vector(arguments['positions'], 6, 'positions')
        else:
            if not self.options['tool_calibrated']:
                raise ValueError('Cartesian motion requires an operator-calibrated hand TCP')
            target = np.eye(4)
            target[:3, 3] = vector(arguments['position'], 3, 'position')
            quaternion = vector(arguments['quaternion'], 4, 'quaternion')
            if abs(np.linalg.norm(quaternion) - 1) > 1e-5:
                raise ValueError('Quaternion must have unit norm')
            target[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
            actual = fk(current, self.tool)
            if np.linalg.norm(target[:3, 3] - actual[:3, 3]) > self.options['max_translation_m']:
                raise ValueError('Cartesian translation step too large')
            angle = Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).magnitude()
            if angle > self.options['max_rotation_rad']:
                raise ValueError('Cartesian orientation step too large')
            goal = ik(target, current, self.tool)
        step = float(np.max(np.abs(goal - current)))
        if step > self.options['max_joint_step_rad']:
            raise ValueError('Joint step too large')
        for fraction in np.linspace(0, 1, max(3, int(np.ceil(step / .002)) + 1)):
            self._check_configuration(current + fraction * (goal - current))
        return goal

    def _hand_goal(self, arguments):
        goal = vector(arguments['positions'], 6, 'hand positions')
        if np.any(goal < self.options['hand_target_min']) or np.any(goal > self.options['hand_target_max']):
            raise ValueError('Hand target outside conservative bounds')
        current = self.hand.state()
        self.last_hand = current
        if any(speed != 0 for speed in current['speeds']) or any(code == 1 for code in current['motor_states']):
            raise ValueError('Hand is already moving; refusing another command')
        if np.max(np.abs(goal - current['positions'])) > self.options['hand_max_step']:
            raise ValueError('Hand step too large')
        return goal.astype(int).tolist()

    def validate(self, name, arguments):
        definitions = {tool['name']: tool['inputSchema'] for tool in self.tools()}
        if name not in definitions:
            raise ValueError(f'Unknown Piper tool {name!r}')
        validate_schema(arguments, definitions[name])
        if name == 'piper_stream':
            if any(not 0 <= age <= self.options['stream_window_s'] for age in arguments.get('seconds_ago', [])):
                raise ValueError('Requested stream age outside retention window')
        if name in self.mutating - {'piper_hold'}:
            self._require_armed()
            state = self.arm.state()
            self._ready(state)
            if state['busy']:
                raise ValueError('An arm trajectory is already active')
            if name == 'revo_move':
                self._hand_goal(arguments)
            else:
                self._joint_goal(name, arguments, state)

    def call(self, name, arguments):
        self.validate(name, arguments)
        if name == 'piper_status':
            return self.status(), []
        if name == 'piper_observe':
            return self._observe()
        if name == 'piper_stream':
            return self._stream(arguments.get('seconds_ago', [0]))
        if name == 'piper_hold':
            return self._hold(), []
        if name == 'revo_move':
            return self._move_hand(arguments), []
        return self._move_arm(name, arguments), []

    def _guard(self, expected=None):
        self._require_armed()
        state = self.arm.state()
        self._ready(state)
        self.last_hand = self.hand.state()
        joints = np.asarray(state['joints'])
        self._check_configuration(joints)
        if expected is not None and np.max(np.abs(joints - expected)) > self.options['max_following_error_rad']:
            raise RuntimeError('Measured arm deviated from the commanded path')
        return state

    def _move_arm(self, name, arguments):
        state = self._guard()
        start = np.asarray(state['joints'])
        goal = self._joint_goal(name, arguments, state)
        distance = float(np.max(np.abs(goal - start)))
        duration = max(self.options['segment_s'], 1.5 * distance / self.options['joint_speed_radps'],
                       np.sqrt(6 * distance / self.options['joint_acc_radps2']))
        count = int(np.ceil(duration / self.options['segment_s']))
        seconds = duration / count
        deadline = time.monotonic() + self.options['move_timeout_s']
        expected = start
        try:
            for index in range(1, count + 1):
                self._guard(expected)
                if time.monotonic() > deadline:
                    raise TimeoutError('Arm motion timed out')
                fraction = index / count
                target = start + fraction ** 2 * (3 - 2 * fraction) * (goal - start)
                self._check_configuration(target)
                self._arm_dispatched = True
                self.arm.segment(target, seconds)
                while True:
                    self._guard(target)
                    if time.monotonic() > deadline:
                        raise TimeoutError('Arm motion timed out')
                    if not self.arm.state()['busy']:
                        break
                    time.sleep(.01)
                expected = target
            while np.max(np.abs(np.asarray(self._guard(goal)['joints']) - goal)) > self.options['joint_tolerance_rad']:
                if time.monotonic() > deadline:
                    raise TimeoutError('Arm failed to settle at the target')
                time.sleep(.02)
            self._arm_dispatched = False
            return {'ok': True, **self._pose(self.arm.state())}
        except BaseException:
            self._hold()
            raise

    def _move_hand(self, arguments):
        goal = self._hand_goal(arguments)
        try:
            self._guard()
            self._hand_dispatched = True
            self.hand.move(goal)
            deadline = time.monotonic() + 1.5
            earliest = time.monotonic() + self.options['hand_segment_ms'] / 1000
            while True:
                self._guard()
                self.last_hand = self.hand.state()
                if time.monotonic() >= earliest and not any(self.last_hand['speeds']):
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError('Hand did not settle')
                time.sleep(.02)
            self._hand_dispatched = False
            errors = np.abs(np.asarray(goal) - self.last_hand['positions'])
            return {'ok': bool(np.all(errors <= 30)), 'hand': self.last_hand,
                    'targets': goal, 'blocked_motors': np.flatnonzero(errors > 30).tolist(),
                    'grasp_verified': False}
        except BaseException:
            self._hold()
            raise

    def _hold(self):
        errors = []
        for name, device in (('arm', self.arm), ('hand', self.hand)):
            if device is None:
                continue
            try:
                device.hold()
                setattr(self, f'_{name}_dispatched', False)
            except Exception as exc:
                errors.append(f'{name}: {exc}')
        if errors:
            raise RuntimeError('Assembly stop unconfirmed: ' + '; '.join(errors))
        return {'ok': True, 'stop_requested': True, 'physical_stop_verified': False}

    def _pose(self, state):
        flange = fk(state['joints'])
        tool = flange @ self.tool
        return {'joints': state['joints'], 'flange_position': flange[:3, 3].tolist(),
                'flange_quaternion': Rotation.from_matrix(flange[:3, :3]).as_quat().tolist(),
                'tcp_position': tool[:3, 3].tolist(),
                'tcp_quaternion': Rotation.from_matrix(tool[:3, :3]).as_quat().tolist(),
                'tcp_calibrated': self.options['tool_calibrated'],
                'pose_source': 'Piper offset-variant DH model; not independently calibrated'}

    def _robot_record(self):
        return {**self._pose(self.arm.state()), 'hand': self.last_hand}

    def status(self):
        state = self.arm.state()
        self.last_hand = self.hand.state()
        try:
            self._ready(state)
            self._check_configuration(np.asarray(state['joints']))
            ready, reason = True, None
        except (RuntimeError, ValueError) as exc:
            ready, reason = False, str(exc)
        return {'robot': 'piper', 'side': 'right', **self._pose(state),
                'arm_serial': state['serial'], 'arm_interface': state.get('interface'),
                'independent_feedback': state.get('independent_feedback'),
                'arm_driver_state': state['driver_state'], 'arm_ready': ready,
                'arm_driver': state.get('driver'), 'arm_firmware': state.get('firmware'),
                'arm_firmware_verified': state.get('firmware_verified', False),
                'motion_ready': ready and self.options['motion_commissioned'] and arming.read(self.config.root)['armed'],
                'not_ready_reason': reason, 'motion_commissioned': self.options['motion_commissioned'],
                'arming': arming.read(self.config.root), 'hand': self.last_hand,
                'camera_geometry': False, 'cameras': ['overview'],
                'workspace_min': self.box_min.tolist(), 'workspace_max': self.box_max.tolist()}

    def _observe(self):
        timestamp, data = self.camera.grab()
        frame_id = uuid.uuid4().hex
        folder = self.config.root / 'agent-workspace/runtime/frames' / frame_id
        folder.mkdir(parents=True)
        (folder / 'overview.jpg').write_bytes(data)
        image = cv.imdecode(np.frombuffer(data, np.uint8), cv.IMREAD_COLOR)
        report = {'frame_id': frame_id, 'time': timestamp, 'camera_geometry': False,
                  **self._robot_record(), 'cameras': {'overview': {
                      'rgb': 'overview.jpg', 'depth': None, 'serial': self.options['cameras'][0]['serial'],
                      'width': image.shape[1], 'height': image.shape[0],
                      'timestamp_ns': int(timestamp * 1e9), 'timestamp_source': 'host_receive'}}}
        (folder / 'meta.json').write_text(json.dumps(report, indent=2) + '\n')
        return {**report, 'directory': f'runtime/frames/{frame_id}',
                'images_in_order': ['overview']}, [image_content(data)]

    def _stream(self, ages):
        root = self.config.root / 'stream'
        now = time.time()
        images, selected = [], []
        for age in ages:
            match = recording.nearest(root, 'overview', int((now - age) * 1000), tolerance_ms=1000)
            if match is None:
                selected.append({'seconds_ago': age, 'available': False})
                continue
            timestamp, path = match
            images.append(image_content(path.read_bytes()))
            selected.append({'seconds_ago': (now * 1000 - timestamp) / 1000,
                             'available': True, 'robot': recording.robot_at(root, timestamp)})
        return {'camera': 'overview', 'frames': selected}, images

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._arm_dispatched or self._hand_dispatched:
            try:
                self._hold()
            except Exception:
                pass
        try:
            if self.camera is not None:
                self.camera.close()
        finally:
            try:
                if self.hand is not None:
                    self.hand.close()
            finally:
                try:
                    if self.arm is not None:
                        self.arm.close()
                finally:
                    if self._lease is not None:
                        self._lease.close()
