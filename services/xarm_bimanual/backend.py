"""Bimanual orchestration over the existing xArm and Wuji controllers.

Like upstream real/rainbow, tools select an arm explicitly and paired calls admit
all targets before dispatch. Independent xArm controllers do not share a clock:
paired moves start concurrently, but do not promise synchronized trajectories.
"""
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
from dataclasses import replace
import base64
import json
from pathlib import Path
import threading
import time
import uuid

import cv2 as cv

from services.xarm import arming
from services.xarm.backend import XArmBackend, CameraService, DEFAULTS, _schema, _validate_arguments
from services.xarm.faults import ControllerFault
from services.xarm.sdk import XArmDriver
from services.wuji.client import WujiClient
from services.wuji.config import MUTATING_TOOLS, TOOL_NAMES
from .config import SIDES, PAIRED_TOOLS, validate_options, arm_options


class _NoCameras:
    def close(self):
        pass


class _Assembly(XArmBackend):
    """Keep the single-arm motion implementation, with whole-rig fault handling."""
    def __init__(self, owner, side, config, arm, hand):
        self.owner, self.side = owner, side
        super().__init__(config, arm=arm, hand=hand, cameras=_NoCameras(), camera_models={})

    def _stop_assembly(self, **kwargs):
        return self.owner._stop_assembly(**kwargs)

    def handle_control_fault(self, exc):
        exc.robot_state = {**getattr(exc, 'robot_state', {}), 'arm': self.side}
        return self.owner.handle_control_fault(exc)


class XArmBimanualBackend(XArmBackend):
    mutating = frozenset({'xarm_move_pose', 'xarm_move_joints', *MUTATING_TOOLS, *PAIRED_TOOLS})
    recovery_tools = frozenset({'xarm_hold'})

    def __init__(self, config, *, arms=None, hands=None, cameras=None):
        configured = validate_options(config.backend_options)
        self.config = config
        self.options = {**DEFAULTS, **configured}
        self.arms = {}
        self._closed = False
        self._stop = threading.Event()
        self._control_fault_lock = threading.Lock()
        self._last_control_fault = None
        self._robot_log_error = None
        self.control_uncertain = False
        self.has_hand = True
        self.last_motion = None
        self.motion_clip_threads = []
        self.models = {c['name']: c for c in configured['cameras']}  # Identities only, not camera transforms.
        self.workspace = config.root/'agent-workspace'
        self.frames_root = self.workspace/'runtime'/'frames'
        self.depth = config.observation_profile == 'rgbd'
        self.stream = config.root/'stream' if self.options['stream_enabled'] else None
        self.motion_archive = config.root/'motion-archive' if self.stream is not None \
            and self.options['motion_archive_enabled'] else None
        self.cameras = cameras
        self._telemetry = None
        try:
            for side in SIDES:
                child_config = replace(config, robot='xarm', backend_options=arm_options(configured, side))
                driver = arms[side] if arms is not None else XArmDriver({**DEFAULTS, **child_config.backend_options})
                hand = None
                try:
                    hand = hands[side] if hands is not None else WujiClient(
                        child_config, lambda side=side: self._hand_fault(side), side=side)
                    self.arms[side] = _Assembly(self, side, child_config, driver, hand)
                except BaseException:
                    if hand is not None:
                        hand.close()
                    close = getattr(driver, 'close', None)
                    if close:
                        close()
                    raise
            if self.cameras is None:
                self.cameras = CameraService(self.options, self.frames_root, config.root/'camera-server.log',
                    self.options['camera_depth_mode'] if self.depth else 'NONE', stream=self.stream)
                if self.stream is not None:
                    self.stream.mkdir(parents=True, exist_ok=True)
                    self.cameras.start()
            if self.motion_archive is not None:
                self.motion_archive.mkdir(parents=True, exist_ok=True)
            # Watch both controllers even when recording is disabled or a hand is moving.
            self._telemetry = threading.Thread(target=self._log_robot, name='bimanual-robot-log', daemon=True)
            self._telemetry.start()
        except BaseException:
            self.close()
            raise

    def _hand_fault(self, side):
        if self._closed:
            return
        self.handle_control_fault(ControllerFault(f'{side} Wuji hand lost healthy control',
                                                  category='hand', state={'hand': side}))

    def tools(self):
        definitions = []
        singles = {side: {t['name']: t for t in arm.tools()} for side, arm in self.arms.items()}
        for name in ('xarm_move_pose', 'xarm_move_joints', *TOOL_NAMES):
            definition = deepcopy(singles['left'][name])
            selector = 'hand' if name.startswith('wuji_') else 'arm'
            definition['inputSchema']['properties'][selector] = {'type': 'string', 'enum': list(SIDES)}
            definition['inputSchema']['required'].append(selector)
            # The selected side enforces its own caps/posture names. The public schema
            # must not accidentally apply the left side's lower cap to the right side.
            for key, prop in definition['inputSchema']['properties'].items():
                right = singles['right'][name]['inputSchema']['properties'].get(key, {})
                if 'maximum' in prop and 'maximum' in right:
                    prop['maximum'] = max(prop['maximum'], right['maximum'])
                if 'default' in prop and prop.get('default') != right.get('default'):
                    prop.pop('default')
            if name == 'xarm_move_pose':
                definition['description'] = (
                    'Move the selected TCP using the existing Cartesian line planner and measured settling. '
                    'position is metres in that arm base; quaternion is [x,y,z,w], omitted preserves orientation. '
                    'relative adds a base-frame displacement. Read that side’s workspace and motion_limits '
                    'from xarm_status. speed_mps may lower its configured speed; timeout_s is 1..120 seconds. '
                    'Idle off-target or stalled motion stops early and alerts the operator.')
            elif name == 'xarm_move_joints':
                definition['description'] = (
                    'Move the selected arm to six absolute joint angles in radians, or preset home. '
                    'Validates the full sampled FK path and executes bounded SDK segments with existing '
                    'limits, progress detection and measured settling. Read that side’s motion_limits.')
            if name == 'wuji_pose':
                postures = sorted(set().union(*(arm.hand_config['postures'] for arm in self.arms.values())))
                if postures:
                    definition['inputSchema']['properties']['name']['enum'] = postures
                definition['description'] = ('Move the selected hand to one of its own configured postures. '
                    'Read wuji_status(hand=...) for available names; a name on the other hand is not sufficient.')
            definition['description'] = (f'Required {selector}: left or right. Uses only that {selector}; '
                'the other side retains its target. Arm coordinates are in the selected arm base frame. '
                'Read xarm_status and wuji_status for that side’s limits and postures. '
                + definition['description'])
            definitions.append(definition)
        for name, single in PAIRED_TOOLS.items():
            definitions.append({'name': name, 'description':
                f'Run {single} for left and right concurrently in one MCP call. Both targets and both '
                'assemblies are checked before either command starts. Each arm target uses its own base '
                'frame. This is not synchronized interpolation or collision planning; verify both sweeps. '
                'Any execution failure disarms and requests stops on all four devices. Results are per side.',
                'inputSchema': _schema({side: deepcopy(singles[side][single]['inputSchema']) for side in SIDES}, SIDES)})
        camera_list = {'type': 'array', 'items': {'type': 'string', 'enum': list(self.models)}, 'minItems': 1,
                       'maxItems': 4}
        definitions += [
            {'name': 'xarm_status', 'inputSchema': _schema(), 'description':
                'Both arms and both hands, identities, per-arm base frames and limits, shared arming and four cameras.'},
            {'name': 'xarm_hold', 'inputSchema': _schema(), 'description':
                'Disarm and independently stop both arms and both hands. Verify each stop; clear uncertainty '
                'only if all four are confirmed. Does not enable or clear faults. MCP calls are serialized; '
                'operator direct disarm/stop bypasses the MCP queue.'},
            {'name': 'xarm_observe', 'inputSchema': _schema({'cameras': camera_list}), 'description':
                'Capture the two side and two wrist RealSense views, plus state of both assemblies. '
                'Saves RGB, optional camera depth, and metadata with camera identities and wrist owners. '
                'Calibration is pending: no camera-to-arm transforms, projected points or base-frame localization.'},
        ]
        stream = deepcopy(next(t for t in self.arms['left'].tools() if t['name'] == 'xarm_stream'))
        stream['inputSchema']['properties']['cameras'] = camera_list
        stream['description'] = 'Review the four-camera rolling recording and both assemblies’ state. ' + stream['description']
        definitions.append(stream)
        return definitions

    def _selected(self, name, arguments):
        selector = 'hand' if name.startswith('wuji_') else 'arm'
        return arguments[selector], {k: v for k, v in arguments.items() if k != selector}

    def _ready(self):
        self._require_armed()
        for side, arm in self.arms.items():
            try:
                arm._motion_state()
                arm.end_effector.require_ready()
            except Exception as exc:
                if getattr(exc, 'operator_required', False):
                    exc.robot_state = {**getattr(exc, 'robot_state', {}), 'arm': side}
                    raise
                raise ControllerFault(f'{side} assembly is not ready: {exc}', category='hand',
                                      state={'arm': side}) from exc

    def validate(self, name, arguments):
        if (name in ('xarm_move_pose', 'xarm_move_joints', 'xarm_move_both', 'xarm_move_joints_both')
                and any(a.hand_config['disabled_joints'] for a in self.arms.values())):
            raise ValueError('arm movement is unavailable in partial-hand diagnostic mode')
        definition = next((t for t in self.tools() if t['name'] == name), None)
        if definition is None:
            raise ValueError(f'unknown tool {name}')
        _validate_arguments(arguments, definition['inputSchema'])
        if name in self.mutating:
            self._ready()
        if name in PAIRED_TOOLS:
            for side in SIDES:
                self.arms[side].validate(PAIRED_TOOLS[name], arguments[side])
        elif name in {'xarm_move_pose', 'xarm_move_joints', *TOOL_NAMES}:
            side, payload = self._selected(name, arguments)
            self.arms[side].validate(name, payload)
        elif name == 'xarm_stream':
            if 'sheet' in arguments and 'seconds_ago' in arguments:
                raise ValueError('give either sheet or seconds_ago')
            ages = arguments.get('seconds_ago', [0])
            if any(age > self.options['stream_window_s'] for age in ages):
                raise ValueError('requested time exceeds the stream window')
            if len(ages) * len(arguments.get('cameras') or self.models) > 12:
                raise ValueError('at most 12 images per call')

    def call(self, name, arguments):
        # Direct callers get the same all-target admission as the MCP frontend.
        try:
            self.validate(name, arguments)
        except Exception as exc:
            if getattr(exc, 'operator_required', False):
                self.handle_control_fault(exc)
            raise
        started = time.time()
        try:
            if name in PAIRED_TOOLS:
                value = self._paired(PAIRED_TOOLS[name], arguments)
                result = value, []
            elif name in {'xarm_move_pose', 'xarm_move_joints', *TOOL_NAMES}:
                side, payload = self._selected(name, arguments)
                value, images = self.arms[side].call(name, payload)
                result = {('hand' if name.startswith('wuji_') else 'arm'): side,
                          'base_frame': f'{side}_base', **value}, images
            else:
                result = getattr(self, '_' + name.removeprefix('xarm_'))(**arguments)
        except Exception as exc:
            if getattr(exc, 'operator_required', False):
                self.handle_control_fault(exc)
            elif name in self.mutating:
                outcome = self._stop_assembly(verify_arm=True, require_healthy=False)
                exc.robot_state = {**getattr(exc, 'robot_state', {}), 'assembly_stop': outcome}
                exc.control_certain = all(v.get('confirmed') is True for v in outcome.values())
                self.control_uncertain = not exc.control_certain
            if name in self.mutating:
                self._schedule_motion_clip(name, started, time.time(), 'failed', exc)
            raise
        if name in self.mutating:
            self.control_uncertain = False
            value = result[0]
            clip = self._schedule_motion_clip(name, started, time.time(), 'completed')
            if clip:
                value['motion_clip'] = clip
            self.last_motion = {'tool': name, 'at': time.time(), **value}
        return result

    def _paired(self, name, arguments):
        gate = threading.Barrier(2)
        results, failures, starts = {}, {}, {}
        def run(side):
            gate.wait(timeout=5)
            self._require_armed()
            starts[side] = time.monotonic()
            return self.arms[side].call(name, arguments[side])[0]
        with ThreadPoolExecutor(max_workers=2, thread_name_prefix='bimanual-move') as pool:
            futures = {pool.submit(run, side): side for side in SIDES}
            for future in as_completed(futures):
                side = futures[future]
                try:
                    results[side] = future.result()
                except Exception as exc:
                    failures[side] = exc
                    self._stop_assembly()  # Stop the peer immediately, before waiting for it.
        if failures:
            exc = next(iter(failures.values()))
            exc.robot_state = {**getattr(exc, 'robot_state', {}), 'completed_sides': results,
                               'failed_sides': {s: str(e) for s, e in failures.items()}}
            raise exc
        return {'results': results, 'dispatch_skew_s': abs(starts['left'] - starts['right']),
                'synchronized_trajectory': False}

    def _stop_assembly(self, *, verify_arm=False, require_healthy=True):
        # Disarm before any potentially blocked transport and fan out across devices.
        disarm_error = None
        try:
            arming.write(self.config.root, False, reason='bimanual stop; operator readiness required')
        except Exception as exc:
            disarm_error = str(exc)
        outcomes, lock = {}, threading.Lock()
        def stop(side, assembly):
            result = XArmBackend._stop_assembly(assembly, verify_arm=verify_arm, require_healthy=require_healthy)
            with lock:
                outcomes.update({f'{side}_{device}': value for device, value in result.items()})
        threads = [threading.Thread(target=stop, args=(s, a), daemon=True) for s, a in list(self.arms.items())]
        for thread in threads:
            thread.start()
        deadline = time.monotonic() + max((a.hand_config['stop_timeout_s']+2.1 for a in self.arms.values()), default=2)
        for thread in threads:
            thread.join(max(0, deadline-time.monotonic()))
        with lock:
            return {f'{s}_{d}': {**outcomes.get(f'{s}_{d}', {'requested': True, 'confirmed': False, 'pending': True}),
                                **({'disarm_error': disarm_error, 'confirmed': False} if disarm_error else {})}
                    for s in self.arms for d in ('arm', 'hand')}

    def status(self):
        reports = {}
        for side, arm in self.arms.items():
            try:
                reports[side] = {**arm.status(), 'base_frame': f'{side}_base',
                                 'xarm_ip': arm.options['xarm_ip'], 'motion_limits': {
                                     key: arm.options[key] for key in ('cartesian_speed_mps',
                                     'cartesian_acc_mps2', 'joint_speed_radps', 'joint_acc_radps2',
                                     'max_translation_m', 'max_joint_step_rad', 'move_timeout_s')}}
            except Exception as exc:
                reports[side] = {'error': str(exc), 'base_frame': f'{side}_base',
                                 'xarm_ip': arm.options['xarm_ip']}
        return {'robot': 'xarm6_wuji_bimanual', 'motion': arming.read(self.config.root), 'arms': reports,
                'cameras': deepcopy(self.options['cameras']), 'depth': self.depth,
                'calibration_status': 'pending', 'shared_base_frame': False,
                'control_uncertain': self.control_uncertain, 'last_controller_fault': self._last_control_fault,
                'robot_log_error': self._robot_log_error,
                'last_motion': self.last_motion, 'stream': None if self.stream is None else {
                    'directory': 'runtime/stream', 'fps': self.options['stream_fps'],
                    'window_s': self.options['stream_window_s'], 'depth_hz': self.options['stream_depth_hz']}}

    def _log_robot(self):
        from services.xarm.camera_server import minute_name
        period = 1. / float(self.options['robot_log_hz'])
        folder = self.stream/'robot' if self.stream else None
        while not self._stop.wait(period):
            for side, arm in list(self.arms.items()):
                if arming.read(self.config.root)['armed']:
                    try:
                        arm._motion_state()
                    except ControllerFault as exc:
                        exc.robot_state = {**getattr(exc, 'robot_state', {}), 'arm': side}
                        self.handle_control_fault(exc)
            if folder:
                try:
                    folder.mkdir(parents=True, exist_ok=True)
                    now = time.time()
                    with (folder/f'{minute_name(now)}.jsonl').open('a') as log:
                        log.write(json.dumps({'t_ms': int(now*1000), **self.status()})+'\n')
                    self._robot_log_error = None
                except Exception as exc:
                    # Recording failure must not terminate controller supervision.
                    self._robot_log_error = str(exc)

    def _observe(self, cameras=None):
        names = cameras or list(self.models)
        frame_id = time.strftime('%Y%m%d%H%M%S') + uuid.uuid4().hex[:6]
        out_dir = self.frames_root/frame_id
        before = self.status()
        captured = self.cameras.request({'op': 'grab', 'cameras': names, 'depth': self.depth,
                                        'out_dir': str(out_dir)})['cameras']
        after = self.status()
        meta = {'frame_id': frame_id, 'time': time.time(), 'calibration_status': 'pending',
                'arms_before': before['arms'], 'arms': after['arms'],
                'depth_units': 'mm (uint16, 0 = invalid); camera coordinates only', 'cameras': {}}
        images, report = [], {}
        for name in names:
            entry, spec = captured[name], self.models[name]
            meta['cameras'][name] = {**entry, 'rgb': Path(entry['rgb']).name,
                'depth': Path(entry['depth']).name if entry.get('depth') else None,
                'serial': spec['serial'], 'mount': spec['mount'], 'arm': spec.get('arm'),
                'calibration_status': 'pending'}
            image = cv.imread(entry['rgb'])
            if image is None:
                raise RuntimeError(f'{name}: captured RGB could not be read')
            scale = min(1., self.options['agent_image_width']/image.shape[1])
            small = cv.resize(image, None, fx=scale, fy=scale, interpolation=cv.INTER_AREA) if scale < 1 else image
            ok, jpeg = cv.imencode('.jpg', small, [cv.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise RuntimeError(f'{name}: preview encoding failed')
            images.append({'type': 'image', 'mimeType': 'image/jpeg', 'data': base64.b64encode(jpeg).decode()})
            report[name] = {'preview_scale': scale, 'width': entry['width'], 'height': entry['height'],
                            'serial': spec['serial'], 'mount': spec['mount'], 'arm': spec.get('arm'),
                            'captured_ms': entry['timestamp_ns']//1_000_000}
        (out_dir/'meta.json').write_text(json.dumps(meta, indent=2))
        return {'frame_id': frame_id, 'directory': f'runtime/frames/{frame_id}', 'images_in_order': names,
                'cameras': report, 'arms': after['arms'], 'calibration_status': 'pending'}, images

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self._telemetry and self._telemetry is not threading.current_thread():
            self._telemetry.join(timeout=8)
        try:
            self._stop_assembly()
        finally:
            for arm in self.arms.values():
                try:
                    arm.close()
                except Exception:
                    pass
            for thread in self.motion_clip_threads:
                thread.join(timeout=5)
            if self.cameras is not None:
                self.cameras.close()
