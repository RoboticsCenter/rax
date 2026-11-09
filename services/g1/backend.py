"""G1 embodiment adapter for the unchanged RAX MCP frontend."""
from copy import deepcopy
from dataclasses import replace
import threading
import time
import uuid

from services.json_io import write_private_json
from services.wuji import arming
from .camera import Cameras
from .config import number, options
from .control import LocomotionWorker, MotionError, healthy
from .transport import DryRunTransport, UnitreeTransport

HAND_TOOLS = {'wuji_status', 'wuji_move_joints', 'wuji_pose', 'wuji_hold'}


def tool(name, description, properties=None, required=()):
    return {'name': name, 'description': description,
            'inputSchema': {'type': 'object', 'additionalProperties': False,
                            'properties': properties or {}, 'required': list(required)}}


class G1Backend:
    mutating = frozenset({'humanoid_move_base', 'wuji_move_joints', 'wuji_pose'})

    def __init__(self, config, *, transport=None, hands=None, cameras=None):
        self.config, self.options = config, options(config.backend_options)
        if config.observation_profile != 'rgb-only':
            raise ValueError('G1 currently requires rgb-only; calibrated RGB-D is not integrated')
        self.hands = {}
        self.worker = None
        self.cameras = None
        self._closed = False
        self._fault = None
        self._monitor_stop = threading.Event()
        self._monitor_thread = None
        try:
            self.cameras = cameras if cameras is not None else Cameras(self.options['cameras'])
            if hands is None:
                from services.wuji.client import WujiClient
                for side, arm in self.options['arms'].items():
                    child = replace(config, backend_options={'end_effector': arm['end_effector']})
                    self.hands[side] = WujiClient(child, lambda: self._hand_fault(), side=side)
            else:
                self.hands = hands
            if transport is None:
                transport = (DryRunTransport if self.options['transport'] == 'dry-run' else UnitreeTransport)(self.options)
            self.worker = LocomotionWorker(transport, self.options, self._permit)
            self._monitor_thread = threading.Thread(target=self._monitor, name='g1-supervisor', daemon=True)
            self._monitor_thread.start()
        except BaseException:
            self.close()
            raise

    def _permit(self):
        if self._closed or self._fault:
            raise MotionError(self._fault or 'G1 backend closed')
        if not self.options['motion_commissioned']:
            raise ValueError('G1 motion is not commissioned')
        armed = arming.read(self.config.root)
        if not armed['armed']:
            raise ValueError(armed['reason'])
        for hand in self.hands.values():
            state = hand.status()
            from services.wuji.partial import control_enabled
            if (not control_enabled(state, hand.cfg) or state.get('fault') or state.get('faults')
                    or state.get('mode') not in ('holding', 'executing')):
                raise MotionError('Both Wuji hands must have fresh prepared control')

    def _hand_fault(self):
        # Monitor performs coordinated cancellation outside Wuji callback threads.
        self._fault = self._fault or 'Wuji controller fault; operator recovery required'

    def _monitor(self):
        handled = False
        while not self._monitor_stop.is_set():
            try:
                state = self.worker.snapshot()
                if state.get('fault'):
                    self._fault = self._fault or state['fault']
                if any(h.status().get('mode') == 'executing' for h in self.hands.values()):
                    healthy(state, self.options)
                if self._fault and not handled:
                    handled = True
                    arming.write(self.config.root, False, reason=self._fault)
                    self._stop_all()
                write_private_json(self.config.root / 'operator' / 'g1-state.json',
                                   {'recorded_unix': time.time(), **self.status()})
            except Exception as exc:
                self._fault = self._fault or str(exc)
            self._monitor_stop.wait(.05)

    def tools(self):
        cfg = self.options
        tools = [
            tool('humanoid_status', 'Measured G1 state, controller health, capabilities and both Wuji hands.'),
            tool('humanoid_observe', 'RGB snapshots and measured state. Receipt times are not calibrated capture times.'),
            tool('humanoid_move_base', 'Bounded body-frame velocity: x forward, y left, positive yaw counterclockwise. '
                 'Uses stock balance control. Returns after command expiry and stop ACK; no position or standstill guarantee. '
                 'Requires commissioned motion and operator arming. No navigation or obstacle avoidance.',
                 {key: {'type': 'number', 'minimum': -cfg[limit], 'maximum': cfg[limit]}
                  for key, limit in (('vx', 'max_vx'), ('vy', 'max_vy'), ('yaw_rate', 'max_yaw_rate'))}
                 | {'duration_s': {'type': 'number', 'minimum': .05, 'maximum': cfg['max_duration_s']}},
                 ['vx', 'vy', 'yaw_rate', 'duration_s']),
            tool('humanoid_stop', 'Request zero walking velocity and hold both hands. Does not disable G1 balancing. '
                 'Returns individual ACKs, not proof of standstill or grip retention. MCP calls are serialized; '
                 'operator disarm interrupts ongoing motion independently.'),
            tool('humanoid_stream', 'Review one of the last 32 explicit RGB observations; not continuous video.',
                 {'seconds_ago': {'type': 'number', 'minimum': 0, 'maximum': 3600}}),
        ]
        from services.wuji.config import tools as hand_tools, DEFAULTS
        first = next(iter(self.options['arms'].values()))['end_effector'] if self.options['arms'] else DEFAULTS
        for definition in hand_tools(first):
            if definition['name'] not in HAND_TOOLS:
                continue
            definition = deepcopy(definition)
            schema = definition['inputSchema']
            schema['properties']['hand'] = {'type': 'string', 'enum': ['left', 'right']}
            schema['required'].append('hand')
            # Each side has its own limits/postures, validated at dispatch.
            if 'duration_s' in schema['properties']:
                schema['properties']['duration_s']['maximum'] = max(
                    [arm['end_effector']['max_duration_s'] for arm in self.options['arms'].values()] or [60.])
            if definition['name'] == 'wuji_pose':
                definition['description'] = 'Move the selected hand to its own configured posture; inspect wuji_status first.'
            tools.append(definition)
        return tools

    def validate(self, name, arguments):
        if not isinstance(arguments, dict):
            raise ValueError('Arguments must be an object')
        definition = next((t for t in self.tools() if t['name'] == name), None)
        if definition is None:
            raise ValueError('Unknown G1 tool')
        schema = definition['inputSchema']
        if set(arguments) - schema['properties'].keys() or set(schema['required']) - arguments.keys():
            raise ValueError('Unknown or missing tool arguments')
        if name == 'humanoid_move_base':
            for key, limit in (('vx', 'max_vx'), ('vy', 'max_vy'), ('yaw_rate', 'max_yaw_rate')):
                number(arguments[key], key, -self.options[limit], self.options[limit])
            number(arguments['duration_s'], 'duration_s', .05, self.options['max_duration_s'])
            self.worker.require_ready()
        elif name == 'humanoid_stream':
            number(arguments.get('seconds_ago', 0), 'seconds_ago', 0, 3600)
        elif name in HAND_TOOLS:
            side = arguments['hand']
            if not isinstance(side, str) or side not in ('left', 'right'):
                raise ValueError('Unknown hand')
            if side not in self.hands:
                if name == 'wuji_status':
                    return
                raise ValueError('Selected Wuji hand is not configured')
            from services.wuji.config import validate_tool
            args = {k: v for k, v in arguments.items() if k != 'hand'}
            cfg = self.options['arms'][side]['end_effector']
            validate_tool(name, args, cfg)
            if name in self.mutating:
                self.worker.require_ready()
                state = self.hands[side].require_ready()
                goal = validate_tool(name, args, cfg, reference=state['target_positions'])
                if cfg['motion_mode'] == 'scurve':
                    from services.wuji.scurve import SCurvePath
                    SCurvePath(state['target_positions'], goal['positions'], cfg, goal['duration_s'],
                               velocity=state.get('command_velocity_rad_s'),
                               acceleration=state.get('command_acceleration_rad_s2'))
                else:
                    from services.wuji.controller import Trajectory
                    Trajectory(state['target_positions'], goal['positions'], cfg, goal['duration_s'])

    def call(self, name, arguments):
        self.validate(name, arguments)
        if name == 'humanoid_status':
            return self.status(), []
        if name == 'humanoid_move_base':
            result = self.worker.move([arguments[k] for k in ('vx', 'vy', 'yaw_rate')], arguments['duration_s'])
            return result, []
        if name == 'humanoid_stop':
            return self._stop_all(), []
        if name in ('humanoid_observe', 'humanoid_stream'):
            frames = (self.cameras.observe() if name == 'humanoid_observe'
                      else self.cameras.stream(arguments.get('seconds_ago', 0)))
            state = self.status()
            metadata = [{k: v for k, v in f.items() if k != 'data'} for f in frames]
            if name == 'humanoid_observe':
                ident = uuid.uuid4().hex
                directory = self.config.root / 'agent-workspace' / 'runtime' / 'frames' / ident
                directory.mkdir(parents=True, exist_ok=True)
                for frame in frames:
                    (directory / (frame['camera'] + '.jpg')).write_bytes(frame['data'])
                write_private_json(directory / 'meta.json', {'time': time.time(), 'cameras': metadata, 'robot': state})
            return {'robot': state, 'cameras': metadata, 'historical_images': name == 'humanoid_stream'}, self.cameras.images(frames)
        side = arguments['hand']
        if side not in self.hands:
            return {'hand': side, 'configured': False, 'fresh': False}, []
        hand = self.hands[side]
        if name == 'wuji_status':
            return hand.status(), []
        if name == 'wuji_hold':
            try:
                return hand.hold(), []
            except Exception:
                self._hand_fault()
                raise
        from services.wuji.config import validate_tool
        state = hand.require_ready()
        goal = validate_tool(name, {k: v for k, v in arguments.items() if k != 'hand'},
                             self.options['arms'][side]['end_effector'], reference=state['target_positions'])
        try:
            return hand.move(**goal), []
        except Exception:
            self._hand_fault()
            self._stop_all()
            raise

    def _stop_all(self):
        result = {}
        if self.worker is not None:
            try:
                result['base'] = self.worker.stop()
            except Exception as exc:
                result['base'] = {'requested': True, 'acknowledged': False, 'error': str(exc)}
                self._fault = self._fault or str(exc)
        for side, hand in self.hands.items():
            try:
                result[side] = hand.hold()
            except Exception as exc:
                result[side] = {'error': str(exc), 'confirmed': False}
                self._fault = self._fault or str(exc)
        return result

    def status(self):
        return {'robot': 'g1', 'model': 'G1 EDU 29-DoF', 'transport': self.options['transport'],
                'capabilities': {'bounded_locomotion': True, 'cartesian_arms': False,
                                 'navigation': False, 'rgbd': False, 'wuji_hands': sorted(self.hands)},
                'motion_commissioned': self.options['motion_commissioned'],
                'armed': arming.read(self.config.root)['armed'], 'fault': self._fault,
                'body': self.worker.snapshot() if self.worker else {},
                'hands': {side: hand.status() for side, hand in self.hands.items()}}

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._monitor_stop.set()
        if self._monitor_thread:
            self._monitor_thread.join(1.)
        errors = []
        for owner in [self.worker, *self.hands.values(), self.cameras]:
            if owner is not None:
                try:
                    owner.close()
                except Exception as exc:
                    errors.append(str(exc))
        if errors:
            self._fault = '; '.join(errors)
