"""Two xArm/Revo pairs behind one serialized MCP frontend and one camera owner."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import replace
import json
import threading
import time

from . import arming
from .backend import XArmBackend, CameraService, DEFAULTS, LAUNCHER_OPTIONS, _schema, _validate_arguments
from .bimanual_config import SIDES, validate_config
from .calibration import CameraModel


class SharedCameras:
    def __init__(self, owner):
        self.owner = owner

    def request(self, *args, **kwargs):
        return self.owner.request(*args, **kwargs)

    def close(self):
        pass  # Only the bimanual backend owns this service.


class BimanualXArmBackend:
    recovery_tools = frozenset({'xarm_hold'})

    def __init__(self, config, *, arms=None, hands=None, cameras=None, monitor=True):
        self.config = config
        settings = validate_config(config.backend_options)
        # Validate both profiles/calibrations before connecting either robot.
        models = {}
        for side, cfg in settings.items():
            unknown = set(cfg) - set(DEFAULTS) - set(LAUNCHER_OPTIONS)
            if unknown:
                raise ValueError(f'unknown {side} xarm options: {sorted(unknown)}')
            models[side] = {c['name']: CameraModel.from_spec(c) for c in cfg['cameras']}
        self.sides = {}
        self._closed = False
        self._stop = threading.Event()
        self._stop_lock = threading.Lock()
        self.control_uncertain = False
        self.last_fault = None
        options = {**DEFAULTS, **settings['left']}
        options['cameras'] = list({c['name']: c for cfg in settings.values() for c in cfg['cameras']}.values())
        stream = config.root/'stream' if options['stream_enabled'] else None
        self.cameras = cameras if cameras is not None else CameraService(
            options, config.root/'agent-workspace'/'runtime'/'frames', config.root/'camera-server.log',
            options['camera_depth_mode'] if config.camera_geometry else 'NONE', stream=stream)
        self.thread = None
        try:
            for side in SIDES:
                child_config = replace(config, robot='xarm', backend_options=settings[side])
                child = XArmBackend(child_config, arm=(arms or {}).get(side), hand=(hands or {}).get(side),
                                    cameras=SharedCameras(self.cameras), camera_models=models[side], arm_name=side)
                self.sides[side] = child
            self.mutating = frozenset().union(*(b.mutating for b in self.sides.values()), {'xarm_move_bimanual'})
            if cameras is None and stream is not None:
                stream.mkdir(parents=True, exist_ok=True)
                self.cameras.start()
            if monitor:
                self.thread = threading.Thread(target=self._watch, name='bimanual-watchdog', daemon=True)
                self.thread.start()
        except BaseException:
            self.close()
            raise

    def tools(self):
        grouped = {}
        for side, child in self.sides.items():
            for tool in child.tools():
                if tool['name'] not in ('xarm_status', 'xarm_hold'):
                    grouped.setdefault(tool['name'], {})[side] = tool
        definitions = [
            {'name': 'xarm_status', 'inputSchema': _schema(),
             'description': 'Read both xArm/Revo pairs, arming and faults. Poses use each arm\'s own base frame.'},
            {'name': 'xarm_hold', 'inputSchema': _schema(),
             'description': 'Disarm and independently stop BOTH arms and BOTH hands; return each stop confirmation.'},
        ]
        for name, choices in grouped.items():
            # oneOf retains each side's actual camera/posture enums and motion limits.
            alternatives = []
            for side, definition in choices.items():
                schema = deepcopy(definition['inputSchema'])
                schema['properties']['arm'] = {'type': 'string', 'enum': [side]}
                schema['required'].append('arm')
                alternatives.append(schema)
            descriptions = {definition['description'] for definition in choices.values()}
            description = (next(iter(descriptions)) if len(descriptions) == 1 else
                           ' '.join(f'{side}: {definition["description"]}' for side, definition in choices.items()))
            definitions.append({'name': name, 'inputSchema': {'type': 'object', 'oneOf': alternatives},
                                'description': description +
                                ' Required arm selects left/right. All geometry is in that arm\'s base frame.'})
        pose_schemas = {side: next(t['inputSchema'] for t in child.tools() if t['name'] == 'xarm_move_pose')
                        for side, child in self.sides.items()}
        definitions.append({'name': 'xarm_move_bimanual', 'inputSchema': _schema(pose_schemas, SIDES),
                            'description': 'Prevalidate left and right Cartesian moves, then dispatch concurrently. '
                            'Targets are in separate arm-base frames. This is not hardware synchronization or '
                            'inter-arm collision planning. A failure stops all four devices.'})
        return definitions

    def _split(self, name, arguments):
        if not isinstance(arguments, dict) or arguments.get('arm') not in SIDES:
            raise ValueError('explicit arm="left" or arm="right" is required')
        args = dict(arguments)
        child = self.sides[args.pop('arm')]
        if name not in {t['name'] for t in child.tools()}:
            raise ValueError(f'{name} is unavailable for this arm')
        return child, args

    def validate(self, name, arguments):
        if name in ('xarm_status', 'xarm_hold'):
            _validate_arguments(arguments, _schema())
            return
        if name == 'xarm_move_bimanual':
            _validate_arguments(arguments, self.tools()[-1]['inputSchema'])
            for side in SIDES:
                self.sides[side].validate('xarm_move_pose', arguments[side])
        else:
            child, args = self._split(name, arguments)
            child.validate(name, args)
        if name in self.mutating:
            if self.control_uncertain:
                raise RuntimeError('bimanual control is uncertain; inspect status and use xarm_hold')
            for child in self.sides.values():
                child._require_armed()
                child._motion_state()
                child.end_effector.require_ready()

    def call(self, name, arguments):
        try:
            self.validate(name, arguments)
        except Exception as exc:
            if getattr(exc, 'operator_required', False):
                self.handle_control_fault(exc)
            raise
        if name == 'xarm_status':
            return self.status(), []
        if name == 'xarm_hold':
            outcome = self._stop_all()
            certain = all(v.get('held') is True for v in outcome.values())
            self.control_uncertain = not certain
            return {'assembly_stop': outcome, 'held': certain, 'control_certain': certain}, []
        try:
            if name == 'xarm_move_bimanual':
                def move(side):
                    try:
                        return self.sides[side].call('xarm_move_pose', arguments[side])[0]
                    except Exception:
                        self._stop_all()
                        raise
                with ThreadPoolExecutor(max_workers=2) as pool:
                    futures = {side: pool.submit(move, side) for side in SIDES}
                    return {'arms': {side: f.result() for side, f in futures.items()},
                            'synchronized': False}, []
            child, args = self._split(name, arguments)
            value, images = child.call(name, args)
            return {**value, 'arm': arguments['arm'], 'coordinate_frame': arguments['arm']+'_base'}, images
        except Exception as exc:
            if name in self.mutating:
                outcome = self._stop_all()
                self.control_uncertain = True
                exc.control_certain = False
                exc.robot_state = {**getattr(exc, 'robot_state', {}), 'bimanual_stop': outcome}
            raise

    def _stop_all(self):
        with self._stop_lock:
            disarm_error = None
            try:
                arming.write(self.config.root, False, reason='bimanual stop requested')
            except Exception as exc:
                disarm_error = str(exc)
            def stop(child):
                try:
                    return child._hold()[0]
                except Exception as exc:
                    return {'held': False, 'error': str(exc)}
            with ThreadPoolExecutor(max_workers=2) as pool:
                futures = {side: pool.submit(stop, child) for side, child in self.sides.items()}
                outcomes = {side: future.result() for side, future in futures.items()}
            if disarm_error:
                for outcome in outcomes.values():
                    outcome.update(held=False, control_certain=False, disarm_error=disarm_error)
            return outcomes

    def handle_control_fault(self, exc):
        if getattr(exc, '_bimanual_fault_handled', False):
            return
        outcome = self._stop_all()
        self.control_uncertain = True
        self.last_fault = {'error': str(exc), 'assembly_stop': outcome}
        exc.robot_state = {**getattr(exc, 'robot_state', {}), 'bimanual_stop': outcome}
        exc.control_certain = False
        from services.controller import operator
        try:
            request = operator.fault_request(self.config.root, 'xarm_controller',
                f'Bimanual control fault: {str(exc)[:2000]}. Both arms and hands received stop requests. '
                'Inspect bimanual_stop for confirmation; operator recovery and rearming are required.')
            exc.operator_request = {key: request[key] for key in ('id', 'kind', 'source', 'status')}
        except Exception as notification_error:
            exc.operator_notification_failed = str(notification_error)
        exc._control_fault_handled = True
        exc._bimanual_fault_handled = True

    def _watch(self):
        was_armed = False
        while not self._stop.wait(.1):
            armed = arming.read(self.config.root)['armed']
            try:
                if armed:
                    for child in self.sides.values():
                        child._motion_state()
                    for child in self.sides.values():
                        child.end_effector.require_ready()
                elif was_armed:
                    self._stop_all()
            except Exception as exc:
                self.handle_control_fault(exc)
            if self.sides['left'].stream is not None:
                from .camera_server import minute_name
                now = time.time()
                try:
                    folder = self.sides['left'].stream/'robot'
                    folder.mkdir(parents=True, exist_ok=True)
                    with (folder/f'{minute_name(now)}.jsonl').open('a') as log:
                        log.write(json.dumps({'t_ms': int(now*1000), 'arms': self.status()['arms']})+'\n')
                except Exception:
                    pass  # Recording failure cannot suppress the next control check.
            was_armed = armed

    def status(self):
        values = {}
        for side, child in self.sides.items():
            try:
                values[side] = {**child.status(), 'coordinate_frame': side+'_base'}
            except Exception as exc:
                values[side] = {'error': str(exc), 'coordinate_frame': side+'_base'}
        return {'robot': 'bimanual_xarm6_revo2', 'arms': values, 'motion': arming.read(self.config.root),
                'control_uncertain': self.control_uncertain, 'last_fault': self.last_fault,
                'collision_checking': 'per-arm workspace and controller limits; no inter-arm collision model'}

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self.thread:
            self.thread.join(timeout=5)
        try:
            for child in self.sides.values():
                try:
                    child.close()
                except Exception:
                    pass
        finally:
            self.cameras.close()
