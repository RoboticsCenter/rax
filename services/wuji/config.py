"""Pure configuration, units and tool contracts shared by host and workspace."""
import math
import re
from copy import deepcopy

JOINT_NAMES = tuple(f'{finger}_S{joint}' for finger in ('thumb', 'index', 'middle', 'ring', 'pinky')
                    for joint in range(1, 5))
TOOL_NAMES = ('wuji_status', 'wuji_move_joints', 'wuji_pose', 'wuji_hold')
MOTION_TOOLS = frozenset({'wuji_move_joints', 'wuji_pose'})
DEFAULTS = {
    'type': 'wuji_hand2', 'address': None, 'handedness': 'right',
    'command_hz': 120., 'max_feedback_age_s': .25, 'max_scheduler_gap_s': .1,
    'parent_timeout_s': 1., 'max_velocity_rad_s': .5, 'max_acceleration_rad_s2': 1.,
    'max_step_rad': .35, 'max_tracking_error_rad': .35,
    'tolerance_rad': .04, 'settle_s': .2, 'settle_timeout_s': 3.,
    'max_duration_s': 60., 'stop_timeout_s': 2., 'stopped_velocity_rad_s': .05,
    'postures': {}, 'lock_paths': [],
}
REQUIRED = {'serial', 'profile_revision', 'lower_rad', 'upper_rad', 'current_limit_a', 'kp', 'kd'}


def number(value, name, minimum, maximum):
    if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be a finite number in {minimum}..{maximum}')
    return float(value)


def vector(value, name='positions'):
    if not isinstance(value, (list, tuple)) or len(value) != 20:
        raise ValueError(f'{name} must contain exactly 20 numbers in thumb/index/middle/ring/pinky order')
    return [number(x, name, -100., 100.) for x in value]


def parameters(value, name, maximum):
    values = value if isinstance(value, (list, tuple)) else [value] * 20
    return [number(x, name, 0., maximum) for x in vector(values, name)]


def validate_config(value):
    if not isinstance(value, dict):
        raise ValueError('end_effector must be an object')
    unknown = set(value) - set(DEFAULTS) - REQUIRED
    if unknown or REQUIRED - set(value):
        raise ValueError(f'Wuji configuration: unknown {sorted(unknown)}, missing {sorted(REQUIRED-set(value))}')
    cfg = {**deepcopy(DEFAULTS), **deepcopy(value)}
    if cfg['type'] != 'wuji_hand2':
        raise ValueError('expected end_effector type wuji_hand2')
    if not isinstance(cfg['serial'], str) or not re.fullmatch(r'[A-Za-z0-9_-]{4,64}', cfg['serial']):
        raise ValueError('Wuji serial is required and must be an exact device serial')
    if cfg['handedness'] not in ('left', 'right'):
        raise ValueError('handedness must be left or right')
    if not isinstance(cfg['profile_revision'], str) or not re.fullmatch(r'[A-Za-z0-9_.-]{1,80}', cfg['profile_revision']):
        raise ValueError('profile_revision must identify the operator control profile')
    if cfg['address'] is not None:
        import ipaddress
        try:
            host, port = cfg['address'].split(':')
            ipaddress.IPv4Address(host)
            if not 1 <= int(port) <= 65535:
                raise ValueError()
        except (AttributeError, ValueError):
            raise ValueError('address must be an IPv4 address and port, e.g. 192.168.1.111:7447') from None
    bounds = {
        'command_hz': (20, 250), 'max_feedback_age_s': (.02, 1), 'max_scheduler_gap_s': (.02, .5),
        'parent_timeout_s': (.3, 5), 'max_velocity_rad_s': (.01, 3), 'max_acceleration_rad_s2': (.01, 25),
        'max_step_rad': (.001, math.pi), 'max_tracking_error_rad': (.01, 1),
        'tolerance_rad': (.001, .2), 'settle_s': (.01, 2), 'settle_timeout_s': (.1, 10),
        'max_duration_s': (.1, 60), 'stop_timeout_s': (.1, 5), 'stopped_velocity_rad_s': (.001, .2),
    }
    for key, (lo, hi) in bounds.items():
        cfg[key] = number(cfg[key], key, lo, hi)
    if cfg['max_scheduler_gap_s'] <= 1 / cfg['command_hz']:
        raise ValueError('max_scheduler_gap_s must exceed the command period')
    for key in ('lower_rad', 'upper_rad'):
        cfg[key] = vector(cfg[key], key)
    if any(lo >= hi for lo, hi in zip(cfg['lower_rad'], cfg['upper_rad'])):
        raise ValueError('every lower_rad must be below upper_rad')
    for key, maximum in (('current_limit_a', 2.), ('kp', 10.), ('kd', 2.)):
        cfg[key] = parameters(cfg[key], key, maximum)
    if any(x <= 0 for x in cfg['current_limit_a']) or any(x <= 0 for x in cfg['kp']):
        raise ValueError('position holding requires positive current_limit_a and kp')
    if not isinstance(cfg['postures'], dict) or len(cfg['postures']) > 100:
        raise ValueError('postures must be an object of at most 100 named 20-joint targets')
    for name, positions in cfg['postures'].items():
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', name):
            raise ValueError('invalid posture name')
        cfg['postures'][name] = target(positions, cfg)
    if not isinstance(cfg['lock_paths'], list) or any(not isinstance(p, str) or not p.startswith('/')
                                                    for p in cfg['lock_paths']):
        raise ValueError('lock_paths must be absolute operator-owned paths')
    return cfg


def target(positions, cfg):
    q = vector(positions)
    for name, v, lo, hi in zip(JOINT_NAMES, q, cfg['lower_rad'], cfg['upper_rad']):
        if not lo <= v <= hi:
            raise ValueError(f'{name} target {v:g} outside configured limits {lo:g}..{hi:g} rad')
    return q


def validate_tool(name, args, cfg):
    if not isinstance(args, dict) or name not in TOOL_NAMES:
        raise ValueError('unknown Wuji tool or invalid arguments')
    allowed = {'positions', 'duration_s'} if name == 'wuji_move_joints' else (
        {'name', 'duration_s'} if name == 'wuji_pose' else set())
    if set(args) - allowed:
        raise ValueError(f'unknown arguments for {name}: {sorted(set(args)-allowed)}')
    if name in MOTION_TOOLS:
        if name == 'wuji_pose':
            pose = args.get('name')
            if not isinstance(pose, str) or pose not in cfg['postures']:
                raise ValueError(f'unknown posture; commissioned names: {sorted(cfg["postures"])}')
            q = cfg['postures'][pose]
        else:
            q = target(args.get('positions'), cfg)
        duration = args.get('duration_s')
        if 'duration_s' in args:
            duration = number(duration, 'duration_s', .1, cfg['max_duration_s'])
        return {'positions': list(q), 'duration_s': duration}
    return {}


def tools(cfg):
    def tool(name, description, properties=None, required=()):
        return {'name': name, 'description': description, 'inputSchema': {
            'type': 'object', 'properties': properties or {}, 'required': list(required),
            'additionalProperties': False}}
    duration = {'type': 'number', 'minimum': .1, 'maximum': cfg['max_duration_s'],
                'description': 'Optional duration; rejected if too short for the configured speed/acceleration.'}
    return [
        tool('wuji_status', 'Right/left Wuji Hand 2 measured and commanded joints (rad), effort (A), '
             'freshness, faults, current action, limits and commissioned postures. Does not enable or move.'),
        tool('wuji_move_joints', 'Move all 20 joints with bounded interpolation and wait for measured settling. '
             'Order: thumb S1..S4, index S1..S4, middle S1..S4, ring S1..S4, pinky S1..S4. '
             'Absolute radians. Requires operator preparation and arming. Holds target after completion; '
             'convergence does not prove a grasp. No collision planning.',
             {'positions': {'type': 'array', 'items': {'type': 'number'}, 'minItems': 20, 'maxItems': 20},
              'duration_s': duration}, ['positions']),
        tool('wuji_pose', 'Move to an operator-configured posture using the same bounds as wuji_move_joints. '
             f'Available names: {", ".join(sorted(cfg["postures"])) or "none commissioned"}.',
             {'name': {'type': 'string'}, 'duration_s': duration}, ['name']),
        tool('wuji_hold', 'Retain the current bounded commanded posture when dispatched. '
             'MCP calls are serialized: this call waits for an active MCP call; operator stop/disarm '
             'is required to interrupt it. '
             'Disarmed, stale or faulty control requests emergency stop instead. Always allowed. '
             'Reports requested/returned/confirmed separately; grip retention is not guaranteed.'),
    ]
