"""Revo 2's six actuators use normalized vendor units, never Wuji radians."""
from copy import deepcopy
import math

MOTORS = ('thumb_flex', 'thumb_rotation', 'index', 'middle', 'ring', 'pinky')
TOOL_NAMES = ('revo_status', 'revo_move', 'revo_pose', 'revo_home', 'revo_hold',
              'revo_set_speeds', 'revo_set_currents', 'revo_touch')
MUTATING_TOOLS = frozenset({'revo_move', 'revo_pose', 'revo_home', 'revo_set_speeds', 'revo_set_currents'})
DEFAULTS = {
    'type': 'brainco_revo2', 'handedness': 'right', 'transport': 'damiao',
    'serial': None, 'firmware': '1.0.22.U', 'hand_id': 127, 'hand_channel': 1,
    'damiao_library': None, 'hand_adapter_firmware': 'dual_app v1.0.0.7',
    'port': None, 'baudrate': 460800, 'touch': False,
    'target_min': [100, 0, 0, 0, 0, 0], 'target_max': [900, 950, 950, 950, 950, 950],
    'max_step': 200, 'duration_ms': 800, 'max_duration_ms': 2000,
    'position_tolerance': 30, 'stopped_speed': 10, 'settle_s': .15,
    'poll_period_s': .05, 'move_timeout_s': 5., 'stop_timeout_s': 3.,
    'max_speed': 500, 'max_current': 400, 'protected_currents_ma': [500] * 6,
    'allow_speed_control': False, 'allow_current_control': False,
    'postures': {}, 'hand_start_timeout_s': 15., 'hand_rpc_timeout_s': 10.,
}


def vector(value, name, low, high):
    if not isinstance(value, (list, tuple)) or len(value) != 6 or any(
            type(v) is not int or not low <= v <= high for v in value):
        raise ValueError(f'{name} needs six integers in {low}..{high}, ordered {MOTORS}')
    return list(value)


def positions(value, cfg):
    result = vector(value, 'positions', 0, 1000)
    if any(not lo <= v <= hi for v, lo, hi in zip(result, cfg['target_min'], cfg['target_max'])):
        raise ValueError('positions outside the configured Revo envelope')
    return result


def validate_config(value):
    if not isinstance(value, dict) or set(value) - DEFAULTS.keys():
        raise ValueError('unknown or invalid Revo configuration')
    cfg = deepcopy({**DEFAULTS, **value})
    if cfg['type'] != 'brainco_revo2' or cfg['handedness'] not in ('left', 'right'):
        raise ValueError('expected brainco_revo2 and explicit left/right handedness')
    for name in ('serial', 'firmware', 'hand_adapter_firmware'):
        if not isinstance(cfg[name], str) or not cfg[name].strip():
            raise ValueError(f'Revo {name} is required')
    if cfg['transport'] not in ('damiao', 'modbus'):
        raise ValueError('Revo transport must be damiao or modbus')
    key = 'damiao_library' if cfg['transport'] == 'damiao' else 'port'
    if not isinstance(cfg[key], str) or not cfg[key].strip():
        raise ValueError(f'Revo {key} is required')
    if type(cfg['hand_id']) is not int or not 1 <= cfg['hand_id'] <= 247:
        raise ValueError('hand_id must be 1..247')
    if type(cfg['hand_channel']) is not int or cfg['hand_channel'] not in (0, 1):
        raise ValueError('hand_channel must be 0 or 1')
    if cfg['baudrate'] not in (19200, 57600, 115200, 460800, 1000000, 2000000, 3000000, 4000000, 5000000):
        raise ValueError('unsupported Modbus baudrate')
    for name in ('touch', 'allow_speed_control', 'allow_current_control'):
        if type(cfg[name]) is not bool:
            raise ValueError(f'{name} must be boolean')
    for name in ('target_min', 'target_max'):
        cfg[name] = vector(cfg[name], name, 0, 1000)
    if any(lo >= hi for lo, hi in zip(cfg['target_min'], cfg['target_max'])):
        raise ValueError('Revo target_min must be below target_max')
    cfg['protected_currents_ma'] = vector(cfg['protected_currents_ma'], 'protected_currents_ma', 1, 2000)
    for name, lo, hi in (('max_step', 1, 1000), ('duration_ms', 20, 2000),
                         ('max_duration_ms', 20, 2000), ('position_tolerance', 1, 100),
                         ('stopped_speed', 0, 100), ('max_speed', 1, 1000), ('max_current', 1, 1000)):
        if type(cfg[name]) is not int or not lo <= cfg[name] <= hi:
            raise ValueError(f'{name} must be an integer in {lo}..{hi}')
    if cfg['duration_ms'] > cfg['max_duration_ms']:
        raise ValueError('duration_ms exceeds max_duration_ms')
    for name in ('settle_s', 'poll_period_s', 'move_timeout_s', 'stop_timeout_s',
                 'hand_start_timeout_s', 'hand_rpc_timeout_s'):
        if type(cfg[name]) not in (float, int) or not math.isfinite(cfg[name]) or not .01 <= cfg[name] <= 60:
            raise ValueError(f'{name} must be finite in .01..60')
    if cfg['hand_rpc_timeout_s'] <= cfg['move_timeout_s'] + cfg['stop_timeout_s']:
        raise ValueError('hand_rpc_timeout_s must exceed move_timeout_s + stop_timeout_s')
    if not isinstance(cfg['postures'], dict):
        raise ValueError('postures must be an object')
    for name, target in cfg['postures'].items():
        if not isinstance(name, str) or not name:
            raise ValueError('posture names must be nonempty strings')
        positions(target, cfg)
    return cfg


def schema(properties=None, required=()):
    return {'type': 'object', 'properties': properties or {}, 'required': list(required),
            'additionalProperties': False}


def tools(cfg):
    vec = lambda lo, hi: {'type': 'array', 'items': {'type': 'integer', 'minimum': lo, 'maximum': hi},
                          'minItems': 6, 'maxItems': 6}
    duration = {'type': 'integer', 'minimum': 20, 'maximum': cfg['max_duration_ms']}
    tool = lambda name, description, props=None, req=(): {
        'name': name, 'description': description, 'inputSchema': schema(props, req)}
    result = [
        tool('revo_status', 'Read Revo 2 identity, positions (0..1000), signed normalized speeds/currents, '
             'motor states and limits. Order: thumb flex, thumb rotation, index, middle, ring, pinky.'),
        tool('revo_move', 'Move six Revo motors in normalized 0..1000 units. Uses native position+duration '
             'or position+speed; give duration_ms OR speeds. Waits for measured settling. '
             'completion=contact also accepts stationary obstruction; this does not verify a grasp.',
             {'positions': vec(0, 1000), 'duration_ms': duration, 'speeds': vec(1, cfg['max_speed']),
              'completion': {'type': 'string', 'enum': ['position', 'contact']}}, ['positions']),
        tool('revo_pose', 'Move to an operator-configured Revo posture; requires position completion.',
             {'name': {'type': 'string', **({'enum': list(cfg['postures'])} if cfg['postures'] else {})},
              'duration_ms': duration}, ['name']),
        tool('revo_home', 'Move to the configured home posture, not firmware homing/calibration.',
             {'duration_ms': duration}),
        tool('revo_hold', 'Cancel ongoing effort/speed with native zero speed then measured-position hold. '
             'May release a grasp; stop confirmation requires measured standstill.'),
    ]
    if cfg['allow_speed_control']:
        result.append(tool('revo_set_speeds', 'Finite native signed speed control in normalized units. '
                           'Positive closes. Ends in measured-position hold, including on disarm.',
                           {'speeds': vec(-cfg['max_speed'], cfg['max_speed']),
                            'duration_ms': duration}, ['speeds', 'duration_ms']))
    if cfg['allow_current_control']:
        result.append(tool('revo_set_currents', 'Finite native signed current control in normalized units, '
                           'NOT amperes or calibrated force. Positive closes. Ends in measured-position hold.',
                           {'currents': vec(-cfg['max_current'], cfg['max_current']),
                            'duration_ms': duration}, ['currents', 'duration_ms']))
    if cfg['touch']:
        result.append(tool('revo_touch', 'Read native Revo 2 Touch fingertip sensor feedback; '
                           'no implicit sensor enabling or calibration.'))
    return result


def validate_tool(name, args, cfg):
    from services.xarm.backend import _validate_arguments
    definition = next((t for t in tools(cfg) if t['name'] == name), None)
    if definition is None:
        raise ValueError(f'unavailable Revo tool {name}')
    _validate_arguments(args, definition['inputSchema'])
    result = dict(args)
    if name in ('revo_move', 'revo_pose', 'revo_home'):
        if name != 'revo_move':
            posture = 'home' if name == 'revo_home' else result.pop('name')
            if posture not in cfg['postures']:
                raise ValueError(f'Revo {posture} posture is not configured')
            result['positions'] = cfg['postures'][posture]
        result['positions'] = positions(result['positions'], cfg)
        if 'speeds' in result and 'duration_ms' in result:
            raise ValueError('give speeds or duration_ms, not both')
        if 'speeds' not in result:
            result.setdefault('duration_ms', cfg['duration_ms'])
        result.setdefault('completion', 'position')
    return result
