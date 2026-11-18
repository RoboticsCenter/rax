"""Pure configuration, units and tool contracts shared by host and workspace."""
import math
import re
from copy import deepcopy

JOINT_NAMES = tuple(f'{finger}_S{joint}' for finger in ('thumb', 'index', 'middle', 'ring', 'pinky')
                    for joint in range(1, 5))
LEGACY_TOOL_NAMES = ('wuji_status', 'wuji_move_joints', 'wuji_pose', 'wuji_home', 'wuji_hold',
                     'wuji_set_effort', 'wuji_set_impedance')
# Smooth control layer (docs/wuji-smooth-control.md): non-blocking streaming targets, timed
# trajectories, impedance presets, stream settings and settle waits.
STREAM_TOOL_NAMES = ('wuji_stream', 'wuji_trajectory', 'wuji_set_profile', 'wuji_stream_config', 'wuji_wait')
TOOL_NAMES = LEGACY_TOOL_NAMES + STREAM_TOOL_NAMES
# Robot-level trace recording (shared trace format; services/dex_eval looks these names up).
TRACE_TOOL_NAMES = ('trace_start', 'trace_stop')
MOTION_TOOLS = frozenset({'wuji_move_joints', 'wuji_pose', 'wuji_home'})
CONTROL_TOOLS = frozenset({'wuji_set_effort', 'wuji_set_impedance'})
STREAM_MOTION_TOOLS = frozenset({'wuji_stream', 'wuji_trajectory', 'wuji_set_profile'})
# High-rate tools skip per-call motion clips and heavyweight whole-rig status reads.
HIGH_RATE_TOOLS = frozenset({'wuji_stream', 'wuji_stream_config', 'wuji_wait'})
MUTATING_TOOLS = MOTION_TOOLS | CONTROL_TOOLS | STREAM_MOTION_TOOLS
DEFAULTS = {
    'type': 'wuji_hand2', 'address': None, 'handedness': 'right',
    'command_hz': 120., 'max_feedback_age_s': .25, 'max_scheduler_gap_s': .1,
    'parent_timeout_s': 1., 'max_velocity_rad_s': .5, 'max_acceleration_rad_s2': 1.,
    'max_step_rad': .35, 'max_tracking_error_rad': .35,
    'tolerance_rad': .04, 'settle_s': .2, 'settle_timeout_s': 3.,
    'max_duration_s': 60., 'stop_timeout_s': 2., 'stopped_velocity_rad_s': .05,
    'motion_mode': 'timed', 'max_jerk_rad_s3': .8, 'reference_timeout_scale': 3.,
    'feedback_transition_s': 1., 'feedback_slow_error_rad': .06,
    'feedback_pause_error_rad': .10, 'settle_position_range_rad': .004,
    'allow_encoder_warning': False, 'max_temperature_c': 65., 'stall_warning_grace_s': 0.,
    'allow_init_current_calib_failed': {},
    'disabled_joints': [], 'isolated_joint_faults': {},
    'max_effort_slew_a_s': .5,
    'effort_velocity_grace_s': 0.,
    'postures': {}, 'lock_paths': [],
    # Smooth control layer. None means "same as the S-curve limit" (no behavior change).
    'stream_max_acceleration_rad_s2': None, 'stream_max_jerk_rad_s3': None,
    'friction_current_a': .08, 'friction_velocity_rad_s': .03, 'max_feedforward_fraction': .5,
    'gain_scale': 1.98, 'max_record_s': 120.,
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
    if cfg['motion_mode'] not in ('timed', 'feedback', 'scurve'):
        raise ValueError('motion_mode must be timed, feedback or scurve')
    if type(cfg['allow_encoder_warning']) is not bool:
        raise ValueError('allow_encoder_warning must be a boolean')
    exceptions = cfg['allow_init_current_calib_failed']
    if (not isinstance(exceptions, dict) or any(
            joint not in JOINT_NAMES or not isinstance(firmware, str)
            or not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+', firmware)
            for joint, firmware in exceptions.items())):
        raise ValueError('allow_init_current_calib_failed must map exact joint names to firmware versions')
    excluded = cfg['disabled_joints']
    if (not isinstance(excluded, list) or len(excluded) >= 20
            or any(not isinstance(name, str) or name not in JOINT_NAMES for name in excluded)
            or len(set(excluded)) != len(excluded)):
        raise ValueError('disabled_joints must be distinct joint names and leave at least one active joint')
    from .partial import ISOLATABLE
    policy = cfg['isolated_joint_faults']
    if (not isinstance(policy, dict) or any(
            joint not in excluded or not isinstance(rule, dict) or set(rule) != {'code', 'firmware'}
            or type(rule['code']) is not int or rule['code'] not in ISOLATABLE
            or not isinstance(rule['firmware'], str)
            or not re.fullmatch(r'[0-9]+\.[0-9]+\.[0-9]+', rule['firmware'])
            for joint, rule in policy.items())):
        raise ValueError('isolated_joint_faults requires disabled joints, supported exact codes and firmware versions')
    if excluded and exceptions:
        raise ValueError('partial-hand mode cannot combine isolation with the current-calibration admission exception')
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
        'command_hz': (20, 1000), 'max_feedback_age_s': (.02, 1), 'max_scheduler_gap_s': (.02, .5),
        'parent_timeout_s': (.3, 5), 'max_velocity_rad_s': (.01, 3), 'max_acceleration_rad_s2': (.01, 25),
        'max_step_rad': (.001, math.pi), 'max_tracking_error_rad': (.01, 1),
        'tolerance_rad': (.001, .2), 'settle_s': (.01, 2), 'settle_timeout_s': (.1, 10),
        'max_duration_s': (.1, 60), 'stop_timeout_s': (.1, 5), 'stopped_velocity_rad_s': (.001, .2),
        'max_jerk_rad_s3': (.01, 100), 'reference_timeout_scale': (1., 5.), 'feedback_transition_s': (.2, 3),
        'feedback_slow_error_rad': (.001, .3), 'feedback_pause_error_rad': (.002, .34),
        'settle_position_range_rad': (.0001, .05),
        'max_temperature_c': (20, 90),
        'max_effort_slew_a_s': (.01, 2.),
        'effort_velocity_grace_s': (0., .02),
        'stall_warning_grace_s': (0., .2),
        'friction_velocity_rad_s': (.001, 1.), 'max_feedforward_fraction': (.05, 1.),
        'gain_scale': (.5, 4.), 'max_record_s': (1., 600.),
    }
    for key, (lo, hi) in bounds.items():
        cfg[key] = number(cfg[key], key, lo, hi)
    for key, hi in (('stream_max_acceleration_rad_s2', 25.), ('stream_max_jerk_rad_s3', 100.)):
        if cfg[key] is not None:
            cfg[key] = number(cfg[key], key, .01, hi)
    # Hard ceiling: host friction feed-forward never exceeds 0.3 A commanded per joint.
    cfg['friction_current_a'] = parameters(cfg['friction_current_a'], 'friction_current_a', .3)
    if cfg['max_scheduler_gap_s'] <= 1 / cfg['command_hz']:
        raise ValueError('max_scheduler_gap_s must exceed the command period')
    if (cfg['motion_mode'] == 'feedback'
            and not cfg['feedback_slow_error_rad'] < cfg['feedback_pause_error_rad'] < cfg['max_tracking_error_rad']):
        raise ValueError('feedback slow/pause errors must increase and stay below max_tracking_error_rad')
    if cfg['motion_mode'] in ('feedback', 'scurve') and cfg['settle_position_range_rad'] > cfg['tolerance_rad']:
        raise ValueError('settle_position_range_rad must not exceed tolerance_rad')
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


def effort(values, limits):
    values = vector(values, 'effort_a')
    for name, value, limit in zip(JOINT_NAMES, values, limits):
        if abs(value) > limit:
            raise ValueError(f'{name} effort exceeds current limit {limit:g} A')
    return values


def impedance(args, cfg):
    if not isinstance(args, dict) or not args or set(args) - {'kp', 'kd', 'current_limit_a'}:
        raise ValueError('provide kp, kd and/or current_limit_a')
    result = {}
    for key, values in args.items():
        result[key] = vector(values, key)
        for name, value, ceiling in zip(JOINT_NAMES, result[key], cfg[key]):
            if value < 0 or value > ceiling or (key != 'kd' and value == 0):
                raise ValueError(f'{name} {key} must be {"nonnegative" if key == "kd" else "positive"} '
                                 f'and no greater than configured ceiling {ceiling:g}')
    return result


def completion_mode(value):
    if value not in ('position', 'contact'):
        raise ValueError('completion must be position or contact')
    return value


def validate_tool(name, args, cfg, *, reference=None):
    if not isinstance(args, dict) or name not in TOOL_NAMES:
        raise ValueError('unknown Wuji tool or invalid arguments')
    if name in STREAM_TOOL_NAMES:
        # Structural checks here; the hand owner validates against its live target.
        from .smooth import check_tool_args
        return check_tool_args(name, args, cfg)
    allowed = {'wuji_move_joints': {'positions', 'duration_s', 'completion'}, 'wuji_pose': {'name', 'duration_s'},
               'wuji_home': {'duration_s'},
               'wuji_set_effort': {'effort_a', 'duration_s'},
               'wuji_set_impedance': {'kp', 'kd', 'current_limit_a'}}.get(name, set())
    if set(args) - allowed:
        raise ValueError(f'unknown arguments for {name}: {sorted(set(args)-allowed)}')
    if cfg['disabled_joints']:
        if name not in ('wuji_status', 'wuji_move_joints', 'wuji_pose', 'wuji_home', 'wuji_hold'):
            raise ValueError('partial-hand mode supports status, position moves and hold only; '
                             'the wuji_stream/wuji_set_profile tools support active joints')
        if args.get('completion', 'position') != 'position':
            raise ValueError('partial-hand mode requires position completion on active joints')
    if name == 'wuji_set_impedance':
        return impedance(args, cfg)
    if name == 'wuji_set_effort':
        duration = number(args['duration_s'], 'duration_s', .1, cfg['max_duration_s']) if 'duration_s' in args else None
        return {'effort_a': effort(args.get('effort_a'), cfg['current_limit_a']), 'duration_s': duration}
    if name in MOTION_TOOLS:
        if name in ('wuji_pose', 'wuji_home'):
            pose = 'home' if name == 'wuji_home' else args.get('name')
            if name == 'wuji_home' and pose not in cfg['postures']:
                raise ValueError('home posture is not configured; ask the operator')
            if not isinstance(pose, str) or pose not in cfg['postures']:
                raise ValueError(f'unknown posture; commissioned names: {sorted(cfg["postures"])}')
            q = list(cfg['postures'][pose])
            if cfg['disabled_joints'] and reference is not None:
                from .partial import disabled_indices
                retained = target(reference, cfg)
                for i in disabled_indices(cfg):
                    q[i] = retained[i]
        else:
            q = target(args.get('positions'), cfg)
        duration = args.get('duration_s')
        if 'duration_s' in args:
            duration = number(duration, 'duration_s', .1, cfg['max_duration_s'])
        result = {'positions': list(q), 'duration_s': duration}
        if 'completion' in args:
            result['completion'] = completion_mode(args['completion'])
        return result
    return {}


def tools(cfg):
    def tool(name, description, properties=None, required=()):
        return {'name': name, 'description': description, 'inputSchema': {
            'type': 'object', 'properties': properties or {}, 'required': list(required),
            'additionalProperties': False}}
    duration = {'type': 'number', 'minimum': .1, 'maximum': cfg['max_duration_s'],
                'description': 'Optional nominal duration; automatically stretched to satisfy configured reference limits. '
                               'Measured settling may add time, bounded by settle_timeout_s. '
                               'S-curve durations are rounded up to a control tick; reference execution allows '
                               'reference_timeout_scale times nominal duration, followed by measured settling.'}
    return [
        tool('wuji_status', 'Right/left Wuji Hand 2 measured and commanded joints (rad), effort (A), '
             'freshness, faults, current action, limits and commissioned postures. Does not enable or move.'),
        tool('wuji_move_joints', 'Move configured active joints with bounded interpolation and wait for measured settling. '
             'In partial-hand mode, copy target_positions and leave disabled_joints entries unchanged; '
             'completion covers active_joints only. Disabled joints are passive, not position-held. '
             'A completed reference that misses position tolerance returns '
             'target_not_reached with reached=false and holding=true without disabling the hand. '
             'Configured S-curve mode limits jerk and publishes matching position/velocity references. '
             'Configured feedback mode slows progress when measured joints lag and requires stable final positions. '
             'Order: thumb S1..S4, index S1..S4, middle S1..S4, ring S1..S4, pinky S1..S4. '
             'Absolute radians. Requires operator preparation and arming. Holds target after completion; '
             'convergence does not prove a grasp. For a deliberate grasp, completion="contact" also accepts '
             'a stable shortfall after the reference finishes, within the unchanged tracking-error guard. '
             'Returns held_short_of_target with reached=false and contact_verified=false while preserving '
             'the target, gains and holding effort. Position mode remains the default. No collision planning.',
             {'positions': {'type': 'array', 'items': {'type': 'number'}, 'minItems': 20, 'maxItems': 20},
              'duration_s': duration,
              'completion': {'type': 'string', 'enum': ['position', 'contact'], 'default': 'position',
                             'description': 'Use contact only for an intentional object-contact closure. '
                                            'Stable shortfall is not proof of contact or retention.'}}, ['positions']),
        tool('wuji_pose', 'Move to an operator-configured posture using the same bounds as wuji_move_joints. '
             'Partial hands move active joints only; disabled entries retain their current reference. '
             f'Available names: {", ".join(sorted(cfg["postures"])) or "none commissioned"}.',
             {'name': {'type': 'string'}, 'duration_s': duration}, ['name']),
        tool('wuji_home', 'Return the hand to its operator-configured home posture through the same custom '
             'motion controller as wuji_move_joints. Uses the configured motion mode, including S-curve '
             'jerk limits. Partial hands return active joints only and retain disabled joint references. '
             'Preserves active gains, damping and current limits. Waits for measured '
             'settling and holds home afterward. Requires operator preparation and arming. Refuses if '
             'home is not configured. This is a posture move, not encoder homing. First support/release '
             'any object and clear the full finger sweep; no collision planning or arm movement.',
             {'duration_s': duration}),
        tool('wuji_hold', 'Retain the current bounded commanded posture when dispatched. '
             'Retains the current feed-forward effort and impedance as well. '
             'During S-curve motion, brake smoothly to a nearby bounded resting target; '
             'if bounded braking is unavailable, request emergency stop. '
             'MCP calls are serialized: this call waits for an active MCP call; operator stop/disarm '
             'is required to interrupt it. '
             'Disarmed, stale or faulty control requests emergency stop instead. Always allowed. '
             'Reports requested/returned/confirmed separately; grip retention is not guaranteed.'),
        tool('wuji_set_effort', 'Ramp twenty signed feed-forward joint currents in AMPERES while retaining '
             'the current position reference and MIT gains. NOT calibrated torque or fingertip force. '
             'Requires arming and a prepared hand. Bounded by active current limits and configured current slew; '
             'position/tracking guards remain active. Effort persists during subsequent hand/arm moves and hold '
             'until changed or stopped. Send twenty zeros to remove the bias smoothly. '
             'Returns after the command ramp, not after physical settling or verified contact. '
             'Order: thumb, index, middle, ring, pinky, S1..S4 each.',
             {'effort_a': {'type': 'array', 'items': {'type': 'number'}, 'minItems': 20, 'maxItems': 20},
              'duration_s': {'type': 'number', 'minimum': .1, 'maximum': cfg['max_duration_s'],
                             'description': 'Optional ramp time, subject to current slew bound; omitted chooses minimum.'}},
             ['effort_a']),
        tool('wuji_set_impedance', 'Adjust per-joint MIT kp, kd and/or current ceilings (A); omitted arrays '
             'retain their active values. Exactly twenty entries per supplied array. Requires arming, a prepared '
             'stationary hand and zero feed-forward effort. Values cannot exceed the operator profile; kp and '
             'current limits must stay positive (pure current-only mode is not exposed). '
             'Targets stay unchanged; changing stiffness can change contact pressure. Readback is verified. '
             'Overrides persist until changed or operator preparation restores the profile; status reports both.',
             {key: {'type': 'array', 'items': {'type': 'number', 'minimum': 0}, 'minItems': 20, 'maxItems': 20}
              for key in ('kp', 'kd', 'current_limit_a')}),
        *stream_tools(cfg, tool),
    ]


def trace_fields(max_seconds):
    return {'controller': {'type': 'string', 'description': 'Condition/controller name (grouping key).'},
            'skill': {'type': 'string'}, 'variant': {'type': 'string'},
            'params': {'type': 'object', 'additionalProperties': True}, 'notes': {'type': 'string'},
            'max_seconds': {'type': 'number', 'minimum': 1, 'maximum': max_seconds}}


def trace_tools(sides, max_seconds):
    """trace_start/trace_stop schemas (robot level; one trial directory for the selected hands)."""
    def tool(name, description, properties):
        return {'name': name, 'description': description, 'inputSchema': {
            'type': 'object', 'properties': properties, 'required': [], 'additionalProperties': False}}
    side_list = {'type': 'array', 'items': {'type': 'string', 'enum': list(sides)}, 'minItems': 1,
                 'maxItems': len(sides), 'description': f'Hands to record (default {", ".join(sides)}).'}
    return [
        tool('trace_start', 'Start recording a trace trial at the hand command rate (shared trace format: '
             'hand_<side>.npz with t, q_ref, q_target, q_meas, dq_meas, current, i_ff, kp, kd, temperature_c; '
             'meta.json). directory: trial directory inside the agent workspace; any other path is replaced '
             'by runtime/traces/<label> and the result reports where the files are. Does not move the hand.',
             {'directory': {'type': 'string'}, 'label': {'type': 'string'}, 'sides': side_list,
              **trace_fields(max_seconds)}),
        tool('trace_stop', 'Stop the active trace trial, write hand_<side>.npz and meta.json (with success '
             'true/false/null and the active current_limit_a), and return the trial directory.',
             {'success': {'type': ['boolean', 'null']}, 'notes': {'type': 'string'},
              'directory': {'type': 'string', 'description': 'Optional; must match the active trial.'}}),
    ]


def stream_tools(cfg, tool):
    """Schemas for the smooth control layer (see docs/wuji-smooth-control.md)."""
    number_ = {'type': 'number'}
    joints = {'type': 'object', 'properties': {name: number_ for name in JOINT_NAMES},
              'additionalProperties': False, 'description': 'Subset by joint name, e.g. {"index_S2": 0.4}.'}
    fingers = {'type': 'object', 'additionalProperties': False,
               'properties': {finger: {'type': 'array', 'items': number_, 'minItems': 4, 'maxItems': 4}
                              for finger in ('thumb', 'index', 'middle', 'ring', 'pinky')},
               'description': 'Subset by finger: four radians S1..S4 each, e.g. {"index": [0.3, 0, 0.5, 0.4]}.'}
    positions = {'type': 'array', 'items': number_, 'minItems': 20, 'maxItems': 20,
                 'description': 'All twenty joints, thumb/index/middle/ring/pinky S1..S4.'}
    per_joint = {'type': ['number', 'array'], 'items': number_, 'minItems': 20, 'maxItems': 20}
    record = {'type': 'object', 'additionalProperties': False, 'properties': trace_fields(cfg['max_record_s'])}
    waypoint = {'type': 'object', 'additionalProperties': False, 'required': ['t'], 'properties': {
        't': {'type': 'number', 'minimum': .001, 'maximum': cfg['max_duration_s'],
              'description': 'Seconds from the call; strictly increasing.'},
        'positions': positions, 'joints': joints, 'fingers': fingers}}
    return [
        tool('wuji_stream', 'NON-BLOCKING smooth target update for policies, MPC and code loops (call at '
             '10-50 Hz, or once). Returns immediately; the hand service interpolates toward the latest target '
             'at its command rate with jerk/acceleration/velocity limits and never stops between targets. '
             'Give exactly one of positions (20 rad), joints {name: rad} or fingers {finger: [S1..S4]}; '
             'omitted joints keep their targets. relative=true adds deltas to the latest target. Without '
             'duration_s the ramp spans the observed call period (10-100 ms); with duration_s it is a '
             'minimum-jerk move. Disabled (partial-hand) joints keep their values. Requires a prepared, '
             'armed, holding hand; blocking moves are refused while the stream reference is moving. '
             'Use wuji_wait to wait for measured settling. No collision planning.',
             {'positions': positions, 'joints': joints, 'fingers': fingers,
              'relative': {'type': 'boolean', 'default': False},
              'duration_s': {'type': 'number', 'minimum': 0, 'maximum': cfg['max_duration_s']},
              'shape': {'type': 'string', 'enum': ['linear', 'minjerk']}}),
        tool('wuji_trajectory', 'Execute timed waypoints as ONE smooth motion through every waypoint '
             '(monotone cubic spline, no stops, zero velocity at the end). Times are seconds from the call; '
             'time is stretched uniformly if a waypoint pair needs more than the velocity/acceleration '
             'limits (reported as time_scale). A new trajectory or stream continues smoothly from the '
             'current goal, so action chunks can be sent before the previous one ends. wait=true (default) '
             'returns after measured settling or timeout; wait=false returns at once. record optionally '
             'saves this call as one trace trial.',
             {'waypoints': {'type': 'array', 'items': waypoint, 'minItems': 1, 'maxItems': 256},
              'wait': {'type': 'boolean', 'default': True},
              'timeout_s': {'type': 'number', 'minimum': .1, 'maximum': 300},
              'record': record}, ['waypoints']),
        tool('wuji_set_profile', 'Select an impedance preset for the next stage: soft, default, firm, hold '
             '(soft gains plus a small hold deadband against twitching) or profile (operator values). Presets '
             f'are effective stiffness/damping divided by the firmware gain scale ({cfg["gain_scale"]:g}x), '
             'clamped to the operator ceilings. Optional kp, kd and current_limit_a (number or 20) override '
             'within the ceilings; current_limit_a is the per-stage current cap. Requires the reference '
             'at rest and a measured stationary hand; readback is verified. Partial hands keep disabled '
             'joints at zero. Returns nominal and effective gains and the error at which the cap saturates.',
             {'name': {'type': 'string', 'enum': ['profile', 'soft', 'default', 'firm', 'hold']},
              'kp': per_joint, 'kd': per_joint, 'current_limit_a': per_joint,
              'hold_deadband_rad': {'type': 'number', 'minimum': 0, 'maximum': .02}}, ['name']),
        tool('wuji_stream_config', 'Stream smoothing settings (software only; applied while the reference is '
             'at rest): filter_tau_s low-pass on targets (0 off), interpolation_s ramp per target (null auto, '
             '0 none), synchronization finger|hand|joint, per-joint max velocity/acceleration/jerk (number or '
             '20, never above the operator profile), friction feed-forward on/off with current (A) and tanh '
             'velocity scale, hold_deadband_rad, and command_hz (at most the configured rate). Omitted '
             'settings are unchanged; reset=true restores defaults. Returns the active settings.',
             {'reset': {'type': 'boolean'},
              'filter_tau_s': {'type': 'number', 'minimum': 0, 'maximum': 2},
              'interpolation_s': {'type': ['number', 'null'], 'minimum': 0, 'maximum': 1},
              'synchronization': {'type': 'string', 'enum': ['finger', 'hand', 'joint']},
              'max_velocity_rad_s': per_joint, 'max_acceleration_rad_s2': per_joint, 'max_jerk_rad_s3': per_joint,
              'friction': {'type': 'boolean'}, 'friction_current_a': per_joint,
              'friction_velocity_rad_s': {'type': 'number', 'minimum': .001, 'maximum': 1},
              'hold_deadband_rad': {'type': 'number', 'minimum': 0, 'maximum': .02},
              'hold_delay_s': {'type': 'number', 'minimum': 0, 'maximum': 5},
              'command_hz': {'type': 'number', 'minimum': 20, 'maximum': cfg['command_hz']}}),
        tool('wuji_wait', 'Wait until the stream/trajectory reference has converged and measured joints settle '
             f'within tolerance_rad={cfg["tolerance_rad"]:g} for settle_s={cfg["settle_s"]:g} (or timeout_s, '
             f'default {cfg["settle_timeout_s"]:g}). Returns settled, reached, measured positions and per-joint '
             'errors; a shortfall (e.g. contact) is reported, not treated as a fault. Does not move the hand.',
             {'timeout_s': {'type': 'number', 'minimum': 0, 'maximum': 60}}),
    ]
