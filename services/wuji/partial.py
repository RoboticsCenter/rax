"""Explicit joint isolation; native telemetry always retains all twenty joints."""
from .config import JOINT_NAMES

# Codes eligible for explicit isolation, not a certification of safe degraded
# hardware. Power, thermal, overcurrent, unknown and active-joint faults still stop.
ISOLATABLE = {
    0x2108: ('Enc1CrcPersistent', 'ImmediateStop', 'ManualClear'),
    0x3208: ('InitCurrentCalibFailed', 'Fatal', 'NonClearable'),
}
DISABLED_STATES = (1, 3)  # Ready / Stopped; Init and unknown states are not proof.


def disabled_indices(cfg):
    return tuple(i for i, name in enumerate(JOINT_NAMES) if name in cfg.get('disabled_joints', ()))


def active_indices(cfg):
    excluded = set(disabled_indices(cfg))
    return tuple(i for i in range(20) if i not in excluded)


def control_enabled(state, cfg):
    excluded = disabled_indices(cfg)
    if not excluded:
        return bool(state.get('fresh') and state.get('enabled'))
    states = state.get('diagnostics', {}).get('states', [])
    return bool(state.get('fresh') and len(states) == 20
                and all(states[i] == 2 for i in active_indices(cfg))
                and all(states[i] in DISABLED_STATES for i in excluded))


def effective_parameters(cfg):
    excluded = set(disabled_indices(cfg))
    return {key: [0. if i in excluded else value for i, value in enumerate(cfg[key])]
            for key in ('kp', 'kd', 'current_limit_a')}


def max_error(cfg, reference, measured):
    return max(abs(reference[i]-measured[i]) for i in active_indices(cfg))


def max_velocity(cfg, velocity):
    return max(abs(velocity[i]) for i in active_indices(cfg))


def validate_isolated_targets(start, goal, cfg):
    for i in disabled_indices(cfg):
        if abs(goal[i]-start[i]) > 1e-9:
            raise ValueError(f'{JOINT_NAMES[i]} is disabled; retain its target_positions entry unchanged')


def isolated_diagnostic(cfg, joint, code, detail, firmware, native_state):
    policy = cfg.get('isolated_joint_faults', {}).get(joint)
    return (joint in cfg.get('disabled_joints', ()) and policy is not None
            and policy['code'] == code and policy['firmware'] == firmware
            and native_state in DISABLED_STATES and code in ISOLATABLE
            and tuple(detail.get(k) for k in ('name', 'severity', 'clear_policy')) == ISOLATABLE[code])
