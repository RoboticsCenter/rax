"""Operator settings; hardware motion is opt-in and firmware-specific."""
from copy import deepcopy
import math

DEFAULTS = {
    'transport': 'dry-run', 'network_interface': 'eth0', 'dds_domain': 0,
    'robot_id': '', 'motion_commissioned': False, 'wuji_mounts_commissioned': False,
    'allowed_fsm_ids': [], 'max_vx': .15, 'max_vy': .10, 'max_yaw_rate': .20,
    'max_duration_s': 2., 'command_lease_s': .25, 'max_feedback_age_s': .2,
    'max_tilt_rad': .35, 'arms': {}, 'cameras': [],
}


def number(value, name, lower, upper):
    if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f'{name} must be finite in [{lower}, {upper}]')
    return float(value)


def options(value):
    if not isinstance(value, dict) or set(value) - DEFAULTS.keys():
        raise ValueError('Unknown G1 backend options')
    cfg = {**deepcopy(DEFAULTS), **deepcopy(value)}
    if cfg['transport'] not in ('dry-run', 'unitree'):
        raise ValueError('transport must be dry-run or unitree')
    for key in ('motion_commissioned', 'wuji_mounts_commissioned'):
        if type(cfg[key]) is not bool:
            raise ValueError(f'{key} must be boolean')
    for key in ('robot_id', 'network_interface'):
        if not isinstance(cfg[key], str) or (cfg['transport'] == 'unitree' and not cfg[key]):
            raise ValueError(f'{key} must identify the hardware connection')
    if type(cfg['dds_domain']) is not int or not 0 <= cfg['dds_domain'] <= 232:
        raise ValueError('dds_domain must be 0..232')
    ids = cfg['allowed_fsm_ids']
    if not isinstance(ids, list) or any(type(i) is not int or i < 0 for i in ids):
        raise ValueError('allowed_fsm_ids must contain commissioned integer FSM IDs')
    for key, maximum in (('max_vx', .3), ('max_vy', .2), ('max_yaw_rate', .4),
                         ('max_duration_s', 2.), ('command_lease_s', .3),
                         ('max_feedback_age_s', .25), ('max_tilt_rad', .4)):
        cfg[key] = number(cfg[key], key, .05, maximum)
    if cfg['command_lease_s'] < .2:
        raise ValueError('command_lease_s must allow the RPC timeout and worker period (at least .2 s)')
    arms = cfg['arms']
    if not isinstance(arms, dict) or set(arms) - {'left', 'right'}:
        raise ValueError('arms must map left/right to end_effector profiles')
    if cfg['transport'] == 'dry-run' and arms:
        raise ValueError('dry-run cannot connect physical Wuji hands')
    serials = []
    for side, arm in arms.items():
        if not isinstance(arm, dict) or set(arm) != {'end_effector'}:
            raise ValueError('each arm needs only an end_effector profile')
        from services.wuji.config import validate_config
        profile = validate_config(arm['end_effector'])
        if profile['handedness'] != side:
            raise ValueError('Wuji handedness must match its side')
        serials.append(profile['serial'])
        arm['end_effector'] = profile
    if len(serials) != len(set(serials)):
        raise ValueError('Wuji serials must be distinct')
    if cfg['transport'] == 'unitree' and cfg['motion_commissioned']:
        if not ids or set(arms) != {'left', 'right'} or not cfg['wuji_mounts_commissioned']:
            raise ValueError('hardware motion requires FSM IDs, both Wuji profiles and commissioned mounts')
    if not isinstance(cfg['cameras'], list):
        raise ValueError('cameras must be a list')
    names = set()
    for camera in cfg['cameras']:
        import re
        if (not isinstance(camera, dict) or set(camera) != {'name', 'url'}
                or not isinstance(camera['name'], str)
                or not re.fullmatch('[A-Za-z0-9_-]+', camera['name'])
                or camera['name'] in names or not isinstance(camera['url'], str)
                or not camera['url'].startswith(('http://', 'https://'))):
            raise ValueError('cameras require distinct names and HTTP JPEG snapshot URLs')
        names.add(camera['name'])
    return cfg
