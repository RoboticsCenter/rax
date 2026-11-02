"""Pure configuration validation for two independently addressed xArm/Revo pairs."""
from copy import deepcopy
from services.revo2.config import validate_config as hand_config

SIDES = ('left', 'right')


def validate_config(options):
    if not isinstance(options, dict) or set(options) - {'arms', 'agent_python', 'protected_paths'}:
        raise ValueError('bimanual options accept arms, agent_python and protected_paths')
    if not isinstance(options.get('arms'), dict) or set(options['arms']) != set(SIDES):
        raise ValueError('bimanual arms must contain exactly left and right')
    arms = deepcopy(options['arms'])
    serials, addresses, modbus_ports, cameras = set(), set(), set(), {}
    for side, cfg in arms.items():
        if not isinstance(cfg, dict) or not isinstance(cfg.get('xarm_ip'), str) or not cfg['xarm_ip'].strip():
            raise ValueError(f'{side} needs an explicit xarm_ip')
        if cfg['xarm_ip'] in addresses:
            raise ValueError('left and right xArm addresses must differ')
        addresses.add(cfg['xarm_ip'])
        hand = hand_config(cfg.get('end_effector'))
        if hand['handedness'] != side:
            raise ValueError(f'{side} arm must configure a {side} Revo hand')
        if hand['serial'] in serials:
            raise ValueError('each Revo hand needs a distinct serial')
        serials.add(hand['serial'])
        if hand['transport'] == 'modbus':
            if hand['port'] in modbus_ports:
                raise ValueError('use a separate Modbus port per hand; shared RS485 buses are not supported')
            modbus_ports.add(hand['port'])
        cfg['end_effector'] = hand
        if not isinstance(cfg.get('cameras'), list) or not cfg['cameras']:
            raise ValueError(f'{side} needs calibrated cameras in its own arm-base frame')
        local_names = set()
        for cam in cfg['cameras']:
            name = cam['name']
            if name in local_names:
                raise ValueError(f'duplicate {side} camera name {name}')
            local_names.add(name)
            capture = {k: v for k, v in cam.items() if k != 'extrinsics'}
            if name in cameras:
                if capture != cameras[name] or cam.get('mount') != 'fixed':
                    raise ValueError('a shared camera must be fixed with identical capture settings and per-arm extrinsics')
            else:
                if any(c['serial'] == cam['serial'] for c in cameras.values()):
                    raise ValueError('a shared physical camera must use the same name on both arms')
                cameras[name] = capture
    # A single camera owner must have one stream configuration.
    for key in ('stream_enabled', 'stream_fps', 'stream_window_s', 'stream_depth_hz', 'stream_jpeg_quality',
                'stream_archive', 'camera_cpus', 'camera_depth_mode'):
        if arms['left'].get(key) != arms['right'].get(key):
            raise ValueError(f'{key} must match between arms')
    return arms
