"""Operator-owned, fail-closed settings for exactly one right Piper/Revo pair."""
from pathlib import Path

import numpy as np

from .kinematics import vector


DEFAULTS = {
    'arm_usb_serial': '002C001B4759530920353131',
    'arm_firmware_profile': 'default',
    'arm_speed_percent': 10,
    'hand_serial': 'BCXTR2362J2500015',
    'hand_firmware': '1.0.22.U',
    'hand_adapter_firmware': 'dual_app v1.0.0.7',
    'hand_channel': 1,
    'hand_id': 127,
    'damiao_library': str(Path.home() / 'Documents/CenterOS-2/dist/openarm-agent/vendor/libdm_device.so'),
    'hand_rpc_timeout_s': 2.,
    'hand_start_timeout_s': 15.,
    'motion_commissioned': False,
    'tool_calibrated': False,
    'tool_translation_m': [0, 0, 0],
    'tool_quaternion_xyzw': [0, 0, 0, 1],
    'hand_extent_m': .25,
    'workspace_min': [.10, -.45, .10],
    'workspace_max': [.70, .45, .70],
    'link_min_z': .08,
    'max_joint_step_rad': .15,
    'joint_speed_radps': .10,
    'joint_acc_radps2': .20,
    'joint_tolerance_rad': .02,
    'max_translation_m': .025,
    'max_rotation_rad': .15,
    'max_following_error_rad': .08,
    'move_timeout_s': 20.,
    'segment_s': .10,
    'hand_max_step': 100,
    'hand_segment_ms': 200,
    'hand_target_min': [100, 0, 0, 0, 0, 0],
    'hand_target_max': [900, 950, 950, 950, 950, 950],
    'cameras': [{'name': 'overview', 'serial': '263322070113',
                 'url': 'http://127.0.0.1:8090/stream?cam=263322070113&type=color'}],
    'stream_enabled': True,
    'stream_fps': 5.,
    'stream_window_s': 600.,
    'agent_python': None,
    'protected_paths': [],
}


def options(value):
    unknown = set(value) - DEFAULTS.keys()
    if unknown:
        raise ValueError(f'Unknown Piper options: {sorted(unknown)}')
    result = {**DEFAULTS, **value}
    for key in ('motion_commissioned', 'tool_calibrated', 'stream_enabled'):
        if type(result[key]) is not bool:
            raise ValueError(f'{key} must be a boolean')
    for key in ('arm_usb_serial', 'hand_serial', 'hand_firmware', 'hand_adapter_firmware', 'damiao_library'):
        if not isinstance(result[key], str) or not result[key]:
            raise ValueError(f'{key} must be nonempty text')
    if not Path(result['damiao_library']).is_absolute():
        raise ValueError('damiao_library must be absolute')
    if result['agent_python'] is not None and (not isinstance(result['agent_python'], str)
                                              or not Path(result['agent_python']).is_absolute()):
        raise ValueError('agent_python must be an absolute interpreter path')
    if result['arm_firmware_profile'] not in ('default', 'v183', 'v188', 'v189', 'v192'):
        raise ValueError('Unsupported Piper firmware profile')
    if (type(result['hand_channel']) is not int or result['hand_channel'] not in (0, 1)
            or type(result['hand_id']) is not int or result['hand_id'] != 127):
        raise ValueError('This single-right-pair backend requires hand ID 127 and channel 0 or 1')
    for key in ('workspace_min', 'workspace_max', 'tool_translation_m'):
        vector(result[key], 3, key)
    if not np.all(np.asarray(result['workspace_min']) < result['workspace_max']):
        raise ValueError('workspace_min must be below workspace_max')
    quaternion = vector(result['tool_quaternion_xyzw'], 4, 'tool quaternion')
    if abs(np.linalg.norm(quaternion) - 1) > 1e-5:
        raise ValueError('Tool quaternion must have unit norm')
    bounds = {'hand_extent_m': (.01, .5), 'link_min_z': (0., 1.),
              'max_joint_step_rad': (.001, .5), 'joint_speed_radps': (.001, .2),
              'joint_acc_radps2': (.001, .5), 'joint_tolerance_rad': (.001, .05),
              'max_translation_m': (.001, .05), 'max_rotation_rad': (.001, .3),
              'max_following_error_rad': (.005, .1), 'move_timeout_s': (1., 60.),
              'hand_rpc_timeout_s': (.5, 5.), 'hand_start_timeout_s': (5., 30.),
              'segment_s': (.05, .1),
              'stream_fps': (.1, 15.), 'stream_window_s': (10., 3600.)}
    for key, (lower, upper) in bounds.items():
        number = result[key]
        if type(number) not in (int, float) or not np.isfinite(number) or not lower <= number <= upper:
            raise ValueError(f'{key} must be finite in [{lower}, {upper}]')
    for key, lower, upper in (('hand_max_step', 1, 100), ('hand_segment_ms', 100, 200), ('arm_speed_percent', 1, 20)):
        if type(result[key]) is not int or not lower <= result[key] <= upper:
            raise ValueError(f'{key} must be an integer in [{lower}, {upper}]')
    minima, maxima = result['hand_target_min'], result['hand_target_max']
    vector(minima, 6, 'hand_target_min')
    vector(maxima, 6, 'hand_target_max')
    for index, (lower, upper) in enumerate(zip(minima, maxima)):
        if type(lower) is not int or type(upper) is not int or not (
                DEFAULTS['hand_target_min'][index] <= lower < upper <= DEFAULTS['hand_target_max'][index]):
            raise ValueError('Hand bounds may only narrow the conservative position envelope')
    cameras = result['cameras']
    if not isinstance(cameras, list) or len(cameras) != 1:
        raise ValueError('Configure exactly one RGB overview camera')
    camera = cameras[0]
    if set(camera) != {'name', 'serial', 'url'} or camera['name'] != 'overview' or not all(
            isinstance(item, str) and item for item in camera.values()):
        raise ValueError('Camera needs name=overview, serial and URL')
    from urllib.parse import urlsplit
    url = urlsplit(camera['url'])
    if url.scheme != 'http' or url.hostname not in ('127.0.0.1', 'localhost') or url.username or url.password:
        raise ValueError('Camera must be an unauthenticated local HTTP stream')
    if not isinstance(result['protected_paths'], list) or any(
            not isinstance(path, str) or not Path(path).is_absolute() for path in result['protected_paths']):
        raise ValueError('protected_paths must contain absolute paths')
    return result
