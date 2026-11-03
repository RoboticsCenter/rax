"""Bimanual configuration and identities; no hardware is opened here."""
from copy import deepcopy
import math
import re

from services.wuji.config import validate_config as hand_config

SIDES = ('left', 'right')
PAIRED_TOOLS = {
    'xarm_move_both': 'xarm_move_pose',
    'xarm_move_joints_both': 'xarm_move_joints',
    'wuji_move_both': 'wuji_move_joints',
}
COMBINED_TOOL = 'xarm_move_combined'
COMBINED_ACTIONS = {
    'arm': {'pose': 'xarm_move_pose', 'joints': 'xarm_move_joints'},
    'hand': {'joints': 'wuji_move_joints', 'pose': 'wuji_pose', 'home': 'wuji_home'},
}
SHARED_OPTIONS = frozenset({
    'cameras', 'camera_cpus', 'camera_depth_mode', 'stream_enabled', 'stream_fps',
    'stream_window_s', 'stream_depth_hz', 'stream_jpeg_quality', 'stream_archive',
    'robot_log_hz', 'motion_archive_enabled', 'motion_archive_padding_s',
    'motion_archive_codec', 'agent_image_width', 'agent_python', 'protected_paths',
    'calibration_manifest', 'unverified_camera_extrinsics',
})


def combined_actions(arguments):
    """One action per requested device; omitted devices retain their targets."""
    if not isinstance(arguments, dict) or not arguments:
        raise ValueError('provide at least one of left_arm, right_arm, left_hand or right_hand')
    actions = {}
    for device, choice in arguments.items():
        if device not in {f'{side}_{kind}' for side in SIDES for kind in COMBINED_ACTIONS}:
            raise ValueError(f'unknown combined device {device}')
        side, kind = device.split('_')
        if not isinstance(choice, dict) or len(choice) != 1:
            raise ValueError(f'{device} requires exactly one of {list(COMBINED_ACTIONS[kind])}')
        action, payload = next(iter(choice.items()))
        if action not in COMBINED_ACTIONS[kind] or not isinstance(payload, dict):
            raise ValueError(f'{device}: expected one supported action with an argument object')
        actions[device] = (side, COMBINED_ACTIONS[kind][action], deepcopy(payload))
    return actions


def validate_options(options):
    from services.xarm.backend import DEFAULTS
    if not isinstance(options, dict) or set(options) - SHARED_OPTIONS - {'arms'}:
        raise ValueError('bimanual options accept arms and shared camera/recording/launcher settings')
    for key in ('stream_enabled', 'stream_archive', 'motion_archive_enabled'):
        if key in options and type(options[key]) is not bool:
            raise ValueError(f'{key} must be boolean')
    for key in ('stream_fps', 'stream_window_s', 'robot_log_hz', 'agent_image_width'):
        value = options.get(key, DEFAULTS[key])
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    arms = options.get('arms')
    if not isinstance(arms, dict) or set(arms) != set(SIDES):
        raise ValueError('arms must configure exactly left and right')
    ips, serials, addresses = set(), set(), set()
    for side in SIDES:
        cfg = arms[side]
        allowed = set(DEFAULTS) - SHARED_OPTIONS - {'gripper_type', 'gripper_max_width_m',
            'gripper_open_position', 'gripper_closed_position', 'gripper_speed', 'gripper_force'}
        if not isinstance(cfg, dict) or set(cfg) - allowed:
            raise ValueError(f'arms.{side} contains unknown or shared options')
        ip = cfg.get('xarm_ip')
        if not isinstance(ip, str) or not ip.strip() or ip in ips:
            raise ValueError('each arm needs a distinct xarm_ip')
        ips.add(ip)
        merged = {**DEFAULTS, **cfg}
        for key in ('workspace_min', 'workspace_max'):
            vector = merged[key]
            if (not isinstance(vector, (list, tuple)) or len(vector) != 3
                    or any(type(v) not in (int, float) or not math.isfinite(v) for v in vector)):
                raise ValueError(f'{side} {key} must be three finite numbers')
        if any(lo >= hi for lo, hi in zip(merged['workspace_min'], merged['workspace_max'])):
            raise ValueError(f'{side} workspace_min must be below workspace_max')
        for key in allowed - {'workspace_min', 'workspace_max', 'xarm_ip', 'end_effector'}:
            value = merged[key]
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f'{side} {key} must be finite and nonnegative')
            if value == 0 and key not in ('settle_s', 'link_min_z', 'workspace_guard_margin_m'):
                raise ValueError(f'{side} {key} must be positive')
        hand = hand_config(cfg.get('end_effector'))
        if hand['handedness'] != side:
            raise ValueError(f'arms.{side} must contain the {side} hand profile')
        if hand['serial'] in serials or (hand['address'] and hand['address'] in addresses):
            raise ValueError('each hand needs a distinct serial and address')
        serials.add(hand['serial'])
        addresses.add(hand['address'])
    cameras = options.get('cameras')
    if not isinstance(cameras, list) or len(cameras) != 4:
        raise ValueError('configure four cameras: two fixed views and one wrist camera per arm')
    names, serials, wrists = set(), set(), []
    fixed = 0
    for camera in cameras:
        if not isinstance(camera, dict):
            raise ValueError('camera must be an object')
        name, serial = camera.get('name'), camera.get('serial')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', name) or name in names:
            raise ValueError('camera names must be unique path-safe identifiers')
        if not isinstance(serial, str) or not serial or serial in serials:
            raise ValueError('camera serials must be distinct nonempty strings')
        names.add(name)
        serials.add(serial)
        if camera.get('mount') == 'tcp' and camera.get('arm') in SIDES:
            wrists.append(camera['arm'])
        elif camera.get('mount') == 'fixed' and 'arm' not in camera:
            fixed += 1
        else:
            raise ValueError('camera mount must be fixed, or tcp with an explicit left/right arm')
        if camera.get('extrinsics') or camera.get('intrinsics'):
            raise ValueError('use calibration_manifest for a verified bimanual bundle, not per-camera legacy files')
        for key in ('width', 'height'):
            if type(camera.get(key, 640 if key == 'width' else 480)) is not int or camera.get(key, 1) <= 0:
                raise ValueError(f'camera {key} must be a positive integer')
    if fixed != 2 or sorted(wrists) != list(SIDES):
        raise ValueError('need two fixed cameras and exactly one wrist camera per arm')
    unverified = options.get('unverified_camera_extrinsics', [])
    if (not isinstance(unverified, list) or any(not isinstance(name, str) for name in unverified)
            or len(set(unverified)) != len(unverified) or set(unverified) - names):
        raise ValueError('unverified_camera_extrinsics must list distinct configured camera names')
    if options.get('calibration_manifest'):
        from .calibration import BimanualCalibration
        BimanualCalibration(options)
    return deepcopy(options)


def arm_options(options, side):
    if side not in SIDES:
        raise ValueError('arm must be left or right')
    return {**deepcopy(options['arms'][side]), 'cameras': [], 'stream_enabled': False,
            'motion_archive_enabled': False}
