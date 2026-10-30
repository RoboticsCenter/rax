"""Thin adapter around UFACTORY's xArm Python SDK.

The backend uses only public read and motion methods. Operator bring-up (power, mode,
state reset, error clearing) stays in scripts/xarm, outside the agent's MCP surface.
"""
from __future__ import annotations

import math
import time

import numpy as np
from scipy.spatial.transform import Rotation


class XArmError(RuntimeError):
    pass


class TargetRejected(XArmError):
    """The xArm controller refused a target or command."""


class MotionStopped(RuntimeError):
    """The motion was interrupted (operator disarm, workspace guard); the arm holds."""


def _code_result(result, op):
    """Return the payload of an xArm SDK call, raising when its code is non-zero."""
    if isinstance(result, tuple):
        code, payload = result[0], result[1:]
    elif isinstance(result, list) and result and isinstance(result[0], int):
        code, payload = result[0], result[1:]
    elif isinstance(result, int):
        code, payload = result, ()
    else:
        return result
    if code != 0:
        exc = TargetRejected(f'{op} returned xArm code {code}')
        exc.operator_required = True
        exc.sdk_code = code
        exc.sdk_operation = op
        raise exc
    if len(payload) == 1:
        return payload[0]
    return payload


def _quat_from_rpy(rpy):
    q = Rotation.from_euler('xyz', rpy).as_quat()
    return -q if q[3] < 0 else q


def _rpy_from_quat(quaternion):
    return Rotation.from_quat(quaternion).as_euler('xyz')


def _call_optional(method, *args, op, **kwargs):
    """Call an SDK method while tolerating older signatures."""
    try:
        return _code_result(method(*args, **kwargs), op)
    except TypeError:
        return _code_result(method(*args), op)


class XArmDriver:
    def __init__(self, options):
        try:
            from xarm.wrapper import XArmAPI
        except ImportError as exc:
            raise RuntimeError('install xarm-python-sdk in the robot Python environment') from exc
        self.arm = XArmAPI(options['xarm_ip'], is_radian=True)
        self.options = options

    def state(self):
        position = np.asarray(_code_result(self.arm.get_position(is_radian=True), 'get_position'), dtype=np.float64)
        joints = np.asarray(_code_result(self.arm.get_servo_angle(is_radian=True), 'get_servo_angle'),
                            dtype=np.float64)[:6]
        state = {'tcp_position': position[:3] / 1000.0, 'tcp_rpy_rad': position[3:6],
                 'tcp_quaternion': _quat_from_rpy(position[3:6]), 'joints': joints,
                 'state': None, 'mode': None, 'error_code': None, 'warn_code': None}
        for name, key in (('get_state', 'state'), ('get_mode', 'mode')):
            method = getattr(self.arm, name, None)
            try:
                if method is not None:
                    state[key] = _code_result(method(), name)
                elif key == 'mode':
                    # SDK 1.18 exposes mode through the controller report
                    # property, without a get_mode RPC.
                    state[key] = self.arm.mode
            except Exception as exc:  # noqa: BLE001 - status should report partial data
                state[f'{key}_error'] = str(exc)
        method = getattr(self.arm, 'get_err_warn_code', None)
        if method is not None:
            try:
                error, warn = _code_result(method(), 'get_err_warn_code')
                state['error_code'], state['warn_code'] = error, warn
            except Exception as exc:  # noqa: BLE001
                state['err_warn_error'] = str(exc)
        return state

    def forward_kinematics(self, joints):
        method = getattr(self.arm, 'get_forward_kinematics', None) or getattr(self.arm, 'get_fk', None)
        if method is None:
            raise XArmError('xArm SDK has no get_forward_kinematics/get_fk method')
        values = _call_optional(method, np.asarray(joints, dtype=np.float64).tolist(), op='get_forward_kinematics',
                                input_is_radian=True, return_is_radian=True)
        pose = np.asarray(values, dtype=np.float64)
        return pose[:3] / 1000.0, _quat_from_rpy(pose[3:6])

    def check_tcp_limit(self, position, quaternion):
        method = getattr(self.arm, 'is_tcp_limit', None)
        if method is None:
            return None
        rpy = _rpy_from_quat(quaternion)
        pose = [*(np.asarray(position, dtype=np.float64) * 1000.0).tolist(), *rpy.tolist()]
        values = _call_optional(method, pose, op='is_tcp_limit', is_radian=True)
        if isinstance(values, (list, tuple, np.ndarray)):
            limited = any(bool(v) for v in values)
        else:
            limited = bool(values)
        return 'xArm controller reports the TCP target is outside its limit' if limited else None

    def check_joint_limit(self, joints):
        method = getattr(self.arm, 'is_joint_limit', None)
        if method is None:
            return None
        values = _call_optional(method, np.asarray(joints, dtype=np.float64).tolist(),
                                op='is_joint_limit', is_radian=True)
        if isinstance(values, (list, tuple, np.ndarray)):
            limited = any(bool(v) for v in values)
        else:
            limited = bool(values)
        return 'xArm controller reports the joint target is outside its limit' if limited else None

    def move_pose(self, position, quaternion, *, speed_mps, acc_mps2, timeout_s):
        rpy = _rpy_from_quat(quaternion)
        xyz_mm = (np.asarray(position, dtype=np.float64) * 1000.0).tolist()
        return _code_result(self.arm.set_position(x=xyz_mm[0], y=xyz_mm[1], z=xyz_mm[2],
                                                  roll=float(rpy[0]), pitch=float(rpy[1]), yaw=float(rpy[2]),
                                                  radius=None, speed=float(speed_mps) * 1000.0,
                                                  mvacc=float(acc_mps2) * 1000.0, is_radian=True,
                                                  wait=False, timeout=float(timeout_s)),
                            'set_position')

    def move_joints(self, joints, *, speed_radps, acc_radps2, timeout_s):
        return _code_result(self.arm.set_servo_angle(angle=np.asarray(joints, dtype=np.float64).tolist(),
                                                     speed=float(speed_radps), mvacc=float(acc_radps2),
                                                     is_radian=True, wait=False, timeout=float(timeout_s)),
                            'set_servo_angle')

    def hold(self):
        method = getattr(self.arm, 'set_state', None)
        if method is not None:
            return _code_result(method(4), 'set_state(hold)')
        method = getattr(self.arm, 'emergency_stop', None)
        if method is not None:
            return _code_result(method(), 'emergency_stop')
        raise XArmError('xArm SDK has no set_state/emergency_stop hold method')

    def gripper_state(self):
        typ = self.options['gripper_type']
        if typ == 'g2' and hasattr(self.arm, 'get_gripper_g2_position'):
            raw = _code_result(self.arm.get_gripper_g2_position(), 'get_gripper_g2_position')
        else:
            raw = _code_result(self.arm.get_gripper_position(), 'get_gripper_position')
        raw = float(raw)
        width = self._raw_to_width(raw)
        return {'width': round(width, 4), 'max_width': self.options['gripper_max_width_m'],
                'raw_position': raw, 'is_grasped': False, 'is_moving': False, 'stale': False}

    def gripper_move(self, width_m, *, speed, force=None):
        raw = self._width_to_raw(width_m)
        typ = self.options['gripper_type']
        if typ == 'g2' and hasattr(self.arm, 'set_gripper_g2_position'):
            speed = int(round(float(speed)))
            speed = max(15, min(225, speed))
            force = int(round(float(force or self.options['gripper_force'])))
            force = max(1, min(100, force))
            return _code_result(self.arm.set_gripper_g2_position(raw, wait=True, speed=speed,
                                                                 force=force,
                                                                 timeout=10),
                                'set_gripper_g2_position')
        method = getattr(self.arm, 'set_gripper_position')
        speed = int(round(float(speed)))
        try:
            return _code_result(method(raw, wait=True, speed=speed), 'set_gripper_position')
        except TypeError:
            return _code_result(method(raw, wait=True), 'set_gripper_position')

    def _width_to_raw(self, width_m):
        if self.options['gripper_type'] == 'g2':
            return int(round(np.clip(float(width_m) * 1000.0, 0.0, 84.0)))
        ratio = np.clip(float(width_m) / float(self.options['gripper_max_width_m']), 0.0, 1.0)
        closed = float(self.options['gripper_closed_position'])
        open_ = float(self.options['gripper_open_position'])
        return int(round(closed + ratio * (open_ - closed)))

    def _raw_to_width(self, raw):
        if self.options['gripper_type'] == 'g2':
            return np.clip(float(raw) / 1000.0, 0.0, float(self.options['gripper_max_width_m']))
        closed = float(self.options['gripper_closed_position'])
        open_ = float(self.options['gripper_open_position'])
        if math.isclose(open_, closed):
            return 0.0
        return np.clip((float(raw) - closed) / (open_ - closed), 0.0, 1.0) * self.options['gripper_max_width_m']

    def close(self):
        method = getattr(self.arm, 'disconnect', None)
        if method is not None:
            method()
