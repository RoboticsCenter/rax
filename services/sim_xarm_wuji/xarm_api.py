"""A mimic of UFACTORY's `xarm.wrapper.XArmAPI` (xarm-python-sdk 1.18.4) for the
simulated controller boxes: the calls RAX's XArmDriver (services/xarm/sdk.py) and the
operator's bring-up (scripts/xarm ready) make, with the SDK's return conventions
((code, payload) tuples, mm and radians or degrees per is_radian).

`SimXArmDriver` is RAX's own XArmDriver with this API in place of the network one; every
line of XArmDriver above the SDK runs unchanged.
"""
from __future__ import annotations

import math
import time

from services.xarm.sdk import XArmDriver

from .rpc import Client, SimError


class SimXArmAPI:
    def __init__(self, port=None, is_radian=False, *, sim_socket, report_period_s=0.004, **kwargs):
        self.ip = str(port)
        self.default_is_radian = bool(is_radian)
        self._client = Client(sim_socket)
        self._report, self._report_at = None, 0.0
        self._period = report_period_s
        try:
            info = self._client.call('xarm.connect', ip=self.ip)
        except (SimError, OSError) as exc:
            raise ConnectionError(f'simulated xArm {self.ip}: {exc}') from None
        self.side, self._serial = info['side'], info['serial']
        self._tcp_offset = info['tcp_offset']
        self._connected = True

    # ---- helpers ---------------------------------------------------------------------------------------------------
    def _call(self, name, **args):
        if not self._connected:
            return -1  # APIState.NOT_CONNECTED
        try:
            return self._client.call(f'xarm.{name}', ip=self.ip, **args)
        except (SimError, OSError):
            return -1

    def _fresh_report(self, force=False):
        now = time.monotonic()
        if force or self._report is None or now - self._report_at > self._period:
            value = self._call('report')
            if isinstance(value, dict):
                self._report, self._report_at = value, now
        return self._report

    def _radian(self, is_radian):
        return self.default_is_radian if is_radian is None else bool(is_radian)

    @staticmethod
    def _angles_out(values, radian):
        return list(values) if radian else [math.degrees(v) for v in values]

    @staticmethod
    def _angles_in(values, radian):
        return [float(v) for v in values] if radian else [math.radians(float(v)) for v in values]

    # ---- properties ---------------------------------------------------------------------------------------------
    @property
    def connected(self):
        return self._connected

    @property
    def state(self):
        report = self._fresh_report()
        return report['state'] if report else 4

    @property
    def mode(self):
        report = self._fresh_report()
        return report['mode'] if report else 0

    @property
    def error_code(self):
        report = self._fresh_report()
        return report['error_code'] if report else 0

    @property
    def warn_code(self):
        report = self._fresh_report()
        return report['warn_code'] if report else 0

    @property
    def tcp_offset(self):
        x, y, z, roll, pitch, yaw = self._tcp_offset
        return [x, y, z, *self._angles_out([roll, pitch, yaw], self.default_is_radian)]

    @property
    def position(self):
        report = self._fresh_report()
        return self._pose_out(report['position'], self.default_is_radian)

    @property
    def angles(self):
        report = self._fresh_report()
        return self._angles_out(report['angles'], self.default_is_radian)

    def _pose_out(self, pose, radian):
        return [*pose[:3], *self._angles_out(pose[3:6], radian)]

    # ---- reads ------------------------------------------------------------------------------------------------
    def get_position(self, is_radian=None):
        report = self._fresh_report(force=True)
        if report is None:
            return -1, None
        return 0, self._pose_out(report['position'], self._radian(is_radian))

    def get_servo_angle(self, servo_id=None, is_radian=None, is_real=False):
        report = self._fresh_report(force=True)
        if report is None:
            return -1, None
        angles = self._angles_out(report['angles'], self._radian(is_radian))
        if servo_id is None or servo_id == 8:
            return 0, angles
        return 0, angles[int(servo_id) - 1]

    def get_state(self):
        report = self._fresh_report(force=True)
        return (0, report['state']) if report else (-1, None)

    def get_err_warn_code(self, show=False, lang='en'):
        report = self._fresh_report(force=True)
        return (0, [report['error_code'], report['warn_code']]) if report else (-1, None)

    def get_robot_sn(self):
        return 0, self._serial

    def get_forward_kinematics(self, angles, input_is_radian=None, return_is_radian=None):
        q = self._angles_in(angles, self._radian(input_is_radian))
        pose = self._call('fk', angles=q)
        if not isinstance(pose, list):
            return -1, None
        return 0, self._pose_out(pose, self._radian(return_is_radian))

    def get_inverse_kinematics(self, pose, input_is_radian=None, return_is_radian=None, limited=True,
                               ref_angles=None):
        pose = [*pose[:3], *self._angles_in(pose[3:6], self._radian(input_is_radian))]
        reference = ref_angles or self._fresh_report(force=True)['angles'][:6]
        value = self._call('ik', pose=pose, reference=list(reference)[:6])
        if not isinstance(value, dict) or not value['ok']:
            return 25, None  # Planning Error
        return 0, self._angles_out(value['angles'], self._radian(return_is_radian))

    def is_tcp_limit(self, pose, is_radian=None):
        return 0, False  # no reduced-mode TCP boundary is configured on the simulated controller

    def is_joint_limit(self, joint, is_radian=None):
        q = self._angles_in(list(joint)[:6], self._radian(is_radian))
        value = self._call('joint_limited', angles=q)
        return (0, bool(value)) if isinstance(value, bool) else (-1, None)

    # ---- motion ---------------------------------------------------------------------------------------------
    def set_position(self, x=None, y=None, z=None, roll=None, pitch=None, yaw=None, radius=None, speed=None,
                     mvacc=None, mvtime=None, relative=False, is_radian=None, wait=False, timeout=None, **kwargs):
        if relative:
            return 10  # MOVE_RELATIVE is not served by the simulated controller (RAX sends absolute targets)
        radian = self._radian(is_radian)
        current = self._fresh_report(force=True)['position']
        pose = [current[0] if x is None else x, current[1] if y is None else y, current[2] if z is None else z]
        rpy = [current[3 + i] if v is None else (v if radian else math.radians(v))
               for i, v in enumerate((roll, pitch, yaw))]
        code = self._call('set_position', pose=[*pose, *rpy], speed=float(speed or 100.0),
                          mvacc=float(mvacc or 2000.0))
        if code == 0 and wait:
            self._wait(timeout)
        return code

    def set_servo_angle(self, servo_id=None, angle=None, speed=None, mvacc=None, mvtime=None, relative=False,
                        is_radian=None, wait=False, timeout=None, radius=None, **kwargs):
        radian = self._radian(is_radian)
        if servo_id not in (None, 8):
            return 10
        q = self._angles_in(list(angle)[:6], radian)
        if relative:
            current = self._fresh_report(force=True)['angles'][:6]
            q = [a + b for a, b in zip(current, q)]
        speed = float(speed) if speed is not None else 0.35
        mvacc = float(mvacc) if mvacc is not None else 1.0
        if not radian:
            speed, mvacc = math.radians(speed), math.radians(mvacc)
        code = self._call('set_servo_angle', angle=q, speed=speed, mvacc=mvacc)
        if code == 0 and wait:
            self._wait(timeout)
        return code

    def _wait(self, timeout):
        deadline = time.monotonic() + (timeout or 600.0)
        time.sleep(0.05)
        while time.monotonic() < deadline:
            report = self._fresh_report(force=True)
            if report['state'] != 1:
                return
            time.sleep(0.02)

    # ---- state ---------------------------------------------------------------------------------------------------
    def set_state(self, state=0):
        return self._call('set_state', state=int(state))

    def set_mode(self, mode=0, detection_param=0):
        return self._call('set_mode', mode=int(mode))

    def motion_enable(self, enable=True, servo_id=None):
        return self._call('motion_enable', enable=bool(enable))

    def clean_error(self):
        return self._call('clean_error')

    def clean_warn(self):
        return self._call('clean_warn')

    def emergency_stop(self):
        return self._call('emergency_stop')

    def set_gripper_enable(self, enable, **kwargs):
        return 0  # the rig's end effectors are Wuji hands, not xArm grippers

    def disconnect(self):
        self._connected = False
        self._client.close()


class SimXArmDriver(XArmDriver):
    """RAX's XArmDriver, unchanged, on the simulated controller box."""

    def __init__(self, options, sim_socket):
        self.arm = SimXArmAPI(options['xarm_ip'], is_radian=True, sim_socket=sim_socket)
        self.options = options
