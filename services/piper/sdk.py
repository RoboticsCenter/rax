"""Direct pyAgxArm Piper adapter with a finite trajectory and an independent watchdog."""
import threading
import time
from pathlib import Path

import numpy as np

from .kinematics import LIMITS, vector
from .transport import PassiveFeedback


def resolve_interface(serial):
    matches = []
    for entry in Path('/sys/class/net').glob('can*'):
        for parent in (entry / 'device').resolve().parents:
            identity = parent / 'serial'
            if identity.is_file() and identity.read_text().strip() == serial:
                matches.append(entry.name)
                break
    if len(matches) != 1:
        raise RuntimeError(f'Expected one Piper CAN adapter with USB serial {serial}; found {matches}')
    return matches[0]


class PiperArm:
    def __init__(self, settings, *, permit=lambda: False, guard=None, sdk=None,
                 feedback=None, interface=None, profile_resolver=None):
        self.settings, self.permit = settings, permit
        self.guard = guard
        self.interface = interface or resolve_interface(settings['arm_usb_serial'])
        self.sdk = sdk
        self.feedback = feedback
        self.firmware = None
        self.firmware_verified = False
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.thread = None
        self.target = self.trajectory = None
        self.streaming = False
        self.fault = None
        self.lease_until = 0.
        self.last_stop_reason = None
        self.closed = False
        try:
            if self.sdk is None or profile_resolver is None:
                from pyAgxArm import AgxArmFactory, create_agx_arm_config, resolve_firmware_profile
                profile_resolver = profile_resolver or resolve_firmware_profile
            if self.sdk is None:
                config = create_agx_arm_config(robot='piper', firmeware_version=settings['arm_firmware_profile'],
                                               interface='socketcan', channel=self.interface)
                self.sdk = AgxArmFactory.create_arm(config)
            self.sdk.connect()
            deadline = time.monotonic() + 2.
            while self.sdk.get_fps() <= 0:
                if time.monotonic() > deadline:
                    raise RuntimeError('Piper SDK receive loop did not become live')
                time.sleep(.01)
            self.firmware = self.sdk.get_firmware(timeout=2.)
            if self.firmware:
                profile = profile_resolver('piper', self.firmware['software_version'])
                self.firmware_verified = profile == settings['arm_firmware_profile']
            self.feedback = feedback if feedback is not None else PassiveFeedback(self.interface)
            deadline = time.monotonic() + 2.
            while self.sdk.get_joint_angles() is None or self.sdk.get_arm_status() is None:
                if time.monotonic() > deadline:
                    raise RuntimeError('No Piper SDK feedback')
                time.sleep(.01)
            self.state()
        except BaseException:
            self.close()
            raise

    def state(self):
        angles = self.sdk.get_joint_angles()
        status = self.sdk.get_arm_status()
        if angles is None or status is None or self.sdk.get_comm_error():
            raise RuntimeError('Piper SDK feedback unavailable or communication failed')
        now = time.time()
        if any(not -.2 <= now - message.timestamp <= .25 for message in (angles, status)):
            raise RuntimeError('Piper SDK feedback is stale')
        sdk_joints = vector(angles.msg, 6, 'Piper feedback')
        independent = self.feedback.check(np.rad2deg(sdk_joints))
        with self.lock:
            busy = self.trajectory is not None
        enabled = independent['enabled']
        return {'serial': self.settings['arm_usb_serial'], 'side': 'right', 'interface': self.interface,
                'joints': np.deg2rad(independent['joints_deg']).tolist(),
                'driver': 'pyagxarm', 'firmware': self.firmware,
                'firmware_verified': self.firmware_verified, 'enabled': enabled,
                'driver_state': 'enabled' if all(enabled) else 'disabled',
                'streaming': self.streaming, 'busy': busy, 'motion_fault': self.fault,
                'ctrl_mode': int(status.msg.ctrl_mode), 'arm_status': independent['arm_status'],
                'error': independent['error'], 'independent_feedback': independent,
                'last_stop_reason': self.last_stop_reason}

    def _healthy(self, state):
        if (not state['firmware_verified'] or state['ctrl_mode'] != 1
                or state['enabled'] != [True] * 6 or state['arm_status']
                or state['error'] or any(state['independent_feedback']['motor_faults']) or self.fault):
            raise RuntimeError('Piper must be enabled, healthy and use the verified firmware profile')

    def segment(self, joints, seconds):
        goal = vector(joints, 6, 'joint target')
        if not 0 < seconds <= self.settings['segment_s']:
            raise ValueError('Arm segment duration outside the bounded command horizon')
        if np.any(goal < LIMITS[:, 0]) or np.any(goal > LIMITS[:, 1]):
            raise ValueError('Joint limits exceeded; SDK clamping is not an acceptable fallback')
        with self.lock:
            if not self.permit():
                raise RuntimeError('Operator motion permit is absent')
            state = self.state()
            self._healthy(state)
            measured = np.asarray(state['joints'])
            if np.any(measured < LIMITS[:, 0]) or np.any(measured > LIMITS[:, 1]):
                raise RuntimeError('Measured arm starts outside legal joint limits; no recovery commanded')
            if self.guard is not None:
                self.guard(measured)
                self.guard(goal)
            if self.trajectory is not None:
                raise RuntimeError('A Piper segment is already active')
            start = np.asarray(state['joints']) if self.target is None else self.target.copy()
            if np.max(np.abs(goal - start)) / seconds > self.settings['joint_speed_radps'] + 1e-6:
                raise ValueError('Piper segment exceeds joint speed limit')
            self.trajectory = (start, goal.copy(), time.monotonic(), seconds)
            self.lease_until = time.monotonic() + seconds + .2
            if self.thread is None:
                self.target = start
                self.sdk.set_speed_percent(self.settings['arm_speed_percent'])
                self.sdk.set_motion_mode('j')
                self.sdk.set_auto_set_motion_mode_enabled(False)
                self.streaming = True
                self.thread = threading.Thread(target=self._stream, name='piper-servo', daemon=True)
                self.thread.start()

    def _stream(self):
        while not self.stop.is_set():
            try:
                with self.lock:
                    state = self.state()
                    self._healthy(state)
                    measured = np.asarray(state['joints'])
                    if self.guard is not None:
                        self.guard(measured)
                    if np.max(np.abs(measured - self.target)) > self.settings['max_following_error_rad']:
                        raise RuntimeError('Piper following error exceeds the watchdog limit')
                    if not self.permit() or time.monotonic() > self.lease_until:
                        self.trajectory = None
                        self.target = measured
                        self.last_stop_reason = 'operator disarmed or motion lease expired'
                    if self.trajectory is not None:
                        start, goal, began, seconds = self.trajectory
                        fraction = min(1., (time.monotonic() - began) / seconds)
                        self.target = start + fraction * (goal - start)
                        if fraction == 1.:
                            self.trajectory = None
                    if np.any(self.target < LIMITS[:, 0]) or np.any(self.target > LIMITS[:, 1]):
                        raise RuntimeError('Measured hold position lies outside legal joint limits')
                    self.sdk.move_j(self.target.tolist())
            except Exception as exc:
                self.fault = str(exc)
                self.streaming = False
                self.trajectory = None
                try:
                    self.sdk.electronic_emergency_stop()
                except Exception:
                    pass
                return
            self.stop.wait(.01)

    def hold(self):
        with self.lock:
            self.trajectory = None
            if not self.streaming:
                return
            measured = np.asarray(self.state()['joints'])
            if np.any(measured < LIMITS[:, 0]) or np.any(measured > LIMITS[:, 1]):
                self.sdk.electronic_emergency_stop()
                raise RuntimeError('Position hold outside limits; damped stop requested instead')
            self.target = measured
            self.lease_until = time.monotonic() + .2
            self.sdk.move_j(measured.tolist())

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=1.)
        try:
            if self.streaming:
                self.sdk.electronic_emergency_stop()
        finally:
            self.streaming = False
            if self.feedback is not None:
                self.feedback.close()
            if self.sdk is not None:
                self.sdk.disconnect()
