"""The xArm 6 controller box, as the simulator runs it behind the xArm SDK.

What RAX's XArmDriver (services/xarm/sdk.py) and operator scripts rely on:
- state 4 (stopped) and motion disabled at power-on; the operator's `ready`
  (clean_error, clean_warn, set_mode(0), set_state(0), motion_enable(True)) makes it
  state 2 (idle); 1 while a motion executes; set_state(4) stops and holds;
- set_servo_angle: a synchronized joint-space move with a trapezoidal profile at the
  given speed (rad/s) and acceleration;
- set_position: a straight TCP line (orientation slerped in step) with a trapezoidal
  profile in mm/s, joint references by inverse kinematics at the controller's 250 Hz.
  An unreachable line raises controller error 21 (Kinematic Error) before moving; a
  line needing joint speed above pi rad/s raises 24 (Speed Exceeds Limit);
- get_position / get_forward_kinematics report the TCP (flange plus `tcp_offset`) in
  mm and rad, roll-pitch-yaw as scipy's extrinsic 'xyz' (RAX's convention);
- collision detection: a joint torque at its limit for collision_time_s raises 31.
Commands queue as on the real controller (wait=False).
"""
from __future__ import annotations

from collections import deque
import math
import threading

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .model import tcp_transform, xarm6_spec
from .spec import ARM_JOINTS

JOINT_SPEED_LIMIT = math.pi  # xArm 6: 180 deg/s per joint
ERR_KINEMATIC, ERR_SPEED, ERR_COLLISION, ERR_JOINT_LIMIT = 21, 24, 31, 23
CODE_ERROR, CODE_NOT_READY, CODE_JOINT_LIMIT = 1, 9, -7


class ArmKinematics:
    """The controller's own model of one xArm 6 (no hand), with its TCP offset."""

    def __init__(self, arm_options, tcp_offset_mm_rad):
        spec = xarm6_spec(arm_options)
        T = tcp_transform(tcp_offset_mm_rad)
        x, y, z, w = Rotation.from_matrix(T[:3, :3]).as_quat()
        spec.body('link6').add_site(name='tcp', pos=T[:3, 3].tolist(), quat=[w, x, y, z])
        for geom in spec.geoms:
            geom.contype = geom.conaffinity = 0
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'tcp')
        self.lower = self.model.jnt_range[:6, 0].copy()
        self.upper = self.model.jnt_range[:6, 1].copy()
        self.effort = self.model.actuator_forcerange[:6, 1].copy()
        self.lock = threading.Lock()
        self._jp = np.zeros((3, self.model.nv))
        self._jr = np.zeros((3, self.model.nv))

    def fk(self, q):
        with self.lock:
            d = self.data
            d.qpos[:6] = q
            mujoco.mj_kinematics(self.model, d)
            return d.site_xpos[self.site].copy(), d.site_xmat[self.site].reshape(3, 3).copy()

    def ik(self, position, rotation, q0, *, iterations=60, tol_m=2e-4, tol_rad=2e-3, converge_m=1e-6,
           converge_rad=1e-5):
        """Damped least squares from q0. Iterates to converge_* (so consecutive path samples
        carry no solver noise into the servos' velocity feed-forward); ok within tol_*."""
        q = np.clip(np.asarray(q0, dtype=float).copy(), self.lower, self.upper)
        with self.lock:
            d, m = self.data, self.model
            for _ in range(iterations):
                d.qpos[:6] = q
                mujoco.mj_kinematics(m, d)
                mujoco.mj_comPos(m, d)
                p = d.site_xpos[self.site]
                R = d.site_xmat[self.site].reshape(3, 3)
                e_p = position - p
                e_r = Rotation.from_matrix(rotation @ R.T).as_rotvec()
                if np.linalg.norm(e_p) < converge_m and np.linalg.norm(e_r) < converge_rad:
                    return q, True
                mujoco.mj_jacSite(m, d, self._jp, self._jr, self.site)
                J = np.vstack([self._jp[:, :6], self._jr[:, :6]])
                err = np.concatenate([e_p, e_r])
                lam = 1e-3 + 0.05 * min(1.0, float(np.linalg.norm(err)))
                dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), err)
                step = np.max(np.abs(dq))
                if step > 0.3:
                    dq *= 0.3 / step
                q = np.clip(q + dq, self.lower, self.upper)
            d.qpos[:6] = q
            mujoco.mj_kinematics(m, d)
            e_p = position - d.site_xpos[self.site]
            e_r = Rotation.from_matrix(rotation @ d.site_xmat[self.site].reshape(3, 3).T).as_rotvec()
            return q, bool(np.linalg.norm(e_p) < tol_m and np.linalg.norm(e_r) < tol_rad)


def trapezoid(distance, v_max, a_max):
    """Duration and a sampler s(t) in [0, 1] of a rest-to-rest trapezoidal profile."""
    distance = abs(float(distance))
    v_max, a_max = max(float(v_max), 1e-9), max(float(a_max), 1e-9)
    if distance < 1e-12:
        return 0.0, lambda t: 1.0
    t_acc = v_max / a_max
    if a_max * t_acc**2 >= distance:  # triangular
        t_acc = math.sqrt(distance / a_max)
        v_max = a_max * t_acc
        t_flat = 0.0
    else:
        t_flat = (distance - a_max * t_acc**2) / v_max
    total = 2 * t_acc + t_flat

    def s(t):
        t = min(max(t, 0.0), total)
        if t < t_acc:
            x = 0.5 * a_max * t**2
        elif t < t_acc + t_flat:
            x = 0.5 * a_max * t_acc**2 + v_max * (t - t_acc)
        else:
            r = total - t
            x = distance - 0.5 * a_max * r**2
        return x / distance
    return total, s


def pose_mm_rad(position, rotation):
    return [*(np.asarray(position) * 1000.0).tolist(), *Rotation.from_matrix(rotation).as_euler('xyz').tolist()]


class XArmController:
    def __init__(self, side, serial, kinematics, options, tcp_offset_mm_rad, initial_q):
        self.side, self.serial, self.kin, self.opts = side, serial, kinematics, options
        self.tcp_offset = [float(v) for v in tcp_offset_mm_rad]
        self.hz = float(options['controller_hz'])
        self.lock = threading.RLock()
        self.power_on(initial_q)

    def power_on(self, q):
        with self.lock:
            self.state, self.mode, self.error_code, self.warn_code = 4, 0, 0, 0
            self.motion_enabled = False
            self.queue = deque()
            self.active = None  # (t0, samples (N, 6))
            self.q_ref = np.asarray(q, dtype=float).copy()
            self.dq_ref = np.zeros(6)
            self.collision_time = 0.0
            self.measured = self.q_ref.copy()
            self.torque = np.zeros(6)

    # ---- operator bring-up (scripts/xarm ready) -----------------------------------------------------------------------

    def clean_error(self):
        with self.lock:
            self.error_code = 0
            return 0

    def clean_warn(self):
        with self.lock:
            self.warn_code = 0
            return 0

    def set_mode(self, mode):
        with self.lock:
            if int(mode) != 0:
                return CODE_ERROR  # only position mode is simulated
            self.mode = 0
            return 0

    def motion_enable(self, enable=True):
        with self.lock:
            self.motion_enabled = bool(enable)
            if not enable:
                self._halt(4)
            return 0

    def set_state(self, state):
        with self.lock:
            state = int(state)
            if state == 4:
                self._halt(4)
            elif state == 0:
                if self.error_code:
                    return CODE_ERROR
                # Accepted before motion_enable (scripts/xarm ready calls it first); the
                # controller only leaves the stopped state once motion is enabled.
                if self.motion_enabled:
                    self.state = 1 if (self.active or self.queue) else 2
            elif state == 3:
                self._halt(3)
            else:
                return CODE_ERROR
            return 0

    def _halt(self, state):
        self.queue.clear()
        self.active = None
        self.q_ref = self.measured.copy()
        self.dq_ref = np.zeros(6)
        self.state = state

    def emergency_stop(self):
        with self.lock:
            self._halt(4)
            return 0

    def fault(self, code):
        with self.lock:
            self.error_code = int(code)
            self._halt(4)

    # ---- motion commands ----------------------------------------------------------------------------------------------

    def _ready_code(self):
        if self.error_code:
            return CODE_ERROR
        if self.state not in (0, 1, 2) or not self.motion_enabled or self.mode != 0:
            return CODE_NOT_READY
        return 0

    def _start_q(self):
        if self.queue:
            return self.queue[-1][-1].copy()
        if self.active is not None:
            return self.active[1][-1].copy()
        return self.q_ref.copy()

    def _enqueue(self, samples):
        self.queue.append(np.asarray(samples, dtype=float))
        self.state = 1

    def set_servo_angle(self, angle, speed, mvacc):
        with self.lock:
            code = self._ready_code()
            if code:
                return code
            goal = np.asarray(angle, dtype=float)[:6]
            if goal.shape != (6,) or not np.isfinite(goal).all():
                return CODE_ERROR
            if np.any(goal < self.kin.lower) or np.any(goal > self.kin.upper):
                return CODE_JOINT_LIMIT
            start = self._start_q()
            delta = goal - start
            duration, s = trapezoid(np.max(np.abs(delta)), min(float(speed), JOINT_SPEED_LIMIT), float(mvacc))
            n = max(2, int(math.ceil(duration * self.hz)) + 1)
            times = np.linspace(0.0, duration, n)
            self._enqueue([start + delta * s(t) for t in times])
            return 0

    def set_position(self, pose_mm_rad_value, speed, mvacc):
        """pose: [x, y, z (mm), roll, pitch, yaw (rad)]; speed mm/s; mvacc mm/s^2."""
        with self.lock:
            code = self._ready_code()
            if code:
                return code
            start_q = self._start_q()
        pose = np.asarray(pose_mm_rad_value, dtype=float)
        p1 = pose[:3] / 1000.0
        R1 = Rotation.from_euler('xyz', pose[3:6])
        p0, R0m = self.kin.fk(start_q)
        R0 = Rotation.from_matrix(R0m)
        distance = float(np.linalg.norm(p1 - p0))
        angle = float(np.linalg.norm((R1 * R0.inv()).as_rotvec()))
        linear = trapezoid(distance, float(speed) / 1000.0, float(mvacc) / 1000.0)
        turn = trapezoid(angle, self.opts['rotation_speed_radps'], self.opts['rotation_acc_radps2'])
        # One synchronized profile: the slower of translation and rotation sets it.
        duration, s = linear if linear[0] >= turn[0] else turn
        n = max(2, int(math.ceil(duration * self.hz)) + 1)
        slerp = Slerp([0.0, 1.0], Rotation.concatenate([R0, R1]))
        q = start_q.copy()
        samples = []
        for t in np.linspace(0.0, duration, n):
            u = s(t)
            q, ok = self.kin.ik(p0 + (p1 - p0) * u, slerp([u]).as_matrix()[0], q)
            if not ok:
                self.fault(ERR_KINEMATIC)
                return CODE_ERROR
            samples.append(q.copy())
        samples = np.asarray(samples)
        if len(samples) > 1 and np.max(np.abs(np.diff(samples, axis=0))) * self.hz > JOINT_SPEED_LIMIT:
            self.fault(ERR_SPEED)
            return CODE_ERROR
        with self.lock:
            code = self._ready_code()
            if code:
                return code
            if not np.allclose(self._start_q(), start_q, atol=1e-9):
                return CODE_ERROR  # the queue changed during planning
            self._enqueue(samples)
            return 0

    # ---- the servo cycle (physics thread) -------------------------------------------------------------------------

    def tick(self, t, measured, torque, dt):
        """Called each physics step: returns (q_ref, dq_ref) for the joint servos."""
        with self.lock:
            self.measured = np.asarray(measured, dtype=float).copy()
            self.torque = np.asarray(torque, dtype=float).copy()
            if self.state in (1, 2) and self.motion_enabled:
                saturated = np.any(np.abs(self.torque) >= 0.95 * self.kin.effort)
                self.collision_time = self.collision_time + dt if saturated else 0.0
                if self.collision_time >= float(self.opts['collision_time_s']):
                    self.collision_time = 0.0
                    self.error_code = ERR_COLLISION
                    self._halt(4)
                    return self.q_ref, self.dq_ref
            if self.active is None and self.queue and self.state == 1:
                self.active = (t, self.queue.popleft())
            if self.active is not None:
                t0, samples = self.active
                x = (t - t0) * self.hz
                i = int(x)
                if i >= len(samples) - 1:
                    self.q_ref = samples[-1].copy()
                    self.dq_ref = np.zeros(6)
                    self.active = None
                    if self.queue:
                        self.active = (t, self.queue.popleft())
                    else:
                        self.state = 2
                else:
                    f = x - i
                    self.q_ref = samples[i] + (samples[i + 1] - samples[i]) * f
                    self.dq_ref = (samples[i + 1] - samples[i]) * self.hz
            return self.q_ref, self.dq_ref

    # ---- reports ---------------------------------------------------------------------------------------------------

    def report(self):
        with self.lock:
            q = self.measured.copy()
            state, mode, error, warn = self.state, self.mode, self.error_code, self.warn_code
        position, rotation = self.kin.fk(q)
        return {'position': pose_mm_rad(position, rotation), 'angles': [*q.tolist(), 0.0],
                'state': state, 'mode': mode, 'error_code': error, 'warn_code': warn,
                'tcp_offset': list(self.tcp_offset), 'serial': self.serial, 'joint_names': list(ARM_JOINTS),
                'torque': self.torque.tolist(), 'motion_enabled': self.motion_enabled}

    def forward_kinematics(self, angles):
        position, rotation = self.kin.fk(np.asarray(angles, dtype=float)[:6])
        return pose_mm_rad(position, rotation)

    def joint_limited(self, angles):
        q = np.asarray(angles, dtype=float)[:6]
        return bool(np.any(q < self.kin.lower) or np.any(q > self.kin.upper))
