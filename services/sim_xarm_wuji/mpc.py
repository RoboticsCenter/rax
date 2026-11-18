"""A fast hand-only model of one Wuji Hand 2 and one object, for MPC and sampling-based
planners inside skills (set the state from measurements, roll out many control
sequences in parallel CPU threads, score them).

    from services.sim_xarm_wuji.mpc import HandModel, object_pose_cost
    model = HandModel('right', object='cube', wrist_pose=((0.45, -0.2, 0.25), (0, 1, 0, 0)))
    model.set_state(q_measured, object_pose=((0.45, -0.2, 0.025), (0, 0, 0, 1)))
    result = model.rollout(q_ref_sequences)              # (N, H, 20) references at 100 Hz
    costs = object_pose_cost(result, target_position=(0.45, -0.2, 0.08))
    best = q_ref_sequences[costs.argmin()]

The model is the twin's hand (wuji-description Hand 2.2, model.py) with:
- the same MIT law, gain factor, torque constants and current caps as the twin's
  firmware, compiled into MuJoCo actuators so `mujoco.rollout` runs it in C++ threads
  with no Python per step: torque = kt clamp(g (kp (q_ref - q) + kd (dq_ref - dq) + i_ff));
- Coulomb joint friction only (no Stribeck stick-slip: MPC wants a smooth model);
- collision primitives fitted to the hand's meshes (capsules) instead of mesh hulls;
- a 2 ms physics step (the twin runs 1 ms);
- the hand fixed at `wrist_pose` (the mount frame in the caller's world frame; see
  wrist_pose_from_tcp), and an optional table plane.
Frames: whatever frame wrist_pose and object poses are given in (e.g. the arm's base).
Fidelity: this is an internal model for planning; check plans on the full twin.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os

import mujoco
from mujoco import rollout as mj_rollout
import numpy as np
from scipy.spatial.transform import Rotation

from . import model as M
from .scenes import Ball, Cube, Marker, SmallBox
from .spec import HAND_DEFAULTS, MJCF_HAND_JOINTS, PEAK_CURRENT_A, PHYSICS_DEFAULTS, SCENE_DEFAULTS, TIP_SITES, TCP_OFFSET

OBJECTS = {'cube': Cube, 'ball': Ball, 'marker': Marker, 'small_box': SmallBox}


def wrist_pose_from_tcp(side, tcp_position, tcp_quaternion_xyzw, *, tcp_offset_mm_rad=TCP_OFFSET,
                        mount_yaw_deg=None, mount_offset_m=None):
    """The hand mount's pose (position, quaternion xyzw) from a measured TCP pose (as
    xarm_status reports it, in the arm's base frame), with the twin's mount assumptions."""
    yaw = SCENE_DEFAULTS['hand_mount_yaw_deg'][side] if mount_yaw_deg is None else mount_yaw_deg
    offset = SCENE_DEFAULTS['hand_mount_offset_m'] if mount_offset_m is None else mount_offset_m
    T_base_tcp = np.eye(4)
    T_base_tcp[:3, :3] = Rotation.from_quat(tcp_quaternion_xyzw).as_matrix()
    T_base_tcp[:3, 3] = tcp_position
    T_flange_tcp = M.tcp_transform(tcp_offset_mm_rad)
    T_flange_mount = np.eye(4)
    T_flange_mount[:3, :3] = (Rotation.from_euler('z', math.radians(yaw)) * Rotation.from_euler('x', math.pi)).as_matrix()
    T_flange_mount[2, 3] = offset
    T = T_base_tcp @ np.linalg.inv(T_flange_tcp) @ T_flange_mount
    return T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_quat()


@dataclass
class RolloutResult:
    """Per rollout n and control step h (after the step): object pose, fingertip
    positions (thumb..pinky), joint positions and motor currents."""
    object_position: np.ndarray      # (N, H, 3)
    object_quaternion: np.ndarray    # (N, H, 4) xyzw
    fingertips: np.ndarray           # (N, H, 5, 3)
    q: np.ndarray                    # (N, H, 20)
    current: np.ndarray              # (N, H, 20) A
    control_dt: float


class HandModel:
    def __init__(self, side='right', object='cube', *, object_kwargs=None, wrist_pose=None, table_z=0.0,
                 timestep=0.002, kp=8.0, kd=0.15, current_limit_a=0.8, gain_factor=HAND_DEFAULTS['gain_factor'],
                 torque_per_amp_scale=1.0, coulomb_friction_a=HAND_DEFAULTS['coulomb_friction_a'],
                 disabled_joints=(), fast_collision=True, nthread=None):
        self.side = side
        spec = mujoco.MjSpec()
        spec.modelname = f'wuji_hand2_{side}_mpc'
        spec.compiler.degree = False
        spec.option.timestep = float(timestep)
        spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        spec.option.impratio = float(PHYSICS_DEFAULTS['impratio'])
        spec.option.iterations, spec.option.ls_iterations = 20, 10
        hand = M.hand_spec(side, PHYSICS_DEFAULTS)
        for key in ('timestep', 'integrator', 'cone', 'impratio', 'iterations', 'ls_iterations', 'tolerance',
                    'jacobian', 'solver'):
            setattr(hand.option, key, getattr(spec.option, key))
        if fast_collision:
            for geom in hand.geoms:
                if geom.group == M.COLLISION and geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                    geom.type = mujoco.mjtGeom.mjGEOM_CAPSULE  # fitted to the mesh by the compiler
        wrist_pose = wrist_pose or ((0.0, 0.0, table_z + 0.30), (0.0, 0.0, 0.0, 1.0))
        position, quat = wrist_pose
        x, y, z, w = quat
        self.mount = spec.worldbody.add_body(name='hand_base', mocap=True, pos=list(position), quat=[w, x, y, z])
        frame = self.mount.add_frame()
        frame.attach_body(hand.worldbody.first_body(), 'hand_', '')
        self.table = table_z
        if table_z is not None:
            spec.worldbody.add_geom(name='table', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.05],
                                    pos=[0, 0, table_z], friction=[0.6, 0.005, 0.0001])
        self.object_name = object
        if object is not None:
            obj = OBJECTS[object](**(object_kwargs or {}))
            body = spec.worldbody.add_body(name='object', pos=[0.0, 0.0, (table_z or 0.0) + obj.rest_height()])
            body.add_freejoint(name='object_free')
            obj.geoms(body)
            self._object_mass = obj.mass
            spec.add_sensor(name='object_pos', type=mujoco.mjtSensor.mjSENS_FRAMEPOS,
                            objtype=mujoco.mjtObj.mjOBJ_BODY, objname='object')
            spec.add_sensor(name='object_quat', type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                            objtype=mujoco.mjtObj.mjOBJ_BODY, objname='object')
        prefix = 'hand_' + M.PREFIX[side]
        for tip in TIP_SITES:
            spec.add_sensor(name=f'{tip}_pos', type=mujoco.mjtSensor.mjSENS_FRAMEPOS,
                            objtype=mujoco.mjtObj.mjOBJ_SITE, objname=prefix + tip)
        for joint in MJCF_HAND_JOINTS:
            spec.add_sensor(name=f'{joint}_q', type=mujoco.mjtSensor.mjSENS_JOINTPOS,
                            objtype=mujoco.mjtObj.mjOBJ_JOINT, objname=prefix + joint)
        for joint in MJCF_HAND_JOINTS:
            spec.add_sensor(name=f'{joint}_force', type=mujoco.mjtSensor.mjSENS_ACTUATORFRC,
                            objtype=mujoco.mjtObj.mjOBJ_ACTUATOR, objname=f'{prefix}{joint}_mit')
        self.model = spec.compile()
        m = self.model
        self.actuators = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'{prefix}{j}_mit')
                                   for j in MJCF_HAND_JOINTS])
        joints = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, prefix + j) for j in MJCF_HAND_JOINTS]
        self.qadr = np.array([m.jnt_qposadr[j] for j in joints])
        self.vadr = np.array([m.jnt_dofadr[j] for j in joints])
        self.kt = M.WUJI_IDENTIFIED_KT * float(torque_per_amp_scale)
        m.jnt_actfrclimited[joints] = 1
        m.jnt_actfrcrange[joints, 0], m.jnt_actfrcrange[joints, 1] = -self.kt * 3.0, self.kt * 3.0
        m.dof_frictionloss[self.vadr] = self.kt * float(coulomb_friction_a)
        m.dof_solref[self.vadr] = M.STIFF_FRICTION_SOLREF
        m.dof_solimp[self.vadr] = M.STIFF_FRICTION_SOLIMP
        self.gain_factor = float(gain_factor)
        from services.wuji.config import JOINT_NAMES
        self.enabled = np.array([name not in set(disabled_joints) for name in JOINT_NAMES], dtype=float)
        if object is not None:
            obody = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'object')
            ojoint = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'object_free')
            scale = self._object_mass / m.body_mass[obody]
            m.body_mass[obody] *= scale
            m.body_inertia[obody] *= scale
            self.obj_q, self.obj_v = int(m.jnt_qposadr[ojoint]), int(m.jnt_dofadr[ojoint])
        self.set_gains(kp, kd, current_limit_a)
        self.data = mujoco.MjData(m)
        mujoco.mj_setConst(m, self.data)
        mujoco.mj_forward(m, self.data)
        self.nthread = int(nthread or os.cpu_count() or 1)
        self._runner = mj_rollout.Rollout(nthread=self.nthread)
        self._datas = [mujoco.MjData(m) for _ in range(self.nthread)]
        self._sensor_slices()
        self.state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS
        self.initial_state = self._get_state()

    def _sensor_slices(self):
        m = self.model
        adr = lambda name: (int(m.sensor(name).adr[0]), int(m.sensor(name).dim[0]))  # noqa: E731
        self.sens = {}
        if self.object_name is not None:
            self.sens['object_pos'], self.sens['object_quat'] = adr('object_pos'), adr('object_quat')
        self.sens['tips'] = [adr(f'{t}_pos') for t in TIP_SITES]
        self.sens['q'] = [adr(f'{j}_q') for j in MJCF_HAND_JOINTS]
        self.sens['force'] = [adr(f'{j}_force') for j in MJCF_HAND_JOINTS]

    def _get_state(self):
        size = mujoco.mj_stateSize(self.model, self.state_spec)
        state = np.empty(size)
        mujoco.mj_getState(self.model, self.data, state, self.state_spec)
        return state

    # ---- parameters and state ---------------------------------------------------------------------------------

    def set_gains(self, kp=None, kd=None, current_limit_a=None):
        """MIT gains (A/rad, A s/rad) and current caps (A): one value or 20, as the firmware takes them."""
        m = self.model
        if kp is not None:
            self.kp = np.broadcast_to(np.asarray(kp, dtype=float), (20,)).copy()
        if kd is not None:
            self.kd = np.broadcast_to(np.asarray(kd, dtype=float), (20,)).copy()
        if current_limit_a is not None:
            self.cap = np.broadcast_to(np.asarray(current_limit_a, dtype=float), (20,)).copy()
        g = self.gain_factor * self.kt * self.enabled
        m.actuator_gainprm[self.actuators, 0] = g
        m.actuator_biasprm[self.actuators, 1] = -g * self.kp
        m.actuator_biasprm[self.actuators, 2] = -g * self.kd
        m.actuator_forcerange[self.actuators, 0] = -self.kt * self.cap * self.enabled
        m.actuator_forcerange[self.actuators, 1] = self.kt * self.cap * self.enabled

    def set_state(self, q, dq=None, object_pose=None, object_velocity=None, wrist_pose=None):
        """Load measurements: hand joints (SDK order), optional object pose ((x,y,z),(x,y,z,w))
        and twist (v, w in the world frame), optional wrist (mount) pose."""
        d = self.data
        mujoco.mj_resetData(self.model, d)
        d.qpos[self.qadr] = np.asarray(q, dtype=float)
        d.qvel[self.vadr] = 0.0 if dq is None else np.asarray(dq, dtype=float)
        if wrist_pose is not None:
            position, (x, y, z, w) = wrist_pose
            d.mocap_pos[0], d.mocap_quat[0] = position, [w, x, y, z]
        else:
            d.mocap_pos[0], d.mocap_quat[0] = self.model.body_pos[self.mount_id()], self.model.body_quat[self.mount_id()]
        if self.object_name is not None and object_pose is not None:
            position, (x, y, z, w) = object_pose
            d.qpos[self.obj_q:self.obj_q + 3] = position
            d.qpos[self.obj_q + 3:self.obj_q + 7] = [w, x, y, z]
            if object_velocity is not None:
                v, omega = object_velocity
                R = Rotation.from_quat([x, y, z, w]).as_matrix()
                d.qvel[self.obj_v:self.obj_v + 3] = v
                d.qvel[self.obj_v + 3:self.obj_v + 6] = R.T @ np.asarray(omega, dtype=float)
        mujoco.mj_forward(self.model, d)
        self.initial_state = self._get_state()
        return self.initial_state

    def mount_id(self):
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'hand_base')

    # ---- rollouts ----------------------------------------------------------------------------------------------

    def controls(self, q_ref, dq_ref=None, i_ff=None):
        """The actuators' input u = kp q_ref + kd dq_ref + i_ff (the firmware's combined command)."""
        q_ref = np.asarray(q_ref, dtype=float)
        u = self.kp * q_ref
        if dq_ref is not None:
            u = u + self.kd * np.asarray(dq_ref, dtype=float)
        if i_ff is not None:
            u = u + np.asarray(i_ff, dtype=float)
        return u * self.enabled

    def rollout(self, q_ref, dq_ref=None, i_ff=None, *, control_dt=0.01, initial_state=None):
        """Roll out N sequences of H references (each held for control_dt) from the state
        last set; returns a RolloutResult sampled after each control step."""
        q_ref = np.asarray(q_ref, dtype=float)
        if q_ref.ndim == 2:
            q_ref = q_ref[None]
        n, h, _ = q_ref.shape
        substeps = max(1, int(round(control_dt / self.model.opt.timestep)))
        u = self.controls(q_ref, dq_ref, i_ff)
        control = np.repeat(u, substeps, axis=1)             # (N, H*substeps, nu)
        start = self.initial_state if initial_state is None else initial_state
        state, sensordata = self._runner.rollout(self.model, self._datas, np.tile(start, (n, 1)), control)
        sensordata = sensordata[:, substeps - 1::substeps]     # (N, H, nsensor)
        get = lambda a: sensordata[..., a[0]:a[0] + a[1]]       # noqa: E731
        if self.object_name is not None:
            position = get(self.sens['object_pos'])
            wxyz = get(self.sens['object_quat'])
            quaternion = np.concatenate([wxyz[..., 1:], wxyz[..., :1]], axis=-1)
        else:
            position = np.zeros((n, h, 3))
            quaternion = np.tile([0.0, 0.0, 0.0, 1.0], (n, h, 1))
        tips = np.stack([get(a) for a in self.sens['tips']], axis=2)
        q = np.concatenate([get(a) for a in self.sens['q']], axis=-1)
        force = np.concatenate([get(a) for a in self.sens['force']], axis=-1)
        return RolloutResult(position, quaternion, tips, q, force / self.kt, substeps * self.model.opt.timestep)

    def close(self):
        runner, self._runner = self._runner, None
        if runner is not None:
            runner.close()


# ---- cost terms (vectorized over rollouts) -------------------------------------------------------------------------

def object_pose_cost(result, target_position, target_quaternion=None, *, w_position=1.0, w_rotation=0.05,
                     terminal_weight=10.0):
    """Squared position error (m^2) and rotation angle^2 (rad^2) to a target, summed over
    the horizon with extra weight on the final step."""
    error = np.sum((result.object_position - np.asarray(target_position)) ** 2, axis=-1)
    if target_quaternion is not None:
        target = np.asarray(target_quaternion, dtype=float)
        dot = np.abs(np.sum(result.object_quaternion * target, axis=-1)).clip(0, 1)
        error = w_position * error + w_rotation * (2 * np.arccos(dot)) ** 2
    else:
        error = w_position * error
    return error.sum(axis=1) + terminal_weight * error[:, -1]


def effort_cost(result, weight=1e-3):
    return weight * np.sum(result.current ** 2, axis=(1, 2))


def fingertip_distance_cost(result, weight=1.0, fingers=(0, 1)):
    """Distance of the chosen fingertips (thumb=0 .. pinky=4) to the object centre."""
    d = np.linalg.norm(result.fingertips[:, :, fingers] - result.object_position[:, :, None], axis=-1)
    return weight * d.sum(axis=(1, 2))


def benchmark(n_rollouts=(16, 64, 256), horizon=50, control_dt=0.01, nthread=None, object='cube', repeats=3, seed=0):
    """Rollout throughput on this machine: rollouts/s and physics steps/s per batch size."""
    import time
    rng = np.random.default_rng(seed)
    model = HandModel('right', object=object, nthread=nthread,
                      wrist_pose=((0.0, 0.0, 0.17), (0.0, 0.0, 0.0, 1.0)))
    q0 = np.zeros(20)
    model.set_state(q0, object_pose=((0.0, 0.0, 0.025), (0, 0, 0, 1)))
    out = []
    for n in n_rollouts:
        q_ref = q0 + rng.normal(0, 0.3, (n, horizon, 20)).cumsum(axis=1) * 0.05
        model.rollout(q_ref[:2], control_dt=control_dt)  # warm-up
        best = math.inf
        for _ in range(repeats):
            started = time.perf_counter()
            model.rollout(q_ref, control_dt=control_dt)
            best = min(best, time.perf_counter() - started)
        steps = n * horizon * int(round(control_dt / model.model.opt.timestep))
        out.append({'rollouts': n, 'horizon_steps': horizon, 'control_dt_s': control_dt,
                    'seconds': round(best, 4), 'rollouts_per_s': round(n / best, 1),
                    'physics_steps_per_s': round(steps / best), 'simulated_s_per_wall_s': round(
                        steps * model.model.opt.timestep / best, 1), 'nthread': model.nthread})
    model.close()
    return out


if __name__ == '__main__':
    import json
    import sys
    threads = int(sys.argv[1]) if len(sys.argv) > 1 else None
    print(json.dumps(benchmark(nthread=threads), indent=1))
