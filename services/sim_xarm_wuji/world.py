"""The simulated rig in one MuJoCo world, stepped at 1 kHz.

Each physics step (1 ms):
1. each hand's firmware applies the last command that has arrived (after its latency)
   and loads the MIT law into its actuators (firmware.py, model.py);
2. each hand joint's dry friction is set from its speed (Stribeck curve between static
   and Coulomb friction, in amperes times the joint's torque constant), so slow
   references stick and slip as the rig's hands do;
3. each arm's controller box advances its planned trajectory and the servos follow
   (xarm_controller.py), with the hand's weight compensated as the xArm's TCP payload;
4. the scene's behaviours run (threads, release, engagement, the bulb's light);
5. mj_step; the firmware reads its currents and updates stall and heat.

`SimWorld` is usable in-process (tests, MPC set-up, notebooks) and is what the sim
server (server.py) runs in real time behind the SDK mimics.
"""
from __future__ import annotations

import threading

import mujoco
import numpy as np

from services.wuji.config import JOINT_NAMES, validate_config

from . import model as M
from .firmware import HandFirmware, stribeck
from .rig import Rig
from .scenes import Scene
from .spec import SIDES, options as sim_options
from .xarm_controller import ArmKinematics, XArmController

HOME_Q = (0.0, -0.70, -1.20, 0.0, 1.90, 0.0)  # services.xarm.backend HOME_Q


class SimWorld:
    def __init__(self, backend_options, *, scene=None, seed=None, randomize=None):
        self.options = backend_options
        self.sim = sim_options(backend_options.get('simulation'))
        if scene is not None:
            self.sim['scene'] = scene
        if seed is not None:
            self.sim['seed'] = int(seed)
        if randomize is not None:
            self.sim['randomize'] = bool(randomize)
        self.rig = Rig(backend_options)
        self.hand_cfg = {side: validate_config(backend_options['arms'][side]['end_effector']) for side in SIDES}
        self.lock = threading.RLock()
        self._build()

    # ---- construction ---------------------------------------------------------------------------------------------------

    def _build(self):
        sim = self.sim
        self.scene = Scene(sim['scene'], seed=int(sim['seed']), randomize=sim['randomize'],
                           table_z=float(sim['scene_options']['table_z']))
        self.spec = M.build_spec(sim, self.rig, self.scene.adders())
        self.model = self.spec.compile()
        self.data = mujoco.MjData(self.model)
        self.ix = M.Indices(self.model)
        self.dt = float(self.model.opt.timestep)
        hand = sim['hand']
        self.kt, self.firmware, self.kin, self.arm = {}, {}, {}, {}
        for side in SIDES:
            kt = M.hand_torque_constants(self.model, side, hand['torque_per_amp_scale'], hand['torque_per_amp'])
            self.kt[side] = kt
            jnt, dof = self.ix.hand_jnt[side], self.ix.hand_dof[side]
            peak = kt * float(hand['max_current_a'])
            self.model.jnt_actfrclimited[jnt] = 1
            self.model.jnt_actfrcrange[jnt, 0], self.model.jnt_actfrcrange[jnt, 1] = -peak, peak
            self.model.dof_damping[dof] = kt * float(hand['viscous_friction_a_s_per_rad'])
            # Dry friction must stick: MuJoCo's default soft friction lets light finger
            # links creep at ~0.4 rad/s under a sub-static load. Stiff settings: ~0.003 rad/s.
            self.model.dof_solref[dof] = M.STIFF_FRICTION_SOLREF
            self.model.dof_solimp[dof] = M.STIFF_FRICTION_SOLIMP
            cfg = self.hand_cfg[side]
            self.firmware[side] = HandFirmware(side, cfg['serial'], side, hand, kt,
                                               sim['hands'][side]['faulted_joints'])
            self.kin[side] = ArmKinematics(sim['arm'], self.rig.tcp_offset[side])
            serial = sim['arms'][side].get('serial') or self.rig.arm_serial[side]
            self.arm[side] = XArmController(side, serial, self.kin[side], sim['arm'], self.rig.tcp_offset[side],
                                            self.initial_arm(side))
        self.servo_lead = {side: np.asarray(sim['arm']['kv']) / np.asarray(sim['arm']['kp']) for side in SIDES}
        self.hand_mass = {side: float(self.model.body_subtreemass[self.ix.hand_bodies[side][0]]) for side in SIDES}
        self._jac = np.zeros((3, self.model.nv))
        self.noise = np.random.default_rng(int(sim['seed']) + 7919)
        self.steps = 0
        self.scene.bind(self.model, self.data)
        self.reset()

    def initial_arm(self, side):
        value = self.sim['arms'][side].get('initial_joints') or self.options['arms'][side].get('home_joints') or HOME_Q
        return np.asarray(value, dtype=float)

    def initial_hand(self, side):
        name = self.sim['hands'][side]['initial_posture']
        postures = self.hand_cfg[side]['postures']
        if name in postures:
            return np.asarray(postures[name], dtype=float)
        return np.clip(np.zeros(20), self.hand_cfg[side]['lower_rad'], self.hand_cfg[side]['upper_rad'])

    def rebuild(self, *, scene=None, seed=None, randomize=None):
        """A different scene needs a new model (objects are compiled in)."""
        with self.lock:
            if scene is not None:
                self.sim['scene'] = scene
            if seed is not None:
                self.sim['seed'] = int(seed)
            if randomize is not None:
                self.sim['randomize'] = bool(randomize)
            self._build()

    def reset(self, *, seed=None, power_cycle=True):
        """Objects to a new draw; with power_cycle, the arms at their initial joints and the
        hands disabled at their initial posture (both devices as at power-on)."""
        with self.lock:
            m, d = self.model, self.data
            if power_cycle:
                mujoco.mj_resetData(m, d)
                for side in SIDES:
                    q = self.initial_arm(side)
                    d.qpos[self.ix.arm_qpos[side]] = q
                    d.ctrl[self.ix.arm_act[side]] = q
                    d.qpos[self.ix.hand_qpos[side]] = self.initial_hand(side)
                    self.firmware[side].power_on()
                    self.arm[side].power_on(q)
            self.scene.reset(m, d, seed)
            # Masses changed: recompute the model's constants. mj_setConst works at qpos0
            # in `d`, so keep the state placed above.
            saved = [a.copy() for a in (d.qpos, d.qvel, d.mocap_pos, d.mocap_quat, d.ctrl, d.qfrc_applied)]
            mujoco.mj_setConst(m, d)
            for target, value in zip((d.qpos, d.qvel, d.mocap_pos, d.mocap_quat, d.ctrl, d.qfrc_applied), saved):
                target[:] = value
            d.time = 0.0 if power_cycle else d.time
            mujoco.mj_forward(m, d)
            for side in SIDES:
                self.arm[side].measured = d.qpos[self.ix.arm_qpos[side]].copy()

    # ---- stepping -------------------------------------------------------------------------------------------------

    def step(self, n=1):
        m, d, ix, hand = self.model, self.data, self.ix, self.sim['hand']
        static, coulomb, v_s = (float(hand['static_friction_a']), float(hand['coulomb_friction_a']),
                                float(hand['stribeck_velocity_rad_s']))
        dt = self.dt
        with self.lock:
            for _ in range(n):
                t = float(d.time)
                for side in SIDES:
                    q = d.qpos[ix.hand_qpos[side]]
                    dq = d.qvel[ix.hand_dof[side]]
                    self.firmware[side].control(t, m, d, ix.hand_act[side], q)
                    m.dof_frictionloss[ix.hand_dof[side]] = self.kt[side] * stribeck(dq, static, coulomb, v_s)
                    q_ref, dq_ref = self.arm[side].tick(t, d.qpos[ix.arm_qpos[side]], d.actuator_force[ix.arm_act[side]], dt)
                    d.ctrl[ix.arm_act[side]] = q_ref + self.servo_lead[side] * dq_ref
                    if self.steps % 4 == 0:
                        d.qfrc_applied[ix.arm_dof[side]] = self._payload(side)
                self.scene.pre_step(m, d, dt)
                mujoco.mj_step(m, d)
                for side in SIDES:
                    self.firmware[side].after_step(d, ix.hand_act[side], d.qvel[ix.hand_dof[side]], dt)
                self.steps += 1

    def _payload(self, side):
        """The arm controller's TCP-load compensation: the hand's weight on the arm joints."""
        mujoco.mj_jacSubtreeCom(self.model, self.data, self._jac, self.ix.hand_bodies[side][0])
        force = self.hand_mass[side] * self.model.opt.gravity
        return -(self._jac[:, self.ix.arm_dof[side]].T @ force)

    @property
    def time(self):
        return float(self.data.time)

    # ---- measurements (what the devices report) ---------------------------------------------------------------

    def hand_measurement(self, side):
        """(q, dq, current A) as the hand's joint_states stream reports them."""
        d, ix, hand = self.data, self.ix, self.sim['hand']
        q = d.qpos[ix.hand_qpos[side]].copy()
        dq = d.qvel[ix.hand_dof[side]].copy()
        if hand['position_noise_rad']:
            q += self.noise.normal(0.0, float(hand['position_noise_rad']), 20)
        if hand['velocity_noise_rad_s']:
            dq += self.noise.normal(0.0, float(hand['velocity_noise_rad_s']), 20)
        return q, dq, self.firmware[side].current.copy()

    # ---- privileged state (simulation only) -------------------------------------------------------------------

    def ground_truth(self):
        """PRIVILEGED: objects, threads, fingertips, palms and TCPs in the left base frame."""
        with self.lock:
            d, ix = self.data, self.ix
            out = self.scene.state(self.model, d)
            out['robot'] = {side: {
                'tcp_position': np.round(d.site_xpos[ix.tcp_site[side]], 5).tolist(),
                'palm_position': np.round(d.xpos[ix.palm[side]], 5).tolist(),
                'fingertips': {JOINT_NAMES[4 * k].split('_')[0]: np.round(d.site_xpos[s], 5).tolist()
                               for k, s in enumerate(ix.tips[side])},
                'hand_q': np.round(d.qpos[ix.hand_qpos[side]], 5).tolist(),
                'arm_q': np.round(d.qpos[ix.arm_qpos[side]], 5).tolist()} for side in SIDES}
            out['contacts'] = self.hand_contacts()
            return out

    def hand_contacts(self):
        """PRIVILEGED: which hand bodies touch which objects, with normal force (N)."""
        m, d = self.model, self.data
        hand_body = {b: side for side in SIDES for b in self.ix.hand_bodies[side]}
        out = []
        force = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            for hb, other in ((b1, b2), (b2, b1)):
                if hb in hand_body and other not in hand_body:
                    mujoco.mj_contactForce(m, d, i, force)
                    out.append({'hand': hand_body[hb], 'link': m.body(hb).name,
                                'other': m.body(other).name or 'world', 'normal_n': round(float(force[0]), 3)})
        return out

    def set_object_pose(self, name, position, quaternion_xyzw=None):
        """PRIVILEGED test hook: place a free object (by scene name)."""
        with self.lock:
            for o in self.scene.objects:
                if getattr(o, 'name_object', o.name) == name and hasattr(o, 'qadr'):
                    self.data.qpos[o.qadr:o.qadr + 3] = position
                    if quaternion_xyzw is not None:
                        x, y, z, w = quaternion_xyzw
                        self.data.qpos[o.qadr + 3:o.qadr + 7] = [w, x, y, z]
                    self.data.qvel[o.vadr:o.vadr + 6] = 0.0
                    mujoco.mj_forward(self.model, self.data)
                    return
            raise KeyError(f'no free object {name!r} in scene {self.scene.name}')

    def render_state(self):
        """What a camera needs to draw the scene now."""
        with self.lock:
            return {'time': self.time, 'qpos': self.data.qpos.tolist(), 'mocap_pos': self.data.mocap_pos.tolist(),
                    'mocap_quat': self.data.mocap_quat.tolist(), 'geom_rgba': self.model.geom_rgba.tolist(),
                    'mat_emission': self.model.mat_emission.tolist(), 'mat_rgba': self.model.mat_rgba.tolist(),
                    'body_quat': self.model.body_quat.tolist()}
