"""Task objects and scenes, selectable by name, with randomized pose, mass and friction.

Objects (all poses in the left base frame, metres, on the table top):
- `cube`: 5 cm cube;            `ball`: 6 cm ball;
- `bottle` + screw cap: a 500 ml bottle; the cap turns about the neck and rises by the
  thread's lead (a MuJoCo joint equality couples lift to turn). Turning
  counter-clockwise seen from above (+ about z) unscrews. Seated at turn 0 (the turn
  joint's lower limit); after `release_turns` the cap leaves the thread and becomes a
  free body. A free cap resting on the neck aligned (within 2.5 mm and 8 deg) and turned
  0.2 rad clockwise catches the thread again;
- `socket` + `bulb`: an E27 lamp holder fixed to the table and an A60 bulb, threaded the
  same way (3.629 mm pitch). A seated bulb lights up (its glass glows), visible to the
  cameras;
- household: `mug`, `marker`, `small_box`.

Thread friction: running torque (Coulomb, the turn joint's frictionloss) and a
breakaway torque until the part has turned `breakaway_rad` away from seated
(ASSUMED: a cap or bulb tightened by hand; a new bottle's tamper band is not modelled).

Every object's ground truth (`Scene.state`) is PRIVILEGED: for simulation checks,
scoring and debriefs, never a skill's input on the real robot.
"""
from __future__ import annotations

from copy import deepcopy
import math

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

VISUAL, COLLISION = 1, 3
THREAD_CHANNEL = 4   # contype bit shared by the invisible thread-start collars and free threaded parts
PARK = np.array([-2.0, 0.0, 3.0])  # hidden parts float here (gravity compensated), clear of every plane
TURN_ARMATURE = 5e-5
STIFF_SOLREF = (0.002, 1.0)                   # 2 physics steps: the stiffest stable time constant
STIFF_SOLIMP = (0.99, 0.999, 0.001, 0.5, 2.0)


def _quat_yaw(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def _ring(body, name, r_in, r_out, z_lo, z_hi, n, **geom):
    """A hollow cylinder of n boxes (collision and rendering)."""
    radius, half_t = (r_in + r_out) / 2, (r_out - r_in) / 2
    half_w = radius * math.tan(math.pi / n) * 1.02
    for k in range(n):
        a = 2 * math.pi * k / n
        body.add_geom(name=f'{name}_{k}', type=mujoco.mjtGeom.mjGEOM_BOX, size=[half_t, half_w, (z_hi - z_lo) / 2],
                      pos=[radius * math.cos(a), radius * math.sin(a), (z_lo + z_hi) / 2],
                      quat=_quat_yaw(a), **geom)


class SceneObject:
    """A free rigid object on the table."""
    kind = 'object'

    def __init__(self, name, pose, *, mass, friction, pose_range=(0.0, 0.0, 0.0), mass_range=(0.8, 1.25),
                 friction_range=None, rgba=(0.8, 0.2, 0.2, 1.0)):
        self.name, self.nominal = name, tuple(pose)
        self.mass, self.friction = float(mass), float(friction)
        self.pose_range, self.mass_range = tuple(pose_range), tuple(mass_range)
        self.friction_range = tuple(friction_range or (0.7 * friction, 1.2 * friction))
        self.rgba = list(rgba)
        self.body_name = f'obj_{name}'
        self.draw = {}

    # geometry -------------------------------------------------------------------------------------------------------
    def rest_height(self):
        raise NotImplementedError

    def geoms(self, body):
        raise NotImplementedError

    def add(self, spec, table_z):
        self.table_z = table_z
        body = spec.worldbody.add_body(name=self.body_name, pos=[self.nominal[0], self.nominal[1],
                                                                  table_z + self.rest_height()])
        body.add_freejoint(name=f'{self.body_name}_free')
        self.geoms(body)
        return body

    # runtime --------------------------------------------------------------------------------------------------------
    def bind(self, model, data):
        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.body_name)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{self.body_name}_free')
        self.qadr, self.vadr = int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])
        self.geom_ids = [g for g in range(model.ngeom) if model.geom_bodyid[g] == self.body]
        self.nominal_mass = float(model.body_mass[self.body])
        self.nominal_inertia = model.body_inertia[self.body].copy()

    def randomize(self, model, rng, randomize):
        mass = self.mass * (rng.uniform(*self.mass_range) if randomize else 1.0)
        friction = rng.uniform(*self.friction_range) if randomize else self.friction
        scale = mass / self.nominal_mass
        model.body_mass[self.body] = mass
        model.body_inertia[self.body] = self.nominal_inertia * scale
        model.geom_friction[self.geom_ids, 0] = friction
        self.draw = {'mass_kg': round(mass, 4), 'friction': round(friction, 3)}

    def place(self, model, data, rng, randomize):
        x, y, yaw = self.nominal
        if randomize:
            dx, dy, dyaw = self.pose_range
            x, y, yaw = x + rng.uniform(-dx, dx), y + rng.uniform(-dy, dy), yaw + rng.uniform(-dyaw, dyaw)
        data.qpos[self.qadr:self.qadr + 3] = [x, y, self.table_z + self.rest_height() + 0.0005]
        data.qpos[self.qadr + 3:self.qadr + 7] = _quat_yaw(yaw)
        data.qvel[self.vadr:self.vadr + 6] = 0.0
        self.draw.update(pose_xy_yaw=[round(x, 4), round(y, 4), round(yaw, 4)])

    def reset(self, model, data, rng, randomize):
        self.randomize(model, rng, randomize)
        self.place(model, data, rng, randomize)

    def pre_step(self, model, data, dt):
        pass

    def state(self, model, data):
        return _body_state(model, data, self.body)


def _body_state(model, data, body):
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
    w, x, y, z = data.xquat[body]
    return {'position': np.round(data.xpos[body], 5).tolist(), 'quaternion_xyzw': np.round([x, y, z, w], 6).tolist(),
            'linear_velocity': np.round(velocity[3:], 5).tolist(),
            'angular_velocity': np.round(velocity[:3], 5).tolist()}


class Cube(SceneObject):
    def __init__(self, pose=(0.50, -0.20, 0.0), *, size=0.05, mass=0.10, friction=0.8, **kw):
        super().__init__(kw.pop('name', 'cube'), pose, mass=mass, friction=friction,
                         pose_range=kw.pop('pose_range', (0.05, 0.05, math.pi / 4)),
                         rgba=kw.pop('rgba', (0.85, 0.2, 0.15, 1)), **kw)
        self.size = size

    def rest_height(self):
        return self.size / 2

    def geoms(self, body):
        body.add_geom(name=f'{self.body_name}_box', type=mujoco.mjtGeom.mjGEOM_BOX, size=[self.size / 2]*3,
                      rgba=self.rgba, friction=[self.friction, 0.005, 0.0001], condim=4)


class Ball(SceneObject):
    def __init__(self, pose=(0.50, -0.20, 0.0), *, diameter=0.06, mass=0.06, friction=0.7, **kw):
        super().__init__(kw.pop('name', 'ball'), pose, mass=mass, friction=friction,
                         pose_range=kw.pop('pose_range', (0.04, 0.04, math.pi)),
                         rgba=kw.pop('rgba', (0.15, 0.4, 0.9, 1)), **kw)
        self.radius = diameter / 2

    def rest_height(self):
        return self.radius

    def geoms(self, body):
        # condim 6: rolling friction keeps a ball from rolling forever on the table.
        body.add_geom(name=f'{self.body_name}_sphere', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[self.radius, 0, 0],
                      rgba=self.rgba, friction=[self.friction, 0.005, 0.0002], condim=6)


class Marker(SceneObject):
    def __init__(self, pose=(0.42, -0.42, 0.3), *, mass=0.015, friction=0.6, **kw):
        super().__init__(kw.pop('name', 'marker'), pose, mass=mass, friction=friction,
                         pose_range=kw.pop('pose_range', (0.04, 0.04, math.pi / 3)),
                         rgba=kw.pop('rgba', (0.1, 0.1, 0.1, 1)), **kw)
        self.radius, self.half_length = 0.0075, 0.06

    def rest_height(self):
        return self.radius

    def place(self, model, data, rng, randomize):
        super().place(model, data, rng, randomize)
        yaw = self.draw['pose_xy_yaw'][2]
        lying = Rotation.from_euler('z', yaw) * Rotation.from_euler('y', math.pi / 2)
        x, y, z, w = lying.as_quat()
        data.qpos[self.qadr + 3:self.qadr + 7] = [w, x, y, z]

    def geoms(self, body):
        body.add_geom(name=f'{self.body_name}_barrel', type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                      size=[self.radius, self.half_length * 0.8, 0], pos=[0, 0, -self.half_length * 0.2],
                      rgba=[0.92, 0.92, 0.92, 1], friction=[self.friction, 0.005, 0.0001], condim=4)
        body.add_geom(name=f'{self.body_name}_cap', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.radius * 1.08, self.half_length * 0.2, 0], pos=[0, 0, self.half_length * 0.8],
                      rgba=self.rgba, friction=[self.friction, 0.005, 0.0001], condim=4)


class SmallBox(SceneObject):
    def __init__(self, pose=(0.55, -0.40, 0.2), *, size=(0.08, 0.05, 0.03), mass=0.08, friction=0.6, **kw):
        super().__init__(kw.pop('name', 'small_box'), pose, mass=mass, friction=friction,
                         pose_range=kw.pop('pose_range', (0.04, 0.04, math.pi / 4)),
                         rgba=kw.pop('rgba', (0.9, 0.75, 0.3, 1)), **kw)
        self.size = size

    def rest_height(self):
        return self.size[2] / 2

    def geoms(self, body):
        body.add_geom(name=f'{self.body_name}_box', type=mujoco.mjtGeom.mjGEOM_BOX,
                      size=[s / 2 for s in self.size], rgba=self.rgba, friction=[self.friction, 0.005, 0.0001],
                      condim=4)


class Mug(SceneObject):
    def __init__(self, pose=(0.40, -0.25, -1.2), *, mass=0.30, friction=0.6, **kw):
        super().__init__(kw.pop('name', 'mug'), pose, mass=mass, friction=friction,
                         pose_range=kw.pop('pose_range', (0.04, 0.04, math.pi / 4)),
                         rgba=kw.pop('rgba', (0.2, 0.55, 0.45, 1)), **kw)
        self.r_out, self.wall, self.height = 0.041, 0.005, 0.095

    def rest_height(self):
        return 0.0

    def geoms(self, body):
        common = dict(rgba=self.rgba, friction=[self.friction, 0.005, 0.0001], condim=4)
        body.add_geom(name=f'{self.body_name}_bottom', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size=[self.r_out, 0.004, 0], pos=[0, 0, 0.004], **common)
        _ring(body, f'{self.body_name}_wall', self.r_out - self.wall, self.r_out, 0.0, self.height, 16, **common)
        for k, (x, z) in enumerate(((self.r_out + 0.012, 0.03), (self.r_out + 0.022, 0.0475), (self.r_out + 0.012, 0.065))):
            body.add_geom(name=f'{self.body_name}_handle{k}', type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                          size=[0.005, 0.011, 0], pos=[x, 0, z],
                          quat=[1, 0, 0, 0] if k == 1 else ([0.924, 0, 0.383, 0] if k == 0 else [0.924, 0, -0.383, 0]),
                          **common)


class ThreadedFixture:
    """A threaded part on its holder: a bottle's cap, or a bulb in its socket.

    The holder carries the threaded part as a child body on a slide (`lift`) and a hinge
    (`turn`) tied by a joint equality lift = lead / (2 pi) * turn; the part's free twin
    (parked out of the scene until released) carries it off the thread."""
    kind = 'threaded'

    def __init__(self, name, *, lead_m, release_turns, running_nm=(0.03, 0.08), breakaway_factor=(1.2, 2.0),
                 breakaway_rad=0.2, seat_tolerance_rad=0.03, engage_turn_rad=0.2, radial_tolerance_m=0.0025,
                 tilt_tolerance_rad=math.radians(8), start='seated', start_turn_range=(0.0, 0.25)):
        self.name = name
        self.lead = float(lead_m)
        self.release_rad = 2 * math.pi * float(release_turns)
        self.lift_release = self.lead * self.release_rad / (2 * math.pi)
        self.running_range, self.factor_range = tuple(running_nm), tuple(breakaway_factor)
        self.breakaway_rad, self.seat_tol, self.engage_rad = breakaway_rad, seat_tolerance_rad, engage_turn_rad
        self.radial_tol, self.tilt_tol = radial_tolerance_m, tilt_tolerance_rad
        self.start, self.start_turn_range = start, tuple(start_turn_range)
        self.draw = {}
        self.events = []

    # names
    @property
    def threaded_body(self):
        return f'obj_{self.name}_threaded'

    @property
    def free_body(self):
        return f'obj_{self.name}_free'

    def add_thread(self, spec, holder, part_geoms, part_mass, collar_z, collar_radius):
        """Under `holder` (an MjsBody): the threaded part (seated pose at the holder's
        origin + thread frame), its equality, and the invisible thread-start collar."""
        part = holder.add_body(name=self.threaded_body, pos=self.thread_origin)
        part.add_joint(name=f'{self.threaded_body}_lift', type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[0, 0, 1],
                       range=[-0.0005, self.lift_release + 0.002])
        # Numerics: a few-gram part has ~1e-6 kg m2 about its axis; MuJoCo's soft
        # constraints then let it creep through its friction and its thread equality.
        # TURN_ARMATURE (5e-5 kg m2, ~1e-3 N m at 20 rad/s2: negligible next to the thread's
        # friction) and stiff solver settings keep creep below ~0.01 rad/s and the thread
        # within 0.03 mm (measured, local_tests/unit/test_sim_xarm_wuji.py).
        part.add_joint(name=f'{self.threaded_body}_turn', type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1],
                       range=[0.0, self.release_rad + 0.05], armature=TURN_ARMATURE,
                       solref_friction=list(STIFF_SOLREF), solimp_friction=list(STIFF_SOLIMP))
        part_geoms(part, f'{self.threaded_body}', contype=1, conaffinity=1)
        self.part_mass = part_mass
        # The thread: lift = lead / (2 pi) * turn (MuJoCo joint equality, polynomial in turn).
        spec.add_equality(type=mujoco.mjtEq.mjEQ_JOINT, name=f'{self.threaded_body}_thread',
                          name1=f'{self.threaded_body}_lift', name2=f'{self.threaded_body}_turn',
                          objtype=mujoco.mjtObj.mjOBJ_JOINT,
                          data=[0.0, self.lead / (2 * math.pi)] + [0.0] * 10,
                          solref=list(STIFF_SOLREF), solimp=list(STIFF_SOLIMP))
        holder.add_geom(name=f'obj_{self.name}_collar', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        size=[collar_radius, 0.0008, 0],
                        pos=[self.thread_origin[0], self.thread_origin[1], self.thread_origin[2] + collar_z - 0.0008],
                        contype=0, conaffinity=THREAD_CHANNEL, group=COLLISION, rgba=[0, 1, 0, 0.3])
        return part

    def add_free(self, spec, part_geoms, part_mass):
        free = spec.worldbody.add_body(name=self.free_body, pos=PARK.tolist())
        free.add_freejoint(name=f'{self.free_body}_free')
        part_geoms(free, self.free_body, contype=1 | THREAD_CHANNEL, conaffinity=1)
        return free

    def bind_thread(self, model, data, holder_body):
        J, B = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_BODY
        self.holder = holder_body
        self.part = mujoco.mj_name2id(model, B, self.threaded_body)
        self.free = mujoco.mj_name2id(model, B, self.free_body)
        lift = mujoco.mj_name2id(model, J, f'{self.threaded_body}_lift')
        turn = mujoco.mj_name2id(model, J, f'{self.threaded_body}_turn')
        free = mujoco.mj_name2id(model, J, f'{self.free_body}_free')
        self.lift_q, self.lift_v = int(model.jnt_qposadr[lift]), int(model.jnt_dofadr[lift])
        self.turn_q, self.turn_v = int(model.jnt_qposadr[turn]), int(model.jnt_dofadr[turn])
        self.free_q, self.free_v = int(model.jnt_qposadr[free]), int(model.jnt_dofadr[free])
        self.part_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == self.part]
        self.free_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == self.free]
        self.part_flags = {g: (int(model.geom_contype[g]), int(model.geom_conaffinity[g]), float(model.geom_rgba[g, 3]))
                           for g in self.part_geoms}
        self.free_flags = {g: (int(model.geom_contype[g]), int(model.geom_conaffinity[g]), float(model.geom_rgba[g, 3]))
                           for g in self.free_geoms}
        self.part_quat0 = model.body_quat[self.part].copy()
        self.part_inertia0 = {b: (float(model.body_mass[b]), model.body_inertia[b].copy()) for b in (self.part, self.free)}
        self.set_part_mass(model, self.part_mass)
        eq = [e for e in range(model.neq) if model.eq_type[e] == mujoco.mjtEq.mjEQ_JOINT
              and model.eq_obj1id[e] == lift]
        if len(eq) != 1:
            raise RuntimeError(f'{self.name}: the thread equality is missing')
        self.equality = eq[0]

    def set_part_mass(self, model, mass):
        for body, (mass0, inertia0) in self.part_inertia0.items():
            model.body_mass[body] = mass
            model.body_inertia[body] = inertia0 * (mass / mass0)

    # state transitions --------------------------------------------------------------------------------------------
    def _show(self, model, flags, visible):
        bodies = set()
        for g, (contype, conaffinity, alpha) in flags.items():
            model.geom_contype[g] = contype if visible else 0
            model.geom_conaffinity[g] = conaffinity if visible else 0
            model.geom_rgba[g, 3] = alpha if visible else 0.0
            bodies.add(int(model.geom_bodyid[g]))
        # The broad phase filters on per-body masks, which mj_setConst recomputes from the
        # geoms: keep them in step, or a shown part would collide with nothing.
        for body in bodies:
            geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == body]
            model.body_contype[body] = int(np.bitwise_or.reduce(model.geom_contype[geoms]))
            model.body_conaffinity[body] = int(np.bitwise_or.reduce(model.geom_conaffinity[geoms]))

    def _set_turn(self, data, turn, velocity=0.0):
        data.qpos[self.turn_q] = turn
        data.qpos[self.lift_q] = self.lead * turn / (2 * math.pi)
        data.qvel[self.turn_v] = velocity
        data.qvel[self.lift_v] = self.lead * velocity / (2 * math.pi)

    def _park(self, model, data):
        self._show(model, self.free_flags, False)
        model.body_gravcomp[self.free] = 1.0
        data.qpos[self.free_q:self.free_q + 3] = PARK
        data.qpos[self.free_q + 3:self.free_q + 7] = [1, 0, 0, 0]
        data.qvel[self.free_v:self.free_v + 6] = 0.0

    def thread_reset(self, model, data, rng, randomize):
        self.events = []
        self.running_nm = rng.uniform(*self.running_range) if randomize else float(np.mean(self.running_range))
        self.breakaway_nm = self.running_nm * (rng.uniform(*self.factor_range) if randomize
                                               else float(np.mean(self.factor_range)))
        self.accumulated = 0.0
        model.body_quat[self.part] = self.part_quat0
        if self.start == 'free':
            self.threaded = False
            self._show(model, self.part_flags, False)
            self._set_turn(data, self.release_rad)
            model.body_gravcomp[self.free] = 0.0
            self._show(model, self.free_flags, True)
        else:
            self.threaded = True
            self._show(model, self.part_flags, True)
            turn = rng.uniform(*self.start_turn_range) if (randomize and self.start == 'loose') else 0.0
            self._set_turn(data, turn)
            self._park(model, data)
        self.broken = not self.seated(data)
        self.was_seated = self.seated(data)
        self.draw.update(running_nm=round(self.running_nm, 4), breakaway_nm=round(self.breakaway_nm, 4),
                         start=self.start)

    def seated(self, data):
        return self.threaded and float(data.qpos[self.turn_q]) <= self.seat_tol

    def thread_step(self, model, data, dt):
        if self.threaded:
            turn = float(data.qpos[self.turn_q])
            seated = turn <= self.seat_tol
            if seated:
                self.broken = False  # a tightened part needs the breakaway torque again
                if not self.was_seated:
                    self.events.append({'event': 'seated', 'time': round(float(data.time), 4)})
            elif turn > self.breakaway_rad:
                self.broken = True
            self.was_seated = seated
            model.dof_frictionloss[self.turn_v] = self.running_nm if self.broken else self.breakaway_nm
            if turn >= self.release_rad:
                self._release(model, data)
            self.on_seated(model, seated)
            return
        self.on_seated(model, False)
        self._set_turn(data, self.release_rad)  # the hidden threaded twin stays at the thread's end
        self._try_engage(model, data, dt)

    def on_seated(self, model, seated):
        pass

    def _release(self, model, data):
        mujoco.mj_kinematics(model, data)
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, self.part, velocity, 0)
        R = data.xmat[self.part].reshape(3, 3)
        data.qpos[self.free_q:self.free_q + 3] = data.xpos[self.part]
        data.qpos[self.free_q + 3:self.free_q + 7] = data.xquat[self.part]
        data.qvel[self.free_v:self.free_v + 3] = velocity[3:]
        data.qvel[self.free_v + 3:self.free_v + 6] = R.T @ velocity[:3]
        model.body_gravcomp[self.free] = 0.0
        self._show(model, self.free_flags, True)
        self._show(model, self.part_flags, False)
        self._set_turn(data, self.release_rad)
        model.dof_frictionloss[self.turn_v] = 0.0
        self.threaded = False
        self.accumulated = 0.0
        self.events.append({'event': 'removed', 'time': round(float(data.time), 4)})

    def _thread_frame(self, data):
        """World pose of the thread frame (the threaded part's seated origin, z along the axis)."""
        R_h = data.xmat[self.holder].reshape(3, 3)
        origin = data.xpos[self.holder] + R_h @ np.asarray(self.thread_origin)
        return origin, R_h

    def _try_engage(self, model, data, dt):
        origin, R_h = self._thread_frame(data)
        p = data.xpos[self.free] - origin
        local = R_h.T @ p
        R_f = data.xmat[self.free].reshape(3, 3)
        tilt = math.acos(max(-1.0, min(1.0, float(R_h[:, 2] @ R_f[:, 2]))))
        aligned = (math.hypot(local[0], local[1]) <= self.radial_tol and tilt <= self.tilt_tol
                   and abs(local[2] - self.lift_release) <= 0.0025)
        if not aligned:
            self.accumulated = 0.0
            return
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, self.free, velocity, 0)
        holder_velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, self.holder, holder_velocity, 0)
        spin = float(R_h[:, 2] @ (velocity[:3] - holder_velocity[:3]))
        self.accumulated = min(0.0, self.accumulated + spin * dt)
        if self.accumulated <= -self.engage_rad:
            self._engage(model, data, R_h, R_f, spin)

    def _engage(self, model, data, R_h, R_f, spin):
        turn = self.release_rad - 0.02
        # Keep the part where it is: a yaw offset in the threaded body's frame.
        relative = Rotation.from_matrix(R_h.T @ R_f).as_euler('zyx')[0]
        offset = relative - turn
        w0 = self.part_quat0
        q = Rotation.from_euler('z', offset) * Rotation.from_quat([w0[1], w0[2], w0[3], w0[0]])
        x, y, z, w = q.as_quat()
        model.body_quat[self.part] = [w, x, y, z]
        self._set_turn(data, turn, min(spin, 0.0))
        self._show(model, self.part_flags, True)
        self._park(model, data)
        self.threaded = True
        self.broken = True
        self.accumulated = 0.0
        self.events.append({'event': 'engaged', 'time': round(float(data.time), 4)})

    def thread_state(self, model, data):
        turn = float(data.qpos[self.turn_q])
        part = self.part if self.threaded else self.free
        return {'threaded': self.threaded, 'seated': self.seated(data), 'removed': not self.threaded,
                'turn_rad': round(turn, 4) if self.threaded else None,
                'turns_from_seated': round(turn / (2 * math.pi), 4) if self.threaded else None,
                'lift_m': round(float(data.qpos[self.lift_q]), 5) if self.threaded else None,
                'release_turns': round(self.release_rad / (2 * math.pi), 3), 'lead_m': self.lead,
                'friction_nm': round(float(model.dof_frictionloss[self.turn_v]), 4),
                'running_nm': round(self.running_nm, 4), 'breakaway_nm': round(self.breakaway_nm, 4),
                'events': list(self.events), 'part': _body_state(model, data, part)}


class Bottle(ThreadedFixture):
    """500 ml PET bottle (66 mm across, 0.2-0.45 kg with water) with a 28 mm screw cap,
    3 mm lead, 1.5 turns of thread (ASSUMED, a common 28 mm finish)."""

    def __init__(self, pose=(0.50, -0.314, 0.0), *, start='seated', mass=(0.20, 0.45), friction=0.6, **kw):
        super().__init__('bottle_cap', lead_m=0.003, release_turns=1.5, start=start,
                         running_nm=kw.pop('running_nm', (0.03, 0.08)))
        self.nominal, self.mass_range, self.friction = tuple(pose), tuple(mass), friction
        self.pose_range = kw.pop('pose_range', (0.03, 0.03, math.pi))
        self.free_pose = kw.pop('free_pose', (0.42, -0.20, 0.0))
        self.r_body, self.h_body, self.r_neck, self.h_neck = 0.033, 0.175, 0.0125, 0.022
        self.thread_origin = (0.0, 0.0, self.h_body + self.h_neck)
        self.cap_mass = 0.003
        self.body_name = 'obj_bottle'
        self.name_object = 'bottle'

    def _cap(self, body, prefix, contype, conaffinity):
        common = dict(rgba=[0.95, 0.95, 0.95, 1], friction=[0.9, 0.005, 0.0001], condim=4,
                      contype=contype, conaffinity=conaffinity)
        body.add_geom(name=f'{prefix}_top', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.0165, 0.0015, 0],
                      pos=[0, 0, 0.0015], **common)
        _ring(body, f'{prefix}_skirt', 0.0145, 0.0165, -0.015, 0.003, 12, **common)

    def add(self, spec, table_z):
        self.table_z = table_z
        bottle = spec.worldbody.add_body(name=self.body_name, pos=[self.nominal[0], self.nominal[1], table_z + 0.0005])
        bottle.add_freejoint(name=f'{self.body_name}_free')
        common = dict(rgba=[0.55, 0.75, 0.9, 0.85], friction=[self.friction, 0.005, 0.0001], condim=4)
        bottle.add_geom(name='obj_bottle_body', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        size=[self.r_body, (self.h_body - 0.03) / 2, 0], pos=[0, 0, (self.h_body - 0.03) / 2], **common)
        bottle.add_geom(name='obj_bottle_shoulder', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        size=[0.022, 0.015, 0], pos=[0, 0, self.h_body - 0.015], **common)
        bottle.add_geom(name='obj_bottle_neck', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        size=[self.r_neck, self.h_neck / 2, 0], pos=[0, 0, self.h_body + self.h_neck / 2], **common)
        bottle.add_geom(name='obj_bottle_neck_ring', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                        size=[0.0175, 0.001, 0], pos=[0, 0, self.h_body + 0.002], **common)
        # The free cap rests on its thread start: skirt bottom (-0.015) at the release lift.
        self.add_thread(spec, bottle, self._cap, self.cap_mass, collar_z=-0.015 + self.lift_release, collar_radius=0.0168)
        self.add_free(spec, self._cap, self.cap_mass)

    def bind(self, model, data):
        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.body_name)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{self.body_name}_free')
        self.qadr, self.vadr = int(model.jnt_qposadr[joint]), int(model.jnt_dofadr[joint])
        self.geom_ids = [g for g in range(model.ngeom) if model.geom_bodyid[g] == self.body]
        self.nominal_mass = float(model.body_mass[self.body])
        self.nominal_inertia = model.body_inertia[self.body].copy()
        self.bind_thread(model, data, self.body)

    def reset(self, model, data, rng, randomize):
        mass = rng.uniform(*self.mass_range) if randomize else float(np.mean(self.mass_range))
        model.body_mass[self.body] = mass
        model.body_inertia[self.body] = self.nominal_inertia * (mass / self.nominal_mass)
        friction = rng.uniform(0.7 * self.friction, 1.2 * self.friction) if randomize else self.friction
        model.geom_friction[self.geom_ids, 0] = friction
        x, y, yaw = self.nominal
        if randomize:
            dx, dy, dyaw = self.pose_range
            x, y, yaw = x + rng.uniform(-dx, dx), y + rng.uniform(-dy, dy), rng.uniform(-dyaw, dyaw)
        data.qpos[self.qadr:self.qadr + 3] = [x, y, self.table_z + 0.0005]
        data.qpos[self.qadr + 3:self.qadr + 7] = _quat_yaw(yaw)
        data.qvel[self.vadr:self.vadr + 6] = 0.0
        self.draw = {'mass_kg': round(mass, 4), 'friction': round(friction, 3),
                     'pose_xy_yaw': [round(x, 4), round(y, 4), round(yaw, 4)]}
        self.thread_reset(model, data, rng, randomize)
        if self.start == 'free':
            fx, fy, _ = self.free_pose
            data.qpos[self.free_q:self.free_q + 3] = [fx, fy, self.table_z + 0.0185]
            data.qpos[self.free_q + 3:self.free_q + 7] = [1, 0, 0, 0]

    def pre_step(self, model, data, dt):
        self.thread_step(model, data, dt)

    def state(self, model, data):
        return {**_body_state(model, data, self.body), 'cap': self.thread_state(model, data)}


class BulbSocket(ThreadedFixture):
    """An E27 lamp holder fixed to the table (a mocap body) and an A60 bulb (60 mm glass,
    E27 base 27 mm, 3.629 mm pitch, 2.5 turns of engagement, 30-60 g; ASSUMED)."""

    def __init__(self, pose=(0.50, -0.40, 0.0), *, start='free', bulb_pose=(0.42, -0.22, 0.0), **kw):
        super().__init__('bulb', lead_m=0.003629, release_turns=2.5, start=start,
                         running_nm=kw.pop('running_nm', (0.01, 0.04)), breakaway_rad=0.15)
        self.nominal, self.bulb_pose = tuple(pose), tuple(bulb_pose)
        self.pose_range = kw.pop('pose_range', (0.03, 0.03, 0.0))
        self.socket_height = 0.035
        self.thread_origin = (0.0, 0.0, self.socket_height)
        self.mass_range = (0.03, 0.06)
        self.body_name = 'obj_socket'
        self.lit = False

    def _bulb(self, body, prefix, contype, conaffinity):
        metal = dict(rgba=[0.75, 0.75, 0.7, 1], friction=[0.6, 0.005, 0.0001], condim=4,
                     contype=contype, conaffinity=conaffinity)
        glass = dict(rgba=[0.95, 0.95, 0.85, 0.9], friction=[0.5, 0.004, 0.0001], condim=4,
                     contype=contype, conaffinity=conaffinity, material=f'{prefix}_glass')
        body.add_geom(name=f'{prefix}_base', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.0132, 0.0125, 0],
                      pos=[0, 0, 0.0125], **metal)
        body.add_geom(name=f'{prefix}_neck', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.016, 0.009, 0],
                      pos=[0, 0, 0.034], **glass)
        body.add_geom(name=f'{prefix}_glass', type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.03, 0, 0],
                      pos=[0, 0, 0.068], **glass)

    def add(self, spec, table_z):
        self.table_z = table_z
        for prefix in (self.threaded_body, self.free_body):
            spec.add_material(name=f'{prefix}_glass', rgba=[0.95, 0.95, 0.85, 0.9], emission=0.0)
        socket = spec.worldbody.add_body(name=self.body_name, mocap=True,
                                         pos=[self.nominal[0], self.nominal[1], table_z])
        common = dict(rgba=[0.15, 0.15, 0.15, 1], friction=[0.6, 0.005, 0.0001], condim=4)
        socket.add_geom(name='obj_socket_base', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.035, 0.005, 0],
                        pos=[0, 0, 0.005], **common)
        socket.add_geom(name='obj_socket_stem', type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.02, 0.0125, 0],
                        pos=[0, 0, 0.0225], **common)
        _ring(socket, 'obj_socket_sleeve', 0.0148, 0.0185, self.socket_height, self.socket_height + 0.026, 12, **common)
        self.add_thread(spec, socket, self._bulb, 0.045, collar_z=self.lift_release, collar_radius=0.012)
        self.add_free(spec, self._bulb, 0.045)

    def bind(self, model, data):
        self.body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.body_name)
        self.mocap = int(model.body_mocapid[self.body])
        self.bind_thread(model, data, self.body)
        self.glass_mats = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, f'{p}_glass')
                           for p in (self.threaded_body, self.free_body)]

    def reset(self, model, data, rng, randomize):
        x, y, _ = self.nominal
        if randomize:
            x, y = x + rng.uniform(-self.pose_range[0], self.pose_range[0]), y + rng.uniform(-self.pose_range[1], self.pose_range[1])
        data.mocap_pos[self.mocap] = [x, y, self.table_z]
        data.mocap_quat[self.mocap] = [1, 0, 0, 0]
        mass = rng.uniform(*self.mass_range) if randomize else float(np.mean(self.mass_range))
        self.set_part_mass(model, mass)
        self.draw = {'socket_xy': [round(x, 4), round(y, 4)], 'bulb_mass_kg': round(mass, 4)}
        self.thread_reset(model, data, rng, randomize)
        if self.start == 'free':
            bx, by, _ = self.bulb_pose
            data.qpos[self.free_q:self.free_q + 3] = [bx, by, self.table_z + 0.0005]
            data.qpos[self.free_q + 3:self.free_q + 7] = [1, 0, 0, 0]
        self.on_seated(model, self.seated(data))

    def on_seated(self, model, seated):
        if seated != self.lit:
            self.lit = seated
            model.mat_emission[self.glass_mats] = 0.9 if seated else 0.0
            model.mat_rgba[self.glass_mats] = [1.0, 0.9, 0.4, 0.95] if seated else [0.95, 0.95, 0.85, 0.9]

    def pre_step(self, model, data, dt):
        self.thread_step(model, data, dt)

    def state(self, model, data):
        return {'socket_position': np.round(data.mocap_pos[self.mocap], 5).tolist(), 'lit': self.lit,
                'bulb': self.thread_state(model, data)}


# ---- scenes --------------------------------------------------------------------------------------------------------
# Nominal places in the left base frame. The arms' bases are 0.628 m apart (left at y = 0,
# right at y = -0.628); y = -0.314 is midway. RAX's per-arm workspace box is x 0.2-0.8.

def _scene(name):
    if name == 'empty':
        return []
    if name == 'cube':
        return [Cube()]
    if name == 'ball':
        return [Ball()]
    if name == 'handover':
        return [Cube(pose=(0.45, -0.12, 0.0))]
    if name == 'bottle':
        return [Bottle(start='seated')]
    if name == 'bottle_loose':
        return [Bottle(start='loose')]
    if name == 'bottle_open':
        return [Bottle(start='free')]
    if name == 'bulb':
        return [BulbSocket(start='free')]
    if name == 'bulb_seated':
        return [BulbSocket(start='seated')]
    if name == 'household':
        return [Mug(), Marker(), SmallBox()]
    if name == 'all':
        return [Cube(pose=(0.40, -0.10, 0.0)), Ball(pose=(0.60, -0.15, 0.0)), Bottle(pose=(0.50, -0.314, 0.0)),
                BulbSocket(pose=(0.62, -0.45, 0.0), bulb_pose=(0.38, -0.50, 0.0)),
                Mug(pose=(0.35, -0.30, -1.2)), Marker(pose=(0.45, -0.52, 0.3)), SmallBox(pose=(0.62, -0.30, 0.2))]
    raise ValueError(f'unknown scene {name!r}; choose one of {", ".join(SCENES)}')


SCENES = ('empty', 'cube', 'ball', 'handover', 'bottle', 'bottle_loose', 'bottle_open', 'bulb', 'bulb_seated',
          'household', 'all')


class Scene:
    def __init__(self, name, *, seed=0, randomize=True, table_z=0.0):
        self.name, self.seed, self.randomize, self.table_z = name, seed, bool(randomize), float(table_z)
        self.objects = _scene(name)
        self.rng = np.random.default_rng(seed)

    def adders(self):
        return [lambda spec, o=o: o.add(spec, self.table_z) for o in self.objects]

    def bind(self, model, data):
        for o in self.objects:
            o.bind(model, data)

    def reset(self, model, data, seed=None):
        if seed is not None:
            self.seed = seed
        self.rng = np.random.default_rng(self.seed)
        for o in self.objects:
            o.reset(model, data, self.rng, self.randomize)

    def pre_step(self, model, data, dt):
        for o in self.objects:
            o.pre_step(model, data, dt)

    def draws(self):
        return {getattr(o, 'name_object', o.name): deepcopy(o.draw) for o in self.objects}

    def state(self, model, data):
        """PRIVILEGED ground truth of every object, in the left base frame."""
        return {'privileged': True, 'frame': 'left_base', 'scene': self.name, 'seed': self.seed,
                'time': round(float(data.time), 4),
                'objects': {getattr(o, 'name_object', o.name): o.state(model, data) for o in self.objects},
                'draws': self.draws()}
