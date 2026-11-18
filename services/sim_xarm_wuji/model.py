"""MuJoCo models of the rig: two xArm 6 arms with Wuji Hand 2.2 hands, cameras, a table
and a scene's objects, built with MjSpec from the vendor assets (assets.py).

- The xArm 6 is generated from UFACTORY's xarm_description data (joint origins from
  `xarm6_default_kinematics.yaml`, inertials from `xarm6_type6_HT_BR2.yaml` (the
  xacro's default), torque limits from `joint_limits_1305.yaml`, meshes from
  `meshes/xarm6_1305`). Its joints carry stiff position servos (the controller box's
  servo loop, ASSUMED gains) and its links are gravity-compensated, as a
  position-controlled industrial arm behaves.
- Each Wuji Hand 2.2 is wuji-description's MJCF, its position actuators replaced by
  one `general` actuator per joint that implements the MIT current law in amperes:
  torque = kt * clamp(g * (kp (q_ref - q) + kd (dq_ref - dq) + i_ff), +-cap). The
  firmware (firmware.py) writes gains into gainprm/biasprm and the combined command
  u = kp q_ref + kd dq_ref + i_ff into ctrl; MuJoCo evaluates the law every 1 ms
  physics step, i.e. at the firmware's 1 kHz.
- The wrist cameras ride link6 at RAX's T_base_cam = T_base_tcp inv(T_cam_tcp); the
  fixed cameras sit at inv(T_cam_left_base); intrinsics are the calibrated K.
"""
from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from . import assets
from .spec import ARM_JOINTS, ARM_LOWER, ARM_UPPER, MJCF_HAND_JOINTS, PEAK_CURRENT_A, PREFIX, SIDES, TIP_SITES

# Geom groups: 1 visual (rendered), 3 collision (hidden), 4 sites/aux.
VISUAL, COLLISION = 1, 3
TABLE_SIZE = (0.65, 0.75, 0.025)       # half extents: covers both arms and the space in front
TABLE_CENTRE = (0.35, -0.314, None)    # x, y in the left base frame (between the arms)


def quat_wxyz(rotation):
    x, y, z, w = rotation.as_quat()
    return [w, x, y, z]


def rpy_quat(rpy):
    return quat_wxyz(Rotation.from_euler('xyz', rpy))


def matrix_pose(T):
    T = np.asarray(T, dtype=float)
    return T[:3, 3].tolist(), quat_wxyz(Rotation.from_matrix(T[:3, :3]))


def _load_yaml(path):
    with open(path) as stream:
        return yaml.safe_load(stream)


# ---- xArm 6 -----------------------------------------------------------------------------------------------------------

def xarm6_spec(arm_options):
    """One xArm 6 (no end effector) as an uncompiled MjSpec, rooted at `link_base`.

    Sites: `flange` (link6 origin, z out of the flange) and `tcp` (set by the caller)."""
    root = assets.xarm_description()
    kin = _load_yaml(root/'config/kinematics/default/xarm6_default_kinematics.yaml')['kinematics']
    inertial = _load_yaml(root/'config/link_inertial/xarm6_type6_HT_BR2.yaml')
    limits = _load_yaml(root/'config/xarm6/joint_limits_1305.yaml')['joint_limits']
    meshes = root/'meshes/xarm6_1305'
    spec = mujoco.MjSpec()
    spec.compiler.degree = False
    for name in ('link_base', *(f'link{i}' for i in range(1, 7))):
        spec.add_mesh(name=f'{name}_visual', file=str(meshes/'visual'/f'{name}.stl'))
        spec.add_mesh(name=f'{name}_collision', file=str(meshes/'collision'/f'{name}.obj'))
    white = spec.add_material(name='xarm_white', rgba=[0.92, 0.92, 0.92, 1], specular=0.3, shininess=0.4)
    base = spec.worldbody.add_body(name='link_base')
    # link_base is fixed: its inertia does not matter (URDF: CoM 0.09103 m up).
    base.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='link_base_visual', group=VISUAL,
                  contype=0, conaffinity=0, material=white.name, density=0)
    base.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname='link_base_collision', group=COLLISION, density=0)
    parent = base
    kp, kv, armature = arm_options['kp'], arm_options['kv'], arm_options['armature']
    for i in range(1, 7):
        joint = kin[f'joint{i}']
        body = parent.add_body(name=f'link{i}', pos=[joint['x'], joint['y'], joint['z']],
                               quat=rpy_quat([joint['roll'], joint['pitch'], joint['yaw']]))
        link = inertial[f'link{i}']
        I = link['inertia']
        body.fullinertia = [I['ixx'], I['iyy'], I['izz'], I['ixy'], I['ixz'], I['iyz']]
        body.mass = link['mass']
        body.ipos = [link['origin']['x'], link['origin']['y'], link['origin']['z']]
        body.explicitinertial = True
        body.gravcomp = 1.0
        effort = float(limits[f'joint{i}']['max_effort'])
        body.add_joint(name=ARM_JOINTS[i-1], type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 0, 1],
                       range=[ARM_LOWER[i-1], ARM_UPPER[i-1]], armature=armature[i-1], damping=0.0,
                       actfrcrange=[-effort, effort], actfrclimited=mujoco.mjtLimited.mjLIMITED_TRUE)
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f'link{i}_visual', group=VISUAL,
                      contype=0, conaffinity=0, material=white.name, density=0)
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=f'link{i}_collision', group=COLLISION, density=0)
        # The controller box's servo: a position actuator with implicit damping.
        spec.add_actuator(name=f'servo{i}', target=ARM_JOINTS[i-1], trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=[kp[i-1]] + [0]*9, biasprm=[0, -kp[i-1], -kv[i-1]] + [0]*7,
                          forcerange=[-effort, effort], forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE)
        parent = body
    parent.add_site(name='flange', pos=[0, 0, 0], group=4, size=[0.004, 0, 0])
    # MuJoCo filters parent-child contacts except when the parent is welded to the world,
    # as link_base is: without this its mesh rubs link1 and fights joint 1.
    spec.add_exclude(bodyname1='link_base', bodyname2='link1')
    return spec


# ---- Wuji Hand 2.2 ---------------------------------------------------------------------------------------------------

def hand_spec(side, physics):
    """wuji-description's Hand 2.2 MJCF with the MIT-law actuators (one per joint, named
    `<prefix><mjcf joint>_mit`) and collision geoms moved to the hidden group."""
    path = assets.wuji_hand_mjcf(side)
    spec = mujoco.MjSpec.from_file(str(path))
    meshdir = (path.parent/spec.meshdir).resolve()
    for mesh in spec.meshes:
        if mesh.file and not Path(mesh.file).is_absolute():
            mesh.file = str(meshdir/mesh.file)
    spec.meshdir = ''
    for actuator in list(spec.actuators):
        spec.delete(actuator)
    for sensor in list(spec.sensors):
        spec.delete(sensor)
    for material in spec.materials:
        material.name = f'wuji_{side}_{material.name}'
    for geom in spec.geoms:
        if geom.material:
            geom.material = f'wuji_{side}_{geom.material}'
        if geom.contype == 0 and geom.conaffinity == 0:
            geom.group = VISUAL
        else:
            geom.group = COLLISION
            geom.condim = int(physics['hand_condim'])
            geom.friction = list(physics['hand_friction'])
            geom.rgba = [0.5, 0.4, 0.8, 0.4]
    prefix = PREFIX[side]
    for name in MJCF_HAND_JOINTS:
        joint = spec.joint(prefix + name)
        peak = float(joint.actfrcrange[1])
        # MIT law: force = gain*ctrl + b1*q + b2*dq, gains written by the firmware.
        spec.add_actuator(name=f'{prefix}{name}_mit', target=prefix + name, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED, biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=[0]*10, biasprm=[0]*10, ctrllimited=mujoco.mjtLimited.mjLIMITED_FALSE,
                          forcerange=[-peak, peak], forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE)
    return spec


STIFF_FRICTION_SOLREF = (0.002, 1.0)
STIFF_FRICTION_SOLIMP = (0.995, 0.999, 0.001, 0.5, 2.0)


HAND_ATTACH = 'hand_'


def hand_name(side, name, rig=True):
    """A hand element's compiled name: '<side>_hand_<l_|r_><mjcf name>' in the rig model,
    '<l_|r_><mjcf name>' in a hand-only model."""
    return f'{side}_{HAND_ATTACH}{PREFIX[side]}{name}' if rig else f'{PREFIX[side]}{name}'


def hand_peaks(model, side, rig=True):
    """The MJCF's peak joint torques (N m), as compiled into the MIT actuators' forcerange."""
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, hand_name(side, f'{j}_mit', rig))
           for j in MJCF_HAND_JOINTS]
    if min(ids) < 0:
        raise RuntimeError(f'the model lacks the {side} hand actuators')
    return np.asarray(model.actuator_forcerange[ids, 1], dtype=float)


# Wuji-identified output torque constants (N m/A) per SDK joint, finger-major
# (thumb, index, middle, ring, pinky) x (S1..S4). Source: the vendor's own Hand 2
# pen-spin actuator identification in wuji-technology/wuji-mjlab (Apache-2.0), which
# also reports 1.911 A per MIT unit, consistent with RAX's measured ~1.98x gain factor.
WUJI_IDENTIFIED_KT = np.array([
    0.4136, 0.3545, 0.2216, 0.1329,
    0.4136, 0.1673, 0.1673, 0.1673,
    0.4136, 0.1673, 0.1673, 0.1673,
    0.4136, 0.1673, 0.1673, 0.1673,
    0.4461, 0.1115, 0.1115, 0.1115,
])


def hand_torque_constants(model, side, scale=1.0, override=None, rig=True):
    """N m/A per SDK joint: Wuji's identified constants (WUJI_IDENTIFIED_KT) times
    ``scale``, unless ``override`` gives one number or 20. ``model``/``side``/``rig``
    are kept for the old MJCF-peak assumption (hand_peaks / PEAK_CURRENT_A)."""
    if override is not None:
        values = [float(override)]*20 if np.isscalar(override) else [float(v) for v in override]
        if len(values) != 20 or min(values) <= 0:
            raise ValueError('torque_per_amp must be one positive number or 20')
        return np.asarray(values)
    return WUJI_IDENTIFIED_KT * float(scale)


# ---- cameras ---------------------------------------------------------------------------------------------------------

OPTICAL_TO_MUJOCO = np.diag([1.0, -1.0, -1.0])  # a MuJoCo camera looks along -z with y up


def add_camera(body, name, T_parent_optical, K, width, height):
    """A MuJoCo camera at an OpenCV optical frame with pinhole intrinsics K."""
    T = np.asarray(T_parent_optical, dtype=float)
    R = T[:3, :3] @ OPTICAL_TO_MUJOCO
    camera = body.add_camera(name=name, pos=T[:3, 3].tolist(), quat=quat_wxyz(Rotation.from_matrix(R)))
    K = np.asarray(K, dtype=float)
    camera.resolution = [int(width), int(height)]
    camera.focal_pixel = [float(K[0, 0]), float(K[1, 1])]
    # MuJoCo's principal offset has the opposite sign of OpenCV's (cx - w/2, cy - h/2);
    # verified against RAX's projection in local_tests/unit/test_sim_xarm_wuji.py.
    camera.principal_pixel = [float(width / 2.0 - K[0, 2]), float(height / 2.0 - K[1, 2])]
    camera.sensor_size = [float(width) * 1e-6, float(height) * 1e-6]  # 1 um pixels: only ratios matter
    camera.fovy = math.degrees(2 * math.atan(height / 2.0 / K[1, 1]))
    return camera


# ---- the rig ---------------------------------------------------------------------------------------------------------

def tcp_transform(tcp_offset_mm_rad):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', tcp_offset_mm_rad[3:6]).as_matrix()
    T[:3, 3] = np.asarray(tcp_offset_mm_rad[:3], dtype=float) / 1000.0
    return T


def build_spec(sim, rig, objects=()):
    """The whole rig as an MjSpec.

    sim: simulation options (spec.options); rig: geometry from the calibration
    (rig.Rig); objects: [(callable(spec, worldbody), ...)] from scenes.py."""
    physics = sim['physics']
    spec = mujoco.MjSpec()
    spec.modelname = 'rax_xarm6_wuji2_bimanual'
    spec.compiler.degree = False
    spec.option.timestep = float(physics['timestep'])
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = float(physics['impratio'])
    spec.option.iterations = int(physics['iterations'])
    spec.option.ls_iterations = int(physics['ls_iterations'])
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1280, 720
    spec.visual.quality.shadowsize = 2048
    spec.visual.headlight.ambient = [0.35, 0.35, 0.35]
    spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
    table_z = float(sim['scene_options']['table_z'])
    world = spec.worldbody
    hx, hy, hz = TABLE_SIZE
    world.add_geom(name='table', type=mujoco.mjtGeom.mjGEOM_BOX, size=[hx, hy, hz],
                   pos=[TABLE_CENTRE[0], TABLE_CENTRE[1], table_z - hz], rgba=[0.55, 0.45, 0.36, 1],
                   friction=[0.6, 0.005, 0.0001], group=VISUAL)
    world.add_geom(name='floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.05],
                   pos=[0, 0, table_z - 0.75], rgba=[0.35, 0.35, 0.38, 1], group=VISUAL)
    world.add_light(name='key', pos=[0.6, -0.3, 1.6], dir=[-0.2, 0.0, -1.0], diffuse=[0.6, 0.6, 0.6],
                    castshadow=True)
    world.add_light(name='fill', pos=[1.2, -0.9, 1.2], dir=[-0.6, 0.5, -0.6], diffuse=[0.3, 0.3, 0.3],
                    castshadow=False)
    for side in SIDES:
        arm = xarm6_spec(sim['arm'])
        hand = hand_spec(side, physics)
        for child in (arm, hand):  # the world's solver settings win; matching them avoids attach noise
            for key in ('timestep', 'integrator', 'cone', 'impratio', 'iterations', 'ls_iterations', 'tolerance',
                        'jacobian', 'solver'):
                setattr(child.option, key, getattr(spec.option, key))
        flange = arm.site('flange')
        yaw = math.radians(sim['scene_options']['hand_mount_yaw_deg'][side])
        mount = Rotation.from_euler('z', yaw) * Rotation.from_euler('x', math.pi)
        frame = flange.parent.add_frame(pos=[0, 0, float(sim['scene_options']['hand_mount_offset_m'])],
                                        quat=quat_wxyz(mount))
        arm.attach(hand, prefix=HAND_ATTACH, frame=frame)
        prefix = HAND_ATTACH + PREFIX[side]
        arm.add_exclude(bodyname1='link6', bodyname2=prefix + 'wrist')
        arm.add_exclude(bodyname1='link5', bodyname2=prefix + 'mount')
        tcp = tcp_transform(rig.tcp_offset[side])
        arm.body('link6').add_site(name='tcp', pos=tcp[:3, 3].tolist(), quat=quat_wxyz(Rotation.from_matrix(tcp[:3, :3])),
                                   group=4, size=[0.004, 0, 0])
        camera = rig.wrist_camera(side)
        if camera is not None:
            # T_tcp_optical = inv(T_cam_tcp); on link6: T_flange_tcp @ T_tcp_optical.
            add_camera(arm.body('link6'), 'wrist_camera', tcp @ np.linalg.inv(camera['T_cam_tcp']),
                       camera['K'], camera['width'], camera['height'])
        position, quat = matrix_pose(rig.base_pose(side))
        base_frame = world.add_frame(pos=position, quat=quat)
        spec.attach(arm, prefix=f'{side}_', frame=base_frame)
    for name, camera in rig.fixed_cameras().items():
        add_camera(world, name, np.linalg.inv(camera['T_cam_left_base']), camera['K'], camera['width'],
                   camera['height'])
    for add in objects:
        add(spec)
    return spec


class Indices:
    """Name-resolved addresses into a compiled rig model."""

    def __init__(self, model):
        m = model
        name = lambda kind, n: mujoco.mj_name2id(m, kind, n)  # noqa: E731
        J, A, S, B, C = (mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mjtObj.mjOBJ_SITE,
                         mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_CAMERA)
        self.arm_qpos, self.arm_dof, self.arm_act = {}, {}, {}
        self.hand_qpos, self.hand_dof, self.hand_act, self.hand_jnt = {}, {}, {}, {}
        self.tcp_site, self.flange_site, self.base_body, self.tips, self.palm = {}, {}, {}, {}, {}
        self.hand_bodies, self.arm_bodies, self.wrist_camera = {}, {}, {}
        for side in SIDES:
            joints = [name(J, f'{side}_{j}') for j in ARM_JOINTS]
            hand = [name(J, hand_name(side, j)) for j in MJCF_HAND_JOINTS]
            act = [name(A, f'{side}_servo{i}') for i in range(1, 7)]
            mit = [name(A, hand_name(side, f'{j}_mit')) for j in MJCF_HAND_JOINTS]
            if min(joints + hand + act + mit) < 0:
                raise RuntimeError(f'the rig model lacks a joint or actuator of the {side} assembly')
            self.arm_qpos[side] = np.array([m.jnt_qposadr[j] for j in joints])
            self.arm_dof[side] = np.array([m.jnt_dofadr[j] for j in joints])
            self.arm_act[side] = np.array(act)
            self.hand_jnt[side] = np.array(hand)
            self.hand_qpos[side] = np.array([m.jnt_qposadr[j] for j in hand])
            self.hand_dof[side] = np.array([m.jnt_dofadr[j] for j in hand])
            self.hand_act[side] = np.array(mit)
            self.tcp_site[side] = name(S, f'{side}_tcp')
            self.flange_site[side] = name(S, f'{side}_flange')
            self.base_body[side] = name(B, f'{side}_link_base')
            self.palm[side] = name(B, hand_name(side, 'wrist'))
            self.tips[side] = [name(S, hand_name(side, t)) for t in TIP_SITES]
            self.wrist_camera[side] = name(C, f'{side}_wrist_camera')
            root = name(B, hand_name(side, 'mount'))
            self.hand_bodies[side] = self._subtree(m, root)
            self.arm_bodies[side] = sorted(set(self._subtree(m, self.base_body[side])) - set(self.hand_bodies[side]))

    @staticmethod
    def _subtree(m, root):
        out = []
        for b in range(m.nbody):
            p = b
            while p > 0 and p != root:
                p = int(m.body_parentid[p])
            if p == root:
                out.append(b)
        return out
