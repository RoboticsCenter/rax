"""Facts and assumptions of the simulated rig, each with its source.

Every constant here says where it comes from. ASSUMED marks a value the rig has not
measured; change it through the session's `simulation` options, not here.

Frames: the simulator's world frame is the LEFT arm's base frame (RAX's
`left_base`, the calibration's reference). Gravity is along its -z (ASSUMED: the
table is level). The right arm's base sits at the calibrated
`T_left_base_right_base`.
"""
from __future__ import annotations

from copy import deepcopy
import math

from services.wuji.config import JOINT_NAMES

SIDES = ('left', 'right')

# ---- Wuji Hand 2.2 -------------------------------------------------------------------------------------------------
# The SDK's flat joint order (RAX services/wuji/config.py JOINT_NAMES) mapped to the
# names in wuji-description's MJCF (without the l_/r_ prefix). S1..S4 are, from the
# palm out, cmc flex, cmc abd, mcp, ip for the thumb and mcp flex, mcp abd, pip, dip
# for the fingers (Wuji docs, "overview"; matches RAX's profile joint limits).
MJCF_HAND_JOINTS = ('thumb_cmc_flex', 'thumb_cmc_abd', 'thumb_mcp', 'thumb_ip') + tuple(
    f'{finger}_{joint}' for finger in ('index_finger', 'middle_finger', 'ring_finger', 'pinky')
    for joint in ('mcp_flex', 'mcp_abd', 'pip', 'dip'))
assert len(MJCF_HAND_JOINTS) == len(JOINT_NAMES) == 20
TIP_SITES = ('thumb_tip', 'index_finger_tip', 'middle_finger_tip', 'ring_finger_tip', 'pinky_tip')
PREFIX = {'left': 'l_', 'right': 'r_'}

# The torque constant is not published by Wuji. ASSUMED (as mooncake sim2real/xarm-wuji):
# the MJCF's actuatorfrcrange (0.6 N m thumb base joints, 2 N m finger mcp flexion,
# 0.2-0.3 N m the rest) is reached at PEAK_CURRENT_A. Scale or override per session.
PEAK_CURRENT_A = 3.0

# Fault codes the sim firmware can raise, with the SDK's describe_error fields that
# RAX's driver checks (services/wuji/sdk.py and partial.py ISOLATABLE).
ERRORS = {
    3: {'name': 'Enc1BitRate', 'severity': 'Warning', 'clear_policy': 'AutoClear'},
    7: {'name': 'Stall', 'severity': 'Warning', 'clear_policy': 'AutoClear'},
    0x2108: {'name': 'Enc1CrcPersistent', 'severity': 'ImmediateStop', 'clear_policy': 'ManualClear'},
    0x3208: {'name': 'InitCurrentCalibFailed', 'severity': 'Fatal', 'clear_policy': 'NonClearable'},
}
# Native joint states (status_word.ext_state): 0 Init, 1 Ready (disabled), 2 Enabled,
# 3 Stopped (after an emergency stop). RAX treats 1 and 3 as disabled.
STATE_NAMES = {0: 'Init', 1: 'Ready', 2: 'Enabled', 3: 'Stopped'}

HAND_DEFAULTS = {
    # RAX parameter sweep 2026-10-27: reported current ~= 1.9870 x nominal PD +
    # 1.9895 x commanded effort (docs/wuji-parameter-sweep-2026-10-27.md); aiowuji
    # fits 1.95 / 1.9. Applied to the whole law, before the current cap.
    'gain_factor': 1.98,
    # Multiplies the per-joint torque constant (Wuji-identified, model.WUJI_IDENTIFIED_KT).
    'torque_per_amp_scale': 1.0,
    # Optional explicit N m/A: one number or 20 (overrides the MJCF-derived values).
    'torque_per_amp': None,
    # Joint friction in amperes of motor current (converted with each joint's torque
    # constant). aiowuji: "breaks free at about 0.075 A of push, slides with
    # 0.02-0.035 A of friction"; 0.1-0.2 A break-free on S1 joints. RAX traces show
    # ~0.013 rad slips at 1-1.5 rad/s under 0.0125 rad/s references
    # (docs/wuji-smoothness-2026-10-27.md). ASSUMED values within those reports: static at
    # the top of the S1 break-free range, so slow ramps stick ~85 % of the time and slip
    # past RAX's 0.25 rad/s burst threshold (the sim's slips peak at 0.3-0.45 rad/s, below
    # the rig's 1-1.5 rad/s: the MIT kd damps them more than on the hand).
    'static_friction_a': 0.2,
    'coulomb_friction_a': 0.03,
    'stribeck_velocity_rad_s': 0.01,
    'viscous_friction_a_s_per_rad': 0.0,
    # aiowuji: the hand applies a target "2 ms after it is sent".
    'command_latency_s': 0.002,
    # Streams: joint_states natively 1 kHz; diagnostics share it on the device and are
    # decimated here to keep the host cost of the driver's callbacks low (ASSUMED OK:
    # RAX's watchdogs need < 0.25 s age).
    'state_rate_hz': 1000.0,
    'diagnostics_rate_hz': 100.0,
    'position_noise_rad': 0.0,
    'velocity_noise_rad_s': 0.0,
    # A sustained obstruction raises Stall (0x0007) without stopping output (Wuji control
    # guide). The firmware's criterion is not published: ASSUMED current within 2 % of
    # the cap and |velocity| < 0.05 rad/s for stall_time_s.
    'stall': True,
    'stall_time_s': 2.0,
    # Joint-MCU temperature: first-order I^2 heating. ASSUMED: 30 C, 0.8 A continuous on a
    # joint settles 25 C higher with a 300 s time constant.
    'thermal': True,
    'ambient_c': 30.0,
    'thermal_tau_s': 300.0,
    'thermal_rise_c_per_a2': 25.0 / 0.64,
    # Firmware feed-forward compensation (set_constant / set_position_table, firmware
    # 2.10+, units shipped from October 2026). RAX measured no effect on its hands
    # (docs/wuji-parameter-sweep-2026-10-27.md), so by default the modes read back but
    # add nothing. When true, Constant adds the Coulomb estimate in the reference
    # velocity's direction and VelocityTable adds tanh(dq_ref / 0.02) times it.
    'firmware_feedforward_effective': False,
    # Firmware ceiling for effort_limit writes (A). ASSUMED.
    'max_current_a': 3.0,
    'firmware_version': '2.13.1',
    'vbus_v': 12.0,
}

# The rig's left hand runs 18 of 20 joints (examples/xarm-wuji-bimanual.json:
# isolated_joint_faults). The simulated hardware carries the same two faults.
RIG_FAULTED_JOINTS = {'left': {'thumb_S3': 0x2108, 'middle_S1': 0x3208}, 'right': {}}

# ---- xArm 6 ---------------------------------------------------------------------------------------------------------
ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 7))
# RAX's software limits (services/xarm/backend.py LOWER/UPPER), the URDF's.
ARM_LOWER = (-2 * math.pi, -2.059, -3.927, -2 * math.pi, -1.692, -2 * math.pi)
ARM_UPPER = (2 * math.pi, 2.094, 0.192, 2 * math.pi, math.pi, 2 * math.pi)
ARM_DEFAULTS = {
    # The controller's servo, ASSUMED stiff (a position-controlled industrial arm):
    # per-joint kp N m/rad and damping N m s/rad, with harmonic-drive reflected inertia.
    'kp': [20000.0, 20000.0, 12000.0, 4000.0, 4000.0, 2000.0],
    'kv': [400.0, 400.0, 240.0, 60.0, 60.0, 30.0],
    'armature': [0.2, 0.2, 0.1, 0.03, 0.03, 0.02],
    # Planner sample rate (the xArm controller's 250 Hz servo cycle).
    'controller_hz': 250.0,
    # Collision detection: a joint torque at >= 95 % of its limit for this long raises
    # controller error 31 (ASSUMED sensitivity).
    'collision_time_s': 0.1,
    # Angular speed for Cartesian rotation (rad/s) when the move is mostly a turn. ASSUMED.
    'rotation_speed_radps': 0.6,
    'rotation_acc_radps2': 2.0,
}
# Calibration arm_identity (calibration/xarm-wuji-bimanual-four-camera-20261117):
# TCP offset (mm, rad) and TCP load (kg, mm). Used when the manifest lacks them.
TCP_OFFSET = (0.0, 0.0, 172.0, 0.0, 0.0, 0.0)

# ---- The scene --------------------------------------------------------------------------------------------------------
SCENE_DEFAULTS = {
    # Table top height in the left base frame. ASSUMED: the arms are bolted to the table
    # top (workspace_min z is 0.02 m).
    'table_z': 0.0,
    # The hand on the flange: wuji-description's mount points its fingers along the
    # mount's -z, so the hand sits on the flange flipped about x. Then turned about the
    # flange axis by this yaw. ASSUMED (as mooncake's twin): mirrored hands, both thumbs
    # toward the other arm; at RAX's home joints the palms then face the arm bases.
    # The rig's own mount has not been recorded: one wrist-camera image settles it.
    'hand_mount_yaw_deg': {'left': 90.0, 'right': -90.0},
    # Wuji's arm-flange mount sits directly on the flange (no adapter plate). ASSUMED.
    'hand_mount_offset_m': 0.0,
}

PHYSICS_DEFAULTS = {
    'timestep': 0.001,   # one step per firmware tick (1 kHz)
    'impratio': 10.0,    # stiffer frictional grasp contacts (elliptic cones)
    'iterations': 30,
    'ls_iterations': 20,
    # A hand contact's sliding / torsional / rolling friction. Torsional resists twisting
    # a cap or bulb between fingertips (~ mu x a 4 mm patch). ASSUMED.
    'hand_friction': [1.0, 0.005, 0.0001],
    'hand_condim': 4,
}

SIM_DEFAULTS = {
    'scene': 'cube',
    'seed': 0,
    'randomize': True,
    'realtime_factor': 1.0,      # 1 = wall-clock pace (the hand services' watchdogs need it)
    'cameras': True,             # serve the four RealSense views (EGL rendering)
    # Ready both arms, arm the session and prepare both hands when the backend starts:
    # in simulation the backend plays the operator. Real sessions never auto-arm.
    'auto_prepare': True,
    # MCP tools sim_ground_truth / sim_reset / sim_set_object_pose (PRIVILEGED, simulation
    # only). Off by default so an agent's tool surface matches the real robot's.
    'privileged_tools': False,
    # MCP tools sim_trace_start / sim_trace_stop: 1 kHz firmware-level traces in the shared
    # trace format, beside the robot service's own trace_start / trace_stop.
    'trace_tools': False,
    'hand': HAND_DEFAULTS,
    'hands': {side: {'faulted_joints': {}, 'initial_posture': 'home'} for side in SIDES},
    'arm': ARM_DEFAULTS,
    'arms': {side: {} for side in SIDES},
    'scene_options': SCENE_DEFAULTS,
    'physics': PHYSICS_DEFAULTS,
}


PER_SIDE_KEYS = {'hands': {'faulted_joints', 'initial_posture'},
                 'arms': {'serial', 'initial_joints'}}


def _block(name, value, allowed):
    if not isinstance(value, dict):
        raise ValueError(f'simulation.{name} must be an object')
    unknown = set(value) - set(allowed)
    if unknown:
        raise ValueError(f'unknown simulation.{name} options {sorted(unknown)}')
    return deepcopy(value)


def options(simulation):
    """The complete simulation options: defaults merged with the session's block."""
    sim = _block('', simulation or {}, SIM_DEFAULTS)
    out = deepcopy(SIM_DEFAULTS)
    for key in ('scene', 'seed', 'randomize', 'realtime_factor', 'cameras', 'auto_prepare', 'privileged_tools',
                'trace_tools'):
        if key in sim:
            out[key] = sim[key]
    for key in ('hand', 'arm', 'physics', 'scene_options'):
        block = _block(key, sim.get(key, {}), SIM_DEFAULTS[key])
        if key == 'scene_options' and 'hand_mount_yaw_deg' in block:
            block['hand_mount_yaw_deg'] = {**out[key]['hand_mount_yaw_deg'],
                                           **_block('scene_options.hand_mount_yaw_deg',
                                                    block['hand_mount_yaw_deg'], SIDES)}
        out[key].update(block)
    for key in ('hands', 'arms'):
        block = _block(key, sim.get(key, {}), SIDES)
        for side, value in block.items():
            out[key][side].update(_block(f'{key}.{side}', value, PER_SIDE_KEYS[key]))
    return out
