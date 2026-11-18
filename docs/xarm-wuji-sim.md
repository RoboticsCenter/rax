# The bimanual xArm 6 + Wuji Hand 2 twin (MuJoCo)

`services/sim_xarm_wuji` simulates the bimanual rig — two xArm 6 arms, a Wuji Hand 2.2 on
each flange, four RealSense views — **below the vendor SDKs**. RAX's robot service runs
on it unchanged: the same `XArmBimanualBackend`, the same hand service process
(`services/wuji/controller.py` with its `WujiDriver`), the same MCP tools, admission,
guards, settling, stop logic, calibration loader and observation code. A skill's
`main(ctx, **params)` therefore runs in simulation exactly as it will on the rig. The
same hand model, stripped down, is available to MPC skills as an internal model
(`mpc.py`).

Nothing here can reach hardware: simulated sessions name `sim-*` arms, `SIM*` hands and
`sim-*` cameras, and validation refuses network addresses or real serials.

## Start a simulated session

One time per checkout (Python 3.12, the robot runtime's interpreter):

```bash
uv pip install --python .venv/bin/python -r requirements-sim.txt
.venv/bin/python -m services.sim_xarm_wuji.assets fetch    # pinned vendor models -> runtime/sim-assets
```

**Run skills without an agent** (prints the MCP socket a ctx connects to):

```bash
.venv/bin/python -m services.sim_xarm_wuji.session --scene bottle [--seed 3] [--privileged]
# RAX_MCP_SOCKET=/tmp/rax-sim-<uid>/mcp-xxxxxx.sock
RAX_MCP_SOCKET=... .venv/bin/python my_skill_runner.py   # with api.runtime.Context() as ctx: main(ctx)
```

**Agent session on the twin** — identical to a real bimanual session (robot
`xarm_bimanual`, same workspace, skills and tools); the `simulation` block selects the
twin (`services/backends.py`):

```bash
.venv/bin/python scripts/configure_session.py --task unscrew_bottle --task-file task.md \
    --backend-options examples/xarm-wuji-bimanual-sim.json
bash scripts/start_agent.sh --config <printed operator-input.json>
```

The backend plays the operator at start-up (`simulation.auto_prepare`, default on):
readies both simulated controllers, arms the session without expiry and prepares both
hands. Afterwards the usual operator paths work and stay in simulation:
`scripts/xarm-wuji-ready --session S`, the dashboard's Prepare, and

```bash
.venv/bin/python -m services.sim_xarm_wuji.operator {prepare|stop|status|reset|stats} --session S [--scene NAME --seed N]
```

**Evaluation harness** (`services/dex_eval`): the registered key is
**`xarm_wuji_bimanual_sim`**; `scripts/dex_eval_run.py PLAN.json --backend sim` picks it.
With that key the backend options may be partial or empty: the example rig fills in.
The harness's in-process route (Frontend + `make_backend`) is what
`local_tests/unit/test_sim_xarm_wuji.py::test_robot_service_runs_unchanged_on_the_sim_backend`
exercises.

## How it fits together

| Process | Real rig | Twin |
|---|---|---|
| robot service | `XArmBimanualBackend` | `SimXArmBimanualBackend` (subclass; only device construction differs) |
| arms | `XArmDriver` → `xarm.wrapper.XArmAPI` → controller box | `XArmDriver` (unchanged) → `SimXArmAPI` → sim server's controller emulation |
| hands | `services.wuji.controller` + `WujiDriver` → `wuji_sdk` | the same `serve()` + `WujiDriver` (`hand_service.py`) → `wuji_sdk` mimic → sim server's firmware emulation |
| cameras | `services.xarm.camera_server` (RealSense) | `camera_server.py`: same socket protocol, MuJoCo renders |
| hardware | the rig | `server.py`: one MuJoCo world at 1 kHz paced to wall-clock time |

The hands' 1 kHz streams (joint_states, joint_diagnostics) and commands pass through
shared memory (`shm.py`, seqlock blocks in `/dev/shm`); parameter calls (gains, limits,
enable/disable) and the xArm API are JSON-line RPCs on a private Unix socket (`rpc.py`).
Frames are stamped with the host's monotonic clock, as RAX's driver expects. Sockets,
shared memory and the hands' device leases are per session, so several simulated
sessions can run on one host at once. `sim_reset scene=…` (privileged) stops and
disarms, rebuilds the world, reconnects and re-prepares both hands (about 0.35 s).

Measured on this machine (16 hardware threads, integrated AMD Radeon 780M, no discrete GPU):
- physics + firmware + controllers: 0.21-0.25 ms per 1 ms step on one core;
  the server holds real time (achieved real-time factor 0.988-1.000 over active
  sessions, with at most one ≤ 95 ms lag, during start-up);
- `wuji_move_joints` of 0.3 rad with the right profile (0.6 rad/s, 3 rad/s², 2 rad/s³):
  reached in 2.1 s, 3 mrad error; a 2-3 cm `xarm_move_pose`: 1.3 mm error;
- `xarm_observe` of all four views with depth: 0.12-0.16 s (0.6 s for the first, which
  starts the camera server); EGL on the integrated GPU, about 12 ms per 1280x720 frame.

## The SDK surface emulated

**`wuji_sdk` (2026.10.12), `services/sim_xarm_wuji/wuji_sdk.py`** — given to RAX's
`WujiDriver(config, sdk=...)`:
- `SdkManager.instance().connect(sn=…, device_name=…, options=ConnectOptions(…))`,
  `get_device`, `disconnect`, `disconnect_all`; connect by `address`/`handedness` is refused
  (not simulated);
- `WujiHand2`: `serial_number`, `device_name`, `info.firmware_version` (`2.13.1`),
  `handedness().get()` (`'Left'`/`'Right'`), `fingertip_online()` (all False, as the
  rig's hands report);
- streams: `joint_states()` / `joint_diagnostics()` with `subscribe_with_callback(cb,
  on_error)` and `subscribe()` (`Subscription.recv()`, `set_rate(hz)`, `close()`); frames
  carry `header.seq/timestamp_us`, `joints[].nid/position/velocity/effort` and
  `joints[].status_word.ext_state/ext_state_name(property)/current_limit_active`,
  `current`, `vbus_v_fb`, `mcu_temp_c_fb`, `error_code_current`; native rates 1 kHz
  (state) and 100 Hz (diagnostics), configurable;
- commands: `joint_command().publish().send([JointCommand(position, velocity, effort)]*20)`
  — MIT targets with **feed-forward current** in `effort` (A), **real-time streaming up
  to 1 kHz** (every physics step reads the latest command);
- parameters: `effort_limit().set/get` (scalar or 20; a write over the 3 A firmware
  ceiling keeps the old limit and raises, as documented), `mit_params().set/get`
  (one `(kp, kd)`/`MitParam` or 20);
- state: `enable/disable/clear_fault(joints=mask)`, `emergency_stop()`;
- firmware compensation: `set_constant(FeedForwardMode, joints)`,
  `set_position_table(on, joints)` (read back; inert unless
  `firmware_feedforward_effective`, since RAX measured no effect on its hands);
- `joint(k)`, `joints()`, `fingers()` handles (`effort_limit`, `error_code`, `status_word`,
  `enable/disable/clear_fault`, `constant`, `position_table`);
- `nid_to_joint_index`, `joint_index_to_nid`, `describe_error`; `FeedForwardMode`,
  `ControlMode`, `LowPass`, `Handedness`, `WujiException`.

Not emulated: discovery, bridge, time sync, tactile/IMU/fingertip data, firmware
upgrade, `realtime_controller` (a first-generation-hand API), Position mode (firmware
3.0.0). The test `test_wuji_sdk_mimic_covers_the_vendor_surface` checks the names RAX
uses against the installed SDK's own stubs.

**`xarm.wrapper.XArmAPI` (1.18.4), `xarm_api.py`** — inside RAX's `XArmDriver`:
`get_position`, `get_servo_angle`, `get_state`, `get_err_warn_code`, `get_robot_sn`,
`get_forward_kinematics`, `get_inverse_kinematics`, `is_tcp_limit`, `is_joint_limit`,
`set_position` (straight TCP line, absolute), `set_servo_angle`, `set_state`,
`set_mode`, `motion_enable`, `clean_error`, `clean_warn`, `emergency_stop`,
`disconnect`, properties `state`, `mode`, `error_code`, `warn_code`, `tcp_offset`,
`position`, `angles`, `connected`; `is_radian` handled as the SDK does. MOVE_RELATIVE is
refused (RAX sends absolute targets).

## The hand model (`firmware.py`, `world.py`, `spec.py`)

Per joint, evaluated by MuJoCo inside every 1 ms step (MIT actuators, `model.py`):

```
current = clamp(g * (kp (q_ref - q) + kd (dq_ref - dq) + i_ff + i_fw), -cap, +cap)
torque  = kt * current        (reported as joint_states effort and diagnostics current)
```

| Constant | Default | Source |
|---|---|---|
| effective gain `g` | 1.98 | RAX parameter sweep: reported current ≈ 1.987 × nominal PD + 1.9895 × effort (aiowuji: 1.95/1.9) |
| torque constant `kt` | MJCF peak torque / 3 A: 0.2 N m/A thumb CMC, 0.667 finger MCP flex, 0.067 MCP abd, 0.1 others; `torque_per_amp_scale` or `torque_per_amp` override | **ASSUMED** (Wuji publishes none; mooncake's assumption) |
| command latency | 2 ms | aiowuji |
| static / Coulomb friction | 0.20 A / 0.03 A, Stribeck speed 0.01 rad/s | aiowuji break-free 0.075-0.2 A, sliding 0.02-0.035 A; RAX traces |
| current cap | as written by the profile (0.8 A on the rig) | RAX profile |
| power-on gains | kp 5, kd 0.1, 1.5 A | firmware 2.12 defaults, vendor examples |
| stall warning (0x0007) | at ≥ 98 % of cap and < 0.05 rad/s for 2 s | **ASSUMED** criterion; output continues, as documented |
| MCU temperature | 30 °C + 25 °C at 0.8 A steady, τ = 300 s | **ASSUMED** |
| left hand | thumb_S3 0x2108, middle_S1 0x3208, never enable | the rig (`isolated_joint_faults`) |

Friction is MuJoCo's dry-friction constraint with its limit set every step from joint
speed (Stribeck: static at rest, Coulomb when sliding), so slow references stick and
slip. At RAX's gains (kp 8, kd 0.15, 0.8 A), a 0.02 rad/s ramp on index S1 is stuck 89 %
of the time and slips at up to 0.43 rad/s (RAX's > 0.25 rad/s burst metric triggers);
without friction it tracks smoothly (peak 0.06 rad/s at the start transient).
Friction constraints use stiff solver settings (MuJoCo's defaults let light finger links
creep at an estimated ~0.4 rad/s under sub-static loads; stiff settings, ~0.003 rad/s).

## The arm model (`xarm_controller.py`)

UFACTORY's xArm 6 kinematics and inertias (`xarm6_default_kinematics.yaml`,
`xarm6_type6_HT_BR2.yaml`, model-1305 meshes and torque limits). At zero joints the
flange is at (207, 0, 112) mm as on the real arm; the TCP is 172 mm out (calibration).
Each joint is a stiff position servo (ASSUMED gains, harmonic-drive armature) with
gravity compensation of the links and of the hand as TCP payload. The controller box:
state 4 / motion disabled at power-on; RAX's bring-up sequence readies it (state 2);
synchronized trapezoidal joint moves; straight TCP lines (orientation slerped) by IK at
250 Hz, refused with error 21 when unreachable or 24 when a joint would exceed π rad/s;
error 31 when a joint torque stays at its limit for 0.1 s (contact, collision); queued
commands; `set_state(4)` stops and holds.

## Scenes and objects (`scenes.py`)

Select with `simulation.scene` (or `sim_reset scene=…`); `seed` and `randomize` draw
pose (x, y, yaw), mass (×0.8-1.25) and friction (×0.7-1.2) per object.

| Scene | Objects |
|---|---|
| `empty` | — |
| `cube` / `handover` | 5 cm cube, 0.1 kg |
| `ball` | 6 cm ball, 60 g |
| `bottle` / `bottle_loose` / `bottle_open` | 500 ml bottle (0.20-0.45 kg) with a 28 mm cap seated / loosened / off the bottle |
| `bulb` / `bulb_seated` | E27 lamp holder fixed to the table, A60 bulb free / seated |
| `household` | mug, marker, small box |
| `all` | everything above, for camera checks |

**Threads.** The cap (or bulb) is a child of its holder on a slide (`lift`) and a hinge
(`turn`) tied by a MuJoCo joint equality `lift = lead / 2π · turn` (cap 3 mm lead, 1.5
turns; E27 3.629 mm, 2.5 turns). Counter-clockwise from above (+z) unscrews. Turn 0 is
the seated stop. Friction on the turn: running torque (cap 0.03-0.08 N m, bulb
0.01-0.04 N m) and a breakaway (×1.2-2) until it has turned 0.2 rad from seated, armed
again whenever it seats. Past the last turn the part is **removed**: a free twin takes
its pose and velocity. A free part resting aligned on its thread start (within 2.5 mm
and 8°; an invisible collar holds it there) and turned 0.2 rad clockwise **engages**
again. Events `seated` / `removed` / `engaged` are recorded with simulation time; a seated
bulb lights up (visible to the cameras).

**Ground truth is PRIVILEGED.** `sim_ground_truth` (MCP, only when
`simulation.privileged_tools` is true), `SimWorld.ground_truth()` and the server's
`sim.ground_truth` return object poses (left base frame, `quaternion_xyzw`), thread
state, fingertip and palm positions, and hand contacts with normal forces, all marked
`"privileged": true`. Skills may use them only in simulation, or where an equivalent
perception skill exists; the tools are off by default so an agent's surface matches the
real robot's.

## Cameras (`camera_server.py`, `rig.py`, `model.py`)

The four configured views are placed exactly where the calibration manifest says
(fixed: inv(`T_cam_left_base`); wrist: `T_base_tcp · inv(T_cam_tcp)`, RAX's convention)
with the calibrated K (principal point included). RGB 1280×720 JPEG, depth aligned to
colour as 16-bit mm PNG (0.1-6 m), rendered on demand; continuous recording only when
`stream_enabled` (off in the example, to stay cheap). A cube's rendered silhouette lies
within 1-2 px of RAX's own projection of its corners (`test_cameras_render_where_rax_projects`).
`calibration/xarm-wuji-bimanual-sim/manifest.json` is the four-camera 2026-11-17
manifest with only the hardware identities replaced (`python -m
services.sim_xarm_wuji.calibration SRC DST`), so RAX's calibration loader reports the
same transforms.

## Traces

The robot service records traces itself (`trace_start` / `trace_stop`, services/wuji,
at the hand command rate). On the twin they are labelled `backend: "sim"`: the mimic
module sets `RAX_BACKEND_KIND = 'sim'`, which `WujiDriver.backend_kind` reads. The
smooth-control tools (`wuji_stream`, `wuji_trajectory`, `wuji_wait`, …) run on the twin
unchanged; e.g. a 30 Hz `wuji_stream` sinusoid on index S1 tracks at 3.5 mrad RMS in the
service's 200 Hz trace.

The twin adds a firmware-level view, with `simulation.trace_tools` (default off):
`sim_trace_start(directory?, label?)` / `sim_trace_stop()` record both hands and arms at
1 kHz as the simulated firmware sees them, in the same shared format
(`services/dex_eval/trace.py`): `hand_<side>.npz` (`t`, `q_ref`, `q_target`, `q_meas`,
`dq_meas`, `current`, `i_ff`, `kp`, `kd`, plus `dq_ref`, `current_limit_a`,
`temperature_c`), `arm_<side>.npz` (`q_ref`, `q_meas`, `torque_nm`) and `meta.json`
(`backend: "sim"`, `trace_source: "service:sim_firmware"`). Disabled joints are NaN in the
measured arrays. The firmware never sees a move's goal, so here `q_target = q_ref`.
`status()['simulation']` marks a session as simulated.

## MPC: the hand-only model (`mpc.py`)

```python
from services.sim_xarm_wuji.mpc import HandModel, wrist_pose_from_tcp, object_pose_cost, effort_cost

wrist = wrist_pose_from_tcp('right', tcp_position, tcp_quaternion_xyzw)   # from xarm_status, right base frame
model = HandModel('right', object='cube', wrist_pose=wrist, kp=8, kd=0.15, current_limit_a=0.8, nthread=16)
model.set_state(q_measured, dq_measured, object_pose=((x, y, z), (qx, qy, qz, qw)))
result = model.rollout(q_ref_sequences, dq_ref=None, i_ff=None, control_dt=0.01)   # (N, H, 20)
costs = object_pose_cost(result, target_position) + effort_cost(result)           # (N,)
```

- One Wuji hand (left/right, optional disabled joints) fixed at a wrist pose, one object
  (`cube`, `ball`, `marker`, `small_box` or none), an optional table plane.
- The same law as the twin (gain factor, kt, caps; `set_gains`), compiled into MuJoCo
  actuators so `mujoco.rollout` runs it in C++ threads without Python per step;
  Coulomb friction only (no stick-slip), capsules fitted to the hand meshes, 2 ms steps.
- `RolloutResult`: object position and quaternion, fingertip positions, joint positions
  and currents per control step; vectorized cost helpers; `wrist_pose_from_tcp` uses the
  twin's mount (exact against the twin).
- Free-space agreement with the full twin over 1 s of finger flexion: 0.6-0.7 mrad RMS,
  3 mrad worst.

Throughput (`python -m services.sim_xarm_wuji.mpc [nthread]`; right hand + cube,
horizons of 50 control steps × 10 ms = 0.5 s, 5 physics steps each):

| Threads | Rollouts/s | Physics steps/s |
|---|---|---|
| 1 | 67 | 16,700 |
| 4 | 241-257 | 60,000-64,000 |
| 8 | 400-454 | 100,000-113,000 |
| 16 (SMT) | 267-584 (16-256 rollouts per call) | 67,000-146,000 |

256 rollouts of 0.5 s cost 0.44 s on 16 threads: about two MPC replans per second at
that size, or 64 rollouts every 0.17 s.

## Assets and versions

| Asset | Source | Version | License |
|---|---|---|---|
| Wuji Hand 2.2 MJCF + meshes | github.com/wuji-technology/wuji-description `hand2/hand2_2/body` | commit 0f02495611a5e5f71ebac50da7efd4f646cc0a75 (release v2026.11.9, 2026-11-10) | MIT |
| xArm 6 kinematics, inertias, limits, model-1305 meshes | github.com/xArm-Developer/xarm_ros2 `xarm_description` | commit ac736c92b02de7ca7e0be6f737d77d46707464d8 (humble, 2026-10-20) | BSD-3-Clause |
| MuJoCo | PyPI `mujoco` | 3.17.0 | Apache-2.0 |

MuJoCo Menagerie has no xArm 6 (only xArm 7 and Lite 6), so the arm's MJCF is generated
from UFACTORY's data files. Assets are fetched by sparse checkout at the pinned commits,
read as data only (XML, YAML, meshes; nothing executed), into `runtime/sim-assets`
(`$RAX_SIM_ASSETS`), with per-file SHA-256 in each `SOURCE.json` (`assets verify`). The
layout matches mooncake's and `services/dex_eval/fingertip.py`, which finds the hand
model there with no configuration.

## Fidelity: assumptions and gaps

- **Torque per ampere** is assumed (MJCF peak at 3 A). Every contact force in the twin
  scales with it; mooncake's unscrew probe found cap turning hinges on it. Measure on the
  rig (known mass hung from a fingertip at a known cap) and set `torque_per_amp`.
- **Gain factor** 1.98 is applied to the whole law, before the cap; whether the hardware
  clamps before or after the factor, and whether torque follows the reported current,
  is not known.
- **Friction** is a per-joint Stribeck model fitted to second-hand reports, identical for
  all joints; no backlash, no position-dependent (cogging) torque, no encoder noise or
  `Enc1BitRate` warnings. The twin's slips peak at 0.3-0.45 rad/s, below the rig's
  1-1.5 rad/s: the MIT kd damps them more in simulation (rotor armature from the MJCF).
- **Screw threads** are kinematic: lift and turn are coupled exactly, so there is no
  cross-threading, jamming, wobble or thread-start search; engagement is a geometric
  rule (aligned within 2.5 mm/8°, 0.2 rad clockwise); thread friction is a constant
  running torque with a breakaway, independent of axial load; seating is a hard stop
  with no tightening torque; tamper bands are not modelled. The turn joint carries
  5e-5 kg m² of armature for numerical stability (negligible next to its friction).
- **Contact** uses convex hulls of the hand and arm meshes (larger than the real
  surfaces in concave places), elliptic friction cones, assumed fingertip friction
  (1.0, torsional 0.005 m); no soft pads, no tactile sensing (as on the rig).
- **Hand mount** orientation is assumed (thumbs toward the other arm; palms toward the
  bases at RAX's home joints) and the adapter thickness zero. One wrist-camera image of
  the rig settles it (`scene_options.hand_mount_yaw_deg`).
- **Arm** servo gains, collision sensitivity, Cartesian rotation speed, joint speed limit
  and payload compensation are assumed; the table top is the base plane; the floor and
  lighting are generic.
- **Thermal** and **stall** models are assumed; firmware 2.13.1 has no derating.
- **Cameras** render undistorted pinhole images with no noise, exposure, motion blur or
  depth artefacts (holes, flying pixels); depth is exact z-depth.
- **Timing**: physics is paced to wall-clock time; under heavy host load the server
  lags (it reports `overruns`), which a hand service sees as slower motion rather than
  stale feedback.
- **MPC model**: Coulomb-only friction, capsule collisions and 2 ms steps; plan there,
  verify on the twin, then on the rig.

## Files

`services/sim_xarm_wuji/`: `assets.py` (pinned fetch/verify), `spec.py` (facts, defaults),
`rig.py` (calibration geometry), `model.py` (MjSpec builders), `scenes.py` (objects,
threads), `firmware.py` (Wuji joint drivers), `xarm_controller.py` (controller box),
`world.py` (the 1 kHz world), `server.py` + `rpc.py` + `shm.py` (the device process),
`wuji_sdk.py` + `xarm_api.py` (SDK mimics), `hand_service.py`, `camera_server.py`,
`backend.py`, `config.py`, `calibration.py`, `operator.py`, `session.py`, `mpc.py`.
Hooks: `services/backends.py` (key `xarm_wuji_bimanual_sim`; `simulation` selects it for
`xarm_bimanual`), `services/xarm_bimanual/config.py` (accepts and validates
`simulation`), `services/xarm_bimanual/operator.py` (arm bring-up, stop and status on the
twin), `services/controller/config.py` (validates the sim key). Example:
`examples/xarm-wuji-bimanual-sim.json`. Tests: `local_tests/unit/test_sim_xarm_wuji.py`.
