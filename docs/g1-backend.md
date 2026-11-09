# G1 EDU 29-DoF with two Wuji hands

`real/g1` starts from `real/piper` commit `924ee0e0768604a46125f77a799dc081a80ea080`.
This first implementation provides stock-controller bounded locomotion, measured
body state, RGB bridge observations and independent left/right Wuji control.
It does not yet implement Cartesian arm planning, stand-up/controller transitions,
localization, obstacle avoidance or coordinated walking and reaching. Those tools
are deliberately absent from discovery and marked unavailable in capabilities.
No physical G1 or simulator validation has been performed.

## Ownership and interface

`services/g1/backend.py` implements the existing `tools`, `validate`, `call`,
`status`, `close` contract. The MCP frontend, CLI bridge, agent harness and browser
assets are unchanged. The existing backend and workspace fact registries receive
G1 entries; the session configuration script validates G1 options. Robot-specific
agent instructions live in the existing control skill layer.

A backend-owned thread samples low-state and the stock locomotion FSM and sends
`SetVelocity(vx, vy, yaw_rate, duration)` with a finite lease. MCP may block while
waiting for the bounded command, but operator disarm and feedback checks continue
in this thread. At completion it sends zero velocity. It never publishes raw
joint commands, disables balance, or silently switches controller modes. Motion
errors latch until the service is restarted and the operator restores readiness.
The native lease must be verified on the target firmware before commissioning;
software cannot guarantee a stop when the robot is unreachable.

The SDK adapter checks RPC return codes, DDS CRC, packet progression (including
32-bit tick wrap), feedback age, finite joint/IMU readings, motor fault words,
roll/pitch bounds and commissioned FSM IDs. A process lease enforces one local
RAX owner per operator-specified robot identity. It does not exclude unrelated
vendor clients or verify the robot serial over DDS. Use an isolated robot network.

Velocity units are m/s, yaw rate rad/s, x forward, y left, positive yaw left.
A successful result means the command window completed and a zero command was
acknowledged. It does not claim distance travelled, target arrival, verified
standstill, collision freedom, or grasp success.

## Tools

- `humanoid_status`: body, controller worker, capabilities, arming and hands.
- `humanoid_move_base(vx, vy, yaw_rate, duration_s)`: bounded body-frame command,
  at most two seconds, default limits 0.15/0.10 m/s and 0.20 rad/s.
- `humanoid_stop`: request zero locomotion and hold both hands, reporting each
  result. Stock balance remains active. MCP serialization means this tool itself
  cannot preempt another MCP call; operator disarm is independent.
- `humanoid_observe`: operator-configured HTTP JPEG snapshots, current body state,
  saved frame evidence in the existing session frame layout. Receipt timestamps
  are not calibrated exposure times. No cameras means an empty image list.
- `humanoid_stream(seconds_ago)`: review the last 32 explicit observations. It is
  not a continuous recording; returned robot state is current, not historical.
- `wuji_status`, `wuji_move_joints`, `wuji_pose`, `wuji_hold`: mandatory explicit
  `hand: left|right`. Missing profiles are visible in status and cannot move.

Both configured hands must be prepared and fresh before locomotion or hand
motion. Wuji faults stop locomotion; body faults during hand motion revoke arming.
The existing hand controller owns its independent watchdog, native feedback,
trajectory limits, diagnostics, partial-joint policy and stop behavior. A stop
cannot guarantee retention of an object.

## Run without hardware

Install the ordinary host dependencies and use this configuration:

```bash
python scripts/configure_session.py --robot g1 --task g1_smoke \
  --backend-options examples/g1-dry-run.json --observation-profile rgb-only
```

Use the existing session launcher and operator arming commands with the printed
session path. `dry-run` performs no physics or balance simulation and cannot open
physical Wuji connections. It exercises the MCP contract and command lifecycle.
Its simulated joint readings are labelled in status. Hardware motion remains off
by default when no options file is supplied.

## Hardware configuration

Use Python 3.11, Unitree's Cyclone DDS installation instructions and
`requirements-g1.txt`. Start from `examples/g1-read-only.json`, supply the real
robot identity and network interface, and verify feedback before commissioning.
Unitree's SDK module initializes DDS globally; use one backend per process.

The `arms` object accepts:

```json
{
  "left": {"end_effector": {"type": "wuji_hand2", "handedness": "left"}},
  "right": {"end_effector": {"type": "wuji_hand2", "handedness": "right"}}
}
```

This is a structural sketch, not a runnable profile. Each `end_effector` also
requires its own serial, profile revision, twenty joint limits, current limits,
gains and validated posture definitions. Use the existing Wuji profile format;
never copy the xArm installation's calibration or diagnostic exceptions onto a
new device. `services.wuji.operator --session PATH --hand left|right` provides
operator-only status, preparation, disable and recovery commands. No MCP tool
enables a hand or changes a diagnostic exception.

Motion admission requires `motion_commissioned: true`, both complete Wuji
profiles, `wuji_mounts_commissioned: true`, and a nonempty `allowed_fsm_ids` list
verified on the target firmware. The mount flag records an operator qualification;
it is not a computed dynamic model. Each hand's mass/inertia, mounting transform,
wrist load and collision geometry must be established before that qualification.
No numeric FSM allowlist is assumed for hardware.

Cameras accept `{ "name": "overview", "url": "http://HOST/snapshot.jpg" }`.
The bridge must serve a fresh JPEG per request; the backend cannot independently
verify exposure freshness. RGB-D and extrinsics need a subsequent adapter.

Set `RAX_ROBOT=g1` and `RAX_BACKEND_OPTIONS` when using the existing dashboard;
select RGB-only. Its server uses G1 snapshots and disarm handling instead of
Franka endpoints. The unchanged browser panel assumes one TCP and scalar gripper
width, so it displays unavailable-field warnings for G1 and Wuji. It is not a
complete humanoid status view. Full telemetry is available through MCP and
`operator/g1-state.json`.

## Reuse and next implementation stages

Wuji source was imported from `real/xarmwujibimanual` commit
`d1e815a9003be7121b787909727588c9ae55df49` (`services/wuji/`). Only the arming import
was localized, the atomic arming helper copied, and the operator CLI extended for
G1 side selection. Reused tests cover its controller, SDK boundary, client,
effort, trajectory and velocity guards; disarm assertions reflect the controller's
existing intentional-disabled behavior.

RAX `humanoid-arena/native` at `a4abb550857803612569f25922e2221d29f00b5c`
and its container counterpart are the existing G1 simulation references. Their
SONIC semantic40 controller and Dex3 model cannot be dispatched directly to
stock G1 locomotion or Wuji hands.

Official upstream references inspected for this implementation:

- [Unitree Python SDK](https://github.com/unitreerobotics/unitree_sdk2_python/tree/814556d15970dd2ecf1c9984e845ca02ab07e206),
  pinned in requirements. Use acknowledged `SetVelocity`, since `Move` discards
  its return code in this version.
- [Unitree arm SDK example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp):
  `rt/arm_sdk` is the intended next-stage arm transport alongside stock locomotion.
- [XR teleoperation](https://github.com/unitreerobotics/xr_teleoperate): reference
  for arm IK and motion-mode coexistence, subject to its controller restrictions.
- [Official Isaac Lab simulation](https://github.com/unitreerobotics/unitree_sim_isaaclab/tree/f42c0819cbd444fe6894cc822e0e1dcc33f3b733):
  G1 Wholebody tasks use `rt/run_command/cmd` with velocity and height. This is
  not the stock hardware RPC and needs a separate adapter plus simulator-side
  command expiry. That adapter is not implemented here.
- [Unitree MuJoCo](https://github.com/unitreerobotics/unitree_mujoco): alternative
  low-level DDS test environment, not assumed to implement stock locomotion RPC.

Next: integrate the Wuji-equipped model and official physics simulator adapter;
then implement bilateral palm goals with IK, complete collision geometry,
stationary-base admission, arm ownership blending and measured tracking through
`rt/arm_sdk`. Finally qualify coordinated walking/reaching with an appropriate
whole-body controller. Navigation additionally needs localization and obstacle
handling. None of these pending capabilities are represented by dummy tools.
