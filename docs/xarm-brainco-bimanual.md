# Two xArm 6 arms with BrainCo Revo 2 hands

Select `robot="xarm_bimanual"` and provide `backend_options.arms.left` and `.right`.
Each contains the existing xArm options plus `end_effector.type="brainco_revo2"`.
The original `xarm` backend still supports Wuji Hand 2 and the native grippers;
it also accepts a single Revo hand. Existing Wuji sessions keep their tool contract.

## Connections found locally on 2026-11-01

| Pair | Arm IP | Revo serial | CAN-FD address |
| --- | --- | --- | --- |
| Left | 192.168.100.14 | BCXTL2204J2600021 | DaMiao channel 1, ID 126 |
| Right | 192.168.100.16 | BCXTR2362J2500015 | DaMiao channel 1, ID 127 |

Sources: the local `rc-packaging/xarm-revo-teleop/config/{left,right}.yaml` and
`hand_bridge/hardware.{left,readonly}.json` pairing records. USB sysfs also reports
two `34b7:6632` DM-USB2FDCAN adapters, serials
`648EA356AB3B7498191305EDEA12B65A` and `1D59B9EBB95A640A28FD34CA64E1483D`.
The USB adapter-to-side assignment has not been freshly verified. Discovery checks
the **hand serial**, never assumes enumeration index 0/1 means left/right, and locks
each selected adapter against another RAX owner.

The left pairing record dated 2026-10-31 reports adapter firmware `dual_app v1.0.0.3`
without FC10 write acknowledgement. The profile requires `dual_app v1.0.0.7` and
refuses mismatches; it does not upgrade firmware or weaken acknowledgement checks.
These are local records, not a fresh live hand/arm probe. This implementation session
could read sysfs but could not open host network/USB devices under its sandbox policy.

## Native API mapping

The adapter imports the official `bc_stark_sdk.main_mod`, pinned to 2.0.9. The
DaMiao module only transports and validates native SDK transactions. It does not
depend on the old teleop application or hand-control scripts. The existing installed
`libdm_device.so` is a native dependency; its example path can be relocated.

| MCP tool | Native API / behavior |
| --- | --- |
| `revo_status` | `get_device_info`, `get_motor_status`, read-only unit/protection/turbo checks |
| `revo_move` | `set_finger_positions_and_durations` or `set_finger_positions_and_speeds` |
| `revo_pose`, `revo_home` | Configured six-motor posture through the same native position controller |
| `revo_hold` | `set_finger_speeds([0]*6)`, then measured position hold and standstill verification |
| `revo_set_speeds` | Optional signed native speed command for a finite duration, followed by hold |
| `revo_set_currents` | Optional signed native current command for a finite duration, followed by hold |
| `revo_touch` | Optional `get_touch_sensor_enabled` and five native `get_single_touch_sensor_status` reads |

All six channels are ordered thumb flex, thumb rotation, index, middle, ring, pinky.
Positions are normalized 0..1000; speed/current are signed normalized values. They
are not Wuji radians, amperes, Cartesian gripper width, or calibrated contact force.
The adapter verifies normalized mode rather than changing it. It also verifies the
expected serial, firmware, protection currents and disabled turbo. The hand's
existing controller handles finger coupling; no 20-joint Wuji mapping is invented.
No firmware gesture, reset, calibration, protection-write or kp/kd tool is exposed.

The native CAN-FD motor writes use registers 1010 (position/time), 1022
(position/speed), 1034 (speed), and 1040 (current). The transport authorizes only the
exact validated transaction, checks CRC and the matching register-range write ACK,
and rejects repeated writes. A missing ACK leaves execution uncertain; it is never
reported as measured completion or automatically retried.

Vendor references: [Python API](https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/python_sdk.html),
[protocol registers](https://www.brainco-hz.com/docs/revolimb-hand/en/revo2/modbus_foundation.html),
[native examples](https://github.com/BrainCoTech/brainco-hand-sdk/tree/main/python/revo2).

## MCP contract

```python
ctx.value('xarm_status')  # both arms and hands
ctx.value('revo_status', arm='left')
ctx.value('revo_move', arm='right', positions=[150, 0, 0, 0, 0, 0], duration_ms=800)
ctx.value('xarm_move_pose', arm='left', position=[0.01, 0, 0], relative=True)
ctx.value('xarm_observe', arm='right')
ctx.value('xarm_hold')  # disarms and independently stops all four devices
```

Every routed tool requires an explicit arm. `xarm_move_bimanual(left={...},
right={...})` accepts the usual Cartesian arguments for each arm. Both targets are
validated before concurrent dispatch. This is not hardware-synchronized motion.
All positions/quaternions/workspace boxes use the **selected arm's base**. There is
no implicit shared-world transform. Pixel lookups reject frames captured for the
other arm, and images/metadata label their arm frame.

Shared fixed cameras use one capture process, the same name/serial/capture settings
in both profiles, and separate extrinsics for each base. A wrist camera belongs to
one arm and cannot be shared as a fixed camera. Do not reuse the Wuji wrist or base
calibration after changing hand mounting or robot placement. The example intentionally
leaves camera identity and calibration paths to be filled from the new rig.

Measured settling is required. `completion="contact"` can report stationary
obstruction with `reached=false`; this is not grasp verification. Timed speed/current
controls are opt-in and end in measured hold, including on disarm; they do not provide
persistent force while carrying an object. `revo_home` requires a commissioned `home`
posture. The example supplies no invented hand home.

## Installation and checks

```bash
uv pip install --python "$RAX_PYTHON" -r requirements-revo2.txt
"$RAX_PYTHON" scripts/check_xarm_revo.py --inventory-only
"$RAX_PYTHON" scripts/check_xarm_revo.py --options examples/xarm-brainco-bimanual.json
"$RAX_PYTHON" scripts/configure_session.py --robot xarm_bimanual \
  --backend-options examples/xarm-brainco-bimanual.json --task bimanual_task
```

The check script only reads identity/state and closes connections; it never enables,
homes, changes modes or commands a motor. Run it on the robot host with device
access. Install the SDK in the frontend's robot Python environment. The native hand
worker runs in a separate process per hand so SDK callbacks and native crashes do not
cross-route hands or kill the arm controller process. Separate direct Modbus ports
are also supported (`transport="modbus"`, `port`, `baudrate`); sharing one RS485 port
between hand workers is rejected.

Configure measured per-arm workspaces, TCP/payload in the xArm controller, camera
extrinsics and hand postures before supervised motion. Then use the existing
`scripts/arm_robot.py SESSION arm --minutes 5` to arm the whole session. The parent
monitor checks both assemblies and a fault/disarm stops all four devices. Worker
commands also check the same arming file while moving; cached telemetry during a
command is timestamped and rejected if stale.

The inherited arm limits are individual-arm checks, **not inter-arm collision
avoidance**. Account for both entire arms, mounted hands and objects. Native library
hangs, USB disconnects and power loss can prevent a software stop; each result
reports whether stop feedback was actually confirmed. No physical motion or grasp
qualification is claimed by the automated tests.

The generated agent workspace advertises the configured Revo tools and separate
arm frames. The existing dashboard's single-arm operator controls and Wuji preparation
scripts are not bimanual controls; use the bimanual session MCP/CLI.

## Verification

The focused backend/workspace regression run passed 425 tests, with the existing
socket integration test excluded because this execution sandbox forbids sockets.
All 39 new Revo/bimanual/transport tests passed. They cover native API arguments,
no writes on read-only startup, side routing, prevalidation of both trajectories,
write ACK/CRC and no retry, disarm, background faults, stale feedback, all-device
stops even when the arming file cannot be written, frame separation and agent tools.

The broader suite passed 936 tests and skipped one; 61 existing tests failed or
could not set up because socket creation/binding was denied. Static checks and
`git diff --check` passed. The SDK was not installed or exercised against live
hands in this sandbox; complete the read-only host check before motion qualification.
