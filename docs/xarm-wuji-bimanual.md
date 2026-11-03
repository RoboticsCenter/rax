# Two xArm 6 arms with two Wuji Hand 2 hands

Use backend `xarm_bimanual` on branch `real/xarmwujibimanual`. It composes the existing
`XArmBackend`, `XArmDriver` and Wuji controller rather than replacing their motion,
S-curve, current, impedance, encoder-warning or feedback logic. Backend `xarm`
still supports existing single-arm sessions with their original tool arguments.

## Upstream references

The implementation follows the official RAX repository's component selection and
combined-command approach:

- [`real/rainbow`, commit 36c926c](https://github.com/robot-agent-exploration/rax/blob/36c926ca3cf15921c6fc746a4bf309c26b0aa3c4/services/rby1/backend.py):
  explicit arm/component selectors, aggregate state and one multi-component tool.
- [`dexjoco/native`, commit 4b577aa](https://github.com/robot-agent-exploration/rax/blob/4b577aa2ba0039ad01707ede236be9913274180e/services/dexjoco/tools.py):
  distinct state/action identities for the two arms and hands. Its Allegro joint
  layout and mirrored left-hand conventions do not apply to Wuji.

The two xArm controllers are independent. Paired calls provide concurrent dispatch,
not RB-Y1 whole-body planning, synchronized arrival or inter-arm collision checks.

## This installation

| Device | Address | Identity |
| --- | --- | --- |
| Left xArm 6 | `192.168.2.205` | `XI130510J56A06` |
| Right xArm 6 | `192.168.1.241` | `XI130505F56A0A` |
| Left Wuji Hand 2 | `192.168.1.110:50001` | `WH2JA01260811007` |
| Right Wuji Hand 2 | `192.168.1.111:50001` | `WH2KA01260823017` |

Studio: [left](http://192.168.2.205:18333/), [right](http://192.168.1.241:18333/).
The operator confirmed the arm sides on November 2, 2026.

Start from [`examples/xarm-wuji-bimanual.json`](../examples/xarm-wuji-bimanual.json).
Settings under `arms.left` and `arms.right` belong to that assembly; cameras and
recording settings are shared. Do not place single-arm settings at the top level.
Both hands need distinct serials, addresses and matching handedness.

The example preserves the right hand's existing control profile and encoder-warning
allowance. The replacement left hand's profile is **uncommissioned**, keeps its
own limits and has no saved postures. At the operator's explicit request, its
profile enables the existing encoder-warning policy and sets
`allow_init_current_calib_failed: {"middle_S1": "2.13.1"}`. This permits the exact
`0x3208 / InitCurrentCalibFailed / Fatal / NonClearable` diagnostic only on that
joint and firmware. The operator reported vendor advice that this can be a false
alarm; the exception itself does not establish that the hardware is healthy.

Native codes and states remain intact, with the permitted fatal diagnostic under
`diagnostic_exceptions` and `diagnostic_exception_history`, separate from encoder
warnings. The policy is off by default and cannot be changed through MCP. Other
faults, including the previously observed overcurrent, still block preparation.
Native enable confirmation, freshness, temperature, tracking, current ceilings and
stop handling remain required. Changing the firmware invalidates this exception.
Restart the disarmed service after an operator configuration change; changing a
policy does not erase an already latched fault. No profile, origin or posture is
inferred by mirroring the other hand.
The workspace boxes are starting configuration values, not a new bimanual clearance
measurement. Verify them and both swept volumes at commissioning.

## Partial-hand diagnostics

The optional `disabled_joints` profile field selects joints that must remain
unpowered. It is empty by default. `examples/wuji-left-partial-profile.patch.json`
is a profile fragment for small unloaded movements on the current left serial:
merge it into `arms.left.end_effector` in a separate operator configuration, not
into the right profile or the top-level backend options. It excludes `thumb_S3`
and `middle_S1`, leaving eighteen active joints. It uses the original branch's
Kp 8 / Kd 0.15 / 1.5 A active control values with reduced trajectory limits.
It is not installed in a bimanual session automatically.
Physical MCP commands on 2026-11-02 demonstrated movement on all eighteen active
joints with the two excluded joints continuously Stopped. Seventeen completed an
outward-and-return position test; ring S1 moved but missed the position tolerance.
The operator reported that visiting Wuji representatives supported testing the
remaining eighteen joints with the faulty pair disabled. Disabled joints are limp;
this is not a qualified grasping or bimanual manipulation profile.

Preparation requires all twenty fresh native states to be disabled. The SDK
boundary writes and reads back zero `kp`, `kd` and current limits for excluded
joints, sends a seed with zero velocity/effort, explicitly disables the excluded
mask and waits for newer disabled feedback before selectively enabling the active
mask. There is no fallback to whole-hand enable. During control, any excluded joint
leaving Ready/Stopped, any active joint leaving Enabled, stale/incomplete feedback,
or a new blocking fault triggers the existing whole-assembly stop.

`isolated_joint_faults` optionally pins an exact native code and firmware to an
excluded joint. The only supported diagnostic categories are `Enc1CrcPersistent`
and `InitCurrentCalibFailed`, with exact SDK severity/clear-policy checks. These
records appear under `isolated_faults` and `isolated_fault_history` with their
original severity and native codes. Temperature, power/overcurrent, unknown and
active-joint faults still block. An unexpected enable remains latched even if the
next frame returns to Stopped. Isolation cannot be combined with the older
`allow_init_current_calib_failed` admission exception.

`enabled` retains its native meaning: all twenty joints Enabled. Partial readiness
is exposed separately as `active_enabled`; `active_joints`, `disabled_joints` and
`partial_hand` make the controlled scope explicit. Raw measured positions are never
rewritten. Tracking and settling use only active joints; whole-hand stop confirmation
still requires fresh disabled states and measured standstill for all twenty.

Use `wuji_move_joints` with twenty entries copied from `target_positions`, changing
only active entries. Any change to an excluded entry is rejected. Its retained
reference is a wire-format placeholder, not a promise to hold the passive joint.
Results identify `completion_scope: active_joints`. Partial mode rejects named
poses/home, contact completion, effort/impedance changes and arm movement. Status
and hold remain available, and existing operator stop/disarm still stop everything.
The ordinary full-hand profile and MCP joint ordering are unchanged.

In partial mode, a completed trajectory that misses the position tolerance returns
`outcome: target_not_reached`, `ok: true`, `reached: false`, and `holding: true`.
This reports the measured shortfall without disabling the hand or stopping the
assembly. Continue an authorized finger sequence from the current target while
feedback remains fresh and the active joints remain enabled. An incomplete
reference, native fault, loss of feedback, isolation violation, or tracking-limit
violation is still an error. The example waits at most one additional second for
position settling; it does not require a precision qualification sweep before use.
The full-hand/right-hand completion behavior is unchanged.
The current fragment uses the original branch's 1.5 A cap on active joints;
excluded joints receive zero gains/current limits at the SDK boundary. The
initial 0.3/0.5 A trials encountered Stall at their current caps. The original
`real/xarmwuji` profile also stopped on native Stall warnings.
Its cleared-Stall recovery is retained: the active native warning must clear
before an operator reconnect. In partial mode, exact configured isolated codes
may remain on the disabled joints, so they do not block unrelated Stall recovery.
The partial profile requests 200 Hz: the observed 1,000 Hz configuration delivered
only about 340 commands/s on this laptop, stretching the tick-based reference and
causing its wall-time deadline to expire. Keep the rate within the host's measured
capacity instead of discarding the scheduling guard.
See the [physical movement record](wuji-left-partial-live-test-2026-11-02.md) for
the measured eighteen-joint movements and the later native ring-S1 stall warning
that stopped the grouped finger sequence.
The [original-controls home test](wuji-left-original-controls-home-2026-11-02.md)
then completed three arbitrary-pose/home cycles with the original branch's
gains, damping and 1.5 A cap, without new native faults. Home was captured from
the left hand's current pose at the operator's request. These were bounded
offsets up to 0.16 rad from that home, not the earlier full-range right-hand sweep.

The included fake-hardware tests cover all three trajectory modes, isolation
violations, exact fault policies, native masks, zero-parameter readback and full-hand
stops. They do not establish safe operation with this hand's recurring hardware
faults; physical qualification remains separate.

## Four cameras; calibration later

| Role | Serial |
| --- | --- |
| `side_a` | `263322070113` |
| `side_b` (replacement D435) | `262422074331` |
| `wrist_left` (D435i, dashboard Camera 1) | `108322071838` |
| `wrist_right` (D435, dashboard Camera 4) | `752112070450` |

The operator confirmed wrist ownership on November 2, 2026. Every wrist entry has
an explicit `arm` field. Update the serial and role together if a camera is replaced.
One shared camera process owns all four RealSense streams; both arm controllers
use it. Stop the standalone camera dashboard before starting a session that opens
the same cameras. A second process cannot capture cameras already owned by it.

RGB observations and rolling history work without extrinsics. An `rgbd` session
may additionally save camera depth, but no camera-to-base or inter-arm transform
is supplied. Old intrinsics/extrinsics files are refused for this uncalibrated
setup. `xarm_pixel_to_position` and observe `project` are intentionally absent.
Each arm's pose remains in its own base frame. Camera calibration is a later task.

## Start and prepare

Use one Python 3.12 environment for the xArm, RealSense and Wuji SDKs and the
pinned S-curve planner. From the repository root, the complete runtime can be
installed into the existing `.venv` with:

```bash
uv pip install --python .venv/bin/python -r requirements-xarm-wuji.txt
export RAX_PYTHON="$PWD/.venv/bin/python"
```

The dependency file pins the runtime versions used on this host. FFmpeg with
`libx264` must also be on PATH for video recording. Installation and imports do
not prepare or move the devices.

```bash
export RAX_BACKEND_OPTIONS="$PWD/examples/xarm-wuji-bimanual.json"
"$RAX_PYTHON" scripts/configure_session.py --robot xarm_bimanual \
  --task bimanual --task-file /path/to/task.md --observation-profile rgb-only
bash scripts/start_agent.sh --config SESSION/operator-input.json
```

The configuration script also selects `xarm_bimanual` automatically when an `arms`
configuration is supplied without `--robot`. Startup discovers and reads the hands;
it does not prepare, enable or home them. The operator prepares the existing owners:

```bash
bash scripts/wuji status --session SESSION --hand left
bash scripts/wuji status --session SESSION --hand right
bash scripts/xarm-wuji-ready --session SESSION
```

The readiness shortcut checks both hands before changing readiness, prepares both
arms through the existing operator path, arms one shared session, and prepares
each disabled hand at its measured posture. Healthy owned holds retain their target,
current bias and impedance. A failure disarms and requests four independent stops.
Individual hand lifecycle commands (`prepare`, `disable`, `stop`, `reconnect`,
qualified recovery) require `--hand` in bimanual sessions. They never create a
second SDK owner. `xarm ready` and `xarm home` refuse ambiguous bimanual use.

The operator dashboard shows both assemblies and its combined preparation and
disarm paths cover both arms/hands. Existing workspaces contain copied skills:
create a fresh bimanual session, or refresh a stopped bimanual session with
`scripts/refresh_wuji_tools.py --session SESSION` before resuming it.

## MCP contract

| Tool | Arguments / result |
| --- | --- |
| `xarm_status` | Both assemblies under `arms.left` and `arms.right`, including identities, limits and hand state |
| `xarm_move_pose`, `xarm_move_joints` | Existing payload plus required `arm: left|right` |
| All seven single-hand `wuji_*` tools | Existing payload plus required `hand: left|right` |
| `xarm_move_both` | `left` and `right`: ordinary pose payloads |
| `xarm_move_joints_both` | `left` and `right`: ordinary joint payloads |
| `wuji_move_both` | `left` and `right`: ordinary hand move payloads, including optional contact completion |
| `xarm_hold` | Disarm, stop and verify all four devices; reports each independently |
| `xarm_observe`, `xarm_stream` | Shared camera names; observe metadata includes both arm states and wrist ownership |

There is no implicit side or `both` selector on single-device tools. Paired calls
validate both targets and readiness before either starts, then run within one
serialized MCP request. Each side uses its original limits and motion monitor.
Results are per side, with dispatch skew reported; a clean result is not proof
of grasp, collision clearance, or synchronized motion. On a partial failure, do
not replay the pair blindly.

Both Wuji processes read the same arming file but have different socket, config,
snapshot and log paths (`wuji-left-*`, `wuji-right-*`). Either owned-hand fault or
arm-controller fault disarms and stops the whole rig. Both controllers are watched
even when recording is off. The operator stop path bypasses the MCP queue;
another MCP request cannot interrupt an active call.

## Hardware-free validation

```bash
.venv/bin/python -m pytest -q local_tests/unit
```

The bimanual tests exercise real backend routing with fake device interfaces,
paired admission and concurrency, partial failure, four-device stop confirmation,
shared arming with distinct hand owners, uncalibrated observations, and generated
workspace/MCP agreement. These tests do not qualify physical bimanual motion or
clear the left hand's previously reported fault.
