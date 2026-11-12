# Two xArm 6 arms with two Wuji Hand 2 hands

Use backend `xarm_bimanual` on branch `real/xarmwujibimanual`. It composes the existing
`XArmBackend`, `XArmDriver` and Wuji controller rather than replacing their motion,
S-curve, current, impedance, encoder-warning or feedback logic. Backend `xarm`
still supports existing single-arm sessions with their original tool arguments.
The xArm motion implementation matches `real/xarmwuji` at `899f0c6`; its Wuji
adapter additionally preserves excluded references for partial-hand saved poses.
The added blanket arm-motion ban for partial hands has been removed from both
single-arm and bimanual routing. Disabled hand joints alone do not prevent arm
movement; the original controller, readiness and path checks still apply.

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
allowance. Its left profile now includes the physically tested partial-hand patch:
18 controlled joints, `thumb_S3` and `middle_S1` isolated, 200 Hz commands, and the
left hand's own saved home. On November 12, the operator reduced the current
ceiling from 0.8 A to 0.6 A on both hands after recurring heating. The operator
subsequently restored both hands to 0.8 A. The two excluded
left joints retain zero current and gains. Both temperature cutoffs are now 67°C
(left unchanged, right raised from 65°C at the operator’s request). Stiffness,
damping and motion limits are unchanged. Earlier physical movement and
calibration evidence below used a 1.5 A ceiling; it does not validate grasping
performance with the reduced limit. Restart the stopped session to load the
updated profile, then use the existing operator preparation controls.

The earlier diagnostic configuration used
`allow_init_current_calib_failed: {"middle_S1": "2.13.1"}`. That optional policy permits the exact
`0x3208 / InitCurrentCalibFailed / Fatal / NonClearable` diagnostic only on that
joint and firmware. The operator reported vendor advice that this can be a false
alarm; the exception itself does not establish that the hardware is healthy.
The current left example instead uses the two-joint isolation policy below and
leaves `allow_init_current_calib_failed` empty.

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
is the physically tested profile fragment for unloaded movements on the current left serial.
It is included in the current example under `arms.left.end_effector`; apply it
there when upgrading an older operator configuration, never to the right hand
or the top-level backend options. It excludes `thumb_S3`
and `middle_S1`, leaving eighteen active joints. It uses the original branch's
Kp 8 / Kd 0.15 / 1.5 A active control values with reduced trajectory limits.
Existing sessions retain their original profiles until explicitly refreshed.
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
Results identify `completion_scope: active_joints`. Named hand poses/home move
active joints and preserve excluded references. Partial mode rejects contact
completion and effort/impedance changes. Arm movement,
status and hold remain available through their existing readiness checks, and
operator stop/disarm still stop everything.
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
Both bimanual hand profiles now request 200 Hz. The former 1,000 Hz setting
delivered about 340 commands/s on this laptop, stretching the tick-based reference.
It also tripped a 50 ms host command-gap check while the right hand held still
with no native faults. The bimanual profiles now use a 250 ms host watchdog,
matching their unchanged 250 ms feedback-age limit. Host delays over 50 ms are
recorded in `command_timing.delays` without stopping otherwise healthy control;
the status includes the soft and hard thresholds and actual measured delay.
S-curve advancement remains one reference step per command, without wall-time
catch-up jumps. Gains, damping, current caps and motion limits are unchanged.
Fault notifications include the controller's specific reason and hand snapshot.
See the [real host-timing validation](wuji-bimanual-host-timing-2026-11-03.md) for
the induced delay, right-finger movement and four-camera holding checks.
See the [physical movement record](wuji-left-partial-live-test-2026-11-02.md) for
the measured eighteen-joint movements and the later native ring-S1 stall warning
that stopped the grouped finger sequence.
The [original-controls home test](wuji-left-original-controls-home-2026-11-02.md)
then completed three arbitrary-pose/home cycles with the original branch's
gains, damping and 1.5 A cap, without new native faults. Home was captured from
the left hand's current pose at the operator's request. These were bounded
offsets up to 0.16 rad from that home, not the earlier full-range right-hand sweep.
The later [extreme-pose tests](wuji-left-extreme-home-2026-11-02.md) exercised
opposing fingers and both alternating five-finger configurations, returning to
that saved home. The profile now also preserves the original branch's exact
`Enc1BitRate (0x0003 / Warning / AutoClear)` allowance. That warning recurred
during the successful repeat and remained visible without stopping movement.
Those early tests used a 0.2-rad per-call cap and staged moves at no more than
0.18 rad per active joint. The current left profile uses a 2.1-rad distance cap,
matching the original right profile, so a saved-home move can be submitted in
one call. Its 0.15-rad/s velocity, 0.5-rad/s² acceleration, 0.5-rad/s³ jerk,
gains, current cap and two disabled joints remain unchanged.

Intentional disarm and stop cancellation are normal stop transitions, not hand
faults. The service still verifies the physical stop and reports native faults,
missing feedback or an unconfirmed stop.
The November 3 live MCP check completed the left saved-home reference in 5.06 s,
with `ok: true`, `holding: true` and a nonfatal 0.01774-rad measured shortfall.
Both the four-device MCP stop and a direct operator disarm completed without
new fault requests. The right profile was unchanged; neither arm received a
movement target. The session's `operator/harness-fix-20261103/` contains the raw
responses, configuration backups, camera check and validation summary.

In the historical side-camera setup,
`unverified_camera_extrinsics: ["side_b"]` kept the replugged Side B image
available while withholding its old base transform and projection marks. The
other three views retained their calibrated geometry. Status reports
`rgb_extrinsics_partial`, with a separate status for every camera. After verifying
or recalibrating the moved camera, the operator can remove its name from this
list and reload the session backend.

The left fragment now includes the operator-selected measured `home`, and
`wuji_home`/`wuji_pose` preserve disabled references during both admission and
dispatch. On November 2, the shared bimanual backend was reloaded and physically
completed `wuji_home(hand="left")`, `wuji_home(hand="right")`, then
`xarm_move_joints(arm="left", preset="home")` and the matching right call,
sequentially. Left home reported a nonfatal 0.01784-rad shortfall and kept holding;
right home returned `reached`. Both arm readbacks settled at
`[0, -0.7, -1.2, 0, 1.9, 0]`, raising their TCPs about 0.119 m. Both hands stayed
fresh and holding without blocking faults; the excluded joints stayed Stopped
with zero gains/current limits. This supersedes the earlier detached-left runtime
state. Raw calls and final readbacks are saved under the bimanual session's
`operator/four-device-home-result.json` and `operator/four-device-home-final-state.json`.

Arm homes are now configured separately as `arms.<side>.home_joints` (six radians)
and reported by `xarm_status` under each arm. On November 3, the operator selected
the current left posture `[0, -0.349066, -1.745329, 0, 2.094395, 0]` for both arms:
the TCP is about 0.521 m above each base, higher than the earlier home above.
This was the first higher-home setting. Configurations without `home_joints`
retain the legacy home. Wuji finger homes are independent.
The live MCP check reached this home on both arms (largest reported joint error
0.001 rad). Right-hand home reached; left-hand home completed with an accepted
0.01825-rad active-joint shortfall after an initial host timing expiry and
reinitialization from measured standstill. All four final stops were confirmed.
Evidence is in the session's `operator/higher-home-20261104T035042Z/` directory.

The operator subsequently positioned each arm manually and saved distinct homes
in the current session and bimanual example: left `[-7, -10, -120, 0, 120, 0]`
degrees and right `[10, -10, -120, 0, 120, 0]` degrees. Configuration stores the
measured radians, with each TCP about 0.583 m above its own base. This capture
sent no motion targets. Evidence is in `operator/save-arm-homes-20261104T043216Z/`.

The included fake-hardware tests cover all three trajectory modes, isolation
violations, exact fault policies, native masks, zero-parameter readback and full-hand
stops. They do not establish safe operation with this hand's recurring hardware
faults; physical qualification remains separate.

## Cameras and extrinsics collection

| Role | Serial | RGB/depth capture profile |
| --- | --- | --- |
| `wrist_left` (left TCP) | `108322071838` | 640×480 at 15 FPS |
| `wrist_right` (right TCP) | `752112070450` | 640×480 at 15 FPS |
| `front` (fixed side view; existing ID) | `348522070593` | 640×480 at 15 FPS |
| `side_right` (fixed side view) | `263222071915` | 640×480 at 15 FPS |

Both `examples/xarm-wuji-bimanual.json` and the explicit
`examples/xarm-wuji-bimanual-four-camera.json` select this setup and
`calibration/xarm-wuji-bimanual-four-camera/manifest.json`. On November 11
(November 12 UTC), a joint 36-pose collection recalibrated all four cameras after
the mounts were moved. All four passed 19 fresh checks each against the frozen fit.
Maximum translation/rotation errors were 1.85 mm / 0.30° for `wrist_left`,
1.62 mm / 0.37° for `wrist_right`, 1.68 mm / 0.38° for `front`, and
3.01 mm / 0.41° for `side_right`. This supersedes the earlier staged calibration.
A later bump prompted a fixed-camera-only recalibration on November 12 UTC.
Using both unchanged wrists as stereo references, `front` and `side_right`
passed 12 training and 8 fresh verification captures each. Current fixed-view
maxima are 1.68 mm / 0.30° (`front`) and 1.69 mm / 0.28° (`side_right`), with
mean reprojection error at most 0.200 px and 0.347 px respectively. Wrist and
inter-base calibrations remain unchanged.
All four RGB extrinsics are verified. The old three-camera bundle is historical
and is not selected.

Each wrist entry has an explicit owning `arm`. One shared camera process owns
all configured streams. Agent skills derive camera names and overview from the
session; `front` remains the overview ID. Both fixed cameras must remain still
through calibration and use. Both wrists can provide stereo references when
shared features are visible.

The explicitly named three-camera and five-camera examples and their calibration
reports are retained for historical rigs. They are not the current installation.
Additional fixed views need unique names/serials and must remain explicitly
unverified until calibrated. Read per-camera status before using geometry.
A selected verified bundle must match camera serials, profiles, owning arms,
arm serials and TCP offsets. Keep each saved image's matching optical model and
capture-adjacent pose together.

`xarm_observe(arm="left" | "right", project=[[x,y,z], ...])` saves camera poses in
both bases and projects points in the selected base (default left). Saved image
coordinates and `K` are native pixels; `preview_scale` describes returned previews.
Status supplies `T_left_base_right_base`. Arm motion targets still use their own
arm's base. An uncalibrated configuration can omit `calibration_manifest` and
continues providing raw observations without transforms or projection.

Use `rgbd` to capture aligned depth on all four cameras at 640×480 and 15 FPS;
the examples record
rolling depth at 1 Hz, while each `xarm_observe` saves a fresh depth image from
every requested camera. Depth PNGs are uint16 millimetres (zero is invalid), aligned
to the saved RGB grid. Observation metadata includes the depth registration source
and scale. Factory depth scale/registration remain independently unqualified and
`xarm_pixel_to_position` is not exposed; see the agent's object-localization skill
for using depth with the verified RGB transforms. The
[bimanual extrinsics workflow](xarm-wuji-bimanual-calibration.md)
collects both wrist sweeps and derives fixed-camera and inter-base transforms
from one stationary ChArUco board, following the original single-arm script.
A full four-camera bundle must include both wrists, both fixed-view transforms
to each arm base, and inter-base alignment. Depth validation remains pending.

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
  --task bimanual --task-file /path/to/task.md --observation-profile rgbd
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

New launcher sessions retain exact operator prompts/replies, intervention notes,
raw Claude transcripts and imported demonstration files. See
[run logging and progress reports](run-logging.md) for importing a video and
exporting a timeline aligned with the existing motion clips. Motion recording
remains unchanged; the operator starts the Claude Code session manually.

## MCP contract

| Tool | Arguments / result |
| --- | --- |
| `xarm_status` | Both assemblies under `arms.left` and `arms.right`, including identities, limits and hand state |
| `xarm_move_pose`, `xarm_move_joints` | Existing payload plus required `arm: left|right` |
| All seven single-hand `wuji_*` tools | Existing payload plus required `hand: left|right` |
| `xarm_move_both` | `left` and `right`: ordinary pose payloads |
| `xarm_move_joints_both` | `left` and `right`: ordinary joint payloads |
| `wuji_move_both` | `left` and `right`: ordinary hand move payloads, including optional contact completion |
| `xarm_move_combined` | Any subset of `left_arm`, `right_arm`, `left_hand`, `right_hand`; one action per device |
| `xarm_hold` | Disarm, stop and verify all four devices; reports each independently |
| `xarm_observe`, `xarm_stream` | Shared camera names; calibrated observe accepts `arm` and `project`, and supplies both base transforms |

There is no implicit side or `both` selector on single-device tools. Paired calls
validate both targets and readiness before either starts, then run within one
serialized MCP request. Each side uses its original limits and motion monitor.
Results are per side, with dispatch skew reported; a clean result is not proof
of grasp, collision clearance, or synchronized motion. On a partial failure, do
not replay the pair blindly.

The combined tool accepts arm actions `{"pose": {<Cartesian payload>}}` or
`{"joints": {<joint/preset payload>}}`, and hand actions
`{"joints": {<20-joint payload>}}`, `{"pose": {"name": "home"}}` or
`{"home": {}}`. Optional hand duration/completion fields remain in the inner
payload and obey that hand's existing policy. Empty requests or multiple actions
for one device are rejected before dispatch. Omitted devices retain their targets.
All selected targets pass the original admission checks before workers start.
During same-side arm/hand execution, only that arm's worker accepts its healthy
hand in `executing` as well as `holding`; ordinary admission still requires a hold.
The original xArm planner, Wuji trajectory implementation, joint isolation, gains,
speed limits and continuous health checks remain in use.

Combined results are keyed by device, with host dispatch skew and per-worker
execution timing. This does not synchronize controller clocks or arrival times.
Execution failure requests stops across the rig and reports completed and failed
devices. The combined MCP call produces one motion clip through the existing
recording implementation. Separate concurrent MCP calls still queue serially.
See the [real combined-motion validation](xarm-wuji-combined-motion-2026-11-03.md)
for four-device motion, subset dispatch, preflight refusals, interruption handling
and the final return home.

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

### High-resolution camera-only preview

`scripts/xarm-camera-viewer --backend-options OPTIONS.json` defaults to
1280×720 color on all configured cameras, 6 fps, and JPEG quality 95 for
live viewing and corner scans. Depth remains enabled by default: native
640×480 depth is aligned to the 1280×720 color image; this does not create
additional depth detail. `--width`, `--height`, and `--fps` override the preview
profile. `--configured-resolution` uses the saved robot camera profile instead.
The four-camera USB setup was verified at 6 fps on 2026-11-12; at 15 fps one
of the two fixed cameras failed to deliver frames. Use the live status to verify
all four feeds after any USB or profile change.

These camera-only defaults do not overwrite the robot session's calibrated
640×480 profile. The saved calibration explicitly checks resolution and frame
rate; validate/install compatible calibration before migrating robot observations
to a new profile. A camera-only resolution override uses live factory intrinsics,
not the old profile's intrinsics or robot projection geometry.
