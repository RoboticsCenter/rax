# xArm + Wuji Hand 2

The `real/xarmwuji` branch retains the xArm native motion planner, camera service,
recordings and session MCP. Select `end_effector.type: "wuji_hand2"` to replace the
G2 tool with twenty-joint hand control. Existing G2 configurations continue to work.

The implementation has hardware-free unit and process tests and an initial
[real-hand MCP test record](wuji-live-test-2026-10-25.md). Basic joint motion,
target holding, a recorded posture and confirmed disable were exercised. Full
manipulation and fault qualification remain pending. The local October 22 vendor
self-check is separate historical evidence. The research and design rationale are in
[xarm-wuji-backend-plan.md](xarm-wuji-backend-plan.md).
The [October 27 backend verification](xarm-wuji-live-mcp-2026-10-27.md) exercises
all eleven tools with the calibrated cameras. xArm checks passed after corrections;
the hand's nonzero positioning trial failed its existing settling tolerance.
The [October 28 v13 qualification](wuji-rax-qualification-2026-10-28.md) now
passes five saved home/fist commands and disarm during actual finger motion,
with both cameras streaming. The operator reported smooth, acceptable motion.
See the [cube task handoff](xarm-wuji-cube-handoff-2026-10-28.md) for the next run.

## Environment and configuration

Use a Linux Python environment containing the existing xArm/RealSense dependencies
(`xarm-python-sdk`, `pyrealsense2`, `numpy`, `scipy`, `opencv-python` or its headless
variant, `pyyaml`, `jinja2`, `pillow`). Install the pinned hand SDK and S-curve planner into that same
environment:

```bash
export RAX_PYTHON=/path/to/robot-env/bin/python
"$RAX_PYTHON" -m pip install -r requirements-wuji.txt
```

Python 3.12 with `wuji-sdk==2026.10.12` matches the successful local hand deployment.
The backend checks the SDK version at connection. It does not import or connect
the SDK when generating workspaces or validating configuration.
S-curve preparation also checks `ruckig==0.19.4` before any motor write.

On this host, `/home/jerry/rax/.venv/bin/python` is prepared with that SDK,
`xarm-python-sdk==1.18.4`, `pyrealsense2==2.59.1.11053` and the test dependencies.
Use it as `RAX_PYTHON` for the commands below. Installing these packages did not
connect to either device.

Video recording requires FFmpeg with `libx264`. The recorder searches PATH, then
the directory containing the active Python interpreter. This host uses FFmpeg
7.0.2 from `imageio-ffmpeg==0.6.0`, linked at `.venv/bin/ffmpeg`. If system FFmpeg
is unavailable, the equivalent environment-local installation is:

```bash
"$RAX_PYTHON" -m pip install imageio-ffmpeg==0.6.0
"$RAX_PYTHON" - <<'PY'
import sys
from pathlib import Path
from imageio_ffmpeg import get_ffmpeg_exe
Path(sys.executable).with_name('ffmpeg').symlink_to(get_ffmpeg_exe())
PY
```

Create that link only when it does not already exist. Motion archives include
camera videos and robot/hand telemetry for successful and failed motion commands.

Copy [examples/xarm-wuji.json](../examples/xarm-wuji.json) into your operator
configuration location and review it before running:

- The example targets the observed arm at `192.168.100.16`, wrist D435
  `262422074331`, and fixed side D435 `348522070593`. Camera references live in
  `calibration/xarm-wuji/`; the G2 transforms are not used. Follow the
  [bring-up checklist](xarm-wuji-bringup-checklist.md) and installed
  [camera calibration report](../calibration/xarm-wuji/README.md).
- It selects right hand `WH2KA01260823017` at `192.168.1.111:50001`. Connection verifies
  serial and side even when the endpoint is supplied. Set `address` to null to
  discover by serial. The endpoint was verified after the
  [firmware 2.13.1 upgrade](wuji-firmware-upgrade-2026-10-27.md); earlier firmware
  2.6.0 sessions used port 7447. Generate new sessions from the current config.
- Its joint bounds follow the published Hand 2 SDK table. They are a commissioning
  profile, not a measured SDK-to-model mapping or self-collision model. Check actual
  joint directions, origin, mounting orientation, TCP, payload and swept clearance.
- The example selects the [v13 S-curve profile](wuji-scurve-2026-10-28.md):
  Kp 8/Kd 0.15 and 1.5 A caps on all twenty joints, with 1 kHz position/velocity
  commands from local Ruckig trajectories. Reference velocity, acceleration and
  jerk limits are 0.6 rad/s, 3 rad/s² and 2 rad/s³. The operator approved this pace
  and gentler starts/stops in the standalone hand test. The RAX integration is
  tested with simulated hardware and now qualified on the saved home/fist paths
  through live RAX. Five commands passed with a worst endpoint error of
  0.038747 rad. Empty-hand qualification does not establish a cube grasp.
  The example explicitly permits only SDK-classified `0x0003` / `Enc1BitRate` /
  `Warning` / `AutoClear`, retaining those events in status. This carries over the
  standalone operator setting; other configurations stop on all nonzero codes by
  default. The encoder issue remains unresolved. See the linked integration note.
  The previous
  [v12 profile](wuji-feedback-motion-2026-10-28.md) used Kp 6/Kd 0.15 on the
  thumb and Kp 5/Kd 0.15 on the other sixteen joints, with 1.5 A caps throughout.
  Feedback-paced trajectories allow stable 0.060 rad endpoint error and up to
  eight extra seconds. Reference speed/acceleration/jerk limits are
  0.12 rad/s, 0.2 rad/s² and 0.8 rad/s³. The per-command displacement limit is
  2.1 rad, enabling full continuous moves on reviewed paths.
  Five small-motion commands and a full arbitrary-pose → fist transition passed
  with these gains; fist error was 0.059642 rad on thumb S2. Softer gains reduced
  measured burst counts in the small-motion comparison, but the operator still
  observed jerkiness on the continuous large move. Smoothness remains unresolved.
  The hand ended disabled; passive retention of the fist is not claimed.
  The preceding [v9 test](wuji-ring-damping-2026-10-28.md) passed one broad
  random-home cycle and stopped the next initialization on pinky S2 error
  of 0.054443 rad. The new tolerance admitted that same target at 0.052530 rad;
  it did not eliminate the static offset.
  The previous [v7 profile](wuji-per-joint-profile-2026-10-27.md), with the thumb
  at 8/0.15, passed all nine near-home whole-hand commands;
  worst error was 0.02223 rad. Measured speed bursts remain, so smoothness is
  still unresolved. The subsequent
  [broad random-pose test](wuji-random-home-2026-10-27.md) failed during its first
  initialization on thumb S2 (0.042127 rad error); no random-home cycle completed.
  The hand was disabled away from home and v7 was retained. The preceding
  [whole-hand test](wuji-uniform-profile-2026-10-27.md) at Kd 0.15 passed eleven commands,
  including a staged home return and two short repeats, but the operator reported
  jerky motion. Increasing Kd to 0.20 modestly reduced measured burst counts in
  two comparisons; peak-speed ranges overlapped. Smoothness remains unresolved.
  RAX applies profiles only on explicit `prepare`. The earlier profiles were also
  read back on disabled hardware. Current is not fingertip force. Gains/current
  limits are read back for every joint before enabling; partial failure refuses
  preparation and requests stop. It does not silently restore unknown old settings.
  The [October 27 gain trials](wuji-gain-tuning-2026-10-27.md) found that lower
  thumb damping helped some moves but did not resolve thumb S2 stalling or qualify
  repeatable home. The later [vendor-profile test](wuji-vendor-profile-test-2026-10-27.md)
  on firmware 2.13.1 also failed thumb-home settling at Kp 5/Kd 0.1/1.5 A.
  The [expanded parameter study](wuji-parameter-sweep-2026-10-27.md) subsequently
  improved repeated thumb-home error to 0.00864–0.00984 rad at Kp 15/Kd 0.15,
  and all twenty joints passed isolated local motions. Coordinated tests stopped
  on a ring S1 encoder-quality warning, which auto-cleared while disabled.
  A [guarded resumption](wuji-shortlist-validation-2026-10-27.md) produced the same
  warning on ring S3 and stopped that comparison. The later moderate-gain ring
  sweep completed 38 returns without detected repeated oscillation or encoder
  warnings. Its earlier ring-only selection was superseded by the uniform
  whole-hand profile; the encoder warning's root cause remains unresolved.
  The [selected-profile home retest](wuji-home-retest-2026-10-27.md) stopped at
  an intermediate pose with thumb/pinky settling errors up to 0.045738 rad
  against the 0.040 rad tolerance, without an encoder event or oscillation stop.
  It also exposed a likely cached-mode race between consecutive MCP moves.
- `postures.home` contains the twenty measured angles selected by the operator
  on October 27, originally under profile `right-hand2-home-20261027-v2`. The
  current `right-hand2-scurve-20261028-v13` retains those same
  angles and the `fist` posture with the thumb outside the curled fingers.
  These are saved joint postures; returns from other task poses still need path
  qualification. There is no configured "open" or force-controlled "grasp"
  posture. Change `profile_revision` when changing limits,
  gains, origin conventions or postures.
- The optional `lock_paths` entry coordinates with the local teleop checkout.
  Remove or update it on another host. Parent directories must exist. Also stop
  vendor demos/Studio and the separate arm-hand adapter before RAX control:
  advisory locks cannot exclude programs that ignore them.

The hand uses its own Ethernet/power connection. The example does not change
network routes, firmware, calibration, xArm TCP, collision settings or payload.
The wrist camera's current extrinsics are TCP-relative; a changed TCP needs an
updated transform even if the camera itself stayed fixed. Fixed camera transforms
remain valid only if the base and cameras stayed fixed.

## Start a session for MCP testing

Choose the artifact root as in the main README, then configure a session:

```bash
export RAX_BACKEND_OPTIONS=/absolute/path/to/your/xarm-wuji.json
"$RAX_PYTHON" scripts/configure_session.py \
  --robot xarm --task wuji_commission \
  --task-text 'Operator-led Wuji MCP testing' \
  --backend-options "$RAX_BACKEND_OPTIONS"
```

The command prints an `operator-input.json` path. To test the MCP directly, without
starting an agent, run the frontend in a terminal using that path:

```bash
export CONFIG=/absolute/path/printed/by/configure_session/operator-input.json
"$RAX_PYTHON" -m services.controller.frontend \
  --config "$CONFIG" --socket /tmp/rax-wuji-mcp-test.sock
```

The frontend starts a private hand process. Connecting reads identity and feedback;
it does not enable motors, home fingers or clear faults. A connection failure is
reported by `wuji_status`, and hand-dependent motion is refused. The log is
`<session>/wuji-controller.log`. With streaming enabled, frontend startup also
starts the existing camera service. `stream_enabled: false` is useful for initial
hand-only checks; camera observations still connect cameras when requested.

Alternatively launch the normal agent using `scripts/start_agent.sh --config ...`.
Workspace generation automatically selects the Wuji tool contract and hand-control
skill. Do not start both frontends for the same session/device.

From another terminal at the repository root, inspect the tools and state:

```python
from auto_research_agent.api.runtime import Context

with Context('/tmp/rax-wuji-mcp-test.sock') as ctx:
    print(ctx.request('tools/list', {}))
    print(ctx.value('wuji_status'))
    print(ctx.value('xarm_status'))
```

These reads do not prepare or move the hand. For real movement, after checking
clearance and the stop setup, the operator explicitly readies the arm, arms the
session, and prepares the hand:

```bash
scripts/xarm ready
scripts/xarm arm 10
scripts/wuji prepare --session /absolute/path/to/session
scripts/wuji status --session /absolute/path/to/session
```

`scripts/xarm` targets the newest session and the configured backend options;
verify the printed session name. `scripts/wuji --session` always targets an explicit
existing hand owner. Preparation requires fresh fault-free feedback with all joints
disabled, applies/verifies the profile, seeds measured positions, enables, and waits
for all twenty joints to report Enabled. It does not open or zero the hand.

Arming expires after the requested duration. The separate process checks arming
and parent liveness while the agent is thinking or waiting for an operator.

## MCP contract

| Tool | Arguments | Result/behavior |
| --- | --- | --- |
| `wuji_status` | none | Measured/target positions, rad/s velocities, effort in A, diagnostics, independent stream ages, profile, limits, posture names, mode and current action. |
| `wuji_move_joints` | `positions`: 20 finite radians; optional `duration_s`, `completion` | Bounded trajectory. Default `position` requires target accuracy. Explicit `contact` also accepts a stable shortfall while preserving the commanded hold. |
| `wuji_pose` | `name`; optional `duration_s` | Same motion path using an operator-configured posture. Unknown names refuse before dispatch. |
| `wuji_home` | optional `duration_s` | Return to configured `home` through the custom movement controller, retaining active gains/damping and motion limits. Waits for measured settling. Requires preparation and arming. |
| `wuji_set_effort` | `effort_a`: 20 signed amperes; optional `duration_s` | Slew-bounded feed-forward current ramp, holding the position reference and MIT gains. Current bias persists after return. Not calibrated fingertip-force control. |
| `wuji_set_impedance` | One or more 20-element arrays: `kp`, `kd`, `current_limit_a` | Adjust active gains/current caps within the operator profile, with device readback. Requires zero feed-forward bias and a measured stationary prepared hand. |
| `wuji_hold` | none | Retain a stationary target; brake a moving S-curve to a bounded rest target. Falls back to emergency stop when disarmed/unreliable or bounded braking is unavailable. |
| `xarm_hold` | none | Disarm and independently stop the arm and hand. Clears `control_uncertain` only when both stops are confirmed. Does not clear controller errors or re-enable either device. |

Joint order is thumb S1–S4, index S1–S4, middle S1–S4, ring S1–S4, pinky S1–S4.
For intentional grasp closure, `completion="contact"` requires the reference to
finish and measured joints to remain stable with low velocity and fresh,
fault-free feedback. A residual above the position tolerance but within the
unchanged tracking guard returns `ok=true`, `outcome="held_short_of_target"`,
`reached=false`, and `contact_verified=false`. The controller keeps the target,
gains, damping, current caps and feed-forward bias. RAX does not stop the assembly
solely for this accepted result. Motion, stale feedback, hardware faults and
tracking-error violations are not accepted as contact completion. Home and named
postures remain position-accuracy moves. Retention must be verified separately.

There is no scalar width or `is_grasped` field for the hand. Wuji sessions do not
publish `xarm_gripper`. The arm tools retain their existing arguments, configured
TCP and quaternion order `[x,y,z,w]`.

Commands enforce configured angle limits, maximum step, velocity, acceleration,
tracking deviation and deadlines. An omitted duration is computed from the bounds;
an explicit duration that is too short is rejected. State and diagnostic streams
must both remain fresh. Missing or duplicated NIDs, nonfinite values, stale
sequences, reported faults and an unexpected enabled state prevent continued
execution. The hand keeps its target during arm motion; calls are sequential,
not a synchronized arm/hand trajectory.

### Arm completion and stalled moves

The default arm deadline is **60 seconds**, with Cartesian `timeout_s` overrides
from **1 to 120 seconds**. Joint moves share one deadline across all SDK segments,
also capped at 120 seconds. Completion still requires both Cartesian position and
orientation tolerance; reaching XYZ with the wrong wrist orientation is not success.

Outside tolerance, an idle, stationary controller fails after **2 seconds**. A
controller reporting motion but making no measurable movement fails after **5
seconds**. The monitor considers TCP translation, rotation and joint movement,
so a progressing in-place rotation is allowed. Each joint segment gets a fresh
progress monitor, while retaining the whole-call deadline. Failure cancels later
segments, requests verified assembly stops, disarms, and posts an operator request
directly through the backend. It does not wait for a second agent tool call to
request help and does not replay the target or relax pose tolerances.

### Effort and impedance semantics

The installed Wuji SDK 2026.10.12 exposes `JointCommand(position, velocity, effort)`
and per-joint `effort_limit` / `mit_params` resources. The vendor's
[teleoperation example](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/retargeting/1.teleop_real.py)
uses these resources with current limits in amperes. The installed SDK type stubs
also label the torque constant as a placeholder; neither SDK effort nor motor current
is a calibrated fingertip force. No newton-valued force command or tactile feedback
controller is claimed by this port.

`wuji_set_effort` streams signed feed-forward current through the same single owner
as joint motion. Its quintic ramp respects `max_effort_slew_a_s` (default **0.5 A/s**),
the configured maximum duration, and each active current cap. The position reference
stays fixed during the ramp; after success, the bias persists through subsequent
joint/arm moves, holds and agent pauses. Twenty zeros ramp the bias away. Measured
joint position, tracking deviation, velocity, feedback freshness, thermal/fault,
arming and parent-process guards remain enforced. Completion acknowledges the final
command write, not measured force convergence, contact or grasp success.

The operator-only `effort_velocity_grace_s` setting defaults to **0** (an immediate
stop above `max_velocity_rad_s`). It can be set up to **0.020 s** to tolerate a
brief measured-speed excursion under current bias. With grace enabled, speed at
or above **twice** the normal bound still stops immediately. Measured displacement
during the excursion is limited to `max_velocity_rad_s * effort_velocity_grace_s`;
reaching either that displacement or the grace deadline also stops. Missing new
feedback cannot extend the deadline. A late low-speed sample does not erase an
expired deadline. The reference speed, current caps and other guards are unchanged.
This is transient tolerance, not additional commanded hand speed or calibrated force.

Status exposes the effective bounds and retained triggering joint, velocities,
positions and source headers under `effort_control.velocity_guard`. `last_trip`
is historical after operator recovery; current `fault`/`mode` determine whether
the hand is healthy. Tolerated excursions are counted and the last event is retained.
Changing this profile setting requires restarting the owner; it is not an agent
tool parameter, and recovery never replays the previous bias.

`wuji_set_impedance` changes only the supplied arrays and keeps the target posture.
The operator profile's `kp`, `kd`, and `current_limit_a` are ceilings; positive kp
and current caps are required, so this does not expose an unconstrained current-only
mode. Clear feed-forward effort first and verify the hand is stationary. Every
resource write is read back, with stop/cancellation and scheduling checks between
writes. Partial failures stop the assembly and report uncertain control. Slow SDK
RPCs can trip the existing scheduling deadline; qualify this path on the mounted
hardware before relying on live gain changes during a grasp.

`wuji_status` separates active `control_parameters` from baseline
`profile_control_parameters`, and commanded `command_effort_a` from measured
`effort_a`. Real operator preparation restores the baseline and zero bias; merely
renewing arming on a healthy hold preserves current settings. Current removal,
lower stiffness or lower caps may release an object. `wuji_hold` retains the bias;
assembly stop/disarm disables the hand. Both new MCP tools are mutating actions
and produce the existing motion clips and audit records.

For an existing session, exit its Claude Code UI and let RAX shut down, then refresh
the copied tool contract and all provided skills before resuming:

```bash
"$RAX_PYTHON" scripts/refresh_wuji_tools.py --session "$CUBE_SESSION"
bash scripts/start_agent.sh --config "$CUBE_SESSION/operator-input.json" --resume
```

The refresh refuses a running session, backs up replaced skill/contract files, records
their new hashes, and preserves task text, conversation, memory, code, references,
custom skills and recordings. It starts no services. On resume, have the agent reread `MCP_SESSION.md` and the hand-control
skill; historical notes that gains were fixed describe the earlier interface. The
operator still performs preparation/arming through the existing controls.

The saved hand home is requested through MCP after operator arming and preparation:

```python
ctx.value('wuji_home')
```

New sessions created from `examples/xarm-wuji.json` include this posture; existing
session configurations are snapshots and do not reload edits to the example.
`scripts/xarm home` continues to reset only the arm. Hand `prepare` holds the
measured posture, so it does not automatically move to `home`.

`wuji_pose(name='home')` remains equivalent. Both use the same custom movement
controller as `wuji_move_joints`, including the configured S-curve trajectory,
gains, damping, current caps and measured settling. Neither resets encoders or
enables motors. Between training episodes, release/support the object and retreat
into clear space, then run `wuji_home` followed by `xarm_move_joints(preset='home')`.
Verify both measured results and fresh status before requesting a scene reset.
Do not force homing after a fault, uncertain state or blocked return path.

Arm joint moves, including the home preset, accept the complete target in one MCP
call. RAX validates the full path before moving and executes longer moves as SDK
segments bounded by `max_joint_step_rad` (1.2 rad by default), accounting for
endpoint tolerance. The agent need not split them into separate calls. Measured
feedback, arming and hand readiness are checked between segments; a stop, failure
or the original overall timeout cancels the rest. Arm speed and acceleration caps
are unchanged. This reduces agent round trips; it does not add collision planning
or qualify an unseen path through the workspace.

The home capture read the disabled hand without enabling motors. Its angles differ
from the earlier tested `test_start` posture by up to 1.00035 rad. Saving a target
does not qualify its return path. The current example admits up to 2.1 rad per
joint per call and chooses a duration that respects the reference limits; larger
commands are rejected. Calls follow one straight path in joint space, without
automatic collision planning or intermediate endpoint stops.

In the selected `scurve` mode, Ruckig produces jerk-limited position/velocity
references, initialized from the measured stationary hand during preparation.
Consecutive commands preserve the commanded position, velocity and acceleration.
At rest, movements are phase synchronized; endpoints have zero velocity and
acceleration. Each published tick advances one nominal millisecond, and missed
slots are skipped without catch-up bursts. Python/Linux scheduling is best effort:
inspect `command_timing` in status for observed rate and maximum gap. The selected
50 ms command-gap guard stops on long stalls. Completion also requires fresh,
stable measured positions within tolerance and low measured velocity. See the
[S-curve integration note](wuji-scurve-2026-10-28.md) for settings and validation.

In the older `feedback` mode, lag smoothly reduces path speed, and matching position and
velocity commands are sent with zero current feedforward. Completion requires
fresh positions inside tolerance with low motion over the full settling window.
The nominal duration plus `settle_timeout_s` bounds the entire call. Status exposes
path progress and speed scale, and results preserve the requested target and
measured error even when catching up times out. Persistent static error is not
guaranteed to disappear. The historical `timed` mode remains the default for
configurations that do not select a motion mode. See the
[implementation and hardware report](wuji-feedback-motion-2026-10-28.md).

During your tool tests, check invalid shapes/limits, disabled and disarmed admission,
small single-joint moves in clear space, posture execution, holding between calls,
fault/status reporting, and the two independent stop paths. Fresh observed
convergence is not proof of grasping; validate retention and release visually.

## Stop and recover

```bash
scripts/wuji stop --session /absolute/path/to/session
scripts/wuji disable --session /absolute/path/to/session
scripts/xarm disarm
```

The dashboard's explicit **Prepare + send** action also handles a retained
thermal-only fault. While disarmed, it requires fresh, stationary, disabled
feedback, zero native error codes and all joint temperatures below the existing
temperature limit for two seconds. It then reconnects the existing hand owner
once, checks fresh fault-free feedback, and continues normal preparation before
sending the reply. There is **no additional temperature margin** and the limit
is not raised. A still-hot, stale, mixed-fault or failed-reconnect state leaves
the reply pending and displays an actionable error instead of nested tracebacks.
Recovery evidence is saved in `operator/thermal-recovery-*.json`. This occurs
only on an explicit operator preparation request; there is no background reset
or replay of an interrupted grasp.

The operator hand commands reach the private hand service directly, bypassing a
long-running MCP dispatcher call. `scripts/xarm disarm` changes the arming files,
queues hand emergency stops and requests arm hold. Dashboard Disarm/Disarm All also
queue hand stops. Dashboard hand state comes from the owner's timestamped snapshot,
without opening another SDK connection.

All MCP calls are serialized, including `wuji_status`, `wuji_hold` and `xarm_hold`.
A hold call from a second MCP client therefore waits behind an active motion
call; it is not a preemptive stop interface. Use the operator's direct stop or
disarm path for interruption. The live concurrency test measured a 2.28 s wait
behind a 2 s stationary motion command; idle disarm independently disabled the
hand in about 42 ms in that test, not a guaranteed worst-case latency.

An emergency/disable result separates `sdk_returned` from `confirmed`. Confirmation
requires new complete feedback, disabled/stopped joint states and low measured
velocity. On the tested firmware 2.6.0 hand, the SDK emergency-stop call left joints
Enabled. The RAX emergency path therefore requests SDK emergency stop followed by
explicit disable, including an attempt to disable if the emergency RPC raises.
A healthy S-curve hold brakes the current reference to a nearby rest target within
the configured joint and derivative limits. If bounded braking is unavailable,
it requests emergency stop. A normal `wuji_hold` reports its rest target with
`confirmed: false`; it does not claim that the hand has physically settled.
If stop delivery or confirmation fails, the service reports uncertainty and does
not resume. A late native SDK return cannot resume an old trajectory.

Support held objects before disarming, disabling or ending a session: grasp
retention is not guaranteed. Software cannot deliver a stop over a failed link or
guarantee a deadline while native code is blocked. Hardware stop/watchdog behavior
and the independent hand supply require the planned physical tests.

After a resolved connection problem, `scripts/wuji reconnect --session ...` explicitly
reconnects without enabling, provided this owner has no unconfirmed active control.
Read state and perform explicit preparation again. Firmware updates, origin changes
and fault clearing remain outside the agent interface. If a stale service socket
remains after a crash, verify its process has exited before removing that socket;
the device-level lease prevents a second service from acquiring the same hand.

## Validation and recording

Run the hardware-free suite in an environment with pytest, the base dependencies,
and the pinned local planner from `requirements-wuji.txt`:

```bash
"$RAX_PYTHON" -m pytest -q
```

Wuji tests inject fake drivers, including in the actual child process and Unix
socket/MCP tests. They cover malformed/missing feedback, arming and heartbeat loss,
delayed native calls, command cancellation, stop failure, target retention,
workspace generation and G2 compatibility. No SDK demo or motion test runs
automatically. Some tests need permission to create local sockets.

For an operator-supervised hardware qualification of `wuji_set_effort` and
`wuji_set_impedance`, first exit the RAX session and put the empty hand in clear
space with the arm stopped and a person at the stop. Preview the exact test:

```bash
"$RAX_PYTHON" scripts/qualify_wuji_effort.py --source-session "$CUBE_SESSION"
```

After confirming physical clearance, add `--execute`. This starts an isolated
production MCP frontend, never an agent. It retains the measured posture, applies
±0.05 A to middle S3, removes the bias, briefly reduces that joint's gains/current
ceiling, restores v13, then disarms and verifies disabled feedback. An independent
monitor stops on stale/faulted feedback or excursion beyond 0.03 rad. No arm motion
is requested. Any failure ends the attempt without rearming; a failed impedance
write can leave partial settings, which must be inspected before further use.
The printed evidence directory contains the report, hand trace, MCP audit and
motion clips when enabled in the source configuration. This validates current
command delivery and bounded behavior, not calibrated fingertip force or grasping.

`xarm_observe` records a hand snapshot in each frame's `meta.json`; replies include
it too. Robot stream records carry `hand` instead of `gripper_width`. Hand motions
also produce the existing camera motion clips when recording is enabled. Inspect
sample ages and device timestamps; camera, arm and hand samples are not exactly
synchronized. Fingertip online flags are exposed, but raw tactile decoding and
force-controlled grasping are not implemented in this first port.
