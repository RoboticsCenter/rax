# Real xArm + Wuji MCP verification — 2026-10-27

All eleven tools exposed by the current xArm/Wuji configuration were exercised
through the production MCP frontend on the real devices. The arm was tested
first; the operator was notified before deliberate finger motion began.

**The xArm checks passed after three corrections. Hand positioning did not pass.**
A +0.100 rad index S3 command ended 0.04655 rad short, outside the unchanged
0.040 rad tolerance. The backend reported failure and confirmed hand disable.
Returning to the recorded starting posture subsequently passed. The planned
twenty-joint sweep was stopped at this failure; no claim of all-joint or grasp
qualification follows from the interface coverage.

## Setup and scope

- xArm6: `192.168.100.16`, firmware 2.7.1, SDK 1.18.4.
- Right Wuji Hand 2: `WH2KA01260823017`, `192.168.1.111:7447`, firmware 2.6.0,
  SDK 2026.10.12. The hand route used `enx6c1ff7618956`, source `192.168.1.11`.
- Wrist and side D435 cameras used the installed calibration in
  `examples/xarm-wuji.json`, with 640×480 RGB-D and 6 fps recording.
- Session-only arm limits: 15 mm/s Cartesian speed, 50 mm/s² acceleration,
  0.15 rad/s joint speed, 0.5 rad/s² acceleration. Existing workspace and
  per-call bounds remained active.
- Hand profile remained 0.5 A, kp 1.5, kd 0.1, 120 Hz, 0.5 rad/s commanded
  velocity, 1 rad/s² commanded acceleration, 0.35 rad maximum step and
  0.040 rad settling tolerance. No gains, currents or tolerances were relaxed.
- The hand held measured angles during arm tests. The arm stayed in controller
  state 4 during the hand tests. `test_start` was a session-only posture copied
  from measured feedback; the main configuration had an empty posture map during
  this test. The later operator-selected home capture is recorded below.
- The test used `libx264` motion recording. Camera-only preview was stopped while
  MCP owned the cameras and restored at `http://localhost:8765/` afterward.

## Tool coverage

| MCP tool | Real result |
| --- | --- |
| `xarm_status` | Correct identity, pose, joints, cameras, hand status, arming and controller state. Mode now reports 0 with SDK 1.18.4. |
| `xarm_observe` | Both RGB-D cameras, saved RGB/depth/metadata, individual camera selection and projected base-frame points passed. |
| `xarm_pixel_to_position` | Converted detected board pixels from each camera to finite base-frame points; returned a per-pixel error for out-of-image coordinates. This exercised the installed geometry; it was not a new calibration. |
| `xarm_stream` | Latest images, historical frames and contact sheets passed. |
| `xarm_move_pose` | Relative +20 mm Z, absolute XYZ change with 3° orientation change, return, timeout handling and recovery passed. |
| `xarm_move_joints` | Explicit +0.030 rad base-joint change, return, actual `home` preset and return from home passed. |
| `xarm_hold` | Stopped the assembly; hand disable was feedback-confirmed and arm state 4 was observed through a separate SDK reader. Subsequent arm motion was refused until hand preparation. |
| `wuji_status` | Correct identity, twenty finite angles, advancing state/diagnostic streams and zero hardware faults. |
| `wuji_move_joints` | Validation and operator-interruption paths passed. The nonzero index S3 trial **failed settling**; details below. |
| `wuji_pose` | Session-only `test_start` over 2 s returned `reached`, maximum joint error 0.003834 rad. |
| `wuji_hold` | Healthy hold retained all twenty targets. Disarmed hold confirmed disable after the stop completed. A hold arriving during an already-running emergency stop was rejected; details below. |

G2's `xarm_gripper` is correctly absent for this configuration and was not tested
against hardware that is no longer fitted.

## Arm results and corrections

The corrected runs passed **35 read-only/admission checks and 33 motion/recording
checks**. These counts include expected rejection and controlled failure checks;
they are not counts of distinct trajectories. Every completed motion archive had
both camera videos and robot telemetry. The deliberately timed-out and interrupted
motions also produced archives marked `failed`.

The start/return TCP was approximately `[0.25310, -0.00134, 0.31412]` m. The
`home` preset is a different position, approximately `[0.23058, -0.00155, 0.43342]`
m. Its interpolated joint path was checked with controller FK before the real
home-and-return test. Final return was within 1 mm of the recorded start.

A 30 mm upward command with a one-second timeout reported a dispatched
`TimeoutError`, stopped the assembly and allowed recovery after preparation.
Independent operator disarm during another bounded arm motion also interrupted
the MCP call. The first separate SDK sample reporting state 4 arrived **73.85 ms**
after the disarm request in this trial. The hand was confirmed disabled as well.
This sampled timing is not a worst-case stop-time bound or a measured braking
distance.

The first runs identified and retained evidence of these problems:

1. **Controller mode was null.** SDK 1.18.4 exposes `mode` as a property without
   `get_mode()`. `XArmDriver.state()` now supports that property.
2. **Declared input constraints were incompletely enforced.** Unknown fields,
   invalid camera names, malformed vectors and negative numeric values sometimes
   reached execution or were accepted. The xArm validator now enforces its
   declared types, vector lengths, enums and bounds before dispatch, including
   rejection of booleans as numbers. It also checks stream history limits and
   incompatible history/contact-sheet arguments. Sixteen invalid-input cases
   and the SDK mode fallback have hardware-free regressions.
3. **Motion video encoding lacked FFmpeg.** The first successful arm movement
   could not finish its video archive. FFmpeg 7.0.2 was installed locally through
   `imageio-ffmpeg==0.6.0` and linked as `.venv/bin/ffmpeg`. The shared recorder now
   finds FFmpeg on PATH or beside the active interpreter, so invoking
   `.venv/bin/python` without activating the environment works. The initial
   failed archive was recovered from saved frames; its original failure log
   remains. Subsequent real arm and hand motion archives completed.

## Hand results and unresolved positioning

The hand run recorded **34 passing checks and one failed positioning check**.
Twenty-two input/admission cases passed, including malformed targets, unknown
postures, disarmed and unprepared motion, excessive step and infeasible duration.
Healthy hold preserved every commanded joint target across two idle seconds.

Independent operator disarm interrupted a two-second MCP hand command that held
the existing posture. The first fresh snapshot with hand disabled and ownership
released appeared **114.59 ms** after the request. This checks interruption of an
active hand command, not stopping fingers with substantial momentum. MCP requests
remain serialized; operator disarm uses the independent private interface.

The nonzero automatic-duration probe requested index S3 from approximately
0.654193 to 0.754193 rad. At settling timeout, its encoder read 0.707646 rad:
about **0.05345 rad (3.06°)** of the requested 0.100 rad (5.73°), with
**0.04655 rad residual error**. The result was `target_not_reached`, followed by a
confirmed assembly stop. No hardware diagnostic fault occurred. This reproduces
the inconsistent settling seen in the [October 25 tests](wuji-live-test-2026-10-25.md);
these measurements do not establish its physical cause.

After inspecting feedback and preparing at the measured posture, the named return
passed with 0.003834 rad maximum error. No other joints received deliberate motion
probes. All three hand command windows, including the two expected/observed failed
commands, have camera videos and robot telemetry. The hand trace contains 381
samples with no hardware diagnostic faults.

During cleanup, immediate `wuji_hold` overlapped the automatic stop triggered by
operator disarm and returned `emergency/disable stop is in progress`. The
controller refused to replace the emergency stop with a hold. This exception is
retained in the report; it is not counted as a passing assertion. A separate
read-only follow-up passed **five final-state checks**, including successful
`wuji_hold` after stop completion, fresh disabled hand feedback, and independent
arm state 4 with no arm error. The final hand posture was within 0.003447 rad of
the recorded start. Both sessions were disarmed and the controller owners closed.

## Evidence and remaining work

After hardware testing and confirmed disable, the complete hardware-free unit
suite passed **268 tests, with one NVENC-dependent test skipped**:

```bash
.venv/bin/python -m pytest -q local_tests/unit
```

The three warnings concerned existing multiprocessing/fork usage in hand-service
tests. `git diff --check` also passed. These software checks do not override the
failed real hand positioning result.

Evidence is local and intentionally excluded from version control:

```text
.local-runtime/backend-mcp-20261027/
  preflight.json
  arm-readonly/report.json
  arm-readonly-fixed/report.json
  arm-motion/report.json
  arm-motion-fixed/report.json
  arm-motion-fixed/arm-motion-trace.json
  hand-live/report.json
  hand-live/hand-trace.json
  hand-final/report.json
```

Each session also retains its exact configuration, MCP audit, frontend/hand/camera
logs, observations and motion archives. The ignored live harness is in the same
directory. Timestamps inside artifacts are UTC on October 28; this report uses
the operator's October 27 local date.

The next hand task is to diagnose and qualify repeatable measured settling before
expanding to all twenty joints or qualifying an episode-reset posture. Physical
E-stop, network/power/process-loss behavior, full workspace/finger clearance,
payload/contact geometry, grasping and repeated manipulation episodes remain
outside this completed test. Calibration files and the main robot configuration
were not changed by this verification.

### Follow-up: operator-selected hand home

After the test, the operator selected the hand's current visible posture as home.
A separate read-only capture collected ten fresh disabled-state samples from the
same hand, with no hardware faults and maximum sample spread 0.000054 rad. Their
per-joint medians, rounded to six decimal places, were saved as
`end_effector.postures.home` in `examples/xarm-wuji.json`, with profile revision
`right-hand2-home-20261027-v2`.

This posture differs from the earlier `test_start` by up to 1.00035 rad; the
successful earlier return does not validate this new target's return paths.
No motors were enabled and no motion test was performed for the capture. The
0.35 rad per-call limit and original control parameters remain active. Raw samples
and comparison are in `home-posture/capture.json` under the evidence directory.

The operator then manually changed the hand posture and requested a return home.
The largest required joint change was 1.35125 rad, exceeding the 0.35 rad per-call
limit. A six-segment return was planned with maximum target changes of 0.22521 rad
and two seconds per segment, keeping the existing control parameters. The first
intermediate MCP move reached its target within 0.03597 rad. The second failed
settling with 0.07170 rad error at thumb S2, above the 0.040 rad tolerance. The
sequence stopped before dispatching the final named `wuji_pose("home")` command.

Final feedback confirmed the hand disabled, zero hardware faults, the session
disarmed, and the arm still in state 4. Maximum remaining error to home was
0.90539 rad. This return attempt **did not reach home**. No gains or tolerances
were changed and no automatic retry followed the settling failure. Configuration,
MCP calls, feedback trace and before/after camera images are in `home-command/`
under the evidence directory. The camera viewer remained available during this
hand-only attempt; the test backend did not take ownership of the cameras.

### Per-joint error analysis of the failed home-return step

Absolute errors against the **second intermediate target**, measured before
disable, in degrees. Bold entries exceed the unchanged 0.040 rad (2.292°)
settling threshold. These are encoder tracking errors, not independently measured
fingertip-position accuracy or the remaining distance to the final home pose.

| Finger | S1 | S2 | S3 | S4 |
| --- | ---: | ---: | ---: | ---: |
| Thumb | 1.57 | **4.11** | 2.05 | 0.22 |
| Index | **3.04** | 1.00 | 0.37 | 1.02 |
| Middle | 1.33 | 1.38 | 0.60 | 1.45 |
| Ring | **2.52** | 0.07 | 0.84 | 1.04 |
| Pinky | 0.57 | 0.75 | 1.70 | **2.66** |

Thumb S2 was requested to move −4.122° cumulatively but moved only −0.013°.
In the final 25 enabled-state samples (0.969 s), its median current was −0.215 A,
peak magnitude 0.231 A, and angle span only 0.000015 rad. Index S1 and ring S1
also barely moved despite changed targets. All 184 trace samples had fresh
feedback and no hardware diagnostic faults. The first step's thumb S2 error was
2.061°, so that step met tolerance despite almost no movement at that joint.

The leading hypothesis is insufficient effective stiffness/compensation to
overcome static friction or load, with binding/contact still possible. This is
an inference, not a diagnosed hardware defect. The recorded current does not show
thumb S2 reaching the configured 0.5 A cap. The gains were read back as kp 1.5,
kd 0.1 on every joint. Wuji documents MIT impedance control; its current
[teleoperation example](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/retargeting/1.teleop_real.py)
uses kp 5.0, kd 0.1. That comparison motivates controlled gain testing, not an
automatic change to this hand. See the vendor's
[control guide](https://docs.wuji.tech/docs/en/wuji-hand/latest/control-guide).

RAX sends zero desired velocity and zero feedforward effort with its position
targets. Device-side friction-compensation settings were not verified. The
near-stationary residual suggests that merely extending the settling timeout is
unlikely to resolve this trial. Separating control tuning from mechanical trouble
requires isolated bidirectional joint tests, measured current/error response and
independent observation of actual joint motion. No new robot movement or control
parameter changes were made during this analysis.

The complete numbers are retained in `home-command/joint-errors.csv` and
`home-command/diagnostic-analysis.json` under the evidence directory.
