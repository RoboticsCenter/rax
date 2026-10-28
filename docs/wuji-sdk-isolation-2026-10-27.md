# Wuji thumb-home command-path isolation — 2026-10-27

**The thumb-home failure reproduces through standalone native Wuji SDK calls,
without MCP, RAX's controller, or its SDK wrapper.** Increasing the native command
rate to 200 Hz and replacing quintic interpolation with linear interpolation did
not resolve the approximately 0.061 rad residual. This isolates a reproducible
symptom below RAX's motion orchestration under the tested settings; it does not
identify a hardware defect or distinguish the SDK from firmware and mechanics.

The operator requested this comparison after the
[gain and repeatability trials](wuji-gain-tuning-2026-10-27.md). All tests used
right hand `WH2KA01260823017`, firmware 2.6.0 and SDK 2026.10.12. The xArm remained
stopped at the same pose. Hand disable and restoration of the original gains
were confirmed after every native test.

## What the earlier tests called

The earlier path was:

```text
MCP wuji_move_joints / wuji_pose
  → services/xarm/backend.py
  → services/wuji/client.py (private controller IPC)
  → services/wuji/controller.py (trajectory, admission and settling)
  → services/wuji/sdk.py (WujiDriver)
  → wuji_sdk JointCommandPublisher.send()
  → hand firmware / actuators
```

RAX ultimately uses the vendor's normal typed API, but those tests were not
independent vendor-only runs. Source inspection confirmed:

- Gains/current are applied through `hand.mit_params().set()` and
  `hand.effort_limit().set()` and read back for all twenty joints.
- `publisher.send()` receives twenty `JointCommand(position, 0, 0)` entries.
  Position is in radians; velocity and effort feedforward are zero.
- Thumb S2 is flat index 1 and stream NID 2. The feedback decoder uses the SDK's
  NID mapping, which skips tactile slots.
- RAX generates a quintic target trajectory at nominally 120 Hz, checks all
  twenty positions against the 0.040 rad tolerance, and stops the assembly after
  an unsuccessful MCP motion. `wuji_pose` resolves the configured posture into
  the same joint-move path; it does not call a separate vendor homing algorithm.

These field definitions and calls agree with Wuji's
[joint-control API](https://docs.wuji.tech/docs/en/wuji-hand/latest/sdk-reference/joint-control).
No unit, argument-order or joint-index mismatch was found in this path.

## Standalone comparison

The independent harness imports `wuji_sdk` directly, with no RAX modules. It
connects to the explicit serial/address, subscribes to raw joint-state and
diagnostic frames, verifies all twenty joints are Ready before preparation,
sets/readbacks the profile, seeds measured positions, enables, and publishes
commands through the native typed publisher. Frame ordering is decoded
independently and checked against the SDK map.

It retains the existing exclusive-device locks and includes its own feedback,
command-gap, tracking-error, temperature and measured-speed watchdog. Cleanup
disables the hand, confirms fresh Ready feedback, restores/readbacks the original
profile, closes the publisher/subscriptions and disconnects. xArm calls are
read-only status/pose queries. Neither RAX nor another publisher owned the hand
during these trials.

The matched conditions were:

- Thumb S2 kp 4.0, kd 0.075; other nineteen joints kp 1.5, kd 0.1.
- Current cap 0.5 A for every joint, zero velocity/effort feedforward.
- The same nineteen-joint target posture as the controlled comparison.
- Thumb target 0.184535 rad, followed by saved thumb home at 0.064535 rad.
- Two seconds per trajectory, 0.040 rad tolerance, 0.2-second settling interval
  and three-second settling timeout after the trajectory.
- The same step/tracking envelope; stop above sampled 2 rad/s, or above 1 rad/s
  sustained for 0.12 seconds. These are experimental stops, not qualified speed limits.

The native harness tests the saved thumb-home angle while holding the other
joints at the shared posture; it does not command whole-hand home. All outward
moves passed, and every home return failed settling without a firmware warning.

| Path | Rate | Interpolation | Measured home-approach start | Home error |
| --- | ---: | --- | ---: | ---: |
| RAX MCP, previous comparison | 120 Hz nominal | Quintic | 0.157005 rad | 0.060725 rad |
| Native SDK | 120 Hz | Quintic | 0.156909 rad | 0.060681 rad |
| Native SDK | 200 Hz | Quintic | 0.156956 rad | 0.060636 rad |
| Native SDK | 200 Hz | Linear | 0.156907 rad | 0.060790 rad |

The home-approach starting angles differed by less than 0.00010 rad. The native
endpoint errors differ from the RAX reference by at most 0.00009 rad (0.0052°).
These are individual matched trials, not estimates of long-run reliability.

## Raw feedback and timing

The native tests stopped moving at 0.12517–0.12532 rad against the 0.064535 rad
target. Mean current during the final second was −0.4810, −0.4811 and −0.4822 A.
Position variation during that second was 0.000021–0.000073 rad.

No position-, velocity- or current-limit flag was reported active on thumb S2
during any native home approach. Although current was near the configured cap,
this does not prove current-limit saturation. No diagnostic code occurred.

Measured command rates were 120.17, 200.12 and 200.14 Hz. The longest send gap
was 10.22 ms at 120 Hz and 6.48 ms at 200 Hz; individual SDK send calls took at
most 0.143 ms. This provides no evidence of command starvation in these trials.
Thumb bus response rate stayed at 100% in both quintic tests; it briefly reported
85% in the linear test, with no increase in the thumb's cumulative bus-timeout
counter in any home approach. Measured thumb supply voltage was 12.31–12.37 V.

Peak SDK-reported thumb velocities were 1.68, 1.58 and 1.49 rad/s across the
respective native trials. These are not independently verified physical output
speeds; see the subsequent [Hand 2 controls research](wuji-hand2-controls-research-2026-10-27.md)
for the external velocity-feedback caveat and offline current-law comparison.
No experiment guard triggered. Maximum reported temperature was 56.23 °C.
Sampling cannot guarantee that shorter physical peaks were absent.

## What is and is not isolated

The failure does not require MCP transport, the RAX trajectory/controller
implementation, or the RAX SDK wrapper. Changing command rate and trajectory
shape within the tested conditions did not remove it. The common configuration
and SDK/firmware/actuator behavior remain candidates.

These were native API calls under matched commissioning settings, not an
unmodified vendor demo or factory-profile qualification. Wuji's
[publishing example](https://docs.wuji.tech/docs/en/wuji-hand/latest/sdk-practice/publish-command)
uses kp 3, kd 0.05, a 1.5 A cap, 200 Hz and linear interpolation toward zero.
The current limit in our comparison was one third of that example's value.
No higher current was applied, and no whole-hand zero motion was requested.

The outstanding distinction is whether the steady residual comes from the
selected control/current profile, firmware compensation or SDK behavior,
posture-dependent load/friction, or an internal mechanical issue. Normal MIT
impedance does permit static error, so nonzero error alone is not evidence of
damage. Repeated large residuals and earlier Stall warnings justify a focused
actuator/profile diagnosis. Current-limit flags remained inactive, so a larger
current cap cannot yet be described as the demonstrated fix.

The next controlled comparison should establish the vendor-supported thumb
operating profile and mechanical condition before changing current. A C API
comparison could further separate Python binding behavior if needed; all native
tests here used the same installed Python SDK, so they do not rule out an SDK
or firmware defect.

## Final state and evidence

Every native trial restored kp 1.5, kd 0.1 and 0.5 A on all twenty joints with
the motors disabled. Final arm state was 4, with error/warning `[0, 0]` and the
unchanged recorded pose. The independent camera viewer remained available.
No production controller code or example configuration was changed.

```text
.local-runtime/wuji-sdk-isolation-20261027/
  direct_sdk.py
  comparison.json
  native-120-quintic/{report.json,trace.json}
  native-200-quintic/{report.json,trace.json}
  native-200-linear/{report.json,trace.json}
```

Reports contain exact native parameter readbacks, raw feedback with stream NIDs
and timestamps, command arrays and timings, diagnostic flags, endpoint results,
stop confirmation, restored parameters and before/after arm state.
