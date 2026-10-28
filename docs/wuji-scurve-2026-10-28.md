# Wuji S-curve integration — October 28, 2026

`examples/xarm-wuji.json` selects `right-hand2-scurve-20261028-v13`, porting the
operator-approved standalone motion into the existing RAX single-writer service.
No extra SDK connection or publisher thread is created. The arm, cameras, joint
bounds and saved `home`/`fist` targets are unchanged.

**Live follow-up:** [RAX qualification](wuji-rax-qualification-2026-10-28.md)
passed five saved-home/fist commands with both camera streams and independent
disarm during finger motion. The operator reported smooth, acceptable motion.
Worst endpoint error was 0.038747 rad; observed command rate was 918.82 Hz with
a 4.951 ms maximum gap. The permitted thumb encoder warning recurred. The
implementation-time validation below is retained as history.

| Setting | Selected value |
| --- | --- |
| Motion mode | `scurve` |
| Local planner | Ruckig 0.19.4, state-to-state API |
| Command rate | 1000 Hz requested, best effort |
| MIT Kp / Kd | 8 / 0.15 on all twenty joints |
| Current limit | 1.5 A per joint; zero host current feedforward |
| Reference velocity / acceleration / jerk | 0.6 rad/s / 3 rad/s² / 2 rad/s³ |
| Tracking-error stop | 0.35 rad |
| Maximum feedback age / command gap | 250 ms / 50 ms |
| MCU temperature stop | 65 °C |
| Endpoint acceptance | 0.060 rad, 0.4 s stable window, ≤0.004 rad position range |
| Endpoint measured speed | ≤0.05 rad/s |
| Extra settling budget | 8 s |

## Reference and control behavior

Preparation requires disabled, fresh, stationary feedback. It seeds the first
reference from measured positions, sets and verifies all twenty MIT gains/current
caps, publishes that pose, and enables only through the existing operator flow.
The planner is checked before any motor write. Subsequent moves start from the
commanded position/velocity/acceleration and retain the final commanded target
between calls, including during arm movement.

Ruckig uses phase synchronization for proportional joint paths from rest, with
its time-synchronization fallback for states where phase synchronization is not
possible. Targets have zero terminal velocity and acceleration. An optional
duration must be at least the feasible duration and rounds upward to a control
tick. Limits bound the reference, not the measured hand. Feedback guards and
settling checks remain active; the controller does not claim that increasing
gains eliminates static error or that reaching a posture proves grasp success.

Each successful normal control tick samples one nominal period of the trajectory.
Missed scheduling slots do not advance the reference by the missed wall time or
trigger catch-up bursts. Short stalls stretch the motion; a long command gap,
stale feedback, excessive tracking error or expired motion budget requests an
emergency stop. The writer sleeps most of each period and spins for the last
0.2 ms. Status snapshots are refreshed at roughly 100 Hz and on state transitions
to leave time for publishing. `command_timing` reports requested/observed Hz,
command count, largest interval and skipped slots. This is not a hard real-time
guarantee; inspect timing under the actual camera/agent workload.

A healthy hold during a move cancels the old command and generates a bounded,
jerk-limited velocity-interface brake from the current reference state. Its rest
position may be ahead of the interrupted position. Planning checks the entire
braking path against joint bounds and the stop time budget; failure falls back to
emergency stop. `confirmed: false` on a normal hold does not certify physical
settling. Emergency/disarm still requests emergency stop plus disable immediately,
without an S-curve delay, and requires fresh disabled/low-speed feedback for
confirmation. Epoch fencing prevents cancelled planning/native calls from sending
later targets. The shared device lease and parent watchdog remain in use.

The older `timed` and `feedback` modes remain available. Unspecified mode/rate
defaults are preserved for existing profiles; the example explicitly selects
the new settings. Tool calls remain serialized, rather than becoming a policy
target streaming API.

## Diagnostics policy

The standalone trials continued the known encoder warning with operator consent.
This example carries that setting as `allow_encoder_warning: true`. The exception
requires **all** of: code `0x0003`, SDK name `Enc1BitRate`, severity `Warning`, and
clear policy `AutoClear`. Active events and per-joint frame counts/timestamps
remain visible in `warnings` and `warning_history`. The default is `false` for
other profiles. No other code or unknown classification is exempt.

Other device faults and temperature-limit crossings remain latched until explicit
reconnection, so a transient fault cannot disappear between control ticks.
Reconnection is permitted only after this owner's active control has stopped.
No encoder calibration, firmware update or device fault clearing is performed.
The encoder issue remains unresolved, and reported joint error is not a
calibrated absolute-accuracy claim.

## Validation and use

The approved standalone report was
`wuji_hand_test/runs/streaming-20261028-025140-646991/report.json`: 52,000 commands,
999.84 Hz observed, 2.745 ms maximum gap, and nominal reference peaks of
0.6 rad/s, 1.0954 rad/s² and 2 rad/s³. Fist maximum error was 0.02934 rad;
final zero-home error was 0.02296 rad. The operator preferred the gentler starts
and stops. Known thumb encoder warnings occurred; all twenty joints ended disabled.
That standalone zero-home differs from RAX's saved nonzero home.

RAX validation uses the real local planner with simulated hand feedback and the
actual private process/Unix socket. Tests cover full-hand reference limits,
rest endpoints, continuity across moves/braking, malformed targets, initialization,
settling failure, scheduling stalls, cancellation during planning/native calls,
warning classification, temperature, parent/arming/feedback watchdogs and stop
confirmation. The integration has not moved the physical hand as part of this port.

A simulated random → saved-home → fist run through the real controller loop,
SDK adapter and native `JointCommand` value type published 10,444 streaming
commands at 994.90 Hz, with a 2.878 ms maximum interval and 48 skipped slots.
The transport and feedback were fake; all three commands completed and the
simulated emergency stop was confirmed. This measures local scheduling overhead,
not network throughput, physical smoothness or accuracy.

Install the pinned requirements with the interpreter that runs RAX, then generate
a **new session** from `examples/xarm-wuji.json` using the normal
[launch instructions](xarm-wuji.md). Existing sessions contain configuration
snapshots and do not pick up edits to the example. Connection alone does not
enable the hand. Read status, arm and explicitly prepare before requesting
`wuji_pose` or `wuji_move_joints` through the usual tools. Inspect measured errors,
warning history and command timing when validating the integrated paths.

```bash
.venv/bin/python -m pip install -r requirements-wuji.txt
.venv/bin/python -m pytest -q local_tests/unit/test_wuji_*.py
```
