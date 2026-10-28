# RAX v13 hand qualification — October 28, 2026

The integrated `right-hand2-scurve-20261028-v13` profile passed its first live
RAX qualification through the production MCP frontend with both calibrated
cameras streaming. The operator confirmed an empty hand, clear finger sweep
and attendance at the emergency stop before execution, then reported the
motion **smooth and acceptable**. The arm remained stationary throughout.

## Results

| Command | Maximum endpoint error (rad) | Elapsed (s) |
| --- | ---: | ---: |
| Initial measured pose → saved home | 0.022627 | 3.534 |
| Home → saved fist | 0.038738 | 4.242 |
| Fist → saved home | 0.017884 | 4.461 |
| Home → saved fist | 0.038747 | 4.285 |
| Fist → saved home | 0.017747 | 4.390 |

All five calls met the unchanged 0.060 rad endpoint tolerance, 0.4 s stable
window, 0.004 rad position-range gate and low-speed condition. The two returns
from fist differed by at most 0.002310 rad across all joints. Across all three
home arrivals, including the approach from near zero, the maximum range was
0.037633 rad; repeatability remains approach-dependent. No admission retry or
extra delay between commands was required.

The run published 19,828 position/velocity commands at 918.82 Hz observed,
with a maximum interval of 4.951 ms and 1,377 skipped slots. Requested rate
remains 1000 Hz. Both RGB-D streams, motion archives and a roughly 100 Hz
private-status recorder were active. The largest nominal reference peaks were
0.6 rad/s, 1.09535 rad/s² and 2 rad/s³. These bound the reference, not measured
physical motion; this is not a hard real-time result.

The configured, previously permitted `thumb_S2` `0x0003` / `Enc1BitRate` /
`Warning` / `AutoClear` event recurred (10,135 diagnostic frames). No other
device fault was recorded. The encoder-warning cause and absolute angle
accuracy remain unresolved; continuation used the existing v13 exception.

For the stop test, a sixth MCP fist move was interrupted by independent
operator disarming after measured finger motion was established (peak reported
speed in the triggering sample: 0.28527 rad/s). Fresh feedback confirmed all
twenty joints disabled and maximum speed 0.000840 rad/s within 61.04 ms of
the disarm timestamp. This is an observed software-disarm result, not a bound
for a physical emergency stop or communication/process loss. The interrupted
move correctly reported failure; it was not retried.

Final cleanup independently confirmed emergency stop/disable. The controller
retained the expected `operator has not armed motion` fault after intentional
disarming; native fault list and current diagnostic codes were empty. The arm
remained at [0.25310, -0.00134, 0.31412] m in state 4. The hand was disabled
partway into the interrupted path, **not returned to home**. Both commissioning
frontends and their camera/hand children were closed after the test.

## Preflight and software correction

The temporary hand route had disappeared. Restoring the documented host route
through `enx6c1ff7618956`, source `192.168.1.11`, restored connection to right
hand `WH2KA01260823017`, firmware 2.13.1, at `192.168.1.111:50001`. The arm
route was unchanged. The temporary route is still not persisted.

The operator confirmed the camera mounts and base had not moved since
calibration. Both 640×480 streams, observations and pixel-to-base tools worked.
Manually chosen cube-top-center pixels gave [0.2610, 0.1382, 0.0464] m from
side and [0.2617, 0.1378, 0.0486] m from wrist, about 2.34 mm apart, with fully
valid 7×7 depth neighborhoods. This scene sanity check does not replace the
installed calibration's reserved-view validation.

The production admission check previously read the parent's 20 Hz hand cache.
A completed command could therefore be followed by a false `executing`
rejection, or validation against an older command target. It now requests the
owner's current status and validates the target using that same snapshot.
Unreachable, stale, disabled, executing or faulted owners still refuse motion;
the child independently enforces command admission. Regression tests cover
both stale-mode directions, an unavailable owner and stale-target validation.
The final repository suite passed **354 tests, with one skipped**; `git diff
--check` was clean. No controller gain, tolerance, posture or reference-limit
change was made during this qualification.

## Scope and next task

The saved home/fist paths, repeated empty-hand returns and moving-hand software
disarm now have live v13 evidence. Payload/CoG, hand contact reference, task
swept clearance, cube grasp/release, retention, loaded transport, task-pose
resets and other failure modes remain unqualified. The historical gain-tuning
failures in the checklist are retained as history, not current v13 results.

The next selected task is cube into bowl. Its current instructions are in
[`examples/xarm-wuji-cube-into-bowl.md`](../examples/xarm-wuji-cube-into-bowl.md).
The operator will start Claude Code; no model or Claude Code session was started
for this qualification. See the [launch handoff](xarm-wuji-cube-handoff-2026-10-28.md).

## Evidence

Session directory:
`.local-runtime/sessions/20261028T101604Z_wuji_v13_qualification_624589/`.
It contains `qualification-plan.json`, `planned-trajectories.json`,
`qualification-report.json`, `hand-trace.jsonl`, `mcp.jsonl`, preflight state,
camera frames, scene-localization results and `motion-archive/`.
Six clips each contain wrist/side video (12 MP4s total), camera timestamps and
robot/hand telemetry. The final clip is intentionally marked failed because
the move was interrupted. `qualify.py` and `run-live-test.py` preserve the
exact local harness; do not reuse the session or overwrite its evidence.
