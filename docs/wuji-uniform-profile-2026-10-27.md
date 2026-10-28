# Uniform whole-hand profile — 2026-10-27

The operator chose Kp 10/Kd 0.15 on all twenty joints. A 1.5 A per-joint cap
matches the prior native-SDK screening and ring tests. The profile was applied,
read back, tested through actual RAX MCP and saved as
`right-hand2-all-kp10-kd015-20261027-v4` in `examples/xarm-wuji.json`.
The saved home angles and production controller implementation are unchanged.
No Kp admission increase is needed; the existing ceiling remains 10.

**Tracking passed; smoothness did not.** All eleven commands passed the existing
0.040 rad settling gate and an additional one-second hold. The operator then
reported visibly jerky motion. This profile is a tracking baseline for a focused
smoothness comparison, not a fully qualified motion profile.

## Motion and measured results

Four small coordinated waypoints moved all twenty joints by +0.02 rad, back,
−0.02 rad, and back to the measured starting center. A staged home return covered
a maximum initial displacement of 0.743822 rad in three segments (each ≤0.248 rad).
Two additional whole-hand ±0.02 rad excursions returned to the saved home.
Every ramp was quintic, three seconds long, published at 120 Hz. All targets
were admitted with the production trajectory validator. Named home returns used
`wuji_pose(name="home")`; other waypoints used `wuji_move_joints`.

The following are worst errors over the one-second enabled home holds:

| Home arrival | Thumb S2 error (rad) | Worst of all twenty joints (rad) |
| --- | ---: | ---: |
| home-first | 0.010938 | 0.020091 |
| home-return-positive | 0.008536 | 0.009115 |
| home-return-negative | 0.001306 | 0.009162 |

Full per-joint endpoint errors are in the [CSV](wuji-uniform-profile-2026-10-27.csv).
These are one staged arrival and two short repeat returns, not repeated returns
from a broad selection of arbitrary poses. They also differ from the earlier
isolated thumb experiment in starting posture and gains on the other nineteen
joints, so they cannot establish that Kp 10 outperforms Kp 15 on the thumb.

## Smoothness limitation

The largest SDK-reported speed was 1.466 rad/s on
thumb S2, although its commanded speed stayed below 0.041 rad/s. At that event,
position error accumulated while the output barely moved; it then changed about
0.01292 rad between native samples eight milliseconds apart. Native-timestamp
position differences independently show a roughly 1.616 rad/s burst. Other
joints also showed abrupt catch-up motion. These are encoder-relative measures,
not independent optical measurements, but the operator's visual observation
corroborates the lack of smoothness.

This is consistent with stick-slip or another breakaway/load effect; it does not
uniquely establish friction as the cause. The commanded trajectory is already
smooth. The configured 0.5 rad/s and 1.0 rad/s² limits constrain the reference
trajectory, not the actuator's measured catch-up speed. Passing endpoint error
and avoiding repeated oscillation do not establish smooth motion.

Peak reported current was 0.491 A, below the 1.5 A cap, and
maximum MCU temperature was 58.82 °C. There were no native
diagnostic codes or additional guard triggers in 73,589
frames (including disabled waiting time). A separate disabled postflight
observed 7,045 clear diagnostic frames and
confirmed all gains and caps. The hand ended disabled; the xArm's full pose was
unchanged in state 4, error-free.

## Software and evidence

The test-only driver retained native diagnostic latching and the existing early
oscillation, temperature, speed and current guards. It used the production hand
controller and MCP path. A bounded wait for fresh holding status between calls
avoided the previously observed cached-mode race; production sequencing remains
unchanged. No friction compensation or velocity target was added to RAX; its
commands still use zero desired velocity and zero host feedforward current.
Existing SDK/controller/service/backend/session checks passed: **73 tests**.

Raw results, native/service traces, scripts, hardware readbacks and before/after
config copies are retained under
[the experiment directory](../.local-runtime/wuji-combined-profile-20261027/).
The follow-up is a controlled damping comparison at unchanged Kp, caps and
reference trajectories, judging measured smoothness as well as final error.
