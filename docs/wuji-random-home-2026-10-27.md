# Broad random-pose home test — 2026-10-27

**The first random starting pose was not reached.** With the saved v7 gains,
outbound step 4 of 10 failed settling on thumb S2: **0.042127 rad** error against
the unchanged 0.040 rad tolerance. The sequence stopped and all twenty joints
were confirmed disabled. No broad random-pose home cycle completed; the other
two planned trials were canceled. There was no native encoder warning or
experimental guard trigger. The xArm stayed at its original pose, state 4,
with no error.

## Scope and method

The operator requested widely different random starting configurations, rather
than small offsets around home. Three twenty-joint configurations were sampled
independently across the intersection of the configured and model limits,
leaving 0.12 rad at each hard-limit boundary. Seed: `2026102703`. Their maximum
joint offsets from home were 1.86846, 1.55863 and 1.53035 rad.

Nominal self-collision screening used the pinned official right Hand 2.2 geometry
from `wuji-technology/wuji-description`, commit
`829c9aa27cb12c3a49d5b8d63c9869dd8f656702`, already stored in the local robot review.
Mesh hashes were checked against its manifest. The screen used the exact wrist
mesh and conservative convex hulls of other links, excluding fixed and directly
adjacent articulation pairs. Required separation was 2 mm for nonadjacent links
and 0.1 mm for bearing housings inside wrist sockets. Six sampled configurations
were rejected; three passed endpoint and home-path checks at joint increments
no larger than 0.002 rad. The minimum accepted separation ratio was about 1.428.

This is a nominal model check, not proof of physical collision freedom. SDK
joint order was matched to the model and limits, but model zero/direction
calibration has not been independently measured. External objects and cables
are absent from the model; Hand 2.2 tactile geometry is included even though
the device reports no online tactile fingertips.

RAX's raw `wuji_pose(name="home")` uses the same **0.35 rad maximum step** as
`wuji_move_joints`. It does not automatically plan a return from a distant pose.
The test therefore planned intermediate MCP moves of at most 0.20 rad, with a
final named home call. Each segment used a three-second quintic trajectory and
one-second endpoint hold. The production limits and 0.040 rad settling gate
were unchanged. A near-home alignment preceded random initialization; it is
not counted as a return from a random pose.

The existing test-only diagnostic, speed, current, temperature and oscillation
guards remained active. Additional test-only geometry checks screened each
fresh measured-to-target segment before dispatch and measured posture roughly
every 40 ms during control. They completed 592 runtime checks without a stop.
These guards and the staging harness are not installed in the production home
tool. Camera streaming was disabled for the test session; the existing viewer
was left running. No arm motion was requested.

## Measured result

Gains stayed at Kp 8/Kd 0.15 on thumb S1–S4 and index S1, and Kp 5/Kd 0.10 on
the other fifteen joints, with 1.5 A caps throughout.

| Command | Maximum arrival error (rad) | Result |
| --- | ---: | --- |
| Initial near-home alignment | 0.020392 | Passed |
| Outbound step 1 | 0.029988 | Passed |
| Outbound step 2 | 0.032940 | Passed |
| Outbound step 3 | 0.032553 | Passed |
| Outbound step 4 | 0.042127 | Failed: thumb S2 |

At the failed waypoint, thumb S2 targeted −0.427049 rad and measured
−0.384921 rad. Its final one-second enabled error stayed within
0.042114–0.042135 rad; reported speed stayed below 0.000402 rad/s. Median
current was −0.667 A, well below the 1.5 A cap. This shows a persistent position
offset during the settling window, rather than an unfinished slow approach;
it does not identify whether friction, compensation or another control/mechanical
effect caused that offset. All other joints were within tolerance at failure;
the next-largest error was pinky S2 at 0.021965 rad.

The failed waypoint was 0.74738 rad from home at its most displaced target
joint; the measured displacement was about 0.73909 rad. Thus even initialization
failed before the first broad random configuration or its return was attempted.
No retries, gain increases or tolerance changes followed the failure.

The motion session recorded 33,501 clear native diagnostic frames, peak current
0.789 A, peak reported speed 1.281 rad/s and peak MCU temperature 58.28 °C.
A separate read-only postflight recorded another 7,010 clear diagnostic frames,
verified the unchanged gains/caps, and confirmed all twenty joints disabled.
The hand remains **away from home**, approximately 0.73915 rad at the worst
joint. The arm's full pose matched its prior stopped pose.

## Evidence and outstanding work

Per-joint arrival errors are in the [CSV](wuji-random-home-2026-10-27.csv).
Plans, mesh-screen metadata, MCP responses, native/service traces, failure
analysis and postflight readback are in the
[experiment directory](../.local-runtime/wuji-random-home-20261027/).
The earlier waiting-only narrow-plan session was canceled without motion and
retained. A Python import-path startup failure was also retained separately;
it occurred before connecting the robot and was corrected before this trial.

The earlier near-home v7 successes remain valid for those tested motions.
Broad random-pose returns are still unqualified. Reliable reset from arbitrary
admissible task poses requires both a validated return path beyond the raw
command-step limit and adequate tracking along that path. No production code,
saved gains, camera calibration or firmware was changed by this test.
