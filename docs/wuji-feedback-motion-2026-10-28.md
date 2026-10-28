# Feedback-paced motion, softer gains and continuous fist test — 2026-10-28

**The full arbitrary-pose → fist transition passed, but the operator still
observed jerky motion.** Softer gains reduced speed-burst counts in the matched
small-motion comparison; larger continuous moves did not solve the remaining
jerkiness. Target convergence and visible smoothness are separate results.

The selected example revision is
`right-hand2-soft-feedback-fist-20261028-v12`. Hardware: right Wuji Hand 2
`WH2KA01260823017`, firmware 2.13.1, SDK 2026.10.12. Camera calibration is unchanged.
The xArm remained stopped and error-free throughout.

## Selected settings

| Setting | Selected value |
| --- | --- |
| Thumb S1–S4 Kp / Kd | **6 / 0.15**, down from 10 / 0.15 |
| Other sixteen joints Kp / Kd | **5 / 0.15**; index S1 previously 8 / 0.15 |
| Current caps | 1.5 A on all twenty joints, unchanged |
| Motion mode | `feedback`; legacy `timed` remains the code default |
| Reference velocity / acceleration / jerk limits | 0.12 rad/s / 0.2 rad/s² / 0.8 rad/s³ |
| Endpoint tolerance | 0.060 rad (3.44°), previously 0.050 rad |
| Stable completion | All joints inside tolerance for 0.4 s, each moving at most 0.004 rad peak-to-peak |
| Extra time budget | 8 s beyond nominal duration, shared by retiming and settling |
| Feedback pacing | Full speed below 0.060 rad reference error; progressively slower up to 0.100 rad, then pause |
| Speed-scale transition | Smooth 1 s transition |
| Per-command joint displacement | **2.1 rad**, previously 0.35 rad, for full continuous paths |
| Hard tracking-error bound / command rate | 0.35 rad / 120 Hz, unchanged |
| Named postures | Existing `home` unchanged; new `fist` |

The local softer-gain test used candidate v11 with the old 0.35 rad admission
limit. The subsequent full-motion session added the 2.1 rad admission limit and
fist target to its test configuration. The selected example names that complete
configuration v12. Independent disabled readback verified the same physical
gains and current caps on all twenty joints. These are host-selected settings,
not a firmware factory reset.

## Production behavior

`services/wuji/motion.py` adds feedback pacing to the existing straight
joint-space quintic trajectory. When a joint lags, the controller smoothly slows
the path clock and gives the device time to catch up. It preserves path geometry,
does not overshoot the requested target, and does not add an integral term,
position bias, current feedforward, or automatic gain/current increases.

The SDK now receives the trajectory's matching velocity as well as position.
Explicit current feedforward remains zero. Wuji's
[control guide](https://docs.wuji.tech/docs/en/wuji-hand/latest/control-guide)
defines the velocity feedback term using requested minus measured velocity.
The old path always requested zero velocity.

Nominal duration accounts conservatively for the velocity, acceleration and jerk
introduced by changing the path speed. These bounds apply to the **reference**,
not measured motor motion. Every speed-scale transition completes before another
begins. Emergency or fault stops can still interrupt a trajectory.

Completion requires fresh position samples over a stable window, rather than
one sample passing through the error band. A timeout reports the requested goal,
current command, measured error, path progress and whether the entire trajectory
finished. It never substitutes an unfinished command for the requested goal or
reports a stalled path as reached. Existing arming, stop, freshness, ownership,
timing, joint-limit and fault checks remain in place.

This is bounded feedback pacing and tolerant completion. Waiting longer can
help a lagging joint; it does not guarantee elimination of a persistent static
offset. Production still does not perform collision planning. The nominal
geometry checks and extra oscillation/current/temperature monitors below belong
to the commissioning harness.

## Hardware results

All **37 dispatched motion commands** across six successful sessions passed
their endpoint holds. All sessions ended with feedback-confirmed disable. Native
monitoring received **306,719 diagnostic frames**, with no native warning or
active guard event. Maximum observed joint MCU temperature was 59.85 °C.

| Session | Commands | Result |
| --- | ---: | --- |
| Recovery with feedback, old gains | 6 | Home error 0.021097 rad |
| Slow timed baseline, old gains | 5 | Worst arrival error 0.023485 rad |
| Same small motions with feedback, old gains | 5 | Worst arrival error 0.023859 rad |
| Previous pinky failure target and return, old gains | 13 | Target reached at 0.052530 rad; home at 0.021342 rad |
| Same small motions with softer gains | 5 | Worst arrival error 0.037681 rad |
| Home alignment, continuous arbitrary pose, continuous fist | 3 | Both full motions reached; thumb S2 near the tolerance boundary |

The previous pinky target passes principally because the tolerance is now
0.060 rad; this does not demonstrate that the old 0.054443 rad offset was cured.
The first three feedback sessions never needed to slow below speed scale 1.

An earlier harness attempt passed an explicit `null` duration, which the MCP
frontend correctly rejected before dispatch. It was disabled and preserved as
`recovery-admission-rejected`. The harness now omits optional durations instead.

### Matched small-motion comparison

Each trial aligned home, then commanded home +0.08 rad, home, home −0.08 rad,
and home across all twenty joints, using 4 s nominal ramps and 1 s endpoint
holds. Metrics exclude initial alignment and use unique device-state timestamps.
Position-derived speed uses centered differences across about 16–18 ms; a burst
is a contiguous interval above 0.25 rad/s for one joint, summed across joints.

| Trial | Peak measured speed | Speed bursts | Thumb bursts | Tracking RMSE |
| --- | ---: | ---: | ---: | ---: |
| Timed, old gains | 1.176 rad/s | 224 | 94 | 0.01026 rad |
| Feedback, old gains | 1.167 rad/s | 237 | 108 | 0.01018 rad |
| Feedback, softer gains | 1.057 rad/s | 166 | 82 | 0.01281 rad |

Feedback alone did not clearly improve smoothness. Softer gains reduced burst
counts by 30% relative to the matched feedback trial, at the cost of more error.
This single ordered comparison is not a statistical or perceptual qualification.

### Full arbitrary-pose → fist transition

The starting target reused previously tested random pose 1, reaching up to
1.868 rad from home. The fist target changes a joint by up to 1.792 rad from that
arbitrary pose. **Each transition was one continuous command**, without small
intermediate targets or endpoint stops within it.

| Motion | Nominal / actual seconds | Minimum speed scale | Worst final error |
| --- | ---: | ---: | ---: |
| Home → arbitrary pose | 34.411 / 35.327 | 0.798 | Thumb S2, 0.059964 rad |
| Arbitrary pose → fist | 33.088 / 33.832 | 0.904 | Thumb S2, 0.059642 rad |

The physical device exercised feedback slowing in both full moves. Their maximum
reference errors were 0.072137 and 0.065533 rad respectively. Both completed
within the bounded extra time, with the required stable window and subsequent
holds; the fist had a two-second hold.

| Finger | Worst joint error at fist arrival |
| --- | ---: |
| Thumb | 0.059642 rad |
| Index | 0.025954 rad |
| Middle | 0.018337 rad |
| Ring | 0.029816 rad |
| Pinky | 0.038061 rad |

The fist is a curled posture with the thumb outside and nominal clearance,
not a forceful squeeze or qualified grasp. Commanded paths were screened at
≤0.002 rad increments with the pinned official Hand 2.2 right-hand geometry,
exact wrist mesh and conservative convex hulls for other links. Nonadjacent
clearance was at least 2 mm, with a 0.1 mm bearing-socket exception; the minimum
clearance ratio along the arbitrary-to-fist path was 1.422. Adjacent articulation
pairs are excluded. The model-to-device mapping is not independently calibrated,
and external objects are not modeled. Actual measured geometry was checked
during the run. This qualifies the tested route, not every possible start pose.

Despite the smooth reference and lack of intermediate stops, the operator
reported **“Still jerky.”** Position-derived peak speed was 1.156 rad/s entering
the arbitrary pose and 1.050 rad/s entering the fist. Larger motion therefore
did not eliminate the speed bursts. The full-motion and small-motion burst
counts cannot be compared directly because distance and duration differ.
Friction or low-level control behavior remains a hypothesis, not an established
root cause. Further host slowdown alone is not supported as a solution.

## Final state, validation and evidence

Independent postflight readback confirmed all twenty joints disabled, zero
current diagnostic codes, selected gains/current caps, and the unchanged stopped
xArm pose. It observed 7,013 additional diagnostic frames without a warning and
performed no enable, motion, configuration write or fault clear. The disabled
hand later differed from the fist target by up to 0.806 rad; the successful fist
measurement refers to the enabled arrival/hold, not passive pose retention.

All **114 targeted software/service/workspace tests passed** in the final run;
`git diff --check` also passed. The software checks cover slow and stalled actuators, bounded timeout, stable
completion, stale/duplicate feedback, command velocity, reference derivatives
during full pause/resume, small and 1.9 rad paths, existing watchdogs in both
modes, and actual child-process/MCP tests with fake hardware. The initial added
large-path fixture exceeded its synthetic joint bounds; it was corrected to
span −0.95 to +0.95 rad without changing production limits.

Raw evidence is under `.local-runtime/wuji-feedback-motion-20261028/`, including
candidate/config snapshots, path screens, frontend reports, native traces,
diagnostic readback and the rejected attempt. Endpoint results are retained in
[the CSV](wuji-feedback-motion-2026-10-28.csv). Historical trials remain intact.

New sessions use the selected example; existing session snapshots do not reload
it. With preparation and arming complete, `wuji_pose(name='fist')` or
`wuji_pose(name='home')` lets the controller choose duration. Explicit durations
must meet the configured reference bounds. No commit or push was performed.
