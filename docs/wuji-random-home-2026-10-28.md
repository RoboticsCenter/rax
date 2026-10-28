# Random-pose retest with stronger thumb — 2026-10-28

Later follow-up: [v9 with higher ring damping](wuji-ring-damping-2026-10-28.md)
completed one broad random-home cycle, then stopped on pinky S2 tracking in
the second initialization. The v8 results below are historical.

**The recovery home return passed; the random-pose campaign stopped on the
ring S3 oscillation detector.** No full random initialization/home cycle completed.
The hand ended disabled, approximately 0.398 rad from home at its most displaced
joint. The arm remained stationary, state 4, with no error.

## Requested configuration

Saved and read back as `right-hand2-thumb10-index8-fingers5-tol050-20261028-v8`
in `examples/xarm-wuji.json`:

| Setting | Previous | Selected |
| --- | --- | --- |
| Thumb S1–S4 Kp/Kd | 8 / 0.15 | **10 / 0.15** |
| Index S1 Kp/Kd | 8 / 0.15 | 8 / 0.15 |
| Other fifteen joints Kp/Kd | 5 / 0.10 | 5 / 0.10 |
| Current cap, all joints | 1.5 A | 1.5 A |
| Endpoint tolerance | 0.040 rad | **0.050 rad** |

The tolerance is an existing configurable acceptance threshold, not a hardware
limit. Increasing it does not itself improve tracking. The global default
remains 0.040 rad; this robot's example explicitly overrides it to 0.050.
The separate 0.35 rad command-step limit remains unchanged, as do trajectory
rate limits, current caps, diagnostics and test-only oscillation guards.
No production controller code was changed.

The same three broad random configurations and nominal geometry screening from
the [previous attempt](wuji-random-home-2026-10-27.md) were retained. Motions used
staged MCP commands, three-second quintic segments, one-second endpoint holds,
and a final `wuji_pose(name="home")`. The geometry model's limitations still
apply; staging and collision checks remain test-harness features.

## Results

Read-only preflight confirmed the hand remained disabled at the previous
stopping pose, with less than 0.001 rad change, clean diagnostics and unchanged
arm pose. A four-command staged recovery from about 0.739 rad away completed
home, with worst home-hold error **0.021584 rad**. Thumb S2's home arrival error
was **0.013759 rad**.

The first random trial then passed near-home alignment and two outbound steps.
On those same outbound targets, thumb S2 error improved:

| Target | Previous Kp 8 | New Kp 10 |
| --- | ---: | ---: |
| Outbound step 1 | 0.029988 rad | 0.011872 rad |
| Outbound step 2 | 0.032940 rad | 0.023078 rad |

This is a single comparison with different approach history. All seven completed
commands and their holds also met the old 0.040 rad threshold; their success
did not depend on relaxing tolerance. The previous failing fourth waypoint
was not reached in this retest.

During outbound step 3, the unchanged test-only detector found three measured
position reversals on **ring S3** over 0.1433 seconds: **0.006904 rad (0.396°)
peak-to-peak**, versus 0.000998 rad commanded travel. The reference was moving
slowly, not strictly stationary. The trace contains a short cluster of measured
reversals; this alone does not establish sustained or visibly apparent trembling.
Reported velocity remained negative through the window despite position
reversals, so the two feedback signals do not independently corroborate each
reversal. No operator visual observation was collected for this event.

Ring S3 still used **5/0.10**. The timing does not establish that increasing
thumb stiffness caused its measured motion. Current in that short window ranged
from −0.115 to +0.041 A, and no native diagnostic/encoder warning occurred.
The detector stopped the campaign; no guard relaxation, retry or subsequent
home motion was attempted. The other two random trials were canceled.

## Final state and evidence

The two motion sessions recorded **36,770** clear native diagnostic frames.
Peak reported speed was 1.542 rad/s during recovery; peak current was 0.643 A,
and peak MCU temperature was 57.60 °C. The first random trial had one
oscillation-guard event and no other guard trigger. Nominal measured-geometry
checks ran 818 times without a clearance stop.

A separate read-only postflight verified the v8 gains/current caps and all
twenty joints disabled, observing another **7,010** clear diagnostic frames.
The saved profile remains v8, provisionally selected by the operator; broad
random-home reliability and smoothness remain unqualified.

See the [per-joint results](wuji-random-home-2026-10-28.csv) and
[raw experiment directory](../.local-runtime/wuji-random-home-20261028/) for
plans, configuration before/after, MCP responses, native traces, guard details
and postflight readback. Configuration and every planned trajectory were
validated before motion. No firmware, camera calibration or arm setup changed.
