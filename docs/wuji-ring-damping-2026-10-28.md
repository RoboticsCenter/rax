# Ring damping and relaxed oscillation detector — 2026-10-28

**One complete broad random-initialization → home cycle passed.** The next
random initialization stopped on pinky S2 settling error, **0.054443 rad** against
the 0.050 rad tolerance. No native encoder warning or active oscillation stop
occurred. All twenty joints ended disabled, away from home; the arm remained
stationary and fault-free. The third random pose was not attempted.

## Selected settings

The operator requested relaxing the experimental oscillation detector and
considering more damping for the ring finger. The saved/read-back revision is
`right-hand2-thumb10-index8-ring5kd015-tol050-20261028-v9`.

| Joints | Kp | Kd | Current cap |
| --- | ---: | ---: | ---: |
| Thumb S1–S4 | 10 | 0.15 | 1.5 A |
| Index S1 | 8 | 0.15 | 1.5 A |
| Ring S1–S4 | 5 | **0.15, previously 0.10** | 1.5 A |
| Other eleven joints | 5 | 0.10 | 1.5 A |

Only ring damping changed in the robot profile. Settling tolerance remains
0.050 rad and the command-step limit remains 0.35 rad. The same three previously
screened broad random targets, staged MCP motions, three-second quintic ramps,
one-second holds and final named home calls were retained. Geometry screening
and its limitations are described in the [original plan](wuji-random-home-2026-10-27.md).

Wuji's [official control guide](https://docs.wuji.tech/docs/en/wuji-hand/latest/control-guide)
defines Kp as position-error feedback and Kd as velocity-error feedback. With
zero requested velocity here, increasing Kd increases opposition to measured
motion. This motivates testing damping for oscillation; increasing Kp instead
increases position correction and does not guarantee better stability.
Neither gain change nor relaxing a detector establishes the fault's root cause.

The **test-only** oscillation threshold rose from **0.006 to 0.010 rad
peak-to-peak** (0.344° to 0.573°). The other detector conditions remain: at least
three reversals above a 0.0005 rad step deadband, command range below 0.001 rad,
and a 0.15-second window spanning at least 0.12 seconds. Native fault latching,
speed/current/temperature checks, geometry checks and production watchdogs
remain active. This detector is not part of the production controller.

Offline regression replay verified that the new threshold permits the recent
0.006904 rad event but still detects both earlier twitch traces, including the
operator-confirmed visible twitch. It also detects both historical warning
traces at the same times as before: about 0.406/0.360 seconds before diagnostic
assertion. A gentle recorded control and a monotonic synthetic control pass.
These retrospective checks do not certify every future oscillation case.

## Motion results

The first campaign recovered home and reached random pose 1. During its return,
the production timing watchdog stopped command streaming. No encoder or
oscillation event triggered that stop. The interrupted return is not counted
as a completed random-home cycle.

After inspecting that timing fault and adjusting only test logging, a fresh
disabled-state check admitted the continuation. The hand recovered home from
approximately 1.506 rad away at its most displaced joint, then repeated the
random campaign with identical v9 gains and stop thresholds.

| Continuation phase | Outcome |
| --- | --- |
| Eight-step recovery home | Passed; worst home-hold error 0.021778 rad |
| Random pose 1 initialization and return, 21 commands | **Passed**; home arrival error 0.021713 rad and hold error 0.021670 rad |
| Random pose 2 initialization | Stopped on outbound step 6: pinky S2 error 0.054443 rad |
| Random pose 3 | Not attempted after the tracking failure |

At random pose 1, ring S1's maximum arrival error was **0.048728 rad**, close
to the selected tolerance. Six completed endpoint holds across the two campaigns
exceeded the previous 0.040 rad tolerance, on ring S1 or pinky S2. Thus the
relaxed settling gate did affect acceptance; these results do not demonstrate
0.040 rad tracking everywhere.

The old 0.006 rad detector continued running as a log-only monitor. It recorded
one thumb S3 event of 0.009120 rad in the first campaign; there were no such
events in the continuation. Separate offline replay restricted to ring joints
found **zero ring events at the old threshold in every new session**. The ring
result therefore is not merely a consequence of raising the stop threshold,
although motion history and limited repetition prevent a causal claim that
Kd alone solved the issue. Smoothness remains unqualified.

At the final failure, pinky S2 targeted 0.235598 rad and measured 0.181156 rad.
Its last-second error remained 0.054443–0.054792 rad, median current was 0.543 A,
and maximum reported speed was 0.00116 rad/s. This is a persistent tracking
offset during the observed window, without current-cap saturation. All other
joints were within 0.050 rad. No further gain or tolerance changes followed it.

## Test logging and timing

The original native logger retained a nested snapshot graph on every send.
Offline full garbage collection over its 7,193 retained records took 65–71 ms;
retaining the same records as serialized strings reduced this below 1 ms.
This makes garbage collection a plausible contributor to the live scheduling
fault, but no GC timing was recorded during that particular pause, so its cause
is not proven.

The continuation serializes each native record before retaining it and writes
the JSON array after control closes. Round-trip checks preserved all 7,193
prior records. Added GC instrumentation measured a maximum pause of **0.326 ms**
in the continuation, with no further timing stop. The production 0.10-second
scheduling watchdog was unchanged. No production controller code was edited.

## Final state and evidence

Across both campaigns, 51 motion commands were dispatched and 49 endpoint holds
completed. One returned command's hold was interrupted by the timing fault;
another command failed settling. Two recovery-home returns and one complete
random-home cycle passed. There were **251,720 clear native diagnostic frames**,
peak current 0.901 A, peak reported speed 1.833 rad/s and peak temperature 58.75 °C.

Independent read-only postflight verified the saved gains/caps, all joints
disabled, another **7,032** clear diagnostic frames and the unchanged arm pose.
The final disabled hand is about **1.155 rad from home** at its most displaced
joint. Disabled relaxation is not included in enabled tracking metrics.

See the [per-joint CSV](wuji-ring-damping-2026-10-28.csv),
[initial campaign and analysis](../.local-runtime/wuji-ring-damping-20261028/),
and [continuation evidence](../.local-runtime/wuji-ring-damping-20261028-logfix/).
The profile remains provisional: ring S1 margin and pinky S2 tracking need
attention before broader random-pose reliability can be claimed.
