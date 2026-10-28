# Selected per-joint profile and whole-hand test — 2026-10-27

Later follow-up: v8 now uses thumb 10/0.15 and 0.050 rad tolerance; its
[October 28 retest](wuji-random-home-2026-10-28.md) completed recovery home but
stopped random initialization on the ring S3 oscillation detector. The hand is
disabled away from home. The preceding [broad random-pose test](wuji-random-home-2026-10-27.md)
failed during initialization on thumb S2, before any random-home cycle. The
near-home v7 results below remain historical results.

**The requested final profile completed all nine whole-hand commands**, including
home alignment and four returns from ±0.06 rad excursions. It is saved in
`examples/xarm-wuji.json` as `right-hand2-thumb8-index8-fingers5-20261027-v7` and read back on the
disabled hardware. The largest arrival error was 0.022227
rad, below the unchanged 0.040 rad tolerance. The hand ended disabled; the xArm
remained stationary and fault-free. No encoder warning or additional guard stop
occurred in the three completed motion trials.

## Current parameters

| Joints | Kp | Kd | Current cap |
| --- | ---: | ---: | ---: |
| Thumb S1/S2/S3/S4 | 8 | 0.15 | 1.5 A each |
| Index S1 | 8 | 0.15 | 1.5 A |
| Other fifteen finger joints | 5 | 0.10 | 1.5 A each |

The settings are independent twenty-element arrays. No firmware reset, flash
save, origin change, camera/calibration change or production controller edit was
performed. Other controls remain: 120 Hz position-only commands, zero desired
velocity and host feedforward current, three-second quintic test ramps, one-second
endpoint holds, 0.5 rad/s reference velocity, 1.0 rad/s² reference acceleration,
0.35 rad command-step/tracking bounds, and the existing 0.040 rad for 0.2 seconds
settling gate with a three-second timeout. The remaining watchdog and experimental
guard inventory is recorded in the [smoothness report](wuji-smoothness-2026-10-27.md).
Reference rate limits constrain the trajectory; they do not guarantee measured
motor speed or visual smoothness.

## Selection based on joint-specific evidence

The first prepared trial detected that the operator had repositioned the disabled
hand, approximately 0.802 rad from home. It stopped before enabling or dispatching
motion. After the operator confirmed positioning complete and hands clear, a
fresh plan returned gradually in five steps, each ≤0.161 rad, then ran two
positive/home/negative/home cycles with 0.06 rad excursions on every joint.
Unlike the previous 0.02 rad screen, these excursions exceed the 0.040 rad
settling tolerance. Every segment was validated by the production trajectory code.

The initial mixed candidate kept the thumb at 10/0.20 and set all sixteen other
joints to the vendor-example baseline 5/0.10, retaining 1.5 A caps. All thirteen
commands passed. A predeclared review rule flagged a non-thumb joint if its later
hold error was at least 0.030 rad on two separate endpoints, or if it failed
settling above 0.040 rad. Only **index S1** met that rule: its two negative
excursions held about **0.03220 rad** error. This is repeatedly near the acceptance
limit, not a controller fault or a failure of those two moves.

Only index S1 was raised to 8/0.15. Repeating the same two cycles reduced its
negative-excursion hold errors to **0.01998 and 0.02027 rad**. Other non-thumb
joints remained at 5/0.10. This test completed all nine commands, but the operator
reported that the unchanged thumb at 10/0.20 remained very jerky.

The operator then explicitly requested **8/0.15 on all four thumb joints** and
another whole-hand test. The final trial retained index S1 at 8/0.15 and the other
fifteen joints at 5/0.10. It repeated the identical home alignment and two
positive/home/negative/home cycles. All nine commands and their one-second holds
completed. No joint had repeated later-hold error ≥0.030 rad in this final run.
No further gain escalation was performed.

## Final trial results

Worst absolute errors across the full one-second enabled endpoint holds:

| Finger | Worst joint error (rad) |
| --- | ---: |
| Thumb | 0.022220 |
| Index | 0.020030 |
| Middle | 0.013125 |
| Ring | 0.020724 |
| Pinky | 0.021179 |

The largest full-hold error was on thumb S4. Worst home-hold error over all twenty
joints was 0.020724 rad. The
[per-joint CSV](wuji-per-joint-profile-2026-10-27.csv) preserves each arrival and
hold separately for every trial. Disabled relaxation is not counted as tracking
error. The final profile was tested near home and through ±0.06 rad motions;
the earlier staged arbitrary-pose return used thumb 10/0.20. Broad arbitrary-pose
returns, loaded grasps and task manipulation remain unqualified for the final
profile.

Passing tracking does not establish smoothness. The operator's jerkiness report
applied to the prior 10/0.20 thumb trial; no visual judgement of the final 8/0.15
trial has yet been provided. Position/current traces remain available for further
analysis. In the matched eight excursion/return moves, the final thumb had a
1.785 rad/s SDK-reported peak and about 1.023 rad/s peak from centered native
position differences, compared with 1.424 and 1.079 rad/s respectively at
10/0.20. Those differing estimates do not establish a clear smoothness improvement;
both profiles still show fast catch-up movement against the slow reference.
Reducing Kp can trade tracking against motion response, as the previous
small-motion comparison showed; this report does not call jerkiness resolved.

## Final hardware state and evidence

The three completed trials comprised **31 commands**
and 151,392 native diagnostic frames, with
no nonzero code or guard stop. The final trial contributed
40,752 frames. Its maximum recorded current was
0.583 A and MCU temperature was
58.95 °C. All twenty joints were confirmed disabled,
fresh and unowned, and a separate read-only postflight verified gains/caps while
observing another 6,998 clear diagnostic
frames. The xArm remained in state 4 with its full pose unchanged and no errors.

Raw scripts, MCP calls, SDK/service traces, profile snapshots, trajectory plans,
assessments and postflight readback are retained under
[the experiment directory](../.local-runtime/wuji-per-joint-profile-20261027/).
The stopped preflight is preserved separately. The production motion code and
its limits are unchanged; the tested SDK/controller/service/backend/session
suite previously passed 73 checks. No commit or external upload was made.
