# Saved-home retest with the selected ring profile — 2026-10-27

**Home was not reached.** One intermediate move passed; the next dispatched
move failed settling. The hand was then confirmed disabled on all twenty joints.
No subsequent motion or gain change was attempted after the settling failure.
The final named `wuji_pose(name="home")` command was never dispatched.

The operator positioned the disabled hand and confirmed hands clear. The test
loaded `examples/xarm-wuji.json`, revision `right-hand2-ring-kp10-kd015-20261027-v3`:
ring S1/S3/S4 used Kp 10/Kd 0.15/1.5 A; the other seventeen joints retained
Kp 1.5/Kd 0.1/0.5 A. The thumb settings that performed better in earlier isolated
experiments are not part of this installed ring-only selection.

## Execution and software timing finding

The initial largest home displacement was 1.198580 rad on index S3. A five-step
linear path in joint coordinates used three-second quintic ramps, with maximum
command changes of 0.239716 rad per step, below the configured 0.35 rad bound.
All segments were validated with the production trajectory implementation.
Preparation applied and read back the selected gains and seeded measured angles.

The first actual MCP `wuji_move_joints` returned `reached`, with worst error
0.038716 rad after 5.149 seconds including settling. The next command was rejected
at validation, before dispatch, because the backend did not see fresh enabled
holding status. Native feedback remained fresh, enabled and fault-free; the
service observer's last pre-stop sample still said `executing`. The code returns
a completion result before publishing the next controller snapshot, while the
client caches status on a 50 ms polling interval. This evidence is consistent
with a stale-mode race between sequential calls, rather than a hardware fault.
The exact cached snapshot inside the rejected admission check was not captured.

The hand was disabled and the original evidence preserved. A second test session
added a bounded wait for fresh, enabled, fault-free `holding` with no active action
before each move. This waits for normal admission; it does not relax guards or
modify the production client/controller. The operator was told about this
continuation. The path was recomputed from fresh measured angles: four remaining
segments, each at most 0.241051 rad and three seconds. Its first segment failed
the actual physical settling criterion after the three-second settling timeout.
No further return was attempted. The production cached-mode issue remains open.

## Tracking at the failed intermediate target

The acceptance threshold was 0.040 rad for every joint, maintained for 0.2 seconds,
with up to three seconds of settling after the ramp. These are **errors relative
to the attempted intermediate waypoint**, sampled by the controller while the
hand was still enabled; they are not errors after reaching home or relaxing while
disabled.

| Finger | Worst joint | Absolute error (rad) | Degrees |
| --- | --- | ---: | ---: |
| Thumb | thumb_S2 | 0.044533 | 2.552° |
| Index | index_S4 | 0.029722 | 1.703° |
| Middle | middle_S3 | 0.020153 | 1.155° |
| Ring | ring_S2 | 0.024058 | 1.378° |
| Pinky | pinky_S4 | 0.045738 | 2.621° |

The joints above tolerance were thumb S2, thumb S3 and pinky S4. Ring S2, which
retains the original low gains, accounts for the ring row; the selected ring
S1/S3/S4 errors were 0.003807,
0.001431 and 0.001493 rad.
All twenty measurements are in the [joint CSV](wuji-home-retest-2026-10-27.csv).
The remaining largest distance to saved home was
0.744005 rad on index S3 because the
return stopped early. This is not an endpoint-accuracy measurement.

The low-gain joints retained static offsets despite nonzero commanded current;
the observed current did not reach their configured caps. This single return does
not distinguish friction/deadband, controller compensation, contact or other
mechanical effects. It does show that the installed ring-only profile is
insufficient to qualify whole-hand home from this starting pose. Earlier successful
isolated thumb profiles have not yet been qualified together with this ring profile.

## Diagnostics and final state

Across both sessions, 113,184 native diagnostic frames
were observed, including disabled waiting time. No diagnostic code or additional
oscillation/speed/current/temperature guard triggered. Maximum observed reported
speed was 1.060 rad/s, current
0.186 A, and MCU temperature
58.55 °C. No independent physical speed or visual
no-twitch observation was collected, so absence of a detector trigger does not
prove absence of every small twitch.

The test used the real RAX MCP frontend/backend and hand service. A test-only
WujiDriver subclass latched native diagnostic events and added the previously
used early oscillation guard plus speed/current/temperature checks. The production
controller handled disable and stop confirmation. Camera streaming and motion
archives were disabled only for these sessions to leave the existing viewer alone.

The hand is disabled, unowned, fresh and fault-free; preparation readback matches
the saved profile. The xArm TCP position was unchanged at
`[0.2531, -0.00134, 0.31412]` m; it ended stopped in state 4 with error code 0 after
the backend's assembly stop (initial state 5). No arm motion command was issued.
The saved home angles, gains, limits and production implementation were unchanged.

Evidence is retained in
[the test directory](../.local-runtime/wuji-home-retest-20261027/), including both
sessions' MCP results, service/native traces, guards, readbacks and scripts.
[summary.json](../.local-runtime/wuji-home-retest-20261027/summary.json) contains
machine-readable metrics. No commit or external upload was made.
