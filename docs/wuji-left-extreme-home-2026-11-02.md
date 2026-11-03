# Left-hand extreme poses and return to home

Real MCP commands moved left Hand 2 `WH2JA01260811007` through opposing-finger
poses and both alternating five-finger patterns, then returned to the previously
saved **current left-hand home**. All five fingers participated through their
eighteen active joints. `thumb_S3` and `middle_S1` remained native Stopped with
zero Kp, Kd and current limits throughout the recorded samples.

## Results

| Physical cycle | Largest active home-return error |
| --- | ---: |
| Index back, pinky forward | 0.010115 rad / 0.58° |
| Index forward, pinky back | 0.010696 rad / 0.61° |
| Thumb/middle/pinky forward; index/ring back | 0.020715 rad / 1.19° |
| Reversed five-finger pattern, successful repeat | 0.017732 rad / 1.02° |

Index/pinky root targets were approximately −53.1° and +83.1°, with the other
flexion joints on those fingers straightened. The five-finger patterns also
varied the thumb and the middle finger's active flexion joints. These were staged
trajectories, with no per-call active-joint change above 0.18 rad. The saved plans
contain the full targets and paths; excluded entries retain the preparation seed
as passive wire placeholders.

The two-finger sequence completed 37 MCP calls without a new native fault.
The first five-finger pattern completed its outbound and home-return paths. The
reversed pattern then reached its outbound target but stopped during return on
ring S3 `Enc1BitRate (0x0003 / Warning / AutoClear)`: the left profile had not yet
enabled the original branch's encoder-warning allowance.

After the native warning cleared, that exact original allowance was restored.
A fresh path from the stopped pose back to home was screened, followed by a
complete repeat of the reversed pattern and home return: 27 MCP calls including
initial alignment. The same ring S3 warning recurred and was recorded across
1,723 native diagnostic frames **without interrupting motion**. This run had
no blocking native faults. The two five-finger returns were therefore achieved
across the initial run and repeat, not one uninterrupted campaign.

The repeat's peak sampled active current was 0.868 A and maximum temperature
60.75 °C. Twenty commands reported a position-tolerance shortfall while remaining
enabled and holding; subsequent commands continued normally. All home returns
were within the original branch's 0.06-rad tolerance.

## Profile and original branch compatibility

The loaded profile is `left-hand2-partial-v13-controls-20261103`: original
`real/xarmwuji` Kp 8, Kd 0.15, 1.5 A active current cap and exact encoder-warning
allowance. Partial-joint isolation remains explicit. The tested command rate is
200 Hz, velocity 0.15 rad/s, acceleration/jerk 0.5, and per-call step cap 0.2 rad.
The repository's partial-profile fragment and validated bimanual draft now match.

The complete `services/xarm/` implementation is identical to `real/xarmwuji`
commit `899f0c6`. The additional ban on arm motion solely because a hand has
disabled joints was removed from single-arm and bimanual admission. Existing
controller, path and readiness checks remain. Bimanual routing still provides
explicit side selection, paired admission and shared fault handling. The obsolete
test and skills requiring the blanket ban were corrected.

## Final state and evidence

The right hand's original owner stayed running, its controller log was unchanged,
and its command count remained zero. Both arms' joint readbacks were unchanged,
with controller error/warning codes zero. Arm motion was not exercised in this
test; original-arm compatibility was checked by exact source comparison.

The final requested hand stop was confirmed for all twenty joints, and the
left-only session is disarmed with no control ownership. Its host retained
`stop arrived during SDK call` from the deliberate end stop; immediately before
that stop it was holding with no host fault. The active native error codes are
zero; the two isolated native fault codes remain visible. The original bimanual
session is still disarmed with its old left child detached. Its updated draft
requires a coordinated reload before normal bimanual operation.

Session directory:
`/home/jerry/rax-runtime/sessions/20261103T034138Z_left_partial_real_probe/`.

- `extreme-home-summary.json`: measured results, warning history and peer checks.
- `extreme-home-{plan,result}.json` and `extreme-home-telemetry.jsonl`: two-finger run.
- `encoder-warning-policy-off-all-fingers-home-*`: preserved interrupted run.
- `all-fingers-home-{plan,result}.json` and `all-fingers-home-telemetry.jsonl`: successful repeat.
- `operator/left-current-home.json`: the operator-selected left home.
- `geometry/`: pinned official collision model and offline path screens.
- `mcp.jsonl`: actual MCP command audit. No simulated hardware tests were run.
