# Wuji shortlist resumption — 2026-10-27

**The guarded resumption reproduced the encoder-quality warning, on a different
joint of the ring finger.** A gentle ring S1 check passed; the following
coordinated ring trial triggered `Enc1BitRate` on ring S3 during its final return.
The harness disabled all twenty joints and stopped the remaining comparison.
There is no new evidence for selecting a best full-hand configuration.

The subsequent [encoder investigation](wuji-encoder-isolation-2026-10-27.md)
analyzes the pre-warning angle/current oscillation, disabled hardware diagnostics,
operator observations and public reports. It adds no further motion trial and
does not yet establish the warning's root cause.

The operator explicitly requested continuing the shortlist on more hand motions
after reviewing the [earlier sweep](wuji-parameter-sweep-2026-10-27.md). This
resumption retained the diagnostic, temperature, feedback, trajectory and speed
stops. The earlier run and its STOP marker were preserved. No reset, manual fault
clear, protection change or arm motion was performed.

## Intended comparison and actual coverage

The first stage was a matched ±0.04 rad flexion cycle for each finger with three
profiles: Kp 15/Kd 0.15 position-only, Kp 5/Kd 0.1 plus 0.20 A motion-friction
feed-forward, and Kp 8/Kd 0.1 with the same feed-forward. Each active joint's
current cap was 1.5 A. Only passing candidates would advance to larger ±0.08 rad
moves, simultaneous flexion, reversals, staggered finger waves and repeats.

These comparisons did not proceed beyond the initial ring check. Of fifteen
prepared per-finger cases, only `flex04-ring-stiff` ran. Prepared specifications
are not test results. The [new trial CSV](wuji-shortlist-validation-2026-10-27.csv)
contains exactly two executed trials:

| Trial | Motion and active profile | Outcome |
| --- | --- | --- |
| Gentle ring S1 | One joint, ±0.02 rad, 4 s ramps; Kp 5/Kd 0.1/1.0 A | Four waypoints met tolerance, no diagnostic |
| Coordinated ring | S1/S3/S4, ±0.04 rad, 2 s ramps; Kp 15/Kd 0.15/1.5 A | Three moves qualified; final return stopped on ring S3 code 3 |

Inactive joints held freshly measured angles at Kp 1.5/Kd 0.1/0.5 A. S2 lateral
joints were held fixed, with a 0.01 rad drift stop. Commands used 120 Hz quintic
interpolation; active endpoint tolerance was 0.02 rad, all-joint tolerance
0.04 rad, with a one-second hold after settling. The arm stayed in state 5 at
the same pose, without errors or warnings.

The gentle check's largest error was 0.01973 rad. Its tolerance was almost as
large as the commanded excursion, so it establishes a warning-free small probe,
not accurate tracking of the whole excursion. The first three coordinated moves
had active endpoint errors 0.00426, 0.00256 and 0.00198 rad. The final return was
interrupted and is not a passed waypoint.

## The recurring warning

| Observation | Earlier coordinated trial | Resumed coordinated trial |
| --- | --- | --- |
| Joint | Ring S1, flat index 12, NID 16 | Ring S3, flat index 14, NID 18 |
| Diagnostic | Code 3, `Enc1BitRate` | Code 3, `Enc1BitRate` |
| Phase | Final return hold | Final return |
| Active gains/current cap | 15/0.15/1.5 A | 15/0.15/1.5 A |
| Reported joint current | −0.130 A | +0.0402 A |
| MCU temperature | 55.0 °C | 53.5 °C |
| Current-limit flag | False | False |
| Communication response/timeouts | 100% / 0 | 100% / 0 |

The installed SDK describes code 3 as excessive Encoder 1 data-quality warning
flags, severity `Warning`, with automatic clearing when signal quality recovers.
It is an encoder diagnostic, not an Ethernet bitrate error. The captured
[second event record](wuji-encoder-recurrence-2026-10-27.json) includes the exact
native sequence/timestamp, diagnostic catalog entry, profile and stop evidence.

At the second event, ring S3 measured 0.36778 rad, its final target was 0.35709 rad,
reported velocity was −0.1044 rad/s, and supply telemetry was 12.394 V. The event
does not indicate current saturation or a recorded network timeout. Telemetry
does not independently establish physical joint velocity or fingertip force.

Both events occurred during coordinated ring flexion at the same gains, but
different joints and different starting postures were involved. The successful
lower-gain S1 probe is not a matched control for S3. These data cannot distinguish
sensor/magnet issues, firmware behavior, vibration/control interactions or another
cause. They also do not establish that this warning explains the earlier thumb
tracking error. Two occurrences make dismissing it as an isolated event
unsupported; changing gains merely to avoid the warning would not establish
reliable feedback.

## Monitoring and final state

A fresh 15-second disabled preflight saw no warning in 15,155 native diagnostic
frames and confirmed the original gains/current caps. The new harness latches
every nonzero diagnostic received from the native stream, so a brief warning
cannot disappear between watchdog checks. An offline synthetic-frame check
verified that a warning stays latched after a following healthy frame. Runtime
stop thresholds were not relaxed.

All twenty joints were confirmed disabled about 28.8 ms after the warning frame.
This is observed feedback timing, not a guaranteed physical stop latency. The
warning was still present at that first disabled snapshot; restoration was
deferred until a separate inspection found it clear about 48.1 seconds later.
The exact automatic-clear time was not measured. Original Kp 1.5/Kd 0.1/0.5 A
settings were then restored and read back for all twenty joints.

A subsequent 15-second disabled check saw no warning in 15,152 diagnostic frames.
No new motion was
performed after the second event. The hand remains disabled and a new STOP
marker prevents further batch runs. Production controller code, gain limits,
saved home and configured operating profile are unchanged.

The next useful investigation is the recurring ring encoder signal, using the
captured native records and Wuji's guidance. The earlier successful isolated
tests remain valid observations, but neither the stiff profile nor the
feed-forward alternatives are qualified for full-hand operation by this run.

## Evidence

Raw traces, exact harness/specifications, measured baseline, comparison plan,
restoration readbacks and disabled monitoring are retained in
`.local-runtime/wuji-shortlist-validation-20261027/`. The updated
[candidate disposition](wuji-tested-candidates-2026-10-27.json) records the
interruption without promoting a profile. All motor commands in this run used
the native SDK directly; no RAX controller or MCP wrapper was involved.

The [local evidence archive](../.local-runtime/wuji-shortlist-validation-20261027/wuji-ring-encoder-recurrence-evidence-2026-10-27.zip)
contains both ring warning events, raw traces, exact harnesses, restoration
evidence, a fresh 1,670-record flash-log export and a SHA-256 manifest. It has not
been sent to anyone. The archive and raw runtime files are ignored by Git.
