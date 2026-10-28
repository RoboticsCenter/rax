# Wuji home-return gain trials — 2026-10-27

Later evidence: the [expanded firmware-2.13.1 parameter study](wuji-parameter-sweep-2026-10-27.md)
found substantially better repeated thumb-home tracking with higher stiffness
and motion feed-forward, then stopped coordinated testing on a ring S1 encoder
warning. The measurements below describe the earlier, narrower gain trials.

The operator requested impedance tuning with repeated home returns. The initial
five gain combinations failed to reach whole-hand home. After the operator
confirmed the thumb was clear, seven additional isolated thumb S2 trials showed
that **lower damping improves some moves, but no tested profile provides reliable
bidirectional returns to the saved home**. At kp 4.0, kd 0.05, thumb S2 reached
its home target twice from the positive side with about 0.0284 rad (1.63°) residual
error, then stalled on the negative-side probe. The sweep was stopped.

A subsequent controlled comparison at kp 4.0 and kd 0.05/0.06/0.075 used one
shared posture and the same positive-side target sequence. **All three failed
their first thumb-home return at 0.06049–0.06072 rad error.** The other thumb
joints had been repositioned since the earlier sweep, so this is a separate
comparison, not a reversal of the earlier successful measurements. No profile
was promoted; the original gains were restored again with motors disabled.

The later [native SDK isolation tests](wuji-sdk-isolation-2026-10-27.md) reproduced
the same approximately 0.061 rad error without MCP or any RAX control modules,
at both 120/200 Hz and with quintic/linear interpolation. The tested configuration
and SDK/firmware/actuator behavior remain candidates; a RAX wrapper is not needed
to reproduce the symptom.

Subsequent [Hand 2 controls research](wuji-hand2-controls-research-2026-10-27.md)
found relevant friction/feedforward implementations and an apparent current-law
scale in the saved native traces. All velocity peaks below are SDK-reported;
they have not been independently verified as physical output speeds.

The hand was confirmed disabled after every motion trial, with the xArm stopped
at its original pose. The original kp 1.5, kd 0.1, 0.5 A profile was restored and
read back while the motors remained disabled after both stages. Final verification
found all twenty joints Ready/disabled, fresh feedback, no diagnostic codes, and
xArm state 4 with no error or warning. No candidate was installed as a recommended
profile. The saved home angles and main configuration are unchanged by tuning.

## Method and constraints

The actual right hand was `WH2KA01260823017`, firmware 2.6.0, SDK 2026.10.12.
Each candidate used a separate MCP session and explicit disabled-state
preparation, with parameter readback before enabling. The arm was never enabled
for movement. The camera viewer remained running independently.

Only kp and kd changed. The 0.5 A current cap, 0.35 rad maximum step, 0.5 rad/s
commanded velocity, 1 rad/s² acceleration, 0.040 rad settling tolerance and
three-second settling timeout were retained. Each trajectory lasted two seconds;
large home returns were divided into steps of at most 0.25 rad. Successful calls
were followed by one second of measured holding to check error and stability.
Independent status samples recorded joint position, target, velocity, current,
freshness, diagnostics, temperature and voltage.

The intended comparison was to reach home, then repeatedly move every joint by
±0.10 rad around home and call `wuji_pose(name="home")` from both directions.
**Those whole-hand repeat cycles never began**, because the initial home approach
did not pass. Later isolated thumb returns are described separately below.
Approach trials started from different measured poses after previous stops;
their residual errors are not a controlled ranking of gains. In particular, the
smallest listed error does not establish kp 3.0 as an optimum.

## Initial whole-hand trials

| kp | kd | Failed target | Largest residual | Joint | Peak sampled current | Result |
| ---: | ---: | --- | ---: | --- | ---: | --- |
| 2.0 | 0.10 | Second approach waypoint | 0.04925 rad / 2.82° | Thumb S2 | 0.230 A | Settling failure; disabled |
| 3.0 | 0.10 | Approach waypoint | 0.04125 rad / 2.36° | Thumb S2 | 0.279 A | Settling failure; disabled |
| 4.0 | 0.10 | Approach waypoint | 0.04714 rad / 2.70° | Thumb S2 | 0.484 A | Settling failure; disabled |
| 4.0 | 0.15 | Saved home | 0.04884 rad / 2.80° | Thumb S2 | 0.493 A | Settling failure; disabled |
| 5.0 | 0.15 | Saved home | Interrupted by firmware warning | Thumb S2 | 0.512 A | `Stall`; disabled |

At kp 4.0, kd 0.15, the other fingers' maximum home errors were approximately
0.02186 rad (index), 0.02033 rad (middle), 0.01889 rad (ring), and 0.02292 rad
(pinky). Thumb S2 prevented the whole-hand success result.

During the kp 5.0 trial, thumb S2 remained near 0.1147 rad against the saved
0.064535 rad home target, drawing approximately −0.483 A when the warning was
observed. The SDK identified code 7 as `Stall`, severity `Warning`, with automatic
clearance after the load was removed. The device reported a prolonged combination
of low motor speed and high current. RAX treats a nonzero diagnostic code as a
reason to stop. The warning cleared after disable; it was not manually cleared.

Brief measured velocity peaks reached 1.17–1.34 rad/s in the larger approach
trials, despite the lower commanded-trajectory bound. The experiments therefore
do not qualify actual motor-speed limits. Final-hold samples for the stalled
thumb were essentially stationary rather than oscillatory. Peak sampled current
in the final trial slightly exceeded the configured 0.5 A cap; a command cap is
not a guarantee that every physical feedback sample remains below it. Maximum
reported joint temperatures during the trials were about 56.1 °C.

## Continuation after the clearance check

The operator inspected the disabled hand and reported: “Thumb is clear; no visible
obstruction.” This checks external contact; it does not establish internal
mechanical condition. The camera viewer does not directly show the thumb.

Only thumb S2 was commanded to move in these trials. The other nineteen joints
held their measured starting targets at kp 1.5, kd 0.1. The thumb's current cap
remained 0.5 A. Targets were ±0.08 rad around the measured starting angle or the
saved thumb home angle, 0.064535 rad. In the final trial an explicit home +0.12 rad
preposition tested whether approach distance affected the result. The full hand
was not commanded to its saved home in these isolated trials.

| Thumb kp | Thumb kd | Test | Result |
| ---: | ---: | --- | --- |
| 3.0 | 0.150 | Positive 0.08 rad probe | Failed settling, residual 0.04600 rad |
| 4.0 | 0.150 | Negative 0.08 rad probe | Failed settling, residual 0.04500 rad |
| 5.0 | 0.150 | Negative 0.08 rad probe | Thumb S2 `Stall`; essentially no movement |
| 4.0 | 0.050 | Both directions and returns around measured starting angle | All four calls passed; maximum errors 0.01924–0.02805 rad |
| 4.0 | 0.075 | Home-centered positive probe and return | Initial target and probe passed; return failed at 0.05077 rad |
| 4.5 | 0.050 | Direct approach to thumb home | Failed at 0.05057 rad; essentially stationary |
| 4.0 | 0.050 | Positive preposition, home, then home-centered probes | Two thumb-home approaches passed at 0.02838 and 0.02843 rad; negative probe triggered `Stall` |

The successful four-call trial used a measured center near 0.115 rad, not the
saved 0.064535 rad home angle. It therefore did not prove repeatable home. At the
saved home angle, the final trial passed both a return from home +0.12 rad and a
return from home +0.08 rad. The subsequent home −0.08 rad target was −0.015465 rad;
thumb S2 stalled near +0.06247 rad with approximately −0.497 A measured current.
RAX disabled the hand on diagnostic code 7. It did not accept that move as reached.

Lower damping also increased brief SDK-reported thumb-velocity peaks: 1.91 rad/s
in the four-call success and 2.09 rad/s in the final trial. The nominal 0.5 rad/s
bound limits the commanded trajectory; these telemetry peaks require validation
against physical output motion. Successful
one-second holds were steady (maximum observed joint peak-to-peak motion below
0.00020 rad), but that does not qualify transient speed or other starting poses.
Across the continuation, peak sampled current was 0.533 A and maximum temperature
was approximately 56.6 °C. No current limit was raised. An independent observer
stopped on hardware diagnostics, persistent stale feedback, temperature ≥65 °C,
or measured speed above 1 rad/s lasting over 0.12 seconds; brief peaks did not
satisfy that duration condition.

Read-only firmware inspection found thumb S2 `soft_limit_enabled=0`; the zero
`soft_pos_min` and `soft_pos_max` values therefore were not enabled position
clamps. Position/current/velocity limit flags were inactive when inspected with
the hand disabled. This does not establish their state during motion. No origin,
firmware, software limits or fault-clear parameters were changed.

## Controlled positive-side comparison

The operator then authorized the proposed repeatability test. Each candidate
kept thumb S2 kp at 4.0 and selected kd 0.05, 0.06 or 0.075. All twenty current
caps remained 0.5 A. The intended sequence was five repetitions of the same two
targets: thumb S2 at home +0.12 rad (0.184535 rad), then at saved home
(0.064535 rad). Each trajectory took two seconds, with the unchanged 0.040 rad
tolerance and three-second settling timeout. Each candidate stopped on its
first unsuccessful motion; retries were not forced through a failure.

The other nineteen joints used one shared target vector captured at the start
of this comparison, at kp 1.5 and kd 0.1. This posture differed substantially
from the previous sweep: for example, thumb S1/S3 were approximately
−0.203/−0.384 rad rather than +0.175/+0.254 rad. It was retained rather than
commanding a whole-hand reposition. Non-probed starting angles between candidates
varied by at most 0.00345 rad. The saved-home approaches began at measured thumb
S2 angles of 0.15710, 0.15600 and 0.15701 rad—a spread of only 0.00110 rad.

| Thumb kd | Outward move | Home attempts passed | Home error | Peak speed over trial | Peak speed during home approach |
| ---: | --- | ---: | ---: | ---: | ---: |
| 0.050 | Passed | 0/1 | 0.060495 rad / 3.466° | 1.969 rad/s | 0.300 rad/s |
| 0.060 | Passed | 0/1 | 0.060594 rad / 3.472° | 1.728 rad/s | 1.728 rad/s |
| 0.075 | Passed | 0/1 | 0.060725 rad / 3.479° | 0.963 rad/s | 0.963 rad/s |

The remaining four repetitions for each candidate were unattempted, so these
are three failed first-return tests, not fifteen executed returns or an estimate
of long-run success probability. All calls used `wuji_move_joints` to isolate the
thumb-home angle; whole-hand `wuji_pose(name="home")` was not exercised here.

All three stopped moving near 0.125 rad against the 0.064535 rad target. During
the final second of enabled holding, thumb S2 position varied by just
0.000014–0.000018 rad, while mean measured current was −0.478, −0.481 and
−0.482 A respectively. This is near the 0.5 A cap, but the traces alone do not
prove that a firmware current-limit flag was active. Every candidate failed
settling; **none produced a firmware diagnostic warning in this comparison**.
Peak measured current was 0.508 A and maximum reported temperature was 55.95 °C.

The experiment added an immediate sampled-speed stop above 2.0 rad/s and kept
the previous stop above 1.0 rad/s sustained for over 0.12 seconds. Neither guard
triggered. These are experiment stop thresholds, not qualified operating speed
limits. Speeds were sampled through the independent status observer, so a shorter
peak could be missed. The first outward move began at a different thumb angle
from the later candidates; whole-trial peaks therefore do not isolate a damping
effect. Home-only speed peaks are shown separately and were not monotonic in kd.

The matched home approaches establish that this damping range did not materially
improve final error in the tested posture. They do not explain why the earlier
posture permitted successful returns. Posture-dependent load/friction remains a
hypothesis, not a diagnosed fault. Expanding the kd grid has little support from
these results; the persistent near-cap, stationary residual needs diagnosis.

Each trial confirmed disabled hand feedback and xArm state 4. The original
kp 1.5, kd 0.1, 0.5 A profile was restored and read back for all twenty joints;
the final arm readback had no error or warning and the same recorded pose.
No production controller or configuration changed. Evidence is separate from
the earlier sweep:

```text
.local-runtime/wuji-repeatability-20261027/
  repeatability.py
  shared-baseline.json
  comparison.json
  kp4-kd005/
  kp4-kd006/
  kp4-kd0075/
  final-restored-profile.json
```

Each candidate directory contains its exact configuration, full MCP responses,
parameter readback, feedback trace, measurements and stop confirmation. Error
figures above use the controller's failed-settling snapshot; a later status
snapshot can already reflect relaxation after disable.

## Interpretation and next step

The approach-dependent residuals, near-zero velocity with substantial current,
and sudden motion with lower damping are consistent with friction/stiction or
load interacting with the current-limited impedance controller. They do not
prove a damaged joint or establish that damping alone is the cause. The trials
also used different starting poses, so their residuals are not a controlled
ranking of gains. No “least error” production profile can be justified yet.

The visible clearance check and disabled soft limits narrow the possibilities,
but internal binding, transmission friction, compensation behavior and
insufficient current for this joint remain unresolved. The next useful step is
a thumb S2 mechanical/actuator diagnosis with the vendor, using the saved traces
and the repeatable high-current, low-motion symptom. A larger current cap was
not tested. Once the stall cause is understood, repeat the same offsets and
directions, check actual speed, and require whole-hand named-home success before
promoting gains. The original low-gain profile remains a commissioning profile,
not a verified fix.

The vendor documents the MIT impedance law and commissioning parameter guidance
in its [control guide](https://docs.wuji.tech/docs/en/wuji-hand/latest/control-guide).
These examples informed the bounded search; they do not qualify a profile for
this individual hand.

## Evidence

All configurations, MCP responses, parameter readbacks, per-trial summaries and
feedback traces are retained in:

```text
.local-runtime/wuji-tuning-20261027/
  comparison.json
  kp2-approach-fixed/
  kp3-approach/
  kp4-approach-retry/
  kp4-kd015-approach/
  kp5-kd015-benchmark/
  restored-profile.json
  continuation-comparison.json
  thumb-s2-kp3-bidirectional/
  thumb-s2-kp4-reverse/
  thumb-s2-kp5-reverse/
  thumb-s2-kp4-kd005/
  thumb-s2-kp4-kd0075-home/
  thumb-s2-kp45-kd005-home/
  thumb-s2-kp4-kd005-home-approach/
  thumb-s2-firmware-inspection.json
  inspection-restored.json
  final-restored-profile.json
```

Two additional aborted attempts are retained separately: `kp2-approach` hit a
test-harness array-format error before dispatching a trajectory; `kp4-approach`
refused an arm state of 5 before enabling the hand. The follow-up explicitly
requested assembly hold and verified state 4 before continuing. The initial sweep's stall
caused overlapping independent stop requests; one earlier request reported
cancellation, and the final stop independently confirmed disabled feedback.

No production controller code was changed for this sweep. One-off tuning code
stays with the ignored evidence. Artifact timestamps are October 28 UTC; this
report uses the operator's October 27 local date.
