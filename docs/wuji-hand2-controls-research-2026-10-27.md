# Wuji Hand 2 controls research — 2026-10-27

**A controls/firmware explanation is credible and deserves a focused test before
concluding that this hand is defective.** The strongest external match is an
independent Hand 2 project documenting friction compensation and an apparent
gain scale that also appears in our saved telemetry. It is not evidence that
another damping sweep alone will fix the saved-home error.

The researched device was right Wuji Hand 2 `WH2KA01260823017`, firmware **2.6.0**, using
`wuji-sdk` **2026.10.12**. This research used web sources and offline analysis of
existing recordings. It did not connect to the hand, execute downloaded control
code, change firmware/configuration, or perform new motion.

Later that day the operator authorized a separate
[upgrade to firmware 2.13.1](wuji-firmware-upgrade-2026-10-27.md). The measurements
below remain evidence from 2.6.0. The subsequent
[vendor-profile test](wuji-vendor-profile-test-2026-10-27.md) on 2.13.1 still
failed initial thumb-home settling, with about 0.05067 rad residual at a 1.5 A cap.

The later [expanded search and parameter study](wuji-parameter-sweep-2026-10-27.md)
adds official real-Hand-2 deployment profiles, 95 thumb trials and all-finger
tests. Higher stiffness and motion feed-forward improved tracking, but a ring S1
encoder warning stopped coordinated qualification. No profile was promoted.

## What others have implemented

Younghyo Park's experimental
[`aiowuji`](https://github.com/younghyopark/aiowuji/blob/0fca23cc855369c9c258ad16c087e8cb00d1a7f7/README.md)
specifically targets **Hand 2**. The author reports slow-motion stick-slip and
tests on firmware 2.10.0. On that hand, firmware compensation had no measurable
effect when calibration bits were all zero. These are one author's observations,
not a replicated fix for thumb-home accuracy.

Its [controller implementation](https://github.com/younghyopark/aiowuji/blob/0fca23cc855369c9c258ad16c087e8cb00d1a7f7/src/aiowuji/controller.py)
streams both trajectory position and velocity and adds bounded current
feedforward proportional to `tanh(desired_velocity / friction_velocity)`.
Our RAX and native comparison commands supply zero desired velocity and zero
feedforward. That is a concrete control-path difference to test. Because this
friction term vanishes at zero desired velocity, it does not directly compensate
the final stationary home bias.

The project's [firmware model](https://github.com/younghyopark/aiowuji/blob/0fca23cc855369c9c258ad16c087e8cb00d1a7f7/src/aiowuji/sim.py)
uses experimentally fitted scales of approximately 1.95 on Kp and 1.9 on Kd,
plus 2 ms command lag. Its
[identification script](https://github.com/younghyopark/aiowuji/blob/0fca23cc855369c9c258ad16c087e8cb00d1a7f7/scripts/sysid_fit.py)
fits gain scales using unsaturated samples and searches command delay. These are
measured model parameters, not vendor specifications or constants to copy into
our controller.

The [host PD example](https://github.com/younghyopark/aiowuji/blob/0fca23cc855369c9c258ad16c087e8cb00d1a7f7/examples/02_host_pd.py)
also supports running a controller outside firmware by zeroing firmware gains
and publishing clipped current. It cautions that reported velocity may be
nonzero while the output is stationary and suggests small derivative gain or
filtered position differences. It implements PD, not integral compensation.

Wuji's [official release notes](https://docs.wuji.tech/docs/en/release-notes)
introduce built-in feedforward with firmware **2.10.0**, requiring SDK
2026.9.22 or later, and restrict applicability to Hand 2 units **shipped from
October 2026 onward**. The tested 2.6.0 firmware predates it; hardware eligibility and
calibration remain unknown. Firmware 2.12.0 changes new/reset defaults to
Kp 5, Kd 0.1 but preserves saved parameters on upgrade. The newer firmware's
default SDK port is 50001 rather than the then-configured 7447. An upgrade therefore
requires compatibility checking and is not an established accuracy fix.

Another integrator's [Gento Apex Hand 2 instructions](https://gento-teleoperation-apex.github.io/Apex_doc_center/end-effectors/wuji2)
document Kp 5, Kd 0.15 and a 1.5 A current limit, with position filtering. This
shows a different operating profile from our 0.5 A commissioning limit; the page
does not provide a matched thumb-home accuracy benchmark. Its values do not
establish that our current limit should be raised.

## What our existing recordings add

The [native SDK comparison](wuji-sdk-isolation-2026-10-27.md) already reproduced
the approximately **0.061 rad / 3.48°** home error without RAX imports. Neither
120 versus 200 Hz nor quintic versus linear interpolation removed it. All three
trials used thumb Kp 4, Kd 0.075 and a 0.5 A limit. At the failed endpoint,
reported current was about −0.48 A and current-limit flags were inactive.
There is no demonstrated publisher starvation or RAX wrapper defect in those
trials, and no demonstrated current-limit saturation.

I reanalyzed the last 0.8 seconds of each positive hold and failed home hold.
Each window contains 97–161 unique state frames after sequence deduplication.
Wuji's [control guide](https://docs.wuji.tech/docs/en/wuji-hand/latest/control-guide)
defines effort as amperes and documents the nominal PD-plus-feedforward law.
With our zero velocity/effort targets, its prediction is:

```text
I_nominal = 4 * (q_command - q_measured) - 0.075 * velocity_reported
```

| Native trial | Hold | Nominal mean current (A) | Reported mean current (A) | Fitted apparent scale |
| --- | --- | ---: | ---: | ---: |
| 120 Hz quintic | Positive | +0.11050 | +0.21894 | 1.9813 |
| 120 Hz quintic | Home | −0.24274 | −0.48103 | 1.9816 |
| 200 Hz quintic | Positive | +0.11031 | +0.21848 | 1.9806 |
| 200 Hz quintic | Home | −0.24266 | −0.48119 | 1.9829 |
| 200 Hz linear | Positive | +0.11059 | +0.21901 | 1.9803 |
| 200 Hz linear | Home | −0.24313 | −0.48202 | 1.9826 |

The scale is the least-squares fit through the origin,
`sum(I_nominal * I_reported) / sum(I_nominal**2)`.
The positive holds are well below the cap, so this observation is not based
only on the near-cap home holds. Applying the external model's 1.95 factor
reduces current prediction RMSE from roughly 0.109–0.239 A to 0.010–0.014 A.

**This is a telemetry-level match, not an independently calibrated measurement
of physical current.** It cannot distinguish firmware scaling from telemetry
scaling, separately identify the derivative gain, or establish command delay.
It also does not show that the scaling causes the home failure. We should not
automatically divide all gains by two.

The stationary endpoint is consistent with a PD controller maintaining force
against friction or another posture-dependent load. With approximately zero
velocity, changing Kd contributes little to the final equilibrium. This explains
why a damping change can improve transient motion without eliminating static
error. It remains an inference: internal binding, contact, spring load, feedback
behavior and firmware remain possible contributors.

The earlier SDK velocity peaks also need qualification. They are **reported
velocity**, not independently verified physical output speed. We should compare
velocity against timestamped position changes before using those peaks to rank
damping profiles. Existing experiment stops remain in place.

## Recommended next comparison

1. Establish the effective current law and velocity interpretation using the
   existing recordings first. Preserve separate command, state and diagnostic
   timestamps; use unsaturated moving samples to study Kd and lag separately.
2. Verify this serial's eligibility and calibration for firmware feedforward
   before considering an upgrade. The release note's shipment restriction and
   the independent author's uncalibrated-hand result both matter.
3. Use the standalone native harness for a matched comparison: current baseline,
   then nonzero trajectory velocity, then bounded motion-friction feedforward.
   Keep the posture, approach direction, current cap and stopping rules fixed.
   Evaluate transient tracking and final home error separately.
4. If the static residual persists, identify holding load versus posture and
   approach direction. A bounded holding-bias term or integral term with
   anti-windup is a possible subsequent experiment, **our proposal rather than
   a verified Hand 2 community fix**. Do not accumulate effort through a stall.
5. Treat current-cap changes as a separate vendor-profile comparison. Near-cap
   current without an active limit flag is insufficient evidence that merely
   increasing the cap will solve the problem.

Another dense Kd sweep has lower diagnostic value than these comparisons.
Successful home qualification still requires repeated returns from both
directions and the intended whole-hand posture.

## Search scope and evidence

The search covered Wuji documentation/release notes, official and independent
GitHub repositories and issues, English/Chinese queries, Reddit, ROS Discourse
and Robotics Stack Exchange. I did not find a publicly documented, resolved
**Hand 2 thumb-home** issue matching our symptom. Neither the official SDK nor
aiowuji repository had GitHub Discussions enabled when checked.

First-generation `wujihandpy` reports, including
[#28](https://github.com/wuji-technology/wujihandpy/issues/28) and
[#29](https://github.com/wuji-technology/wujihandpy/issues/29), were excluded as
Hand 2 fixes. Simulation-only results were not treated as hardware validation.

The independent code was inspected at commit
`0fca23cc855369c9c258ad16c087e8cb00d1a7f7` (2026-10-21). Downloaded source and
offline calculation output are retained under
`.local-runtime/wuji-controls-research-20261027/`; native inputs remain under
`.local-runtime/wuji-sdk-isolation-20261027/`. These are ignored local evidence,
not production dependencies. The report date uses the operator's local time.
