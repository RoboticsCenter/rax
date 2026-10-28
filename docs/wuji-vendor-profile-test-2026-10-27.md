# Wuji Hand 2 vendor-profile test — 2026-10-27

Follow-up: the [expanded parameter study](wuji-parameter-sweep-2026-10-27.md)
found better tracking with other gains and motion feed-forward. Coordinated
qualification later stopped on a ring S1 encoder warning. This report preserves
the earlier matched comparison with the teleoperation example's profile.

**The vendor profile improved the thumb S2 home error by about 20%, but did not
meet the existing 0.040 rad settling tolerance.** On firmware 2.13.1, Kp 5,
Kd 0.1 and a 1.5 A limit left 0.05067 rad (2.90°) error. The commissioning
baseline, Kp 1.5/Kd 0.1/0.5 A, left 0.06318 rad (3.62°). Both trials stopped
after their initial failed home approach. No diagnostic warning occurred.

The operator explicitly requested testing the vendor's operating profile after
the [firmware upgrade](wuji-firmware-upgrade-2026-10-27.md). Wuji's current
[Hand 2 teleoperation example](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/retargeting/1.teleop_real.py)
sets Kp 5, Kd 0.1 and 1.5 A for all joints; its
[control guide](https://docs.wuji.tech/docs/en/wuji-hand/latest/control-guide)
points to that example for recommended values. These exact parameters were set
and read back on all twenty joints for the vendor trial.

## Method

The native SDK harness was adapted from the previous
[SDK isolation experiment](wuji-sdk-isolation-2026-10-27.md). It imports
`wuji_sdk` directly and does not use MCP, RAX's controller or its SDK wrapper.
The installed SDK remains 2026.10.12. The hand is right `WH2KA01260823017`,
connected at `192.168.1.111:50001`.

Both trials used one shared posture captured with the motors disabled after
the firmware upgrade. Only thumb S2 received a changed target: saved home at
0.064535 rad. The other nineteen joints held the shared measured posture.
Between trials their initial measured angles differed by at most 0.00001574 rad;
thumb S2 starting angles differed by 0.00048161 rad. This is a matched comparison
on the new firmware, not an identical-posture repeat of the older firmware tests.

Each trajectory used 120 Hz quintic position interpolation over two seconds,
zero desired velocity and zero effort feedforward, followed by up to three
seconds for settling. Success required all joints within 0.040 rad for 0.2 seconds,
then a one-second stable hold. Motors were enabled only after parameter readback
and seeding measured positions. The xArm remained in state 5 with no error or
warning; no arm writes were performed.

The retained experiment stops include nonzero joint diagnostics, stale/invalid
feedback, temperature ≥65 °C, a command gap over 0.1 seconds, unexpected loss
of Enabled state, tracking error over 0.35 rad, and SDK-reported velocity above
2 rad/s or above 1 rad/s for over 0.12 seconds. SDK-reported velocity is not an
independently verified measurement of physical output speed.

## Results

| Measurement | Commissioning baseline | Vendor profile |
| --- | ---: | ---: |
| Kp / Kd | 1.5 / 0.1 | 5 / 0.1 |
| Current limit | 0.5 A | 1.5 A |
| Thumb start | 0.128191 rad | 0.127709 rad |
| Thumb final | 0.127717 rad | 0.115207 rad |
| Final error | 0.063182 rad / 3.62° | 0.050672 rad / 2.90° |
| Mean reported current, last second | −0.18735 A | −0.50177 A |
| Peak absolute reported current | 0.21903 A | 0.54027 A |
| Position variation, last second | 0.00002396 rad | 0.00002170 rad |
| Peak SDK-reported velocity | 0.00732 rad/s | 0.53379 rad/s |
| Maximum reported temperature | 56.76 °C | 56.58 °C |
| Thumb current/velocity/position limit flags active | No | No |
| Active diagnostic codes | None | None |
| Initial home approach | Failed settling | Failed settling |

The held fingers' endpoint errors were below 0.000012 rad. That demonstrates
holding this posture, not their ability to reach different targets. The thumb
was the failing joint. Command rates were approximately 120.00 Hz, with maximum
send gaps below 9.70 ms; there is no sign of command starvation in these trials.

The vendor trial planned three positive-side returns from home +0.12 rad,
followed by three negative-side returns from home −0.08 rad, conditional on a
successful initial home approach. **None of those cycles began.** These results
are one initial approach per profile, not six failed repeats or full-hand home
qualification.

## Interpretation and final state

The larger profile made a measurable difference, but a 1.5 A ceiling did not
make the controller deliver 1.5 A. At the stationary vendor endpoint it reported
about 0.50 A, without an active current-limit flag. This does not support the
configured current cap as the explanation for this remaining error.

The prior apparent current-law scaling persists in the new firmware's saved
telemetry. Fitting reported current against nominal PD current over the last
0.8 seconds gives approximately 1.9794 for the baseline and 1.9799 for the vendor
profile. For the latter, `1.98 * 5 * 0.05067 ≈ 0.502 A`, consistent with the
observed stationary effort. This is not an independent physical-current
calibration or proof of the firmware's internal law.

Both gain and current limit changed, so this comparison does not separately
identify their effects. Static friction/load, mechanical constraints and
compensation behavior remain candidates. The firmware upgrade and vendor
profile have not established reliable home. No feedforward switches, origin,
calibration, factory reset or persistent parameter writes were used.

Each trial disabled the hand and restored/read back the existing Kp 1.5,
Kd 0.1 and 0.5 A profile. A separate final inspection again confirmed all twenty
joints Ready/disabled, no active errors and the original arm pose. The vendor
profile was not installed in the main configuration after failing the test.

## Evidence

```text
.local-runtime/wuji-vendor-profile-20261027/
  compare_profile.py, shared-baseline.json
  baseline-kp15-kd01-cap05/{report.json,trace.json}
  vendor-kp5-kd01-cap15/{report.json,trace.json}
  baseline.log, vendor.log
  comparison.json, additional-analysis.json
.local-runtime/wuji-firmware-upgrade-20261027/
  vendor-test-preflight.json, vendor-test-final.json
```
