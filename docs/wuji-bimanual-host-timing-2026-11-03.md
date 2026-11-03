# Bimanual Wuji host timing: November 3, 2026

The right hand stopped while holding after a successful two-arm positioning
command. Its native diagnostics reported no faults. The host requested 1,000 Hz,
achieved approximately 340 Hz, and tripped its 50 ms command-gap watchdog.

Both bimanual profiles now request 200 Hz with a 250 ms host watchdog. The
unchanged feedback-age limit is also 250 ms. Delays over 50 ms are recorded in
`command_timing.delays`; they do not stop otherwise healthy control unless the
hard watchdog is exceeded. Status exposes both thresholds and the measured delay.
Fault notifications carry the specific reason and hand snapshot instead of only
"lost healthy control". Gains, damping, current caps, joint isolation, tracking
limits and reference velocity/acceleration/jerk limits are unchanged.

Real hardware validation used the existing bottle session:

- A deliberate 110.9 ms pause of the right-hand owner during a stationary hold
  produced a 117.5 ms command interval. Both hands stayed armed and holding,
  retained their targets, and resumed commands without faults.
- A 0.1-rad right index-S1 target change produced 0.0897 rad measured movement.
  The outward move and return both reported `reached`, with about 0.010 rad
  maximum final error. Neither arm nor the left hand received a movement target.
- A 120-second stationary hold passed with all four camera streams and twelve
  additional four-camera observations. Rates were approximately 185 Hz left and
  190 Hz right. Arm poses and hand targets remained unchanged; no new faults
  or operator requests appeared.
- The final stop was confirmed on both arms and both hands. The session was
  preserved and left disarmed at its current poses for the operator to resume.

This validates the revised timing policy for the tested delays and load; the
hard watchdog and native/feedback protections remain active. Raw snapshots,
configuration backups, movement results, load samples and a validation summary
are in session `20261103T073444Z_unscrew_bottle_b1d812`, under
`operator/host-timing-20261103/`.
