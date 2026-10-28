# Wuji current and impedance MCP qualification — 2026-10-28

Both `wuji_set_effort` and `wuji_set_impedance` passed a supervised, empty-hand
hardware trial through the production MCP frontend. The operator confirmed
clearance/stop attendance and exited the existing RAX session first. No LLM agent
was started. The arm was already stopped in state 4 and received no motion command.

This qualifies the exercised current commands and impedance writes, not calibrated
fingertip force, contact control, a grasp, or every joint/current combination.

## Setup and procedure

- Right Wuji Hand 2 `WH2KA01260823017`, firmware 2.13.1, SDK 2026.10.12.
- Profile `right-hand2-scurve-20261028-v13`: Kp 8, Kd 0.15, current cap 1.5 A;
  existing velocity, tracking, freshness and 50 ms scheduling guards retained.
- Prepared the measured posture; no saved posture replay.
- Middle S3 (flat index 10) received 0, +0.05, 0, −0.05, 0 A, each with a
  0.5 s ramp and 1 s observation hold. Other joint biases remained zero.
- With zero bias, changed only middle S3 to Kp 7.2, Kd 0.14, cap 1.0 A, then
  restored the original settings. Both operations verified hardware readbacks.
- Independent hand feedback recorder enforced a 0.03 rad excursion bound and
  stopped on unhealthy feedback. Disarmed at completion and confirmed all joints
  disabled with zero commanded bias. No retry/rearming was needed.

## Measurements

| Measurement | Result |
| --- | --- |
| MCP operations completed | Five effort ramps, two impedance writes |
| Trace samples | 928 total, 916 enabled |
| Largest measured joint excursion from prepared pose | 0.00541449 rad (0.310°), middle S3 |
| Mean middle S3 shift during +0.05 A hold | +0.00160573 rad |
| Mean middle S3 shift during −0.05 A hold | −0.00515467 rad |
| Largest reported joint speed | 0.215967 rad/s, below 0.6 rad/s guard |
| Observed command frequency | 891.97 Hz; requested 1000 Hz |
| Largest command interval | 11.230 ms, below 50 ms guard |
| Disabled/low-speed feedback after disarm | 27.16 ms in this trial |
| Arm TCP before and after | [0.25310, −0.00134, 0.31412] m, controller state 4 |
| Native faults / warnings | None |

The SDK's reported motor current is the result of feedback plus feed-forward;
it need not equal the requested bias. For example, mean reported middle S3 current
was +0.07321 A during the +0.05 A hold and −0.01790 A during the −0.05 A hold.
These values are not fingertip-force measurements. Small motion and signed bias
delivery were measured; no subjective visual quality rating was collected.

At completion, hardware parameter readbacks were Kp 8, Kd approximately 0.15
(float32 representation), cap 1.5 A on all twenty joints. All twenty states were
Ready/disabled and commanded effort was zero. The retained controller fault
`operator has not armed motion` is the expected disarm cause. The frontend and
private hand owner exited normally. The latency above is an observation, not a
worst-case guarantee; disarm was tested after removal of the bias, not during a
nonzero-current ramp.

## Evidence and reuse

Evidence directory:
`/home/jerry/rax-runtime/sessions/20261028T135200Z_wuji_effort_qualification_0634eb/`

- `qualification-plan.json`, `qualification-report.json`, `hand-trace.jsonl`
- `mcp.jsonl`, `frontend.log`, `wuji-controller.log`
- Seven `motion-archive/*/` entries with wrist and side recordings

Reusable harness: [`scripts/qualify_wuji_effort.py`](../scripts/qualify_wuji_effort.py).
Run without `--execute` to preview; use `--source-session "$CUBE_SESSION"`.
Hardware-free validation: 82 effort/SDK/backend tests plus four harness tests
(success/restore, MCP failure cleanup, unstopped-arm refusal, excursion abort)
passed before this live trial.

After the test, the stopped cube session's tool contract and all provided skills
were refreshed with `scripts/refresh_wuji_tools.py`; prior versions are backed up
under its `operator/tool-refresh-backup-ln7i66dy/`. Its conversation and task memory
were preserved. The operator starts/resumes the agent using the official launcher.
