# Read-only Piper/Revo bring-up — 2026-10-28

Reference: freshly fetched `origin/real/xarmwuji`, commit
`9e34222cec179a3d778bf375c9db8aea888abf49`.

Ran `scripts/piper.py check` through the new direct SDK adapters; JSON report:
`/tmp/rax-piper-direct-check.json` (machine-local, not a permanent repository artifact).
The check passed communication only. **No enable, trajectory, hand position,
current command, session arming or left-arm motion was sent.** Hand adapter
discovery can issue identity reads on the other USB adapter before finding the
configured right-hand serial.

## Results

- Right Piper USB serial: `002C001B4759530920353131`, currently `can1`, 1 Mbit/s.
- Direct `pyAgxArm` connection: disabled, all six motors disabled, firmware error
  0. SDK query verified `H-V1.2-1`, `S-V1.8-2`, firmware profile `default`.
  RAX did not change motor state.
- Independent passive CAN feedback: current, all motor fault flags 0.
- Raw joints (degrees): `[6.440, -2.425, 1.129, -4.352, 23.167, 82.249]`.
- Direct SDK joints agree with raw CAN, including out-of-range J2/J3. The old
  evaluation owner had reported those two joints as zero; it is no longer used.
  **No automatic clamp, recovery, limit relaxation or enable is implemented by
  RAX.** Unsafe starting positions continue to block motion.
- Selected Revo serial: `BCXTR2362J2500015`; verified installed right profile,
  DaMiao channel 1, ID 127. Six idle motor states, normalized positions
  `[200, 300, 0, 0, 0, 0]`. Direct `bc-stark-sdk 2.0.7` via the repo-owned
  DaMiao adapter verified protected currents 500 mA, normalized units, turbo off.
  No physical grasp/force success inferred. The native hand SDK runs in an
  isolated worker; no evaluation Python modules are imported.
- Overview D435 `263322070113`: received and decoded a fresh RGB JPEG from
  Spyglass (424×240 stream catalog). No camera SDK was opened by RAX.

A read-only full-stack smoke test also built the Piper workspace, constructed
the real `PiperBackend`, and called `piper_status` and `piper_observe` through
the actual RAX frontend/audit path. It reported `pyagxarm` and `bc-stark-sdk`,
saved the overview image/metadata, and correctly reported `motion_ready: false`.
Machine-local evidence: `/tmp/rax-piper-direct-smoke/session/readonly-smoke.json`.
No paid agent session was started. A separate scientific Python environment
was provisioned at `~/rax-agent-env`, without robot SDKs, for agent-written code.

## Still required before motion

The operator must safely recover/support and enable the arm, independently verify
the DH variant, measure the hand tool
transform and table/workspace/envelope, and qualify both arm/finger stops and
failure cases under supervision. Only then enable `motion_commissioned` in a
new operator configuration and separately arm that session. Cartesian commands
also require `tool_calibrated`. Neither flag was enabled during this bring-up.
