# Piper / Revo 2 real-hardware MCP tests — 2026-10-28

## Scope and outcome

Two real server sessions exercised all **nine exposed MCP tools** over the private
Unix socket, including JSON-RPC initialization, initialized notification, ping,
tool discovery, tool calls, audit logging and clean server shutdown. No backend
or SDK mocks were used. Across both sessions, **32 tool calls** matched their
expected success or refusal outcomes; seven returned RGB images decoded correctly.

This is **not a completed physical-motion qualification**. Joint, Cartesian and
finger trajectory calls were tested for safe refusal only. The stationary
assembly hold was exercised on hardware, but stopping a moving assembly and
physical emergency-stop behavior remain untested.

## Selected hardware

- Right Piper USB serial `002C001B4759530920353131`, resolved as `can1`.
- Piper firmware `H-V1.2-1`, `S-V1.8-2`; verified `default` SDK profile.
- Right Revo 2 serial `BCXTR2362J2500015`, firmware `1.0.22.U`, channel 1, ID 127.
- Overview D435 serial `263322070113`, RGB only.

The selected Piper SDK feedback agreed with independent passive CAN feedback.
No left-pair motion commands were issued.

## Tool matrix

| Tool | Real-server test | Result |
| --- | --- | --- |
| `status` | Session budgets, live robot summary, final state, unexpected arguments | Passed |
| `piper_status` | Arm/hand identity, firmware, joints, readiness and independent feedback; unexpected arguments | Passed |
| `piper_observe` | Fresh RGB evidence, image payload and saved metadata; invalid camera argument | Passed |
| `piper_stream` | Default selection, ages 0/1/2 seconds, unavailable old history, negative/out-of-retention ages and empty selection | Passed |
| `piper_move_joints` | Current-joint target refused before dispatch; malformed five-joint target | Passed refusal tests; trajectory untested |
| `piper_move_pose` | Current model-pose target refused before dispatch; missing quaternion | Passed refusal tests; trajectory untested |
| `revo_move` | Current finger target refused before dispatch; noninteger target | Passed refusal tests; trajectory untested |
| `piper_hold` | Invalid arguments refused; valid stationary assembly hold acknowledged | Passed stationary test only |
| `request_operator` | Help/reset file round trips, invalid kind/timeout, actual 30-second timeout | Passed protocol tests |

An unknown tool was also rejected. The three valid-shaped trajectory requests
reported validation-stage errors, `no_action_executed: true` and
`control_uncertain: false`, because motion is not commissioned.

Operator round trips used explicitly labeled **synthetic test acknowledgments**
with `done: false`; no human approval, scene reset or safety qualification was
claimed. The timeout case waited without a synthetic reply.

`gemini_generate` was **not exposed or tested**: no operator-selected OpenRouter
credential was supplied. No paid inference request was made.

## Stationary hold

Before dispatch, fresh overview RGB showed an apparently empty right hand and
feedback reported all six finger speeds zero. The arm was disabled and not
streaming. `piper_hold {}` returned `ok: true`, `stop_requested: true` and
`physical_stop_verified: false`. Before/after joint and finger-position feedback
was identical. The arm remained disabled and the session remained unarmed.

This verifies the stationary stop-request path, not stopping distance, stop latency
under motion, grasp retention, or an independent physical emergency stop.

## Remaining physical-test blockers

- All six arm motors are disabled; control mode is standby. No enable attempted.
- J2 is −2.425 degrees and J3 is +1.129 degrees, outside configured command limits.
- The model TCP x coordinate is about 0.0501 m, below the configured 0.10 m
  workspace minimum. The tool/hand geometry is uncalibrated.
- `motion_commissioned` remains false; the sessions were never armed.
- `tool_calibrated` remains false, independently blocking Cartesian motion.

Do not bypass these guards to make a test pass. A supervised operator must first
establish a supported, legal starting configuration, verify kinematics and the
physical workspace/hand envelope, qualify stop behavior, and commission the
setup. Cartesian testing additionally requires a measured hand TCP. Then use a
new, separately armed session for small joint/finger excursions, return checks,
Cartesian tests and supervised interruption/failure tests.

## Machine-local evidence

- Preflight: `/tmp/rax-piper-tool-preflight-20261028.json`.
- Main run: `/tmp/rax-piper-mcp-hardware/20261028T140811Z/report.json`.
- Stationary hold: `/tmp/rax-piper-mcp-hardware/20261028T140901Z/report.json`.
- Cross-run semantic checks:
  `/tmp/rax-piper-mcp-hardware/verification-20261028.json`.
- Each run directory contains full `wire-*.json` requests/responses, image payloads,
  `mcp.jsonl`, recordings, operator request files and `server.log`.
- Temporary driver: `/tmp/rax_piper_mcp_hardware_test.py`.

These `/tmp` artifacts are machine-local and may be removed by system cleanup.

## Later supervised control-mode handoff attempt

At 2026-10-28 14:16 UTC, after the operator confirmed presence at the emergency
stop with the workspace clear, the selected right arm reported all motors enabled,
no reported faults, teaching control mode 2, and all joints within configured
limits. Half a second of sampled joint feedback confirmed a stationary start.

The direct SDK received one `set_speed_percent(5)` and one `set_motion_mode('j')`
request. No joint target, enable, disable, reset, role-change or firmware command
was sent. During the subsequent two-second observation, the measured maximum
joint change was zero, but feedback remained in mode 2. **The attempted handoff
failed; CAN control was not established.** Evidence:
`/tmp/rax-piper-can-handoff-20261028.json`.

Further automatic role changes or motor cycling were not attempted. AgileX
pyAgxArm discussion #53 (maintainer responses June 5 and September 16, 2026)
states that seamless leader/follower switching was introduced in firmware 1.8-5,
with a leader-to-follower bug in 1.8-5 through 1.8-8 fixed in 1.8-9. This rig is on
1.8-2, so a safe seamless transition must not be assumed. This does not establish
the firmware as the sole cause of the rejected mode request. Mechanically
supported, manufacturer-appropriate recovery must be established before any
motor-cycle procedure; no firmware update was attempted.

The legacy right-arm server was also inspected read-only: its status reported
`server_state: disabled`, `streaming: false`, `busy: false` while hardware feedback
reported enabled motors and mode 2. It was not commanding a hold according to
that status. Neither legacy server was stopped or restarted.
