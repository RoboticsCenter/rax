# Left Hand 2: physical partial-hand movement

Follow-up: the [original-controls home test](wuji-left-original-controls-home-2026-11-02.md)
completed all three bounded arbitrary-pose/home cycles at Kp 8 / Kd 0.15 / 1.5 A,
with no new native faults. The lower-current trials below remain historical evidence.

Tested on 2026-11-02 (America/Los_Angeles), through the normal MCP `Frontend`
and Wuji controller, in a separate left-hand-only session. No simulated device
was used for these movement results.

- Device: `WH2JA01260811007`, firmware `2.13.1`, `192.168.1.110:50001`.
- Excluded: `thumb_S3` (`0x2108`) and `middle_S1` (`0x3208`). Their native faults
  remained visible. Their gains and current limits were written and read back as
  zero, and selective enable addressed only the other eighteen joints.
- Active limits: 0.3 A, kp 8, kd 0.15, velocity 0.15 rad/s, acceleration
  0.5 rad/s², jerk 0.5 rad/s³, maximum target step 0.05 rad.
- The right hand retained its original owner process and unchanged SDK command
  log throughout. Both arm joint readbacks were unchanged.

## Individual movements

Every active joint physically moved in response to a 0.04-rad MCP command.
Seventeen also passed the return movement and 0.015-rad position tolerance. Ring
S1 moved but missed that position tolerance; its return was not attempted in this
sweep. These are unloaded small movements, not grasping or bimanual qualification.

| Joint | Measured outward movement (rad) | Return test |
| --- | ---: | --- |
| thumb_S1 | 0.037206 | Passed |
| thumb_S2 | 0.031636 | Passed |
| thumb_S4 | 0.032259 | Passed |
| index_S1 | 0.031157 | Passed |
| index_S2 | 0.034918 | Passed |
| index_S3 | 0.036969 | Passed |
| index_S4 | 0.032028 | Passed |
| middle_S2 | 0.035673 | Passed |
| middle_S3 | 0.031883 | Passed |
| middle_S4 | 0.037815 | Passed |
| ring_S1 | 0.022383 | Not attempted after position shortfall |
| ring_S2 | 0.026099 | Passed |
| ring_S3 | 0.031061 | Passed |
| ring_S4 | 0.034803 | Passed |
| pinky_S1 | 0.034891 | Passed |
| pinky_S2 | 0.029048 | Passed |
| pinky_S3 | 0.033708 | Passed |
| pinky_S4 | 0.035086 | Passed |

Across 3,889 telemetry samples from these sweeps, both excluded joints remained
Stopped, their configured gains/current limits stayed zero, and no new native
fault appeared. Maximum recorded MCU temperature was 59.33 °C. Earlier 0.02-rad
probes at lower gains or a tighter 0.006-rad tolerance are retained in the evidence;
they demonstrated movement but did not all meet the precision criterion.

## Continued finger movement

At the operator's request, partial-hand precision shortfalls were changed to
informational results: a completed bounded reference may return
`target_not_reached`, `ok: true`, `reached: false`, `holding: true`. It no longer
raises a backend exception that stops the assembly. Native faults, disabled-joint
isolation, feedback freshness, current/joint bounds and emergency stop remain
enforced. Full-hand completion behavior is unchanged.

The 1,000-Hz profile delivered about 340 commands/s on this laptop. Reducing the
settling wait exposed a reference deadline failure, so the profile was corrected
to 200 Hz. The final physical run measured 184.3 commands/s with a maximum command
interval of 0.0411 s, below the existing 0.05-s limit.

The final sequence completed seven MCP movements while remaining enabled and
holding between commands: thumb, index and middle bend/return, then ring bend.
During ring return, the device reported `ring_S1: Stall (0x0007)`, a Warning with
AutoClear policy. The backend requested emergency stop and disable. The native
warning cleared after unloading; the host retains its record. This was a new
native stall warning, not a failed precision test, and the firmware did not itself
disable the hand. Pinky movement in this final sequence was not attempted.

The final sequence's seven completed commands reached their tolerance; it did not
exercise the newly nonfatal precision-shortfall return path. The actual hardware
run therefore confirms continued normal movement, not physical coverage of every
new result branch. No mock test is counted as physical validation.

Final readback confirmed all twenty joints disabled/Ready or Stopped, fresh
feedback, no controller ownership, and the session disarmed. The two original
native fault codes remain. The ring stall remains visible as a retained fault.

## Evidence and session state

Artifacts are under:
`/home/jerry/rax-runtime/sessions/20261103T034138Z_left_partial_real_probe/`

- `physical-test-summary.json`: measured movements, isolation samples and peer checks.
- `motion-sweep-result.json`, `remaining-ring-pinky-result.json`: full MCP results.
- Corresponding `*-telemetry.jsonl`: real feedback samples.
- `finger-sequence-result.json`: final grouped finger sequence and stall stop.
- `finger-sequence-1000hz-result.json`: retained timing-failure evidence.
- `final-after-finger-sequence.json`: final states and unchanged right/arm checks.
- `wuji-left-controller.log`: native parameter readback, selective enable and stop logs.
- `mcp.jsonl`: actual MCP request/result audit.

The left-only MCP socket is `/tmp/rax-left-partial-real-mcp.sock`. The original
bimanual frontend remains disarmed; its former left child was stopped to release
exclusive ownership without restarting or commanding the right hand. The updated
`operator-input.partial-hand-draft.json` in the original session is validated but
not loaded. A coordinated session reload is needed before normal bimanual use.
