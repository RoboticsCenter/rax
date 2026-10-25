# Wuji Hand 2 live MCP test — 2026-10-25

The production MCP frontend and Wuji controller were exercised on the real right
hand. These are initial joint-space tool checks, not full manipulation or fault
qualification. No arm trajectories were commanded.

## Hardware and connection

- Hand: `WH2KA01260823017`, right, firmware `2.6.0`, SDK `2026.10.12`.
- Hand endpoint: `192.168.1.111:7447`, reached through `enx6c1ff7618956`.
- Host hand address: temporary `192.168.1.10/24`, added to that adapter with
  `nmcli device modify`. Existing xArm routes and the Wi-Fi default route were retained.
- The current xArm endpoint is `192.168.100.16` on `enx6c1ff772bfe8`; the test
  session overrides the older address in the repository example.
- The operator confirmed an empty hand, finger clearance and access to the hardware stop.

Initial SDK discovery found nothing because the hand subnet was absent and the
hand-side adapter had no carrier. After connection and address configuration,
MCP feedback verified all twenty joints Ready/disabled, zero active error codes,
and fresh state and diagnostics. Initial reported joint temperatures were about
32–38 °C and bus voltages about 12.3–12.5 V.

## Tool results

The control profile used a 0.5 A limit, kp 1.5 and kd 0.1. All movements used
bounded interpolation; only index S3 received a substantial changed target.

| Tool/check | Observed result |
| --- | --- |
| `wuji_status` | Verified identity, twenty measured joint positions, independent fresh streams, and fault-free diagnostics. Disconnected status also correctly reported timeout and absent feedback before the network was restored. |
| Preparation | Applied/read back the profile, seeded measured positions, and confirmed all joints Enabled without a deliberate trajectory. |
| `wuji_move_joints`, strict probe | Requested index S3 +0.060 rad over 2 s with a test-only 0.010 rad tolerance. Reported `target_not_reached`, with 0.0272 rad residual error. This stricter precision check **failed**. |
| `wuji_move_joints`, original tolerance | With the backend's original 0.040 rad tolerance, requested another +0.060 rad over 2 s. Measured change 0.0252 rad; maximum error 0.0349 rad. Returned `reached` within that tolerance. Other measured joints changed by at most about 0.000014 rad. |
| `wuji_hold` | Returned `held`; commanded targets stayed identical and measured joints remained stationary. `confirmed: false` correctly distinguished holding a target from confirmed motor disable. |
| `wuji_pose("test_start")` | Used the twenty measured starting angles recorded earlier. Returned `reached` after a 2 s move; maximum error 0.0129 rad. This is a test posture, not a general open/grasp posture. |
| `xarm_hold` after the stop fix | Returned a feedback-confirmed hand stop; all twenty joints disabled. Arm hold was acknowledged, without a claim of independently verified physical stopping. |

The operator could not see the initial tiny movements. A subsequent visibility
test commanded index S3 +0.250 rad (14.3°) over 3 s, paused for 3 s, and returned
over 3 s. Feedback measured **12.12°** of outward movement, then a final angle
**1.00°** from the starting angle. Both calls met the original 0.040 rad tolerance;
their maximum errors were 0.0386 and 0.0176 rad. Visual confirmation from the
operator is separate from these encoder measurements.

At the operator's request, the same 0.250 rad movement was repeated with a
ten-second countdown. This time the bend measured **11.86°**, but residual error
was **0.04296 rad**, exceeding the unchanged 0.040 rad tolerance. The tool returned
`target_not_reached`; its failure path confirmed hand disable. The planned pause
and return were therefore not executed. Final feedback was fresh, all twenty
joints disabled, and hardware fault codes zero. This repeat demonstrates that
position settling is not yet consistently within even the original tolerance;
the earlier successful calls must not be read as full positioning qualification.

The operator then confirmed seeing the small movement and requested a larger,
quicker one. Index S3 was commanded **+0.340 rad (19.5°) over 1.5 s**, within the
existing step, velocity and acceleration bounds. Feedback measured **17.78°**
of movement, with maximum error **0.02975 rad**; the tool returned `reached`.
The hand was then confirmed disabled, with zero hardware faults, and the session
was disarmed. No gain, current or limit changes were made for this test.

The visibility test ended disarmed, with a feedback-confirmed disable. Disarming
before the explicit hold also latched the expected operator-disarm condition in
the controller; this is distinct from a hardware diagnostic fault.

## Expanded live MCP coverage

The follow-up suite exercised all four Wuji MCP tools with the same gains,
current cap, limits and 0.040 rad completion tolerance.

| Check | Result |
| --- | --- |
| Tool discovery and status | All four Wuji tools present, G2 absent, correct hand identity, twenty finite joint values, and advancing state/diagnostic sequences across ten samples. |
| Input and admission rejection | **22 cases passed**, all rejected at validation with `no_action_executed: true`: wrong/missing targets, boolean/string values, out-of-range targets, unexpected fields, invalid durations, unknown/missing postures, disarmed motion, armed-but-unprepared motion, excessive step and too-short duration. |
| Healthy `wuji_hold` | Preserved all twenty commanded targets across twelve samples over three idle seconds; hand stayed enabled with fresh feedback. |
| `wuji_move_joints`, automatic duration | Requested index S3 −0.200 rad. Measured −9.90°; returned `reached` after 1.279 s, with 0.0274 rad maximum error. |
| `wuji_pose`, explicit duration | Called `test_start` over 1.5 s. Returned `reached` after 1.710 s, with 0.0297 rad maximum error; index S3 changed +2.07°. |
| Disarmed hold | `wuji_hold` remained available and returned a confirmed emergency-disable result. |
| Independent idle disarm | With the hand holding and no motion tool running, operator disarm produced fresh Ready/disabled feedback for all twenty joints in **42 ms** in this test. Control ownership cleared. |
| Final stop/status | `wuji_status`, `wuji_hold` and `xarm_hold` remained available after disarm; final hand disable was confirmed. |

**Concurrency limitation:** a second MCP client issued `wuji_hold` during a
two-second command that retained the existing posture. The hold waited **2.276 s**
and ran after the first command completed. The MCP frontend serializes requests;
MCP hold/status calls cannot preempt a running tool. Operator stop/disarm uses the
separate private interface. Tool descriptions, hand-control instructions and the
operator guide now state this distinction explicitly. No dispatcher concurrency
behavior was changed.

The motion trace contains 341 samples, including 97 during execution, with no
observer errors, stale samples or hardware diagnostic faults. Maximum observed
tracking error was 0.0411 rad during motion. Peak reported joint velocity was
**0.529 rad/s**, slightly above the 0.5 rad/s *commanded-trajectory* limit. That
limit bounds generated targets, not measured motor speed; measured response and
position accuracy still require tuning and qualification. The sampled trace is
not a bound on unobserved peaks or worst-case stop latency.

Additional evidence:

```text
1792932336323984417-mcp-contract-disarmed.json
1792932392176841086-mcp-armed-admission.json
1792932438473722605-mcp-motion-cases.json
1792932438473722605-mcp-motion-feedback.jsonl
1792932522910686083-mcp-stop-watchdog.json
```

Both modified skills validated, and all four targeted workspace/tool-selection
regressions passed after the documentation corrections. This suite covers the
MCP interface; it does not qualify movement of every joint or finger.

## Fixes found through real testing

1. **Result polling:** reconnect completed on hardware while the operator client
   timed out reading its result. The client now tolerates individual result-read
   timeouts up to the operation deadline, polling the same command ID without
   resending the hardware operation.
2. **Emergency disable:** on this device, the native `emergency_stop()` call
   returned while all twenty diagnostic states remained Enabled. RAX correctly
   refused to claim a confirmed disable. Its emergency path now requests native
   emergency stop followed by explicit `disable()`, including a disable attempt
   if the first RPC raises. This corrected path was verified on the real hand.
3. **Failure reporting:** hand execution errors now retain the current hand
   snapshot and both assembly-stop outcomes instead of allowing a null hand
   value in the transport error to replace the measured state.

The hardware-free regression suite after these fixes passed **233 tests**, with
**5 skipped**. Fault-injection unit tests cover the new timeout and disable behavior.

## Evidence and remaining scope

Raw MCP audit, configurations, before/after feedback, and per-test reports are in:

```text
.local-runtime/sessions/20261025T112620Z_wuji_live_mcp/
```

Useful files include `01-wuji-status-reconnected.json`, `02-prepare.json`,
`03-wuji-move-joints.json` (the failed strict probe),
`03-after-failed-motion.json`, the timestamped `*-move.json`, `*-hold.json`,
`*-pose.json`, `*-stop.json`, and `04-visible-index-movement.json`.
The countdown repeat is `1792928322920936873-visible-repeat.json`.
The larger, quicker move is `1792928442684894277-quicker-larger-move.json`.

The measured residual errors show that the current conservative profile has not
demonstrated 0.010 rad positioning accuracy. Gains and current were not increased
during these tests. Full-hand posture clearance, grasping, carrying, tactile use,
and physical communication/power-loss behavior remain untested by this session.

The hand subnet change is temporary and can disappear on adapter reconnection.
The original device/profile examples remain commissioning inputs; use the
recorded test session's actual addresses when reproducing this host setup.
