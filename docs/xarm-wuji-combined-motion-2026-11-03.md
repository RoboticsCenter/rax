# Combined arm and hand motion: real hardware validation

Validated `xarm_move_combined` on both xArm 6 controllers and both Wuji Hand 2
devices on November 3, 2026. Tests used the real MCP frontend and existing
commissioned profiles. No mock devices were used.

The tool accepts any nonempty subset of `left_arm`, `right_arm`, `left_hand` and
`right_hand`. Arms take one `pose` or `joints` action; hands take one `joints`,
`pose` or `home` action. Each action contains the ordinary single-device payload.
All targets are validated before dispatch. One worker per selected device uses
the existing controller; omitted devices retain their targets.

| Check | Observed result |
| --- | --- |
| Initial reset | Both arms and both hands returned home before implementation testing |
| Four-device movement | Both arms rose 10 mm while index/middle finger joints on both hands moved; all four returned successful measured completion |
| Concurrent execution | Host dispatch skew 2.26 ms; 17 telemetry samples across 1.983 s showed both arms moving while both hands executed |
| Measured finger overlap | During that overlap, index S1 changed by 0.0531 rad on the left and 0.0198 rad on the right |
| Preflight refusals | Empty request, multiple actions on one device, invalid peer-arm target and unknown peer-hand posture all rejected without movement or loss of existing holds |
| Subset commands | Left arm plus right hand, then right arm plus left hand; omitted targets remained unchanged |
| Interrupted execution | Deliberate operator disarm cancelled the combined action and confirmed all four stops; results identified each failed device |
| Final reset | One combined call returned all four devices home; both hands reported `reached`, with maximum errors 0.01036 rad left and 0.01807 rad right |
| Final state | All four stops confirmed, session disarmed, no pending operator requests |

The first interruption test exposed an intentional-stop race: a peer could
deliver the stop before another arm worker read telemetry, causing state 4 to be
misreported as a controller fault. The bimanual adapter now classifies state 4
after disarm as cancellation while checking the other controller-health fields.
The real retest produced no new arm/hand fault alerts. The test-generated alert
was resolved with an explicitly attributed validation note; its audit is retained.

Worker starts are concurrent, but controller clocks and arrival times are not
synchronized. The existing motion recorder produced one combined clip with all
four camera videos and robot telemetry. No gains, current caps or motion limits
were changed for these tests.

Raw requests, responses, telemetry overlap, final status and `validation-summary.json`
are under the session's `operator/combined-motion-20261103/` directory. The tested
session is `20261103T073444Z_unscrew_bottle_b1d812`. Its conversation, reference
video and prior recordings were preserved; the operator resumes the agent.
