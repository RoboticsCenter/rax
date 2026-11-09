---
name: robot-interface
description: Operate the G1 EDU locomotion controller and two Wuji hands through bounded MCP tools.
---

# G1 EDU and Wuji hands

Use `humanoid_status` first. Check transport, capabilities, body freshness, faults,
arming and each hand's readiness. `dry-run` is a protocol exerciser, not a physics
simulation. It has no hands or camera unless an observation bridge is configured.

`humanoid_move_base(vx, vy, yaw_rate, duration_s)` commands body-frame velocity in
m/s and rad/s for at most two seconds: x forward, y left, positive yaw left.
It has no navigation, localization or obstacle avoidance. A completed command
window is not proof of distance travelled or standstill. Observe after movement.
The backend retains Unitree's stock balance controller. Cartesian arm motion,
whole-body reference sequences and autonomous controller-mode changes are unavailable.

`wuji_status(hand="left"|"right")` reports per-hand measured and commanded state.
`wuji_move_joints` uses explicit `hand` and twenty absolute radians in thumb,
index, middle, ring, pinky order, S1..S4 each. Preserve disabled entries from
`target_positions`; partial-hand completion applies to active joints only.
`wuji_pose(hand, name)` selects that hand's configured posture. Never infer a
left-hand posture by mirroring the right. Position convergence and contact-mode
shortfall do not establish successful grasping. Read the tool descriptions for
bounded trajectories and measured completion semantics.

`humanoid_stop` requests zero locomotion and a hold for both hands, preserving
G1 balance control. `wuji_hold(hand)` addresses one hand. MCP dispatch is serial;
these tools cannot preempt a blocking call. Operator disarm is monitored outside
MCP. Stop ACKs are not measured standstill or guaranteed grip retention.

`humanoid_observe` captures configured RGB snapshots and saves frame evidence.
Receipt times are not calibrated exposure times; RGB-D/extrinsics are unavailable.
`humanoid_stream(seconds_ago)` reviews the last 32 explicit observations, not a
continuous video. State returned with historical imagery is current, not synchronized.
Use `request_operator` for missing preparation, faults or physical scene resets.
