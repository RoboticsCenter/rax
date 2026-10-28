---
name: robot-interface
description: Control the single right AgileX Piper and its BrainCo Revo 2 hand through RAX, respecting arming, joint, TCP, hand and workspace limits.
---

# Single Piper / Revo 2 interface

Only the **right pair** belongs to this session. Never access the other arm, the
CAN, USB, either vendor SDK, or the camera HTTP endpoint directly.

- `piper_status {}`: six measured joints in radians, flange pose and estimated TCP,
  arming and commissioning, direct arm driver state and six hand motor readings.
- `piper_observe {}`: fresh overview RGB, robot state and saved frame metadata.
- `piper_move_joints {positions: [j1,j2,j3,j4,j5,j6]}`: absolute radians, small steps;
  speed/acceleration capped, path checked by FK, arm/hand envelope checked at runtime.
- `piper_move_pose {position: [x,y,z], quaternion: [x,y,z,w]}`: absolute base-frame
  metres and unit quaternion, only after the operator calibrates the hand TCP.
  The move follows a joint-space path, **not** a straight Cartesian line.
- `revo_move {positions: [thumb_flex, thumb_opposition, index, middle, ring, little]}`:
  six integer normalized values (0–1000 scale), not radians or a gripper width.
  Conservative motor bounds and per-call step limits apply. There is no generic
  open/grasp action, current-control tool, stored action, or automatic hand homing.
- `piper_hold {}`: best-effort stop/position hold; never disable motors, open a
  loaded hand, or assume this substitutes for the physical emergency stop.
- `piper_stream {seconds_ago: [0, 2, 10]}`: recent RGB evidence and robot records.

`motion_commissioned` is false until the operator qualifies this port. Arming is a
separate, expiring operator decision. RAX does not enable or recover a disabled
arm. A readiness failure requires operator help, not a retry around the guard.
No home preset exists. Ask the operator for a safe neutral pose/reset.

Joint limits (degrees for reference only): ±150, [0,179.91], [-170,0], ±99.98,
±69.9, ±120. Tool arguments use **radians**. State uses a Piper modified-DH offset
model; until `tcp_calibrated` is true, TCP equals the configured unverified tool
transform, not a calibrated grasp center. Hand extent/table checks are conservative
spherical bounds, not collision detection for people or arbitrary obstacles.

An obstructed finger or reached target does **not** prove a grasp. `grasp_verified`
is always false: inspect RGB for secure contact and slip before any lift. Never
interpret motor current or a stall as calibrated force/tactile feedback.
