---
name: robot-interface
description: The real UFactory xArm 6 motion tools, configured end effector, frames, limits, arming and failure handling. Read before composing any robot action or writing robot code.
---

# UFactory xArm 6 robot interface

One 6-DoF UFactory xArm 6 arm with a {{ robot.effector_name }} and three Intel RealSense D435i
cameras. A person stands at the emergency stop. Everything you command happens
physically.

## Frames and units

- Base frame: x forward from the robot base, y to the robot's left, z up. Metres,
  radians, quaternions `[x, y, z, w]`.
- The TCP is the tool pose reported by the xArm controller. Every pose tool uses this
  TCP, not a raw flange pose.
- Keep the current `tcp_quaternion` from
  `{{ robot.status }}` unless you intentionally need a new wrist orientation.
- Objects' metric positions come from [object-localization](../object-localization/SKILL.md).

## Tools

- `{{ robot.status }}`: pose, joints, end-effector feedback, controller state/error codes, whether
  motion is armed, workspace box.
- `{{ robot.observe }}`: camera images now (`left`, `right`, `wrist`).
- `xarm_move_pose {position, quaternion?, relative?, tolerance_mm?, timeout_s?}`:
  Cartesian TCP move through the xArm controller and wait until settled. At most
  0.35 m per call, default cap 50 mm/s. `relative: true` adds `position` to the current
  TCP position. Requires operator arming.
- `xarm_move_joints {positions | preset: "home"}`: joint-space move with six joint
  angles in radians, or the backend home preset. The path is sampled with controller FK
  and checked against the workspace box before moving. Requires operator arming.
{% if robot.has_hand %}
- `wuji_status`, `wuji_move_joints`, `wuji_pose`, `wuji_hold`: see
  [hand-control](../hand-control/SKILL.md) before moving fingers or grasping.
- `xarm_hold`: stop the arm and request hand emergency stop. Inspect each reported
  result; delivery and physical stopping are distinct. Always allowed, but queued
  behind an active MCP call. The operator's direct stop/disarm path bypasses MCP.
{% else %}
- `{{ robot.gripper }} {action: open | move | grasp, width?, force?, speed?}`: position
  control for the xArm gripper. `speed` is an integer in the xArm SDK units. `force`
  only applies when the operator configured a G2 gripper; for the standard gripper,
  verify object retention from images.
- `xarm_hold`: stop/hold the current motion. Always allowed.
{% endif %}

## Limits and arming

- Motion tools work only while the operator has armed motion. If `status`
  says motion is not armed, stop and report; never try to work around it.
- The operator may disarm at any time. A running move then sends a hold command and the
  tool returns the measured state.
- Targets outside the workspace box, too far per call, outside joint limits, or refused
  by the xArm controller are not safe to push through. Choose a shorter move, a higher
  clearance, a different orientation, or go `home` first.
- Treat the arm as stiff position-control hardware. Approach from above,
  descend in short steps near contact, and check the measured pose plus fresh images
  after each contact-sensitive step.

{% if robot.has_hand %}
## Hand geometry and carrying

The configured TCP is a fixed tool frame, not the moving fingertips. Finger motion
changes the hand's reach and clearance. TCP workspace checks do not check every
finger, link, object or cable. Use the operator-verified mount/TCP and images.
The wrist-camera calibration must match the configured TCP.

The hand must be prepared and holding with fresh feedback before arm motion. Keep
its accepted target while carrying. Arm and hand motions are sequential; these
tools do not provide a synchronized 26-joint trajectory. Hand faults or disarming
request stops on both devices. Supported release and recovery are described in
[hand-control](../hand-control/SKILL.md).
{% else %}
## Grasping pattern

1. Observe, localize the object centre/top from a fresh frame.
2. Open the gripper; move above the object at a safe height.
3. Yaw so the fingers close across the object's narrow dimension.
4. Descend to grasp height in short steps, close/grasp, lift vertically, then inspect
   the wrist and side cameras to confirm the object moved with the gripper.
{% endif %}

## In your own code

Your Python reaches the same tools through `api.runtime.Context` (see
`MCP_SESSION.md`): `ctx.value('xarm_move_pose', position=[...])`. Robot calls are
serialized with your direct calls; keep scripts short, check every reply, and stop on
the first unexpected result.
