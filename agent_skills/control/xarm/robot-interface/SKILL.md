---
name: robot-interface
description: The real UFactory xArm 6 motion and gripper tools, frames, limits, arming and failure handling. Read before composing any robot action or writing robot code.
---

# UFactory xArm 6 robot interface

One 6-DoF UFactory xArm 6 arm with an xArm gripper and three Intel RealSense D435i
cameras. A person stands at the emergency stop. Everything you command happens
physically.

## Frames and units

- Base frame: x forward from the robot base, y to the robot's left, z up. Metres,
  radians, quaternions `[x, y, z, w]`.
- The TCP is the tool pose reported by the xArm controller. Every pose tool uses this
  TCP, not a raw flange pose.
- Gripper pointing straight down: keep the current `tcp_quaternion` from
  `{{ robot.status }}` unless you intentionally need a new wrist orientation.
- Objects' metric positions come from [object-localization](../object-localization/SKILL.md).

## Tools

- `{{ robot.status }}`: pose, joints, gripper, controller state/error codes, whether
  motion is armed, workspace box.
- `{{ robot.observe }}`: camera images now (`left`, `right`, `wrist`).
- `xarm_move_pose {position, quaternion?, relative?, tolerance_mm?, timeout_s?}`:
  Cartesian TCP move through the xArm controller and wait until settled. At most
  0.35 m per call, default cap 50 mm/s. `relative: true` adds `position` to the current
  TCP position. Requires operator arming.
- `xarm_move_joints {positions | preset: "home"}`: joint-space move with six joint
  angles in radians, or the backend home preset. The path is sampled with controller FK
  and checked against the workspace box before moving. Requires operator arming.
- `{{ robot.gripper }} {action: open | move | grasp, width?, force?, speed?}`: position
  control for the xArm gripper. `force` only applies when the operator configured a G2
  gripper; for the standard gripper, verify object retention from images.
- `xarm_hold`: stop/hold the current motion. Always allowed.

## Limits and arming

- Motion and gripper tools work only while the operator has armed motion. If `status`
  says motion is not armed, stop and report; never try to work around it.
- The operator may disarm at any time. A running move then sends a hold command and the
  tool returns the measured state.
- Targets outside the workspace box, too far per call, outside joint limits, or refused
  by the xArm controller are not safe to push through. Choose a shorter move, a higher
  clearance, a different orientation, or go `home` first.
- Treat the arm and gripper as stiff position-control hardware. Approach from above,
  descend in short steps near contact, and check the measured pose plus fresh images
  after each contact-sensitive step.

## Grasping pattern

1. Observe, localize the object centre/top from a fresh frame.
2. Open the gripper; move above the object at a safe height.
3. Yaw so the fingers close across the object's narrow dimension.
4. Descend to grasp height in short steps, close/grasp, lift vertically, then inspect
   the wrist and side cameras to confirm the object moved with the gripper.

## In your own code

Your Python reaches the same tools through `api.runtime.Context` (see
`MCP_SESSION.md`): `ctx.value('xarm_move_pose', position=[...])`. Robot calls are
serialized with your direct calls; keep scripts short, check every reply, and stop on
the first unexpected result.
