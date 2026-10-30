---
name: robot-interface
description: The real UFactory xArm 6 motion tools, configured end effector, frames, limits, arming and failure handling. Read before composing any robot action or writing robot code.
---

# UFactory xArm 6 robot interface

One 6-DoF UFactory xArm 6 arm with a {{ robot.effector_name }} and the configured Intel RealSense
cameras: `{{ robot.cameras | join('`, `') }}`. A person stands at the emergency stop. Everything you command happens
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
- `{{ robot.observe }}`: camera images now (`{{ robot.cameras | join('`, `') }}`).
- `xarm_move_pose {position, quaternion?, relative?, speed_mps?, tolerance_mm?, timeout_s?}`:
  Cartesian TCP move through the xArm controller and wait until settled. Read the
  live tool description for this session's translation and speed caps; do not assume
  repository defaults apply. `speed_mps` selects a speed from 0.001 m/s up to the
  configured ceiling for this call; omit it to use that ceiling. Select a lower speed
  near contact or when carrying requires it. `relative: true` adds a base-frame displacement to the
  current TCP position. It does not translate along rotated tool axes. Requires operator arming.
  `timeout_s` accepts 1–600 seconds; the repository default is 300 seconds, and the
  live tool description reports this session's configured default. The call returns
  as soon as the target settles; the timeout is only a maximum. For an unusually slow
  move, request `timeout_s: 600`. This changes the time allowance, not motion speed.
  A timed-out move stops both devices. When feedback confirms both stopped,
  `control_uncertain` is false; timeout alone does not latch uncertainty. If either
  stop cannot be confirmed, uncertainty remains true. Inspect the measured pose and
  fresh views, and restore arm/hand readiness through operator controls before
  another move; do not automatically replay the failed target.
- `xarm_move_joints {positions | preset: "home"}`: joint-space move with six joint
  angles in radians, or the backend home preset. The path is sampled with controller FK
  and checked against the workspace box before moving. The configured move timeout
  (default 300 seconds) covers the entire call, including all segments. Requires operator arming.
{% if robot.has_hand %}
- `wuji_status`, `wuji_move_joints`, `wuji_pose`, `wuji_home`, `wuji_hold`,
  `wuji_set_effort`, `wuji_set_impedance`: see
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
{% if robot.has_hand %}
  The hand also requests motor disable; a held object may be released.
{% endif %}
- Joint targets, including `xarm_move_joints(preset="home")`, need only one call.
  The host validates the complete path and handles longer moves in bounded SDK
  segments. Do not split a verified-clear joint path merely to satisfy the SDK
  segment limit. Arming and hand readiness are checked during execution; a stop
  or failure cancels the remaining segments.
- Targets outside the workspace box, Cartesian translations too far per call, outside joint limits, or refused
  by the xArm controller are not safe to push through. Choose a shorter move, a higher
  clearance or a different orientation within observed clearance. Use `home` only
  after establishing clearance for the whole assembly along that path.
- Treat the arm as stiff position-control hardware. Approach from above,
  descend in short steps near contact, and check the measured pose plus fresh images
  after each contact-sensitive step.

{% if robot.has_hand %}
## Hand geometry and carrying

The configured TCP is a fixed tool frame, not the moving fingertips. Finger motion
changes the hand's reach and clearance. TCP workspace checks do not check every
finger, link, object or cable. Use the available mount/TCP evidence and fresh images;
distinguish measured geometry from nominal models and experimental estimates.
The wrist-camera calibration must match the configured TCP.

The hand must be prepared and holding with fresh feedback before arm motion. Keep
its accepted target, current bias and impedance while carrying. Arm and hand motions are sequential; these
tools do not provide a synchronized 26-joint trajectory. Hand faults or disarming
request stops on both devices. Supported release and recovery are described in
[hand-control](../hand-control/SKILL.md).

An operator reply and arming are separate state changes. The dashboard's combined
**Ready & send reply** prepares/reuses a healthy hand hold and renews arming before
answering; **Send reply only** does not. In either case, check live status and fresh
images after the reply. Enabled holding preserves its target/bias/gains; preparation
from disabled seeds newly measured angles, restores the baseline gains/caps, and
zeros feed-forward effort. Do not replay a pre-disable finger target automatically.
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
