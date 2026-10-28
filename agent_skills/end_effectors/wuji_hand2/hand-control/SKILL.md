---
name: hand-control
description: Shape, hold and release objects with the real twenty-joint Wuji Hand 2 through its MCP tools. Use before finger motion, grasping, carrying or release; covers joint order, target retention, feedback and recovery.
---

# Wuji Hand 2 control

Read `wuji_status` before moving. Confirm the configured side, `profile_revision`,
fresh state, no faults, and `mode: holding`. The operator prepares/enables the hand
and arms the session. Request help if it is unprepared; agent tools do not enable,
clear faults, change gains/current limits, or recalibrate joint origins.

## Commands and state

`wuji_move_joints` takes `positions`: exactly twenty **absolute radians**, ordered
thumb S1–S4, index S1–S4, middle S1–S4, ring S1–S4, pinky S1–S4. Each finger has
four entries. Read configured limits from status; do not reuse Allegro, left-hand,
or another firmware/origin profile's numbers. A zero vector is a motion target,
not a hold or a guaranteed open pose.

Start a small change from `target_positions` while checking measured `positions`.
Modify only the intended joints. The backend bounds the per-call change, velocity,
acceleration and tracking error. Optional `duration_s` must allow those bounds;
omitting it lets the backend choose. A rejection calls for revising the target,
not retrying the same command.

With `motion_mode: scurve`, the backend also limits reference jerk and sends
matching position/velocity commands. Movements start from the preceding commanded
state and finish at zero reference velocity/acceleration. Requested durations may
round up to a control tick; measured settling can take extra time. Status exposes
motion progress, reference derivatives and actual `command_timing`. These are
reference limits, not measured tracking guarantees or permission to change gains.

`wuji_pose(name=...)` uses an operator-commissioned posture listed in `postures`.
The list can be empty. Never invent names or assume an untested open/grasp pose.
Both motion tools wait for measured settling and keep the final target afterward.
They do not certify self-collision clearance or a successful grasp.

`effort_a`, motor current and current limits are **amperes**, not calibrated contact
force or joint torque. Optional fingertip-online flags report availability, not
contact. Use images and actual object motion to assess a grasp.

## Grasp, carry, release

1. Observe and localize the object. Choose a commissioned preshape and approach
   with clearance for every finger, the thumb and the palm. Arm tools move the
   configured TCP; it is not a moving fingertip frame.
2. Close gradually with bounded targets. Observe the hand and object after each
   meaningful change. A finger that stops short can indicate contact, a limit,
   collision or a fault; it does not establish object retention. Do not add an
   arbitrary squeeze offset or raise current to overcome an unexplained stop.
3. Preserve the accepted **commanded** hand target while carrying. Copying measured
   contact angles into a new target can remove holding effort. The service keeps
   publishing the target during arm moves and agent deliberation.
4. Lift a short distance with the arm, inspect wrist and side images, and verify
   that the object moved with the hand before continuing. Watch for slipping and
   rotation. Replan the posture if retention is poor.
5. Support the object before releasing with a verified posture; confirm separation
   in images, then retreat with clearance.

## Hold and failures

`wuji_hold` retains a stationary commanded posture when control is healthy.
If an S-curve is still moving when dispatched, it brakes smoothly to a nearby
bounded rest target and reports that target; physical settling is not confirmed.
It requests emergency stop if bounded braking is unavailable, the session is disarmed or state
is unreliable. `xarm_hold` requests stops on both the arm and hand. These calls
report request, SDK return and confirmation separately. Grip retention after an
emergency stop, disable, session end or lost connection is not guaranteed.

MCP calls are serialized. `wuji_hold` and `xarm_hold` from another MCP client wait
behind an active tool call; they cannot interrupt it immediately. The operator's
direct stop/disarm path bypasses that queue. Keep motion calls bounded and do not
rely on a second MCP client as an emergency interrupt.

A fault, stale feedback, unexpected enabled state or `control_uncertain` means stop
and request operator help. A timeout/target-not-reached is not grasp success. The
backend can stop both devices after a failed motion; check status again before
planning recovery. Reconnection never resumes an interrupted trajectory.

An operator may configure the specific auto-clearing encoder warning to be logged
under `warnings`/`warning_history`. This does not waive other faults or establish
encoder accuracy. Agent tools cannot change that policy or clear latched faults.

Record successful postures with the side, profile revision, serial, firmware,
mount/TCP and camera-calibration context, and decisive before/after frames in
memory. Reacquire object geometry on each attempt. Follow
[robot-interface](../robot-interface/SKILL.md) for arm frames and limits and
[object-localization](../object-localization/SKILL.md) for image geometry.
