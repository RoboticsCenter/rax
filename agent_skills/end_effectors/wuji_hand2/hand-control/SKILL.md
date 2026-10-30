---
name: hand-control
description: Shape, grasp and release with the twenty-joint Wuji Hand 2 through MCP. Covers bounded geometry probes, joint trajectories, current/impedance controls, feedback, retention and recovery.
---

# Wuji Hand 2 control

Read `wuji_status` before moving. Confirm the configured side, `profile_revision`,
fresh state, no faults, and `mode: holding`. The operator prepares/enables the hand
and arms the session. Request help if it is unprepared; agent tools do not enable,
clear faults, raise the operator's parameter ceilings, or recalibrate joint origins.

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

For a deliberate grasp closure where object contact is expected, call
`wuji_move_joints(positions=closure_target, completion="contact")`. The reference
must finish and measured joints must remain stable for the configured settling
window, with low measured velocity and error within `max_tracking_error_rad`.
An endpoint shortfall can then return `outcome: held_short_of_target`, `ok: true`,
`reached: false`, `holding: true`, and `contact_verified: false`. The hand keeps
the commanded target, gains, damping, current caps and feed-forward bias; RAX
does not disable it solely for this accepted shortfall. Inspect per-joint errors,
images and actual object motion. A stable obstruction is not proof of a grasp;
verify retention with a short lift only when the scene supports that action.
Do not reflexively call `xarm_hold` or increase the target after this outcome,
because that would respectively release the hold or add unverified closing effort.

Omit `completion` (or use `"position"`) for ordinary positioning and release.
`wuji_home` and named `wuji_pose` postures always require position accuracy.
Contact completion does not relax the tracking-error guard, joint limits,
freshness, encoder/fault checks, current caps, or speed/acceleration/jerk limits.
Unstable feedback or an unfinished trajectory still cannot complete as a contact hold.

With `motion_mode: scurve`, the backend also limits reference jerk and sends
matching position/velocity commands. Movements start from the preceding commanded
state and finish at zero reference velocity/acceleration. Requested durations may
round up to a control tick; measured settling can take extra time. Status exposes
motion progress, reference derivatives and actual `command_timing`. These are
reference limits, not measured tracking guarantees.

`wuji_pose(name=...)` uses an operator-configured posture listed in `postures`.
The list can be empty. A saved home or fist is not automatically a qualified grasp
or collision-free preshape for this scene. Never invent names; use bounded
`wuji_move_joints` experiments to develop task-specific postures and record the evidence.
`wuji_home()` moves to the configured `home` posture through the same custom
controller as `wuji_move_joints` and `wuji_pose`. It retains active gains, damping,
current limits and current bias; S-curve mode uses the same bounded jerk and
position/velocity references. Omit `duration_s` to let the controller choose the
duration. Missing home configuration is an error; do not substitute zeros.
This moves to a saved posture; it does not recalibrate encoders or enable the hand.

All three motion tools wait for measured settling and keep the final target afterward;
only explicit contact completion permits a stable target shortfall.
They do not certify self-collision clearance or a successful grasp.

`effort_a`, motor current and current limits are **amperes**, not calibrated contact
force or joint torque. Optional fingertip-online flags report availability, not
contact. Use images and actual object motion to assess a grasp.

## Learn geometry through bounded probes

Full CAD registration or measured contact offsets may be unavailable. Use nominal
kinematics and previous experience as hypotheses, tied to hand revision, joint
signs/zeros, posture, wrist orientation and mount. Finger offsets change with joint
angles; the index tip is not necessarily the whole hand's lowest point. Track
uncertainty rather than assigning a single fixed TCP-to-contact offset to every pose.

Choose a small, reversible action whose full sweep is clear in the available views
and whose result distinguishes useful hypotheses: a one-joint perturbation, a small
clear-space wrist rotation, or an improved view. Change one factor at a time and
compare before/after images and measured joints. Update the estimated opposition,
contact region and lowest-point envelope before approaching the object. Qualification
is evidence for the next bounded step, not a requirement to finish a complete hand model.

Continue while evidence supports a bounded next action. Ask for specific help when
occlusion or uncertainty prevents bounding that action. A person at the stop is a
backstop, not a clearance measurement. Do not interpret missing tactile/depth data,
motor current, a stalled finger or a settled joint target as proof of contact or grasp.

## Current bias and compliant contact

`wuji_set_effort(effort_a=[...], duration_s=...)` ramps twenty signed feed-forward
currents in amperes, in the same joint order. It retains the position reference and
MIT gains. Omit `duration_s` to choose a ramp that respects `max_slew_a_s`; a shorter
requested ramp is rejected. The tool reports completion of the command ramp, not
physical settling, contact or grasp success. The motor combines position/velocity
feedback with this bias under the active hardware current cap. Current is not a
calibrated fingertip force, and its sign must be established for each intended joint.

The bias persists through arm moves, finger moves, `wuji_hold`, and pauses while
armed and healthy. Inspect `command_effort_a` alongside measured `effort_a`, images,
tracking errors and diagnostics. To remove the bias, ramp to twenty zeros with
`wuji_set_effort`. Reducing bias or stiffness can release an object; support it first.
Disarm, faults and emergency stop cancel the bias and request motor disable.

`effort_control.velocity_guard` reports the operator's measured-speed policy.
An optional, bounded grace period tolerates brief excursions; sustained overspeed,
the immediate hard cutoff, or excessive displacement still stop the hand. This
does not change reference speed or grant permission to increase force through a
fault. `last_trip` retains the triggering feedback as history after recovery;
use current `fault`, `mode`, freshness and diagnostics to assess readiness.

`wuji_set_impedance(kp=[...], kd=[...], current_limit_a=[...])` accepts one or more
twenty-element arrays; omitted parameters retain their active values. First clear
the feed-forward bias and verify the measured hand is stationary. Values cannot
exceed `profile_control_parameters`; kp and current limits must remain positive.
The tool verifies device readback and retains the commanded position target.
Lower gains can soften contact, and lower current caps can limit available effort;
neither proves a safe or calibrated contact force. Watch tracking error when using
lower stiffness; the normal error guard still applies. A slow or failed parameter
write can fault and disable the hand rather than waive its scheduling watchdog.

`control_parameters` reports the active gains/caps; `profile_control_parameters`
reports the operator's baseline and ceilings. Overrides persist until changed or
the operator actually prepares a disabled hand again, restoring the baseline and
zero feed-forward bias. Renewing arming on an already healthy hold preserves them.
Record active gains, caps and bias with every successful grasp/preshape; a posture
without these parameters no longer fully specifies the control state.

## Grasp, carry, release

1. Observe and localize the object. Choose or develop an evidence-supported preshape and approach
   with clearance for every finger, the thumb and the palm. Arm tools move the
   configured TCP; it is not a moving fingertip frame.
2. Close gradually with bounded targets and `completion="contact"` for expected
   contact. Observe the hand and object after each
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

## Return home between episodes

Begin training episodes from both home positions and finish each episode by
returning both devices home before requesting the next scene reset. This includes
failed attempts when recovery is supported by fresh, fault-free state.

1. Support and release any held object with a verified release posture. Confirm
   the hand is empty, then retreat the arm in bounded steps until the full hand
   sweep and arm return path have clearance. Home is not a release command.
2. If a task applied feed-forward current bias, ramp it to zero with
   `wuji_set_effort` after release. Keep the commissioned gains/damping and caps.
3. Call `wuji_home()` and require measured completion. Check `wuji_status` for
   fresh, fault-free holding at the settled home target.
4. Call `xarm_move_joints(preset="home")` and require measured completion. Check
   `xarm_status`, `wuji_status` and fresh images before reporting the reset complete.
5. Only then call `request_operator(kind="reset", ...)` for the next layout.
   After the operator replies, check status and observations again before starting.

Run these moves sequentially. If a return is refused, faults, times out, or the
path cannot be established, stop and request specific operator help; do not start
another episode or run homing unconditionally in an exception/finally handler.
Record reset completion or the reason it was blocked with each episode result.

## Hold and failures

`wuji_hold` retains a stationary commanded posture when control is healthy.
It also retains the active current bias and impedance; it is not a force-off command.
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

After recovery, `xarm_hold` can verify a stopped assembly and clear
`control_uncertain` without a movement probe. It requests hand disable and does
not ready, prepare or arm either device; operator readiness is still required
before homing or another episode. An xArm controller fault already opens a
dashboard help request; a subsequent help call waits on that existing request.

An operator may configure the specific auto-clearing encoder warning to be logged
under `warnings`/`warning_history`. This does not waive other faults or establish
encoder accuracy. Agent tools cannot change that policy or clear latched faults.

Record successful postures with the side, profile revision, serial, firmware,
mount/TCP and camera-calibration context, and decisive before/after frames in
memory. Reacquire object geometry on each attempt. Follow
[robot-interface](../robot-interface/SKILL.md) for arm frames and limits and
[object-localization](../object-localization/SKILL.md) for image geometry.
