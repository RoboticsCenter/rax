---
name: hand-control
description: Shape, grasp and release with the twenty-joint Wuji Hand 2 through MCP. Covers bounded geometry probes, joint trajectories, current/impedance controls, feedback, retention and recovery.
---

# Wuji Hand 2 control

{% if robot.bimanual %}
Every single-hand tool requires `hand="left"` or `hand="right"`. Read each hand's
status separately. Examples below apply to the explicitly selected hand. Use each
side's loaded targets, limits, home, current bias, impedance and warning policy;
do not infer a posture by mirroring or copying the other hand.
`wuji_move_both(left={...}, right={...})` takes the usual move payload for each
hand, validates both, then starts them concurrently. It does not guarantee exact
synchronization or mutual collision clearance. An execution fault stops all four
devices; a healthy `wuji_hold(hand=...)` applies only to that hand.
For arm and finger motion together, use `xarm_move_combined` with optional
`left_arm`, `right_arm`, `left_hand` and `right_hand` actions. Hand actions are
`{"joints": {...}}`, `{"pose": {"name": "..."}}` or `{"home": {}}`, containing
the ordinary tool payload. The same partial-joint, completion and motion policies
apply. See [robot-interface](../robot-interface/SKILL.md) for the combined schema.
{% endif %}

Read `wuji_status` before moving. Confirm the configured side, `profile_revision`,
fresh state, no faults, and `mode: holding`. The operator prepares/enables the hand
and arms the session. Request help if it is unprepared; agent tools do not enable,
clear faults, raise the operator's parameter ceilings, or recalibrate joint origins.

`command_timing` separates host timing from native diagnostics. Its `delays` and
`skipped_slots` are informational while feedback is fresh and control remains
holding/executing without faults. Do not stop or request recovery solely because
a soft timing warning was recorded. Read `warning_threshold_s`, `stop_threshold_s`
and the current `fault`; a host watchdog trip is distinct from a native hand error.
An unfinished S-curve can also exceed its wall-time deadline when the achieved
command rate is below the requested rate, even without a command-gap watchdog
trip. Report that timing failure separately from native joint diagnostics; it is
not an accepted completed-reference shortfall.

{% if robot.has_hand %}
## This session's hand profiles

These values come from the launch configuration; live status is authoritative.

| Hand | Profile revision | Controlled joints | Passive joints | Saved postures |
| --- | --- | --- | --- | --- |
{% for side, profile in robot.hand_profiles.items() %}
| {{ side }} | `{{ profile.profile_revision }}` | {{ profile.active_joint_count }}/20 | {{ profile.disabled_joints | join(', ') or 'none' }} | {{ profile.postures | join(', ') or 'none' }} |
{% endfor %}
{% endif %}

For `partial_hand: true`, use `active_enabled`, not all-joint `enabled`, to assess
readiness. The configured passive joints and their isolated diagnostic codes do
not block supported moves. Position moves, saved poses/home and hold are supported;
`completion="contact"`, `wuji_set_effort` and `wuji_set_impedance` are unavailable
for that hand. Keep its commissioned gains/caps and zero feed-forward bias.
The other hand keeps its own full or partial profile.
Home moves only the active joints. A passive joint may remain elsewhere, so an
active-joint home result does not establish the complete finger's home shape.

## Commands and state

`wuji_move_joints` takes `positions`: exactly twenty **absolute radians**, ordered
thumb S1–S4, index S1–S4, middle S1–S4, ring S1–S4, pinky S1–S4. Each finger has
four entries. Read configured limits from status; do not reuse Allegro, the other hand's,
or another firmware/origin profile's numbers. A zero vector is a motion target,
not a hold or a guaranteed open pose.

Start a small change from `target_positions` while checking measured `positions`.
Modify only the intended joints. The backend bounds the per-call change, velocity,
acceleration and tracking error. Optional `duration_s` must allow those bounds;
omitting it lets the backend choose. A rejection calls for revising the target,
not retrying the same command.

For a deliberate grasp closure on a **full-control hand**, where object contact is expected, call
`wuji_move_joints({% if robot.bimanual %}hand=selected_hand, {% endif %}positions=closure_target, completion="contact")`. The reference
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
`wuji_home` and named `wuji_pose` postures use position completion; partial hands
report a completed reference's position shortfall without disabling control.
Contact completion does not relax the tracking-error guard, joint limits,
freshness, encoder/fault checks, current caps, or speed/acceleration/jerk limits.
Unstable feedback or an unfinished trajectory still cannot complete as a contact hold.
For a partial hand, use ordinary position completion, preserving the passive
entries from `target_positions`. A completed reference may report an informational
shortfall with `ok: true`, `holding: true`; inspect the measured result and continue
the supported sequence without another preparation solely for that shortfall.

With `motion_mode: scurve`, the backend also limits reference jerk and sends
matching position/velocity commands. Movements start from the preceding commanded
state and finish at zero reference velocity/acceleration. Requested durations may
round up to a control tick; measured settling can take extra time. Status exposes
motion progress, reference derivatives and actual `command_timing`. These are
reference limits, not measured tracking guarantees.

`wuji_pose({% if robot.bimanual %}hand="left", {% endif %}name=...)` uses an operator-configured posture listed in `postures`.
The list can be empty. A saved home or fist is not automatically a qualified grasp
or collision-free preshape for this scene. Never invent names; use bounded
`wuji_move_joints` experiments to develop task-specific postures and record the evidence.
`wuji_home({% if robot.bimanual %}hand="left"{% endif %})` moves to the configured `home` posture through the same custom
controller as `wuji_move_joints` and `wuji_pose`. It retains active gains, damping,
current limits and current bias; S-curve mode uses the same bounded jerk and
position/velocity references. Omit `duration_s` to let the controller choose the
duration. Missing home configuration is an error; do not substitute zeros.
This moves to a saved posture; it does not recalibrate encoders or enable the hand.

All three motion tools wait for measured settling and keep the final target afterward.
Full-hand mode accepts stable shortfalls only with explicit contact completion;
partial-hand mode also reports completed position-reference shortfalls without stopping.
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

This section applies only to a hand with `partial_hand: false`; skip these commands
for a partial hand, including a redundant command to set its bias to zero.

`wuji_set_effort({% if robot.bimanual %}hand=selected_hand, {% endif %}effort_a=[...], duration_s=...)` ramps twenty signed feed-forward
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

`wuji_set_impedance({% if robot.bimanual %}hand=selected_hand, {% endif %}kp=[...], kd=[...], current_limit_a=[...])` accepts one or more
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
2. Close gradually with bounded targets. Use `completion="contact"` for expected
   contact on a full-control hand, or ordinary position completion on a partial
   hand. Observe the hand and object after each
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

{% if robot.bimanual %}
Apply the procedure below to each side with explicit `hand` and `arm` selectors.
Before starting, verify that each hand actually lists a commissioned `home`
posture in its own status. Use the session profile table above; do not substitute
the other hand's values or zeros. Get an operator-defined reset only if a home
is actually absent.
Release/support anything spanning the hands before homing either side. Order
the returns so neither arm crosses the other. Independent returns may use the
combined tool only when the whole arm-and-finger sweeps have established clearance.
In this setup `xarm_hold` stops both arms and both hands, not only the selected side.
{% endif %}

Begin training episodes from the commissioned reset state and finish each episode by
returning {% if robot.bimanual %}each assembly{% else %}both devices{% endif %} home before requesting the next scene reset. This includes
failed attempts when recovery is supported by fresh, fault-free state.

1. Support and release any held object with a verified release posture. Confirm
   the hand is empty, then retreat the arm in bounded steps until the full hand
   sweep and arm return path have clearance. Home is not a release command.
2. If a full-control hand has applied feed-forward current bias, ramp it to zero
   with `wuji_set_effort` after release. Skip this tool for a partial hand, whose
   bias remains zero. Keep the commissioned gains/damping and caps.
3. Call `wuji_home({% if robot.bimanual %}hand="left"{% endif %})` and inspect completion and measured active joints. Check
   the live profile's `max_step_rad`; this is a per-command distance cap, not a
   speed setting. A saved-home command may span the full configured distance
   while retaining its trajectory velocity, acceleration and jerk limits. Do not
   impose a smaller historical cap or split a valid saved-home move unnecessarily.
   Check `wuji_status` for fresh holding with no blocking faults. For a partial hand,
   a completed-reference shortfall with `ok: true`, `holding: true` can continue
   through the reset sequence; record that shortfall instead of claiming exact
   home convergence or requiring an extra precision test.
4. Call `xarm_move_joints({% if robot.bimanual %}arm="left", {% endif %}preset="home")` and require measured completion. Check
   `xarm_status`, `wuji_status` and fresh images before reporting the reset complete.
   Read the current arm `home_joints`; the operator can change it independently
   of the saved finger home.{% if robot.bimanual %} Left and right arm homes can differ.{% endif %}
5. Only then call `request_operator(kind="reset", ...)` for the next layout.
   After the operator replies, check status and observations again before starting.

Keep dependent release, retreat and return steps sequential; combine only returns
whose complete sweeps are established as independent. A validation refusal with `no_action_executed: true`
and certain control leaves the existing holds intact; resolve the reported input
or profile issue without automatically calling an assembly stop. Ask the operator
for configuration changes if needed. An execution fault, uncertain control or an
unclear physical path requires stopping and specific operator help; do not start
another episode or run homing unconditionally in an exception/finally handler.
Record reset completion or the reason it was blocked with each episode result.

## Hold and failures

`wuji_hold` retains a stationary commanded posture when control is healthy.
It also retains the active current bias and impedance; it is not a force-off command.
If an S-curve is still moving when dispatched, it brakes smoothly to a nearby
bounded rest target and reports that target; physical settling is not confirmed.
It requests emergency stop if bounded braking is unavailable, the session is disarmed or state
is unreliable. `xarm_hold` requests stops on {% if robot.bimanual %}both arms and both hands{% else %}both the arm and hand{% endif %}. These calls
report request, SDK return and confirmation separately. Grip retention after an
emergency stop, disable, session end or lost connection is not guaranteed.

MCP calls are serialized. `wuji_hold` and `xarm_hold` from another MCP client wait
behind an active tool call; they cannot interrupt it immediately. The operator's
direct stop/disarm path bypasses that queue. Keep motion calls bounded and do not
rely on a second MCP client as an emergency interrupt.

A blocking fault, stale feedback, unexpected active-joint state or `control_uncertain` means stop
and request operator help. A timeout/target-not-reached is not grasp success. The
backend can stop {% if robot.bimanual %}all four devices{% else %}both devices{% endif %} after a failed motion; check status again before
planning recovery. Reconnection never resumes an interrupted trajectory.
Configured `isolated_faults`, allowed `warnings` and diagnostic exceptions are
reported separately from blocking `faults`; do not turn their presence alone into
an additional stop condition. Use the backend's current result and readiness.

`Stall` (`0x0007`, Warning/AutoClear) is distinct from an accepted position
shortfall. The backend records it and stops; operator recovery can reconnect once
the native Stall has cleared and the hand is disabled and stationary. In partial
mode the exact configured isolated faults may remain on excluded joints; they do
not block recovery from an unrelated cleared Stall. Do not interpret a raised
current cap as commanded current or proof that it fixed the stall.

After recovery, `xarm_hold` can verify a stopped assembly and clear
`control_uncertain` without a movement probe. It requests hand disable and does
not ready, prepare or arm either device; operator readiness is still required
before homing or another episode. An xArm controller fault already opens a
dashboard help request; a subsequent help call waits on that existing request.

An operator may configure the specific auto-clearing encoder warning to be logged
under `warnings`/`warning_history`. This does not waive other faults or establish
encoder accuracy. Agent tools cannot change that policy or clear latched faults.

An operator may separately allow `InitCurrentCalibFailed` (`0x3208`) for an exact
joint and firmware in `diagnostics_policy.allow_init_current_calib_failed`.
These remain labelled with their original SDK severity under
`diagnostic_exceptions` and `diagnostic_exception_history`; they are not evidence
of healthy current sensing. Read them alongside native joint states and error
codes. In full-hand mode all joints must still confirm enabled control before motion. Other faults
and all feedback/thermal/tracking/current guards remain active. Agent tools cannot
add exceptions or change their scope; a native enable refusal requires operator
help, not another bypass or automatic retry.

When `partial_hand` is true, the operator has explicitly excluded joints from
control. Require `active_enabled: true`, fresh feedback and `mode: holding`; native
`enabled` still means all twenty joints and is therefore false. Read `active_joints`,
`disabled_joints`, `isolated_faults` and their native states. The excluded joints
must remain Ready/Stopped with zero configured gains and current limits. Their
faults remain visible and do not establish that the hand is healthy.

Partial mode supports `wuji_move_joints`, saved poses/home, status and hold. Copy `target_positions`
before changing active entries; leave excluded entries exactly unchanged even if
their measured positions drift. They are passive and are not held at those entries.
Movement completion and tracking apply only to active joints.
For partial-hand moves, `target_not_reached` with `holding: true` means the bounded
reference finished and the position shortfall is informational. It does not
disable the hand or require another preparation. Inspect fresh holding status and
the measured result, then continue the authorized finger sequence; do not invent
a mandatory precision sweep or stop solely because `reached` is false. A real
fault, incomplete reference, loss of feedback or isolation violation still stops.
Saved hand poses/home apply only to active joints; the backend keeps excluded
references unchanged. Contact completion and current/impedance tools are refused.
Arm movement uses the original xArm controller checks; partial-hand mode alone
does not block it. A disabled joint changes the finger geometry; do not infer usable grasping or retention from
an active-joint motion result. Stop when the backend reports a blocking fault or isolation violation. Agent
tools cannot change the exclusion mask, isolated-fault policy or enable motors.

Record successful postures with the side, profile revision, serial, firmware,
mount/TCP and camera-calibration context, and decisive before/after frames in
memory. Reacquire object geometry on each attempt. Follow
[robot-interface](../robot-interface/SKILL.md) for arm frames and limits and
[object-localization](../object-localization/SKILL.md) for image geometry.
