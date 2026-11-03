---
name: manual-robot-control
description: Solve the task on the real {{ robot.name }} by choosing robot actions directly from observations. Use for solving the task by hand, recovery, and preparing the scene for script tests.
---

# Manual robot control

This is real hardware with people nearby. Use complete supported moves through
verified clear space, and smaller, slower steps near contact or uncertain geometry.
Inspect meaningful results and stop when the next action cannot be bounded. First learn how
to complete the task yourself through the robot tools, then automate what worked.

## First look

1. Read `TASK.md` and identify the required object relationships, orientation,
   placement, robot end state and any episode limit. Write a short completion checklist in your notes.
2. Call `status` and read `MCP_SESSION.md`. Inspect the robot with
   `{{ robot.status }}` and the scene with `{{ robot.observe }}`. If the robot reports
   that motion is not armed, do not try to work around it: the operator arms motion.
3. View the current camera images. Identify the relevant objects and a short sequence
   of manipulation stages.
4. Ground the next target in current observations with
   [object-localization](../object-localization/SKILL.md), and choose an action whose
   full path has established clearance. Use [robot-interface](../robot-interface/SKILL.md)
   for motion tools, speed and limits. For arm travel through verified clear space,
   use a complete supported move; reserve smaller steps for contact or uncertain geometry.
5. Inspect the reply and new images, revise the next action, and save useful
   parameters and evidence. Work through the checklist until every condition holds.

## Observe and choose actions

Use these skills as needed rather than loading every reference up front:

- [Robot interface](../robot-interface/SKILL.md) for this robot's motion tools,
  frames, limits and conventions. Read it before composing actions.
{% if camera_geometry %}
- [Object localization](../object-localization/SKILL.md) for metric positions of
  visible object points from the current images and depth.
{% else %}
- [Object localization](../object-localization/SKILL.md) to locate visible object
  points from the current images.
{% endif %}
- [Wrist-camera inspection](../wrist-camera-inspection/SKILL.md) for a closer view
  with an empty end effector when the {{ robot.overview }} is insufficient.
- [Experience memory](../experience-memory/SKILL.md) to record and reuse tested
  motion strategies under matching observable conditions.
{% if robot.has_hand %}
- [Hand control](../hand-control/SKILL.md) for posture-dependent geometry, bounded
  learning probes, joint-current/impedance control, and grip retention.
{% endif %}

{{ block("images.forwarding") }}

## Operator demonstrations and steering

When the operator supplies a demonstration video or image, read
[references/demonstrations.md](references/demonstrations.md). Imported files under
`context/` are references for strategy and task intent; re-localize the real scene
and choose targets in this robot's current frames. Record which source segments
you inspected and which decisions they informed. An available filename is not
evidence that you inspected the demonstration.

Apply new operator steering to the ongoing task and update `memory/PROGRESS.md`.
If the operator physically intervened, observe again before continuing. Keep the
original wording separate from your interpretation, and distinguish an assisted
attempt from an autonomous success in your run record.

## Motion and feedback

Distinguish commanded targets from measured states: the reply's measured pose tells
you what physically happened. Real tracking is imperfect; check the measured error
before the next step rather than assuming the target was reached.

Observe the result of each meaningful action and adapt. If a motion tool fails or
reports `control_uncertain`, do not repeat the motion blindly: observe, reason about
what happened, and continue only from a known state. If the robot refuses a target
(workspace or joint limit), treat it as a wall, not a fault. If localization fails or
is uncertain, distinguish what was observed from what was inferred and take a new
view or another bounded information-gathering action. Do not use an unverified
coordinate guess as permission to descend or contact an object.
{% if robot.has_hand %}

Incomplete hand geometry does not by itself end the task. Use the hand-control
probe workflow to learn from small, reversible experiments in observed clear space.
State the hypothesis, observation and next decision; ask the operator for a specific
missing observation only when the next action cannot be bounded. Keep advancing
toward the physical task rather than substituting a report for a feasible next probe.
{% endif %}

## Confirm success

Partial visible progress or a clean tool return is not completion. Check every
condition in `TASK.md` against fresh images from more than one camera. Record the
successful strategy and evidence in `memory/`. Continue into automation and further
physical tests only within the task's requested scope. For a single-episode task,
leave the requested stable end state and report the result.
