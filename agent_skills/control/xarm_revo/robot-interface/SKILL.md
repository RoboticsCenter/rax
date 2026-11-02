---
name: robot-interface
description: Control xArm arms carrying BrainCo Revo 2 hands through the backend MCP tools.
---

Read `xarm_status` before motion. Arming is operator-owned; never enable, reset or
clear hardware errors through another route. `xarm_hold()` disarms and independently
stops all configured arms and hands. Inspect each stop confirmation.

{% if robot.bimanual %}
Every tool except `xarm_status`, `xarm_hold` and `xarm_move_bimanual` requires
`arm="left"` or `arm="right"`. There is no default arm. A hand's arm selector
identifies the hand mounted on that arm. `xarm_status` contains `arms.left` and
`arms.right`. All positions and quaternions are in the selected arm's own base frame;
left-base points are not right-base points. Observation metadata records this frame.
Use the same arm for observing, pixel lookup and the subsequent move.

`xarm_move_bimanual(left={position: [...]}, right={position: [...]})` validates both
targets before concurrent dispatch. It has no hardware synchronization or collision
planner between arms. Keep both trajectories and mounted hands clear of one another.
Start with sequential moves and verify the scene after each step.
{% endif %}

`xarm_move_pose` takes metres, xyzw quaternions and optional relative translations
in the selected base frame. `xarm_move_joints` takes six radians or `preset="home"`.
The inherited xArm controller checks workspace bounds, joint limits, FK and measured
completion. Home is an arm preset, not a verified collision-free path for a new rig.
Check the other arm and the entire hand envelope before each motion.

Use `revo_move`, `revo_pose`, `revo_home`, `revo_status`, `revo_hold` and any configured
optional Revo tools described by [hand-control](../hand-control/SKILL.md).
There is no scalar gripper-width control or Wuji impedance interface on this embodiment.

Observe with `xarm_observe`; review recorded frames with `xarm_stream`. Camera names
are {{ robot.cameras | join(', ') }}. Consult the live tool schemas for exact limits.
`runtime/stream/index.json` describes camera recordings. With bimanual operation,
robot records contain both arms. Each hand's feedback timestamp is independent.

After an execution failure, inspect status and all stop outcomes. Do not replay the
failed command. Operator recovery and rearming may be required. A controller's stop
ACK alone does not prove standstill, and a hand obstruction does not prove a grasp.
