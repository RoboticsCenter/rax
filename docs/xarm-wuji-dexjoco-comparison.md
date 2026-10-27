# xArm + Wuji comparison with RAX's simulated dexterous-hand branch

Report date: 2026-10-27.

**Finding: the Wuji integration follows the DexJoCo branch's architecture and
agent workflow for dexterous control. It is a hardware-specific adaptation, with
an explicit multi-joint hand interface, measured feedback, visual verification,
and reusable agent skills. Full behavioral parity is not established: contact
completion, coordinated arm/hand trajectories, and demonstrated manipulation
remain material differences.**

This report was restored after the user requested removal of the uncommitted
implementation work. The checkout is back at `dc8991a`; the camera/calibration
changes reviewed earlier are no longer installed. Findings about that discarded
work are identified below as historical. Restoring this document does not restore
the implementation changes.

The original assessment compared the locally available Git snapshots of
`origin/dexjoco/native` at `bcd748cfcff5e2fe53d5025afc239c89285e9e1e` and
`origin/dexjoco/container` at `e600e99a7436cb0b2f6e3f43cdf93afef6491b8b` with
`real/xarmwuji` at `dc8991a00352fb30393ae604648ab60833bc77ab` plus the then-current
working-tree changes. Remote branch freshness was not checked. The compared
DexJoCo tool definitions, backend, simulation session, hand-control skill, and
robot-interface skill are identical between those two simulation snapshots.

The reference robot is a simulated Franka Panda with a 16-joint Allegro hand
(two assemblies for bimanual tasks). The target is one real xArm 6 with a
20-joint Wuji Hand 2. The existing Wuji design plan already identifies the
DexJoCo skills as adaptation sources and distinguishes reusable manipulation
principles from Allegro-specific assumptions. See
[the original adaptation rationale](xarm-wuji-backend-plan.md#7-skills-found-on-the-other-rax-branches).

The following similarities are supported by the committed implementation:

| Area | DexJoCo reference | xArm + Wuji implementation | Assessment |
| --- | --- | --- | --- |
| Agent access | The agent and its Python programs use a host-owned MCP service. | The existing MCP frontend dispatches xArm and Wuji tools; the agent does not own the SDK connection. | Same architectural pattern. |
| Execution ownership | A host backend delegates to an isolated simulator process. | `XArmBackend` delegates hand control through `WujiClient` to a private controller process with one SDK writer. | Same separation of agent decisions from execution; different process implementation. |
| Hand command representation | Sixteen absolute finger-joint targets in radians per hand, carried in each action row. | Twenty absolute finger-joint targets in radians via `wuji_move_joints`; `wuji_pose` supplies a configured target. | Same joint-target abstraction, adapted to Wuji anatomy. |
| Feedback | Observations expose measured hand joints and flange pose. | Status exposes measured and commanded hand joints, arm state, velocities, current, diagnostics, and freshness. | Comparable observability with additional hardware diagnostics. |
| Agent skills | Separate `robot-interface`, `hand-control`, and perception skills explain control and grasping. | Workspace generation installs those same skill roles, selecting Wuji through an end-effector configuration axis. | Same composition approach. |
| Manipulation workflow | Preshape, close gradually, retain commanded finger targets while carrying, and inspect grasp/release in images. | The Wuji hand skill specifies those same phases, with hardware-specific limits and recovery. | Instruction-level alignment; physical success remains to be demonstrated. |
| Perception | RGB/RGB-D observations, wrist/overview views, and pixel back-projection support scene-based decisions. | Existing xArm observation and geometry tools are reused. | Same perception role; real calibration must match the physical setup. |
| Reusable controllers | Explore through tools, save evidence in memory, then write observation-driven Python functions using the same tools. | The manual/autonomous-control skills and Python MCP client support that workflow. | Same exploration-to-code workflow; evaluation infrastructure differs. |
| Input rejection and evidence | Tool validation precedes simulator dispatch; motion replies and artifacts record execution. | Wuji validates shapes, limits, arming, and readiness before dispatch; audit logs, hand snapshots, and motion clips record execution. | Same separation of rejected requests from execution failures. |

Simulation evidence: [tool schemas and layouts][sim-tools],
[backend dispatch][sim-backend], [execution and hold semantics][sim-session],
[hand-control instructions][sim-hand], and [workspace composition][sim-recipes].
Hardware evidence: [MCP frontend](../services/controller/frontend.py),
[xArm backend](../services/xarm/backend.py),
[Wuji client](../services/wuji/client.py),
[controller](../services/wuji/controller.py),
[workspace setup](../services/workspace/setup.py), and
[Wuji hand-control instructions](../agent_skills/end_effectors/wuji_hand2/hand-control/SKILL.md).

The interfaces correspond as follows; this is a conceptual mapping, not a
drop-in API conversion:

| DexJoCo capability | Wuji-session counterpart | Translation required |
| --- | --- | --- |
| `dexjoco_status` and observed robot state | `xarm_status`, `wuji_status` | Different state layouts and hardware-health fields. |
| Arm portion of `dexjoco_step` | `xarm_move_pose`; also `xarm_move_joints` | World-frame flange position/rotation vector becomes xArm-base TCP position/quaternion `[x,y,z,w]`. |
| Hand portion of `dexjoco_step` | `wuji_move_joints`, `wuji_pose` | Allegro `ff,mf,rf,th` order becomes Wuji `thumb,index,middle,ring,pinky`, four joints each. Values and limits require commissioning. |
| `dexjoco_observe` | `xarm_observe` | Real observations include a hand snapshot; camera, arm, and hand samples are not exactly synchronized. |
| `dexjoco_pixel_to_position` | `xarm_pixel_to_position` in RGB-D sessions | Uses real depth/calibration and xArm-base coordinates instead of simulator world coordinates. |
| `dexjoco_hold` | Related to `wuji_hold`, but not equivalent | DexJoCo's first hold seeds measured arm/hand state. Healthy Wuji hold preserves the existing commanded hand target. `xarm_hold` requests assembly stopping. |

Several differences are deliberate adaptations to real hardware:

- **Control runs while the agent thinks.** The DexJoCo harness advances physics
  only on action/hold calls. Wuji continuously publishes targets at the configured
  rate (120 Hz by default), including between calls. This is a software scheduling
  target, not a hard real-time guarantee. Arming, feedback, scheduling, and parent
  watchdogs operate independently of the agent's next tool call.
- **Enable and recovery belong to the operator.** Preparation verifies the
  current/gain profile, seeds measured joint positions, and confirms enable.
  Emergency handling requests SDK stop plus explicit disable and checks new
  feedback. These mechanisms have no equivalent need in a paused simulation.
- **Geometry and units are specific to the robot.** Wuji joint arrays, posture
  values, TCP offsets, quaternion ordering, and camera transforms cannot be copied
  from Allegro. Rejecting out-of-limit commands and avoiding an assumed zero/open
  posture are appropriate adaptations.
- **Capabilities are selected honestly.** Wuji sessions publish hand tools and
  omit `xarm_gripper`; G2 sessions retain their existing interface. The port does
  not advertise DexJoCo's robot-only preview or expert demonstration tools.

The remaining differences limit a stronger equivalence claim:

1. **Contact-limited closure is not equivalent.** The DexJoCo hand skill explicitly
   recommends maintaining a flexion target beyond the measured contact angle to
   produce squeeze. Wuji currently requires every joint to settle within its
   configured position tolerance: the defaults are 0.040 rad for 0.2 seconds,
   with a 3-second settling timeout after the trajectory. If contact prevents
   convergence beyond that timeout, the controller returns `target_not_reached`.
   Although the controller initially keeps the target, the client raises
   `HandError` and the MCP backend requests arm and hand stops. Thus the ordinary
   MCP path does not preserve an unsuccessful contact-limited closure for carrying.
   This follows from the code and is not a claim that every possible grasp fails.
   The default 0.35 rad tracking bound can also reject a large contact offset.
   A commissioned way to distinguish acceptable grasp contact from obstruction or
   fault is needed before claiming equivalent grasp behavior; simply copying the
   simulator's squeeze offsets or increasing tolerances would not establish it.
   Sources: [DexJoCo hand skill][sim-hand],
   [Wuji controller](../services/wuji/controller.py),
   [client error handling](../services/wuji/client.py), and
   [backend assembly-stop handling](../services/xarm/backend.py).

2. **Arm and finger trajectories are sequential.** A DexJoCo row updates arm pose
   and finger targets together, including both arms on bimanual tasks. Wuji's
   continuous process preserves a hand posture during xArm motion, but the exposed
   MCP API does not coordinate changing finger targets with an arm trajectory.
   This supports the intended staged approach/close/carry/release workflow at the
   interface level; it does not reproduce simultaneous in-hand manipulation or
   bimanual action rows.

3. **Grasp postures and geometry are not yet commissioned.** The Wuji example has
   an empty `postures` map. The discarded calibration work recorded a controller
   TCP using the old 172 mm gripper reference, rather than a measured Wuji
   palm/grasp reference. It supplied provisional wrist/side calibration and a
   flange-relative transform for future TCP changes. That fit's maximum residual
   was 3.363 mm / 1.455 degrees, exceeding the automatic 1-degree rotation
   threshold. Those uncommitted transforms, configuration changes, scripts, and
   the calibration report were removed at the user's request. These measurements
   describe the earlier review, not the restored checkout's active calibration.
   The committed [example configuration](../examples/xarm-wuji.json) has reverted
   to its checkpoint camera setup.

4. **Simulation research facilities were not ported.** Robot-only motion preview,
   recorded Allegro expert trajectories, automatic seeded resets, benchmark
   scoring, parallel simulated environments, and frozen-controller formal
   evaluation are features of the DexJoCo branch. The physical branch uses
   operator-assisted resets and physical task verification. Their absence does
   not negate the shared MCP/skill design, but means this is not a complete port
   of the simulation branch's feature set. See [DexJoCo backend guide][sim-guide]
   and [physical autonomous-control workflow](../agent_skills/task/autonomous-control/SKILL.md).

The follow-up discussion also established the distinction between homing and
episode reset:

| Setup | Implemented reset behavior |
| --- | --- |
| xArm + G2 on `real/franka` | `scripts/xarm home` returns the arm to `HOME_Q`; `--open` also opens the gripper. The agent can call `xarm_move_joints(preset="home")`. |
| xArm + Wuji | The arm-home command remains, without resetting the fingers. `--open` is rejected for Wuji. There is no `wuji home` operator command or predefined hand-home posture. |
| DexJoCo | `start_episode` closes the previous episode and creates the next environment, resetting arm, hand, and scene. It advances to the next layout and consumes an episode. There is no separate hand-home MCP tool. |

Wuji can execute `wuji_pose(name="home")` only after an operator configures and
commissions a twenty-joint posture with that name. The current example's empty
posture map does not supply one. Named postures obey the same movement bounds as
joint commands, so a return from a distant posture may require intermediate
targets. `prepare` enables holding at the measured posture; `wuji_hold` preserves
the accepted target. Neither resets the fingers. A physical between-attempt reset
routine still needs a verified hand-home posture and an appropriate sequence with
arm homing. Sources: [xArm operator commands](../scripts/xarm),
[Wuji operator commands](../services/wuji/operator.py),
[posture validation](../services/wuji/config.py),
[simulation episode frontend][sim-frontend], and [simulation reset][sim-session].

Verification distinguishes implementation coverage from hardware qualification.
The preceding review ran 118 relevant hardware-free tests successfully, with three
multiprocessing deprecation warnings. That result applies to the implementation
before rollback, including the now-discarded calibration helpers. It is not a
fresh test result for the restored checkpoint. The tests covered tool selection,
workspace contracts, G2 compatibility, target retention, bounded trajectories,
blocked-target failure, process watchdogs, stop behavior, and calibration helpers.
The comparison added source inspection; it did not perform a
simulation-versus-hardware equivalence test. The relevant native/container
simulation files were confirmed identical using `git diff`. No real robot or
simulator was run for the comparison or document restoration.

The committed [real-hand test record](wuji-live-test-2026-10-25.md) documents all
four Wuji tools, small index-finger movements, a recorded test posture, target
holding, input rejection, and confirmed disable. It also records inconsistent
settling, including failure at the original 0.040 rad tolerance. Those tests did
not demonstrate full-hand grasping, carrying, or release. The later calibration
record reviewed before rollback described arm movement while the hand held its
measured posture, without a grasped object; that record is no longer in the checkout.

The supported acceptance statement is: **RAX's Wuji port implements the same
high-level pattern for agent-driven dexterous control as DexJoCo: host-owned MCP
execution, explicit hand-joint commands, measured feedback, perception, and
hand-aware skills. It supports further hardware commissioning; behavioral
equivalence for grasping remains unverified.** To establish that stronger claim,
resolve contact-completion semantics, commission the hand's postures and geometry,
and demonstrate repeated observe–approach–close–lift–carry–release sequences with
measured state and visual evidence. Add coordinated arm/hand trajectories only if
the intended task requires the simulator's simultaneous control behavior.

[sim-tools]: https://github.com/RoboticsCenter/rax/blob/bcd748cfcff5e2fe53d5025afc239c89285e9e1e/services/dexjoco/tools.py
[sim-backend]: https://github.com/RoboticsCenter/rax/blob/bcd748cfcff5e2fe53d5025afc239c89285e9e1e/services/dexjoco/backend.py
[sim-session]: https://github.com/RoboticsCenter/rax/blob/bcd748cfcff5e2fe53d5025afc239c89285e9e1e/services/dexjoco/session.py
[sim-hand]: https://github.com/RoboticsCenter/rax/blob/bcd748cfcff5e2fe53d5025afc239c89285e9e1e/agent_skills/control/dexjoco/hand-control/SKILL.md
[sim-recipes]: https://github.com/RoboticsCenter/rax/blob/bcd748cfcff5e2fe53d5025afc239c89285e9e1e/services/workspace/recipes.py
[sim-guide]: https://github.com/RoboticsCenter/rax/blob/bcd748cfcff5e2fe53d5025afc239c89285e9e1e/docs/DEXJOCO_BACKEND.md
[sim-frontend]: https://github.com/RoboticsCenter/rax/blob/bcd748cfcff5e2fe53d5025afc239c89285e9e1e/services/controller/frontend.py
