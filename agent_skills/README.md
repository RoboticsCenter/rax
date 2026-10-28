# Agent skills

Every skill an agent can receive lives here, once. The workspace template
(`auto_research_agent/`) holds only the instructions and the agent-side MCP client;
`services/workspace/build.py` builds each new session's `.agents/skills/`.

An existing workspace contains copies, not live links to these sources. Updating
this directory does not update a running session. For a stopped Wuji session, use
`scripts/refresh_wuji_tools.py --session SESSION` to refresh the tool contract and
all provided skills/references, with backups and hashes. Task-specific memory,
code, recordings and custom skills are preserved. Resume through the official
launcher, then reread the relevant refreshed skills. See
[`docs/xarm-wuji.md`](../docs/xarm-wuji.md) for commands and current tool semantics.

## Layout: by axis

Each workspace takes the layers of its setup, in this order (`SKILL_LAYERS` in
`services/workspace/recipes.py`); a later layer adds to or replaces files of the same
skill, and an absent layer is skipped:

| Layer | Contents |
| --- | --- |
| `task/` | `manual-robot-control`, `autonomous-control`, `experience-memory` |
| `perception/common/` | `live-stream`, `wrist-camera-inspection` |
| `perception/<mode>/` | perception for one observation mode, any robot (none yet) |
| `perception/<mode>-<robot>/` | `object-localization` for that mode and robot (`depth-franka`, `rgb-only-franka`, `depth-xarm`, `rgb-only-xarm`) |
| `control/common/` | control for any robot (none yet) |
| `control/<robot>/` | the robot's `robot-interface` and motion skills (`franka`, `xarm`) |
| `end_effectors/<type>/` | hand-specific guidance selected by backend configuration (`wuji_hand2`) |
| `auto-research/` | `gemini` |

`<mode>` is `depth` or `rgb-only` (from the observation profile) and `<robot>` is the
backend key (`franka`, `xarm`). Agents see one flat `.agents/skills/` with stable, robot-neutral
names, so task skills link to `robot-interface` or `object-localization` without
branching on the robot or the observation mode.

Robot-specific facts a shared source needs come from `robot.*` (`ROBOT_FACTS` in
`services/workspace/setup.py`): tool names, cameras, the overview camera.

## Templates

Every `.md` is rendered with Jinja (`services/workspace/templates.py`) against
`Setup.context()` plus `vocab.*`; unknown names fail the build. Reusable passages are
registered blocks in `agent_blocks/blocks.toml`, inserted with `{{ block("<id>") }}`.
