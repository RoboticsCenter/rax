---
name: dex-skill-library
description: Find, run, compare, compose and add dexterous skills and their competing variants (code-as-policy, MPC, RL, imitation, hybrid) in the rig's skill library at skill-library/{{ robot.skill_library }}/. Read it before grasping, rotating, handing over or placing with the Wuji hands, and before writing a hand primitive of your own.
---

# The rig's dexterous skill library

`skill-library/{{ robot.skill_library }}/` in your workspace is a git clone of this rig's **dexterous skill
library**: primitive skills (power-grasp, pinch-grasp, in-hand-rotate, screw, unscrew, regrasp, handover, place,
...) for the {{ robot.name }}, each with **several competing variants**. You are on your own branch,
`session/<this session>`.{% if robot.skill_library_merge %} The tool `skill_library_merge` checks your branch and
merges it into the library's `main`, where the next session finds it.{% else %} This session's library is
read-only: nothing you commit is merged.{% endif %}


| Where | Written by | Holds | Lifetime |
|---|---|---|---|
| `.agents/skills/` | people (the harness) | how to use the robot, tools, limits; read-only | every session |
| `memory/` | you | this task's notes, runs, frames | this task |
| `skill-library/{{ robot.skill_library }}/` | agents, through merges | executable primitive variants with evidence | every later session |
| `skill-library/dex/`, `skill-library/dexmate/` | other robots' sessions | Mooncake references (screw-lightbulb) | never merged |

The harness skills and live tool schemas take precedence: a library skill never changes a limit, a safety rule
or what a tool does. [hand-control](../hand-control/SKILL.md) and
[robot-interface](../robot-interface/SKILL.md) govern every motion a skill makes.

## 1. Find

1. Read `skill-library/{{ robot.skill_library }}/INDEX.md`, then pick up merges since your workspace was made:
   `git -C skill-library/{{ robot.skill_library }} fetch origin && git -C skill-library/{{ robot.skill_library }} merge origin/main`.
2. Pick the **primitive** your step needs, then a **variant**. Variants live in
   `skills/<primitive>--<representation>[-<tag>]/`. Choose by its frontmatter:
   - `hands` and `requires.hand_features`: the left hand runs 18/20 joints (`thumb_S3`, `middle_S1` passive) and
     refuses contact completion, effort and impedance; a variant needing those runs only on the right hand.
   - `requires`: a `sim_model` (mpc), a `checkpoint` (rl/imitation), a GPU, or a perception input you must supply.
   - `control.mode`: `streaming` variants need the streaming hand API; today the tools block, so they refuse.
   - `status` (prefer `verified`) and `COMPARISON.md` (success rate and smoothness per variant on this rig).
3. Read only the chosen variant's SKILL.md (and the skills in its `uses:`). Its `## Preconditions`,
   `## Checks`, perception-checkpoint table and `params.json` (tested ranges, tuning recipes) say when it applies.

## 2. Run

From the workspace root:

```python
import importlib.util, json
from api.runtime import Context

def load(slug):
    path = f"skill-library/{{ robot.skill_library }}/skills/{slug}/code/run.py"
    spec = importlib.util.spec_from_file_location(slug.replace("-", "_"), path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module

grasp = load("power-grasp--code-synergy")
with Context() as ctx:
    pre = grasp.check_preconditions(ctx, side="right")          # {ok, why, failed: [precondition ids]}
    if pre["ok"]:
        result = grasp.main(ctx, side="right", lift_m=0.03)     # {ok, measured: {status, ...}, why}
        held = grasp.success(ctx, result["measured"])            # the skill's own two-signal re-check
    print(json.dumps(result, indent=1, default=str))
```

- Parameters are overrides of the variant's `params.json`; values outside a tested range run but are listed in
  `measured.params_warnings`. Measure or tune them with each entry's `tune` recipe on a new object or posture.
- `measured.status` is `success`, `failed_precondition` (refused before moving), `failed` (no harm done),
  `dropped`, `timeout` or `aborted`; on any failure the skill leaves the hand **holding** (`wuji_hold`).
- Look at each **perception checkpoint** yourself in a scene the evidence does not cover; pass
  `observe_cameras=[...]` where a variant offers it, and check the frames it saved.
- **Dry-run first, without the robot**: `sys.path.insert(0, "skill-library/{{ robot.skill_library }}/lib")`,
  `from dexlib.testing import FakeCtx`, and call `main(FakeCtx(blocks=...), ...)` to exercise its logic. FakeCtx is
  kinematic only; it is not evidence.
- One script moves the robot at a time; MCP calls are serialized.

## 3. Compose

A task is a chain of primitives: perceive -> approach -> grasp -> transport -> regrasp / rotate -> place ->
release. Chain variants so that one's **postconditions** are the next one's **preconditions**, and pass what the
next one needs from `measured` (a grasp's `contact_baseline` and `closing` feed the hold checks of in-hand skills).
Before each link call its `check_preconditions`; on a failure, re-check from the furthest stage still feasible
rather than repeating blindly, and cap retries (`dexlib.contract.retry`). Split only where the robot is in a
stable, describable state, never in the middle of a contact. Keep the chain in `code/<project>/` and record each
run in `memory/RUNS.md`.

## 4. Add or improve a variant

1. `cp -r skills/_template skills/<primitive>--<representation>-<tag>` and rename `code/_template` to the package
   (the folder name with `_` for `-`). Keep `code/run.py` unchanged.
2. Fill **every** frontmatter field (README.md "Skill anatomy": primitive, representation, hands, requires,
   control, preconditions/postconditions as `"<id>: <checkable statement>"`, progress stages), the seven sections
   and the perception-checkpoint table.
3. Write the procedure in the package through `dexlib.io.HandIO` only (it applies limits and the partial-hand rules);
   reuse `dexlib` (postures/synergies, contact evidence, approach waypoints, contract/retry, params) and other
   variants' packages through `uses:`. Every tunable goes in `params.json` with its tested range and tuning recipe.
4. Implement `check_preconditions` (one check per precondition id), `main` (typed statuses via
   `dexlib.contract`) and `success` (two independent signals where possible).
5. Dry-run on FakeCtx, then run on the robot. Record **every** run, failures included, as one line in
   `evidence/runs.jsonl`: `date, session, commit, version, backend (real|sim), trial (the trial directory in the
   shared trace format), params, success, status, unattended, conditions, smoothness`.
   `dexlib.trace.SampleRecorder` writes a coarse trial directory today (endpoints of blocking moves); full-rate
   traces come with the streaming hand API.
6. Add the INDEX.md row and the variant's row in the primitive's COMPARISON.md table.

`verified` needs at least 2 unattended successful **real** runs of the current `version`; bump `version` when
behaviour changes. Never delete another session's variant: set `status: deprecated` and say why.

{% if robot.skill_library_merge %}
## 5. Commit and merge

```bash
cd skill-library/{{ robot.skill_library }}
git add -A && git commit -m "<variant>: what changed"
git fetch origin && git merge origin/main
```

Then call `skill_library_merge` with a one-line `message`. It answers `merged: true`, or `problems` (format and
evidence rules a changed skill breaks: fix, commit, call again), or `conflicts` (merge `origin/main`, keep both
sessions' evidence lines, commit, call again). A branch may change only `INDEX.md`, `COMPARISON.md` and
`skills/` (not `skills/_template/`); `lib/`, `methods/`, `README.md` and `library.json` are maintainer-owned.
Merge when a variant is first automated (as `draft`), when its evidence changes, and before your final report.
{% endif %}
