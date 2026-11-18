# Dexterous-hand runbook for the RAX robot machine

**Who reads this:** Claude Code and the operator on the **RAX robot computer**, the machine
wired to the two xArm 6 arms, the two Wuji Hand 2 hands and the RealSense cameras
(Tailscale name `jerry-blade-15-mid-2019-base`; checkout `/home/jerry/rax`; artifact root
`/home/jerry/rax-runtime`). It tells you what to run there and in what order, and what to
send back. A separate analysis machine wrote the code, but it cannot reach the robot.
Results travel to it as `.tar.gz` files that the operator copies over.

To load this as Claude Code context on the robot machine, add this line to the repo's
`CLAUDE.md` (or `CLAUDE.local.md`): `@docs/dex-robot-runbook.md`

---

## 0. Rules for Claude on this machine (read first)

1. **The operator moves the robot, not you.** Never arm a session, type `READY`/`RUN`, or
   start a hardware run without the operator's explicit go-ahead in this conversation. Prepare
   commands, run dry runs, explain what will move, then wait. The operator stands at the
   e-stop for every hardware run.
2. **Never change firmware, the SDK version or current caps on your own.** Firmware upgrades
   or downgrades, `pip install wuji-sdk==...`, and caps above **0.8 A** are operator decisions.
   The right hand is already on firmware 3.0.0 (see §1). Leave the left hand alone.
3. **Push nothing unless the operator asks.** The RAX `dex` branch and the mooncake skill
   library are maintained from the analysis machine. Local fixes go on a local branch, and you
   tell the operator about them. The operator may ask you to push the RAX `dex` branch;
   never push to `real/*` or `main` branches.
4. **Stall is the main hazard.** If any Wuji joint sits still with its current pinned at the
   current limit, the firmware raises `Stall` (0x0007). RAX then emergency-stops the hand 0.1 s
   later (`stall_warning_grace_s`). Every grip must keep effective kp × lead ≤ about 0.7 × cap
   (see §1). Stop and report on any Stall warning.
5. **Heat:** stop a run if the hottest hand MCU goes above **55 °C**. Let it cool below 50 °C
   before starting another. The hands ran at 54–59 °C during the batteries.
6. **One hand, empty first.** Use the right hand only, at reduced speed, empty-hand or
   free-air first, then light soft objects. Left-hand and bimanual skills come only after the
   right hand is verified (§6.4).
7. **Always export.** Every hardware run ends with a `.tar.gz` under `~/battery/` or
   `~/dex-trials/`, which the operator copies to the analysis machine (§7).

---

## 1. Current state (as of 2026-11-18)

### Code and branches

| What | Where | Notes |
|---|---|---|
| RAX code | `RoboticsCenter/rax`, branch **`dex`** | Smooth hand control, motion battery, eval tools, skill-library wiring, MuJoCo twin. `git pull` on `dex`. |
| Skill library | `XCZhang03/mooncake`, branch **`skill-library/xarm-wuji-bimanual/dex`** | 20 draft skills. RAX loads it as library `xarmwujibimanual`. |
| M5 (firmware Position mode) code | **This machine only, uncommitted** (`wuji_m5_battery.py`, `wuji_m5_smoke.py`, battery edits) | Preserve it before pulling (§2.1). |
| Research report | analysis machine: `reports/Dexterous skill policy representations.md` | Background only. |

### Hardware facts measured on this rig

| Fact | Value / evidence | Consequence |
|---|---|---|
| Effective PD gain | about **1.98×** the commanded kp/kd (reported current ≈ 1.987 × nominal PD) | Effective stiffness ≈ kp × 1.98 A/rad: about 16 A/rad at the operator profile 8/0.15 and about 10 A/rad at the default preset 5/0.1 |
| Stall e-stop | Firmware `Stall` when a joint is still with its current at the limit; RAX e-stops after 0.1 s | Hold with lead ≤ 0.7 × cap ÷ stiffness (0.022 rad at kp 8 with a 0.5 A cap). Recover: `bash scripts/wuji recover-stall --session S --hand right` |
| Self-contact | `thumb_S2` stalled at −0.818 rad going to the commissioned fist (−1.25) when the fingers led the thumb | Whole-hand closes must not let the thumb cross already-flexed fingers |
| thumb_S2 encoder | Repeated `Enc1BitRate` warnings; about 0.019 rad of hold jitter | Treat thumb_S2 hold noise as sensor noise; watch it |
| Heat | 54–59 °C peaks during batteries; thermal guard tripped once | Short holds, 0.8 A cap, cooldowns |
| Stick-slip | Slips peak at about 1.0 rad/s (1.45 rad/s with soft gains) on 0.01–0.6 rad/s moves | This is the jerkiness and crackle; see the battery results below |
| Left hand | `thumb_S3` and `middle_S1` disabled (faulted, passive); 0.15 rad/s profile | Report it separately; it is slow; long left-hand streams dropped the hand service twice in the twin |
| Right hand firmware | **3.0.0** (serial WH2KA01260823017), upgraded 2026-11-17, MIT mode restored afterwards | RAX's fault exceptions were written for 2.13.1; check `wuji status` before every session |
| Left hand firmware | 2.13.1 (assumed unchanged) | Do not upgrade |
| Stream limits (new defaults on `dex`) | right 5 rad/s² / 30 rad/s³, left 2.5 / 15 | Streamed skills need jerk ≥ 15 rad/s³. Only **new** sessions get them. |
| Friction feed-forward | 0.04 A was the best value tested (config default 0.08 may exceed breakaway) | Skills set 0.04 themselves; the battery patch uses 0.04 |

### Motion-control results so far (right hand, one trial per item)

Conditions: **M0** today's blocking rest-to-rest S-curve; **M1** streaming; **M2** M1 plus
0.04 A friction feed-forward; **M3** M2 with the soft default preset; **M4** M3 plus online
replanning; **M5** firmware 3.0.0 Position mode with its 40 Hz command filter.

| Finding | Evidence |
|---|---|
| M2 is the best of M0–M4, but only modestly | thumb_S2 SPARC ratio 3.25 → 2.47, stick-slip 60 → 36; whole-hand slips −40–50% |
| M3 (soft gains) and M4 (online replan) are worse | more slips, slip peaks of about 1.45 rad/s |
| Slow ramps stay rough under every condition | SPARC ratio 5–7× the commanded reference |
| **M5 is the leading candidate** | At matched 30 Hz sampling: single-joint SPARC ratio 1.0–1.7 vs 1.5–2.1 for M2; tracking about 0.002 rad vs 0.007 |
| **Not yet known for M5** | Whether it removes stick-slip (it was recorded at 30 Hz, which cannot see 8 ms slips); contact and stall behavior; a same-firmware comparison with M2 |

**Current default for skills: M2-style streaming** (profile gains plus 0.04 A friction
feed-forward). M5 needs the rerun in §4 before it becomes the default.

---

## 2. Setup and update (each time before working)

### 2.1 Preserve the local M5 work, then update RAX
```bash
cd /home/jerry/rax
git status                     # expect the uncommitted M5 files / battery edits
git switch -c m5-local         # keep them on a local branch (do NOT push)
git add -A scripts local_tests services && git commit -m "M5 position-mode battery (local, firmware 3.0.0)"
git switch dex && git pull     # stream limits, smoke script, twin fixes, this runbook
```
If `git status` is already clean, just run `git switch dex && git pull`. Tell the operator
what was committed to `m5-local`.

### 2.2 Environment
```bash
cd /home/jerry/rax
export RAX_PYTHON=$PWD/.venv/bin/python
export RAX_ARTIFACT_ROOT=/home/jerry/rax-runtime
export RAX_MOONCAKE_REPO=git@github.com:XCZhang03/mooncake.git
uv pip install --python .venv/bin/python -r requirements-xarm-wuji.txt -r requirements-dex-eval.txt
```
Record the versions (they go into every export):
```bash
"$RAX_PYTHON" -m pip freeze | grep -iE "wuji|ruckig|numpy|scipy|mujoco"
git rev-parse HEAD
```

### 2.3 Refresh the skill library
```bash
"$RAX_PYTHON" -m services.skill_library sync xarmwujibimanual
```
If it reports no host library, skip it: the next session creates one from
`skill-library/xarm-wuji-bimanual/dex`. Check the variants with
`"$RAX_PYTHON" -m services.skill_library list <checkout>`, or read the library's `INDEX.md`.

### 2.4 Hands on and healthy
Both hands need power. "fresh positions and diagnostics required; ages_s={'state': None...}"
means a hand is off or disconnected. With a session (§3):
```bash
bash scripts/wuji status --session SESSION --hand right
bash scripts/wuji status --session SESSION --hand left
```
Check the firmware, mode, faults and temperature. Report anything unexpected before moving.

---

## 3. Create a fresh session (needed for the new profiles and library)

Old sessions keep the profiles they were created with (no stream limits). Create a new one:
```bash
cd /home/jerry/rax
export RAX_BACKEND_OPTIONS=$PWD/examples/xarm-wuji-bimanual.json
"$RAX_PYTHON" scripts/configure_session.py --robot xarm_bimanual --task <task-name> \
  --task-file <task.md> --observation-profile rgbd
# The last line printed is SESSION/operator-input.json
```
Confirm the profile picked up the limits: `stream a/j 5.0/30.0` (right) and `2.5/15.0` (left).
The dry run in §4.1 prints them.

---

## 4. Procedure A: motion battery (low-level controller comparison)

### 4.1 One-shot smoke test with export (M0–M4, right hand)
```bash
bash scripts/wuji_smoke_export.sh SESSION/operator-input.json --name <name> --dry-run   # plan only
bash scripts/wuji_smoke_export.sh SESSION/operator-input.json --name <name>             # hardware
```
- **What it does:** the battery copies the session, starts only the hand owners and never
  connects the arms. It prints the arming command for its own session copy.
- **Arming:** the operator runs `"$RAX_PYTHON" scripts/arm_robot.py <printed battery session>
  arm --minutes 60` in a second terminal, then types `READY`.
- **Output:** `~/battery/<name>.tar.gz` (trials, `report/`, `battery.log`, `env.txt`, `patch.json`
  and the battery session's hand logs). On failure it prints `FAILED`, packs the logs only, and
  shows the hand-service log tail.
- **Options:** `--hand left` uses half the stream limits. For the full protocol, edit the script
  or run the battery directly with `--repeats 10 --speeds 0.1,0.3,0.6 --shuffle`.

### 4.2 Run the battery directly (custom runs)
```bash
"$RAX_PYTHON" scripts/wuji_motion_battery.py --session SESSION/operator-input.json \
  --hands right --conditions M0_baseline,M2_stream_friction_ff \
  --items single_joint,stick_slip,fist_pinch,tapping,hold,waypoints5 \
  --repeats 3 --shuffle --seed 1 --hand-patch patch.json --start-temp-c 55 --out ~/battery/<name> [--execute]
"$RAX_PYTHON" scripts/dex_eval_report.py ~/battery/<name> --group-by controller --baseline M0_baseline \
  --out ~/battery/<name>/report
```
`patch.json` example: `{"all": {"stream_max_acceleration_rad_s2": 5, "stream_max_jerk_rad_s3": 30,
"friction_current_a": 0.04}}`. The patchable keys are command_hz, current_limit_a, kp, kd,
max_acceleration/jerk, stream_max_acceleration/jerk and friction_current_a. Raising
`current_limit_a` above 0.8 A is an operator decision.

---

## 5. Procedure B: decisive M5 rerun (highest priority)

This settles whether firmware Position mode (M5) should become the default. Run it on the
**right hand** (firmware 3.0.0):

1. **Record at ≥ 200 Hz.** Adapt the local `wuji_m5_battery.py` (branch `m5-local`) so the
   trace samples the hand's state stream at ≥ 200 Hz, ideally the native 1 kHz callback, not
   30 Hz polling. Write the standard trace format, documented in `docs/dex-eval.md` and
   `services/wuji/trace.py`:
   - per-hand `hand_right.npz` with `t`, `q_ref`, `q_target`, `q_meas`, `dq_meas`, `current`,
     `i_ff`, `kp`, `kd`, plus optional `temperature_c` and `current_limit_a`;
   - `meta.json` with the controller `M5_position_firmware`, the correct `nominal_rate_hz`,
     the firmware and SDK versions in `notes`, and `backend: "real"`.
   
   Validate a trial with `"$RAX_PYTHON" -c "from services.dex_eval.trace import
   validate_trial_dir as v; print(v('<trial dir>'))"`.
2. **Same-firmware baseline:** run `M2_stream_friction_ff` (and `M0_baseline`) with
   `scripts/wuji_motion_battery.py` on the same hand, now on firmware 3.0.0 in MIT mode.
   This separates the effect of the new firmware from the effect of Position mode.
3. **Design:** the items `single_joint, stick_slip, fist_pinch, tapping, hold, waypoints5`;
   at least 3 repeats per condition × item; M0/M2/M5 interleaved or alternated in blocks;
   the same start temperature rule (≤ 55 °C); speed 0.3 rad/s (add 0.1 if time allows).
4. **Contact probe for M5** (separate, operator present): with the hand palm-up and a soft
   foam block, close index and thumb onto it under M5. Use torque limits around 0.1–0.2 N·m
   and watch for `Stall` or overcurrent. Record whether M5 can hold an object gently (a
   bounded position lead) without stalling. If it cannot, skills will switch back to MIT
   for contact.
5. **Export:** `tar czf ~/battery/m5-rerun.tar.gz -C ~/battery <dirs>` plus a short
   `README.txt`: what ran, firmware and SDK versions, anything odd, and what was heard
   (crackle or no crackle per condition).

---

## 6. Procedure C: skill trials on the real right hand (through a RAX agent)

The library skills run inside a normal RAX session: the RAX agent (a separate Claude in the
session sandbox) reads its `dex-skill-library` guide and runs one skill at a time. Each skill
records its own trace and returns `{ok, measured: {status, ...}, why}`.

### 6.1 Task file for the RAX agent
Write `~/dex-skill-trials.md`, create a fresh session with it (§3), ready the rig, and start:
```bash
bash scripts/xarm-wuji-ready --session SESSION
bash scripts/start_agent.sh --config SESSION/operator-input.json
```
Suggested task file content (adjust the ladder step):
```
Try dexterous library skills on the RIGHT hand only, one at a time, with me at the e-stop.
Read the dex-skill-library guide, then each skill's SKILL.md and params.json before running it.
Before every motion, tell me what will move and wait for my OK.

Order:
1. Move the right arm so the palm faces up (ask before moving); record the TCP quaternion as
   palm_up_tcp_quaternion. Run in-hand-rotate--code-gait with empty_hand=true, cycles=2.
2. in-hand-rotate--code-keyframe, empty hand, 2 cycles.
3. in-hand-rotate--mpc-cem with place_object=none (empty palm).
4. power-grasp--mpc-cem with on_robot=false (plan only, no motion) for an object I describe;
   show me the planned grasp against a camera frame.
5. unscrew--code-wrist free-air check only: rotate wrist joint 6 by +0.2 rad to confirm tighten_j6_sign.
6. With the palm up and a light rigid ~60 mm cylinder resting in the open right hand
   (object_observed=true, no lift, cameras on): run power-grasp--code-synergy, then
   power-grasp--code-human (grasp_type diameter_wrap, the measured diameter) as an A/B.

After each run report measured.status, any current-guard/watch events, peak current, max hand
temperature, and the trace directory.
STOP and tell me on any Stall warning, any joint near its current cap, or hand temperature above 55 C.
Do NOT run handover, place or regrasp (they need both arms). Do not merge into the library.
```

### 6.2 Skill catalog (all `draft`; none has run on hardware)

| Primitive | Variant | Kind | First real test | Key params / notes |
|---|---|---|---|---|
| power grasp | `power-grasp--code-synergy` (v2) | code | ~60 mm light cylinder in the open palm, no lift | either hand; engine shared by the v2 grasps |
| | `power-grasp--code-contact` (v2) | code | after synergy; needs `contact_s` from the synergy run's first contact | right hand only |
| | `power-grasp--code-human` | code | A/B against synergy on the same object | `grasp_type` diameter_wrap / power_sphere; human taxonomy postures retargeted with wuji-retargeting |
| | `power-grasp--mpc-cem` | MPC | `on_robot=false` (plan only) | plans on the twin; finds no held grasp in the twin yet |
| | `power-grasp--mpc-cem-rh` | MPC | plan only first | receding-horizon close from joints and currents |
| pinch grasp | `pinch-grasp--code-synergy` (v2) | code | ~20–30 mm object | thumb and index required |
| | `pinch-grasp--code-lateral` | code | a card or flat object | thumb onto the side of the index |
| precision grasp | `precision-grasp--code-tripod` | code | ~30 mm object | fingertip-first tripod |
| in-hand rotation | `in-hand-rotate--code-gait` | code | `empty_hand=true, cycles=2`, palm up | needs `palm_up_tcp_quaternion`; about 4.7 s per cycle at the new stream limits; did not turn objects in the twin |
| | `in-hand-rotate--code-keyframe` | code | empty hand | baseline loop |
| | `in-hand-rotate--mpc-cem` | MPC | `place_object=none` | 10–15 Hz replans; needs an object-pose estimate for real use |
| | `in-hand-rotate--rl` | RL slot | refuses until a checkpoint exists | the proprioceptive RL policy is still training on the DGX Spark |
| screw / unscrew | `unscrew--code-wrist` | code | free-air +0.2 rad J6 sign check, then a loosened cap on a clamped bottle | rotates about the object axis (the grip sits about 64 mm off J6); grip 0.32 A per joint |
| | `screw--code-wrist` | code | after unscrew works | E27 pitch 3.63 mm; backward thread-start search |
| | `unscrew--code-fingers` | code | loose cap only | likely torque-limited |
| bimanual | `handover--code`, `handover--code-humanlike` | code | only after both hands are verified: `rehearsal: true`, empty hands | geometry needs commissioning (TCP separation, quaternions) |
| | `place--code`, `regrasp--code-place`, `regrasp--code-bimanual` | code | after handover | |

### 6.3 What every skill run must report
`measured.status` (success | failed_precondition | failed | dropped | timeout | aborted), `why`,
the trace directory, peak current per digit, any `watch`/back-off events, the max temperature,
the `close_lead` used, and the operator's verdict. Collect the RAX session directory afterwards
(§7).

### 6.4 Ladder before the left hand or bimanual
Right hand empty-hand rotation passes, then a right-hand grasp holds a light object 5/5 times
with no Stall, then the same grasp on the left hand (slow, 0.15 rad/s), then a handover
rehearsal with empty hands.

### 6.5 Optional: the scripted comparison runner
`scripts/dex_eval_run.py PLAN.json --real --session SESSION` attaches to a **running** RAX
session's robot service. It never arms; the operator types `RUN` per condition, confirms each
reset and gives each verdict. RAX has no service-only launch mode, so use it only while the
session's agent is told to stand by and not move anything. Grasp skills ship
`eval/plan-real.json`. Ask the operator first.

---

## 7. Exporting results to the analysis machine

Pack each run and tell the operator the path. The operator copies it to the analysis machine.
```bash
# battery runs: wuji_smoke_export.sh already writes ~/battery/<name>.tar.gz
# skill sessions:
tar czf ~/dex-trials/<yyyymmdd>-<what>.tar.gz -C /home/jerry/rax-runtime/sessions <session-dir-name>
```
Include a `README.txt` in every archive:
- what ran;
- the firmware per hand and the SDK version;
- the `git rev-parse HEAD` of RAX and of the library checkout;
- the operator's observations: sound, smoothness, anything odd;
- which trials were aborted and why.

Never include credential files. The session directories hold no model keys, but check before
packing anything outside them.

---

## 8. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| `fresh positions and diagnostics required; ages_s={'state': None…}` | hand powered off or disconnected | power or reconnect the hand; `bash scripts/wuji status` |
| `hand streaming requires a prepared, idle, holding hand (mode is stopping)` | a Stall or other fault stopped the hand mid-run | read `wuji*-controller.log`; `bash scripts/wuji recover-stall --session S --hand right` once the device's AutoClear has cleared; then prepare again |
| `Stall` during a fist or whole-hand close | thumb collides with flexed fingers, or a grip pins the cap | lower the closing fraction; keep the thumb from crossing; check the lead bound |
| Thermal stop | hand above its guard | `recover-thermal` after cooling; longer rests; lower cap |
| xArm errors 21/24 during Cartesian skill moves | straight-line Cartesian path faults | prefer joint-space moves; split into smaller legs; report to the analysis machine |
| Hand service stops answering during long left-hand streams | seen twice in the twin under host load | avoid long left-hand streams; report with logs |
| Streamed skill refuses on stream limits | old session without the new profile limits | create a fresh session (§3) |
| `thumb_S2 Enc1BitRate` warnings | known encoder data-quality issue on this unit | note it; stop if it becomes persistent or faults |

---

## 9. Where things are documented in this repo
- `docs/wuji-smooth-control.md`: the streaming layer, presets, friction, battery, tuning order.
- `docs/dex-eval.md`: trace format, metrics (SPARC, stick-slip, hold, current, timing, audio),
  runner, report, blind rating.
- `docs/xarm-wuji-sim.md`: the MuJoCo twin (analysis-side; not needed on the robot machine).
- `docs/xarm-wuji-bimanual.md`: rig setup, readiness, operator lifecycle commands.
- `agent_skills/end_effectors/wuji_hand2/dex-skill-library/SKILL.md`: how the RAX agent uses the
  library.
