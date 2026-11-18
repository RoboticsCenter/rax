# Dexterous skill evaluation harness (`services/dex_eval`)

This harness ranks competing variants of each dexterous primitive (grasp, in-hand rotation,
screw/unscrew, handover) on the bimanual xArm 6 + Wuji Hand 2 rig. Variants may be
code-as-policy, MPC or RL. They are ranked by task success first, and then by how smooth and
natural their motion is. It implements the evaluation protocol in
`research_notes/Dexterous skill policy representations/low_level_smooth_hand_control.md`
(section 5), and the same reports summarize the low-level free-space motion battery
(`scripts/wuji_motion_battery.py`).

```
scripts/dex_eval_run.py     run randomized, interleaved trials (mock | sim | real) -> results.jsonl + trace dirs
scripts/dex_eval_report.py  metrics, rankings, flags and plots -> report.md, summary.json, trials.csv, plots/
scripts/dex_eval_rate.py    blind pairwise video ratings -> Bradley-Terry naturalness ranking
```

Analysis needs only numpy, scipy and matplotlib (`requirements-dex-eval.txt`). Fingertip
SPARC additionally needs `mujoco` and a wuji-description checkout.

## Quick start (no hardware)

```bash
python scripts/dex_eval_run.py plan.json --out /data/dex-eval/grasp-1     # mock backend by default
python scripts/dex_eval_report.py /data/dex-eval/grasp-1                   # -> /data/dex-eval/grasp-1/report/
python scripts/dex_eval_rate.py build /data/dex-eval/grasp-1 --session /data/dex-eval/grasp-1/rating --camera front
python scripts/dex_eval_rate.py serve --session /data/dex-eval/grasp-1/rating   # http://127.0.0.1:8765
python scripts/dex_eval_rate.py rank  --session /data/dex-eval/grasp-1/rating
python scripts/dex_eval_report.py /data/dex-eval/grasp-1 --ratings /data/dex-eval/grasp-1/rating
```

A minimal `plan.json`:

```json
{
  "name": "grasp-compare",
  "trials": 10,
  "seed": 20261117,
  "sides": ["right"],
  "skill_paths": ["/path/to/skills"],
  "conditions": [
    {"skill": "grasp", "variant": "cap_waypoints", "controller": "C1-stream",
     "params": {"object": "cylinder", "approach_speed": {"uniform": [0.2, 0.4]}}},
    {"skill": "grasp", "variant": "mpc_v2", "params": {"object": "cylinder"}},
    {"skill": "grasp", "variant": "rl_ema", "entry": "/path/to/rl_grasp.py:main", "params": {}}
  ]
}
```

## Shared contracts

### Trace directory (one per trial)

| File | Content |
|---|---|
| `hand_<side>.npz` | `t` (T,) float64 monotonic s; `q_ref` (T,20) interpolated reference sent to the hand, rad; `q_target` (T,20) latest caller target; `q_meas` (T,20); `dq_meas` (T,20), NaN if unavailable; `current` (T,20) A, NaN if unavailable; `i_ff` (T,20) feed-forward A; `kp`, `kd` (T,20). Disabled joints are NaN in the measured arrays. Optional extra arrays are kept (`temperature_c` (T,20) is used for thermal metrics). |
| `arm_<side>.npz` | optional: `t`, `q_ref` (T,6), `q_meas` (T,6) |
| `meta.json` | `sides`, `joint_names` (20, SDK order), `nominal_rate_hz`, `controller`, `skill`, `variant`, `params`, `success` (bool or null), `started_at`, `backend` (`real`/`sim`/`mock`), `notes`. Extra keys are allowed. The runner adds `trial_id`, `block`, `condition_id`, `run`, `seed`, `result`, `skill_source` and `trace_source`, and optionally `audio.start_t`, `audio.offset_s` and `current_limit_a`. |
| `audio.wav` | optional, 48 kHz |
| `video_<camera>.mp4` | optional; used by the rating tool |

`services.dex_eval.trace` reads, writes and validates this format.
`validate_trial_dir(path)` returns `(errors, warnings)`; `load_trial(path)` raises
`TraceError` listing every violation. The validator treats the following as errors:
missing arrays, wrong shapes, decreasing time, missing meta keys, an invalid
`backend`/`success`/`sides`, and a listed side without its file. Duplicate timestamps and
partially NaN command arrays are only warnings. A joint counts as active when `q_meas` is
finite in at least 90% of its samples.

### Skill entry and outcome

`main(ctx, **params) -> {"ok": bool, "measured": dict, "why": str}`, where
`measured["status"]` is one of `success | failed_precondition | dropped | timeout | aborted`.
Task success is derived in this order:

1. a boolean `meta.success` (an operator verdict or external verifier) wins;
2. otherwise `ok and measured.status == "success"`;
3. otherwise `ok` alone, when the status is missing.

When a skill raises, or returns something malformed, the trial is recorded as `aborted`,
with the exception and the shape problems kept in the results line.

## Trial runner (`scripts/dex_eval_run.py`)

**Design.** For `trials` blocks, each block runs every condition once in a fresh, seeded
random order. Trials are therefore randomized, interleaved and balanced, so drift affects
every condition equally. Use at least 10 trials per condition; the runner warns below that.
Parameter leaves can be sampled per trial, reproducibly from the recorded seed:
`{"uniform": [a, b]}`, `{"loguniform": [a, b]}`, `{"normal": [mu, sd]}`, `{"randint": [a, b]}`
or `{"choice": [...]}`. The schedule, seed, plan, git commit and argv go into `run.json`.

**ctx.** Every trial calls `main(ctx, **params)` with the controller runtime's ctx
(`auto_research_agent/api/runtime.py: Context`: `call`, `value`, `request`, `images`).

- `mock` / `sim`: the runner builds the production `services.controller.frontend.Frontend`
  in process, around a backend. `InProcessContext` subclasses the runtime `Context` and
  replaces only its socket transport, keeping the JSON round trip, so tool calls,
  validation, errors and the `mcp.jsonl` audit log behave exactly as for agent code.
  - `mock` is `services.dex_eval.mock_rig.MockRigBackend`. It is a simulated-time
    bimanual hand and arm plant: a second-order tracker plus breakaway friction, so slow
    references stick-slip as on the real hands. It also exposes `trace_start`/`trace_stop`.
  - `sim` constructs a backend registered in `services.backends`, by default the first key
    containing `sim`, or the one given with `--sim-robot`. Until the simulation backend is
    registered, `--backend sim` stops with a clear message.
- `real`: connects the runtime `Context` to the operator's **already running** robot
  service socket (default `harness.claude_cli.launcher.socket_path(<session>)`, or
  `--socket`).

**Real-robot safety.**
- Real runs require `--real` together with `--session <session dir>`, and an interactive
  terminal.
- The runner **never starts, enables or arms the robot**. It only reads
  `services.xarm.arming` before every trial. If the session is not armed, it asks the
  operator to arm it with `scripts/arm_robot.py`, or to quit.
- Each condition needs a typed `RUN` confirmation before its first trial. Every trial is
  preceded by a scene-reset prompt (Enter / skip / quit) and followed by the operator's
  success verdict, which is stored in `meta.success`.
- Ctrl-C asks for a best-effort `xarm_hold` (which disarms) and re-raises.
- A test asserts that the runner sources never call `arming.write`.

**Skill resolution** (`services/dex_eval/skills.py`):
1. An explicit `entry` (`pkg.module:function` or `path/file.py[:function]`).
2. The host-side skill library. `services.skill_library` is probed for one of
   `load_skill_main`, `load_skill`, `resolve_skill`, `get_skill` or `skill_main`; another
   change adds this function. It is called with `library_name`, `skill/variant`,
   `skill_variant`, `variant`, then `skill`, and may return a callable, a module or object
   with `main`, or a `.py` path. Use `--skill-loader module:function` to plug in a
   differently named loader.
3. `skill_paths` / `--skill-path`: `<dir>/<skill>/<variant>.py`, `<dir>/<skill>/<variant>/main.py`,
   `<dir>/<skill>_<variant>.py`, `<dir>/<variant>.py`, `<dir>/<skill>.py`.

**Trace recording** (`--trace auto|service|poll|none`):
- `service`: the robot service writes the trace. The runner calls the first tool pair the
  ctx lists among `trace_start/trace_stop`, `wuji_trace_start/stop`, `dex_trace_start/stop`,
  `record_start/stop`, `wuji_record_start/stop` and `trace_record_start/stop`, or the pair
  given with `--trace-tools START,STOP`. The trial directory is passed when the start tool
  accepts one of `directory`, `trace_dir`, `out_dir`, `path` or `output_dir`. A label goes
  in `label`/`trial_id`/`name`. Files written elsewhere, reported as `directory`, are
  copied in.
- `poll` (the `auto` fallback): polls `wuji_status`-shaped telemetry at `--poll-hz`
  (default 100). Real runs read the hand services' read-only `status` op. In-process
  backends are called directly, bypassing the frontend's dispatch lock, so polling
  continues during blocking moves. The mapping is:
  - `q_ref` = `target_positions` (the per-tick commanded reference);
  - `q_target` = `motion.requested_positions`;
  - `q_meas`, `dq_meas`, `current` = `positions`, `velocity_rad_s`, `effort_a`;
  - `i_ff` = `command_effort_a`;
  - `kp`, `kd` = `control_parameters`;
  - `temperature_c` = `diagnostics.mcu_temp_c_fb`.

  `meta.trace_source = "poll"`. A polled trace is coarse: its timing metrics describe
  the polling, not the commands, and slips shorter than a few poll periods are
  under-resolved.
- `--audio-device hw:X,Y` records 48 kHz mono `audio.wav` with `arecord`. It stores
  `meta.audio.start_t` on the monotonic clock. Refine alignment with `meta.audio.offset_s`,
  for example from a sync clap.

Output: `run.json`, `results.jsonl` (one line per trial) and `trials/<NNNN>_<skill>_<variant>/`.
A results line holds `trial_id`, `index`, `block`, `condition_id`, `skill`, `variant`,
`controller`, `params`, `backend`, `seed`, `trace_dir`, `started_at`, `duration_s`,
`result`, `status`, `ok`, `success`, `success_source`, `error`, `trace_source`,
`recorder_errors`, `trace_errors` and `skill_source`. With `--evaluate` it also holds a
`summary`.

## Metrics (`services/dex_eval/metrics.py`, `audio.py`, `fingertip.py`)

**Analysis pipeline.** Each hand trace is resampled linearly onto a uniform grid at its
median sample rate. Disabled joints are excluded, and interior NaN gaps are bridged.
- **Moving joints** are active joints whose reference spans at least 0.01 rad.
- **Reference motion** means joint-space reference speed above
  max(2% of its peak, 0.005 rad/s).
- **The analysis window** covers all reference motion, padded by 0.5 s.
- **Smoothness metrics** low-pass positions at 20 Hz (2nd-order Butterworth, zero phase)
  before differentiating, as the protocol recommends (20–30 Hz).
- **The reference** is processed identically, so each metric has a self-referenced twin.

| Metric (summary key) | Definition | Default parameters |
|---|---|---|
| SPARC (`sparc`, `sparc_ref`, `sparc_joint_median`) | Balasubramanian et al. 2015: the negative arc length of the normalized Fourier magnitude spectrum of the speed profile, over [0, ωc]. ωc is adaptive: the highest frequency ≤ 10 Hz whose magnitude is still ≥ 0.05. `nfft = 2^(ceil(log2 T)+4)`. Hand: joint-space speed ‖dq/dt‖ over moving joints; per joint: \|dq_j/dt\|. Closer to 0 is smoother. Minimum-jerk ≈ −1.40. | 0.05 threshold, 10 Hz cap, padlevel 4 |
| SPARC ratio (`sparc_ratio`) | measured SPARC / reference SPARC for the same trial (worst hand). 1 = as smooth as commanded. | |
| Fingertip SPARC (`fingertip_sparc`, `_ref`) | SPARC of each fingertip-site speed via MuJoCo FK on the official Wuji Hand 2 MJCF, averaged over fingers. Disabled joints take their reference value. Skipped (`available: false`) without `mujoco` or the model. | ≤ 200 Hz FK rate |
| LDLJ (`ldlj`, `ldlj_ref`) | −ln( (t2−t1)³ / v_peak² · ∫‖d²v/dt²‖² dt ) on the joint-velocity vector over the analysis window. The research note writes (t2−t1)⁵, which is only dimensionless with an amplitude normalization; with v_peak the exponent is 3. Minimum-jerk = −ln(720/1.875²) = −5.32. | |
| RMS jerk (`rms_jerk`) | sqrt(mean ‖d³q/dt³‖²) over the window, rad/s³ (noise-sensitive) | |
| Tracking (`tracking_rms_rad`, `_rms_motion_rad`, `_max_rad`, `_final_max_rad`) | e = q_ref − q_meas over active joints: pooled RMS; RMS during reference motion; max \|e\|; worst joint's mean \|e\| over the last 0.2 s | |
| Lag (`tracking_lag_s`) | Delay that maximizes the sum of per-joint normalized cross-correlations of reference and measured velocities, with parabolic sub-sample refinement; positive = measurement lags | ±0.5 s search |
| Overshoot (`overshoot_max_rad`, `_max_rel`) | For each reference move (contiguous motion, amplitude ≥ 0.02 rad), the largest excursion of q_meas beyond the move's final reference in the direction of motion, until the next move | |
| Stick-slip (`stick_slip_count`, `stick_slip_peak_speed`, `burst_count`) | Spike: \|v_meas\| > max(2 × local reference peak speed (±0.1 s), \|v_ref\| + 0.25 rad/s), with v_meas a centred difference over 16 ms. A spike is a **stick-slip event** if, over the preceding 0.15 s, the joint was nearly still (median \|v\| ≤ 0.05 rad/s), the error reached ≥ 0.003 rad and grew by ≥ 0.0005 rad from its window minimum, and the spike moves toward the reference. Other spikes are **bursts**. Reports counts, per-event time/joint/peak speed/jump/error. Peak speeds are 16 ms window speeds, so 8 ms slips read low. | thresholds from docs/wuji-smoothness-2026-10-27.md |
| Hold oscillation (`hold_p2p_max_rad`, `hold_rms_max_rad`, `hold_reversals_per_s`, `hold_oscillation_events`) | Where the reference is constant (\|v_ref\| < 1e-3 rad/s for ≥ 0.5 s, first 0.3 s discarded): the largest peak-to-peak of q_meas over 1 s windows; RMS about the mean; direction reversals per joint-second (0.0005 rad hysteresis); oscillation events = 0.15 s windows with span > 0.006 rad and ≥ 3 reversals (the existing RAX test detector) | |
| Current (`current_rms_a`, `current_peak_a`, `current_frac_near_cap`) | Over active joints: RMS; peak \|I\|; fraction of samples where any joint is within 5% of its cap. The cap comes from `meta.current_limit_a` or `meta.params.current_limit_a` (scalar, 20-list or per side), else `--current-cap` (0.8 A) | |
| Temperature (`temperature_max_c`, `temperature_rise_c`) | From an optional `temperature_c` array: max, and the largest per-joint rise between the first and last second | |
| Command timing (`timing_*`) | From raw `t`: achieved rate and its ratio to `nominal_rate_hz`; interval SD (jitter), p99 and max; gaps = intervals > 1.5 periods; skipped slots = Σ(round(dt/period) − 1) over gaps; max gap in periods | |
| Arm (`arm_sparc`, `arm_sparc_ratio`, `arm_ldlj`, `arm_tracking_rms_rad`, `arm_tracking_lag_s`) | The same pipeline on `arm_<side>.npz` (6 joints) | |
| Audio level (`audio_laeq_db`, `_laeq_motion_db`, `_background_db`, `_motion_above_background_db`) | IEC 61672 A-weighting (bilinear; within 0.3 dB to 4 kHz, about 1 dB low at 10 kHz). Leq in dB re full scale, so **relative only**. Motion and hold are split by the trace's reference motion. The background is the 10th percentile of 125 ms frame levels, or a `--audio-background` recording | |
| Clicks (`audio_clicks`, `_clicks_motion`, `_click_rate_motion_hz`, `_excess_clicks_motion`) | 2 kHz–0.45 fs band-pass; 1 ms frame energy (0.5 ms hop) more than 10 dB above its 100 ms running median; 10 ms refractory. Times are on the trace clock. Excess = motion clicks − hold click rate × motion time. Also reports clicks within 25 ms of stick-slip events and the dominant tone (to test the 200 Hz command-ripple hypothesis) | |

Across hands, the summary averages SPARC and LDLJ. It takes the worst hand for ratios,
errors, holds, currents and timing, and sums event counts.

**Interpretation notes.**
- SPARC is robust to broadband noise (by design). Noise can even make it read slightly
  better, because noise raises mean speed and the normalized spectrum falls below 0.05
  sooner. Rankings therefore treat SPARC ratios below 1 as 1.
- Fast stick-slip, with slips every few tens of ms during fast motion, lies above the
  20 Hz pre-filter and the 10 Hz SPARC cap. It is invisible to SPARC and LDLJ by
  construction, which is why the stick-slip detector and the click counter are separate
  metrics.
- LDLJ is very noise-sensitive (Balasubramanian 2015), so treat it as secondary.

## Thresholds (`services/dex_eval/thresholds.py`)

| Key | Rule | Meaning |
|---|---|---|
| `sparc_ratio` | ≤ 1.15 | measured SPARC within 15% of the trial's own reference SPARC |
| `stick_slip_count` | ≤ 1 | per trial |
| `hold_p2p_max_rad` | < 0.004 | over every 1 s hold window |
| `hold_oscillation_events` | ≤ 0 | |
| `tracking_rms_motion_rad` | < 0.03 | |
| `current_frac_near_cap` | < 0.05 | |
| `temperature_rise_c` | < 5 | |
| `timing_rate_ratio` | ≥ 0.95 | |
| `timing_max_gap_periods` | < 3 | |
| `audio_motion_above_background_db` | ≤ 10 | relative to the trial's own background |
| `audio_excess_clicks_motion` | < 1 | relative to the trial's own hold click rate |
| `arm_sparc_ratio` | ≤ 1.15 | |

The thresholds are self-calibrated at three levels:
1. **Per trial.** SPARC, audio and stick-slip are judged against the trial's own commanded
   reference, background and reference speed.
2. **Per study.** `--calibrate-from <condition>` relaxes each threshold to the
   95th percentile (5th for lower bounds) of an ideal or reference condition, times 1.1.
   Examples are a simulated ideal tracker or a reference replay. This keeps thresholds
   from sitting below the rig's sensing floor. Calibration never tightens a default;
   calibrating from the baseline you want to beat would defeat it.
3. **Overrides.** `--thresholds file.json` takes `{key: value}`,
   `{key: {"op", "value", "description"}}`, or `null` to drop a threshold.

The report flags a condition when the **median** of its trials fails a threshold, and it
shows how many trials pass.

## Report (`scripts/dex_eval_report.py`)

**Inputs** can be runner output directories, `results*.jsonl` files, or any tree of trace
directories. Each trial's trace metrics are cached in `<trial>/metrics.json`, keyed by the
metric configuration and the file stamps; `--recompute` refreshes them. Trials with
unusable traces still count toward success.

**Grouping.**
- Skill mode groups by `meta.skill` (one section per primitive), with conditions as rows.
- Controller mode is selected automatically when every trial is a battery trace (no
  `skill`, or `skill` in `battery`, `wuji_motion_battery`, `motion_battery`). It groups
  by battery item (`params.item`/`motion`, else `variant`, or `--item-key`) with
  `meta.controller` as rows, plus an "all items" section.

**Ranking.** Rows are ordered by:
1. success rate (point estimate; Wilson 95% interval shown);
2. the median rank metric, with a percentile-bootstrap 95% CI over 2000 resamples. The
   metric is `sparc` for skills and `sparc_ratio` for batteries, or `--rank-metric`;
3. median stick-slip count;
4. median motion RMS tracking error.

"Overlaps best" marks rows whose interval overlaps the top row's, meaning the ordering is
not settled. `--baseline <row>` adds Mann–Whitney U tests on SPARC, SPARC ratio and
stick-slip, Holm-adjusted within the group.

**Contents.**
- Tables: ranking, outcomes by status, secondary metric medians, threshold flags.
- Plots: per-condition distributions of the rank metric and SPARC, with the commanded
  reference median and the pass threshold. For each condition, a representative trial
  (the one closest to the median SPARC) shows `q_ref` vs `q_meas` for up to four joints
  (those with the most stick-slip events first). Each joint has a 0.5 s zoom on its
  strongest event, and stick-slip events are marked.
- `summary.json` (machine-readable aggregates) and `trials.csv` (every trial's summary).
- `--ratings <rating session>` adds the naturalness ranking.

## Blind pairwise naturalness ratings (`scripts/dex_eval_rate.py`)

**`build`** pairs videos (`video_<camera>.mp4`) of the same primitive under different
conditions:
- each condition pair gets `--per-pair` comparisons (default 3);
- clips are sampled from different trials, sides are randomized, and the order is shuffled
  with `--seed`;
- clips are exposed only under opaque ids (`clips/<id>.mp4`, symlinked, or copied with
  `--copy`). The un-blinding key is `key.json`; keep it away from raters.

**Collecting ratings.**
- `serve` starts a loopback-only page with the two videos side by side, synchronized
  replay, and A / Tie / B buttons (keys a, t, b). It supports HTTP Range requests so
  videos can be scrubbed.
- `rate --rater NAME [--player mpv]` collects ratings in a terminal.
- Both append to `ratings.jsonl` and skip pairs that rater has already rated.

**`rank`** fits a Bradley–Terry model (Hunter's MM algorithm) per primitive. A tie counts
as half a win for each side, and a 0.5 pseudo-win prior keeps strengths finite. It prints
log-strengths with bootstrap 95% CIs over ratings. The protocol asks for at least 5
raters.

## Low-level battery

`scripts/wuji_motion_battery.py` (written separately) writes ordinary trace directories
with `meta.controller` set to the condition: baseline, streaming, friction compensation,
soft gains or continuous replanning. Point the report at its output directory:

```bash
python scripts/dex_eval_report.py <battery output dir> --group-by controller [--item-key item]
```

## Tests

`local_tests/unit/test_dex_eval_*.py`: synthetic-signal tests for every metric, against
closed-form or known answers:
- Gaussian SPARC arc length, minimum-jerk LDLJ and RMS jerk;
- injected delays, overshoot of a ζ = 0.3 step, injected stick-slip and holding
  oscillation;
- IEC A-weighting points and injected clicks.

They also cover:
- trace validation;
- runner tests with fake skills on the mock rig: interleaving, seeds, failures, the polling
  fallback, the library adapter, interrupts and real-mode guards;
- report generation on skill and battery fixtures;
- rating sessions, including the HTTP server.

The fingertip FK tests run when `mujoco` and wuji-description are installed:

```bash
RAX_WUJI_DESCRIPTION=/path/to/wuji-description python -m pytest -q local_tests/unit -k fingertip
```
