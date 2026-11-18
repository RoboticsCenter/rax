# Wuji Hand 2 smooth control layer

This layer lets dexterous skills (code-as-policy, MPC, RL) command the Wuji hands smoothly
instead of through blocking, rest-to-rest moves. It addresses the diagnosis in
`research_notes/Dexterous skill policy representations/low_level_smooth_hand_control.md`:

| Likely cause (ranked) | What this layer does |
|---|---|
| 1. Stick-slip under plain PD, no friction compensation | Optional `tanh` friction feed-forward on the reference velocity, bounded and counted toward the cap |
| 2. Stop-go from blocking rest-to-rest calls with 0.4 s settle waits | Non-blocking stream targets and timed trajectories that never stop between targets; settling is checked only when asked (`wuji_wait`) |
| 3. Holding twitch from stiff gains and noisy velocity | Impedance presets that account for the ~1.98x firmware gain scale, and a host hold deadband |
| 4. Cap saturation at about 0.05 rad lag | Per-stage caps within the operator ceiling; status reports the error at which the cap saturates |
| 5. Unfiltered staircase targets and timing jitter | Interpolation and jerk-limited tracking at the writer rate, cheaper ticks, GIL hand-off tuning, optional low-pass filter |
| 6. Faulted left-hand joints | Disabled joints never move, never receive feed-forward, and keep zero gains in every profile |

Nothing here changes the SDK or firmware versions, and nothing was run on hardware while
building it. Without the new tools, the existing seven tools behave as before; an idle
stream sends exactly the previous hold command.

## Architecture

```
skill / agent code                      robot service (frontend + backend)        hand owner process (one per hand)
ctx.hand('right').stream(q)  ── MCP ──►  wuji_stream ── dry-run check ──► IPC ──►  socket thread: validate, queue (epoch)
ctx.hand(...).trajectory(...)                                                      writer thread, every tick:
trace_start / trace_stop                                                             goal g(t) ─► [lookahead] ─► [low-pass]
                                                                                     ─► online Ruckig (v/a/j, rest target)
                                                                                     ─► joint-limit clamp ─► q_ref, dq_ref
                                                                                     + hold-deadband offset (position)
                                                                                     + friction feed-forward (current)
                                                                                     ─► guards ─► SDK publish ─► trace row
```

* **Goal.** Each target starts a segment from the current goal value. Without `duration_s` it
  is a linear ramp over 1.5x the observed target period (10–150 ms; 50 ms for a one-off
  target), so a 10–100 Hz stream keeps a constant goal velocity even with arrival jitter.
  With `duration_s` it is a minimum-jerk ramp. A timed trajectory is a monotone cubic
  (PCHIP) spline through the waypoints. It starts with the current goal's velocity and
  ends at rest, and it never overshoots between waypoints. Time is stretched uniformly if
  a waypoint pair needs more than the velocity or acceleration limits.
* **Tracker.** Online Ruckig (the pinned 0.19.4) replans every tick toward the goal with zero
  target velocity. When the goal stops, the reference comes to rest at the last target
  without overshooting it. It never stops between targets while targets keep arriving.
* **Lookahead.** Rest-target tracking trails a moving goal by its braking distance. Because
  the goal is a known function of time, the tracker aims at the goal half a braking time
  ahead, which cancels the steady lag. It never aims beyond the commanded target.
* **Synchronization.** By default each finger is one time-synchronized Ruckig group, and the
  fingers are independent, so staggered finger timing is preserved. `hand` makes all 20
  joints one group; `joint` makes every joint independent.
* **Reference time base.** The writer advances the tracker by the elapsed wall time, at most
  5 periods per tick. Small scheduling delays do not slow the reference (risk 8 in the
  diagnosis); longer stalls stretch it rather than jump.
* **Ownership.** The stream runs only while the hand is `holding` with no blocking job.
  Blocking moves, `set_effort`, `set_impedance` and `set_profile` are refused while the
  stream reference moves. Once it is at rest, they start from the exact stream state.
  A blocking move's final target becomes the stream's new target.

## API

### MCP tools (all sessions with a Wuji hand)

The seven existing tools are unchanged. Bimanual sessions add `hand="left"|"right"` to each
new single-hand tool.

| Tool | Blocking? | Mutating? | What it does |
|---|---|---|---|
| `wuji_stream` | no | yes | Set the latest target: `positions` (20), `joints` {name: rad} or `fingers` {finger: [S1..S4]}; `relative`; optional `duration_s`/`shape`. Omitted joints keep their targets. |
| `wuji_trajectory` | `wait` (default true) | yes | Timed waypoints `[{t, positions\|joints\|fingers}]` as one smooth motion. A new call continues from the moving goal (action chunks). `record` saves the call as one trace trial. |
| `wuji_set_profile` | short (SDK readback) | yes | Preset `profile\|soft\|default\|firm\|hold`, optional `kp`, `kd`, `current_limit_a` (number or 20), `hold_deadband_rad`. Needs the reference at rest and a measured stationary hand. |
| `wuji_stream_config` | no | no | Software settings (table below); applied at the next rest. `reset=true` restores the defaults. |
| `wuji_wait` | yes | no | Until all accepted targets are applied, the reference has converged, and measured joints settle within `tolerance_rad` for `settle_s` (or `timeout_s`). |
| `wuji_stream_both` | no | yes | Bimanual: both hands' `wuji_stream` payloads in one call. Both are validated before either is accepted. |
| `trace_start` | no | no | Start a trace trial: `directory` (used only inside the agent workspace), `label`, `sides`, `controller`, `skill`, `variant`, `params`, `notes`, `max_seconds`. |
| `trace_stop` | until saved | no | Write `hand_<side>.npz` and `meta.json` (`success`, `notes`); returns `directory`. |

Rejected arguments (limits, disabled joints, max step, unknown names) are validation errors
with `no_action_executed: true`, and nothing stops. Transport or owner faults stop the
assembly, exactly as for the old tools. A `wuji_hold` during streaming brakes the stream to
rest with its own jerk and acceleration limits and drops queued targets. Disarm, stop,
faults, the tracking-error, feedback-age and scheduler watchdogs, and the temperature limit
all behave as before. After a stop, streaming needs a new preparation.

### Python: `ctx` in agent controllers and skills (`api/hand.py`)

```python
from api.runtime import Context
from api.hand import Rate

def main(ctx, **params):                                   # mooncake skill entry point
    hand = ctx.hand('right')                               # single-hand sessions: ctx.hand()
    hand.configure(friction=True, filter_tau_s=0.03)       # wuji_stream_config
    hand.profile('default', current_limit_a=0.6)           # preset + per-stage cap
    with hand.recording(controller='M2_stream_friction_ff', skill='pinch', params=params) as trial:
        rate = Rate(30)
        for q in policy():                                 # 20 rad, or hand.stream(index=[...]), joints={...}
            hand.stream(q)                                 # returns immediately
            rate.sleep()
        hand.trajectory([(0.5, q1), (1.0, q2), (1.6, q3)]) # one smooth motion; waits by default
        result = hand.wait()
        trial.success = result['settled']
    return {'ok': result['settled'], 'measured': {'trace': trial.path, **result}, 'why': 'pinch closed'}
```

Function API for skill libraries (for example `dexlib.io.bind_streaming`):

| Need | Exact call |
|---|---|
| (a) stream a 20-joint target, non-blocking | `api.hand.stream(ctx, side, q20, **options)` (options: `relative`, `duration_s`, `shape`) |
| (b) follow a timed waypoint trajectory | `api.hand.follow(ctx, side, times, positions, wait=True, timeout_s=None)` |
| (c) impedance profile and current cap | `api.hand.profile(ctx, side, 'default', current_limit_a=0.6)` |
| (d) is streaming available? | `api.hand.available(ctx, tools=None)` |
| (e) trace from inside a skill | `with api.hand.record(ctx, sides, directory, controller=..., skill=...) as started: ...` |

`side` is `'left'` or `'right'`; it is ignored in single-hand sessions. Binding:
`io.bind_streaming(follow=hand.follow, stream=hand.stream, record=hand.record, available=hand.available)`.

### Owner IPC (operator scripts, battery)

Direct, non-SDK operations on the hand owner's private socket: `stream`, `trajectory`,
`stream_config`, `record` (`start` with an absolute `directory`, `stop`, `status`) and
`stream_status`. All of them accept `dry_run` validation where it applies. `set_profile` is a
normal job (`operation(root, 'set_profile', {...})`). The helpers in
`services/wuji/client.py` are `direct`, `wait_settled` and `record_stop`.

## Parameters and defaults

### New profile keys (operator configuration, `end_effector`)

| Key | Default | Bounds | Meaning |
|---|---|---|---|
| `stream_max_acceleration_rad_s2` | `null` = `max_acceleration_rad_s2` | 0.01–25 | Stream tracker acceleration ceiling |
| `stream_max_jerk_rad_s3` | `null` = `max_jerk_rad_s3` | 0.01–100 | Stream tracker jerk ceiling |
| `friction_current_a` | 0.08 | 0–0.3 per joint | Default friction magnitude (commanded A) |
| `friction_velocity_rad_s` | 0.03 | 0.001–1 | `tanh` velocity scale v0 |
| `max_feedforward_fraction` | 0.5 | 0.05–1 | Bias plus friction never exceeds this fraction of the active cap |
| `gain_scale` | 1.98 | 0.5–4 | Telemetry-fitted firmware gain scale used by presets and reports |
| `max_record_s` | 120 | 1–600 | Longest trace recording (memory: about 1.9 MB/s at 1 kHz per hand) |

Velocity is always bounded by the existing `max_velocity_rad_s`, which the driver enforces.
`command_hz` is the writer's maximum rate.

### Runtime stream settings (`wuji_stream_config`; preparation restores them)

| Setting | Default | Meaning |
|---|---|---|
| `interpolation_s` | `null` (auto) | Ramp per target: 1.5x the observed period, clamped 10–150 ms; 50 ms when one-off; 0 = jump |
| `filter_tau_s` | 0 | First-order low-pass on the goal (EMA); adds about tau of lag |
| `lookahead` | true | Aim half a braking time ahead along the known goal |
| `synchronization` | `finger` | `finger`, `hand` or `joint` Ruckig groups |
| `max_velocity_rad_s` / `max_acceleration_rad_s2` / `max_jerk_rad_s3` | profile ceilings | Per joint (number or 20); cannot exceed the ceilings |
| `friction` | false | Friction feed-forward on or off |
| `friction_current_a`, `friction_velocity_rad_s` | profile defaults | Per joint, at most 0.3 A |
| `hold_deadband_rad` | 0 | Hold deadband (0–0.02); the `hold` preset sets 0.004 |
| `hold_delay_s` | 0.2 | Rest time before the deadband blends in (0.2 s blend in, 0.05 s blend out) |
| `command_hz` | profile `command_hz` | Writer rate, at most the profile value; S-curve moves use it too |

### Feed-forward and hold details

* Friction: `I_ff = F * tanh(dq_ref / v0)` on active joints only. It is zero at rest. It is
  withheld on a joint measured above 1.5x `max_velocity_rad_s`. It is clamped so that
  `|bias + I_ff| <= max(max_feedforward_fraction * cap, |bias|)`. Telemetry shows that
  commanded feed-forward is also applied at about 1.99x (parameter-sweep doc), so 0.08 A
  commanded reads about 0.16 A.
* Hold deadband: once the reference has rested for `hold_delay_s`, the command becomes
  `q_ref + clip(q_meas_filtered - q_ref, +/-band)`, with a 20 ms position filter. Inside
  the band the P term is about zero, which removes twitching against friction and
  backlash. Outside it the spring acts from the band edge. An encoder glitch can move the
  command by at most the band.

### Impedance presets

Presets are effective stiffness and damping divided by `gain_scale`, then clamped to the
operator ceilings (`kp`, `kd`) per joint:

| Preset | Effective K (A/rad) / D (A·s/rad) | Nominal kp / kd at 1.98 | Notes |
|---|---|---|---|
| `soft` | 6 / 0.12 | 3.03 / 0.061 | Close to the vendor gentle profile (3/0.05) |
| `default` | 10 / 0.20 | 5.05 / 0.101 | Vendor default (5/0.1); M3 in the research plan |
| `firm` | 16 / 0.30 | 8.08 / 0.152, so 8 / 0.15 on the current profiles | Current production |
| `hold` | 8 / 0.12 | 4.04 / 0.061 | Plus `hold_deadband_rad` 0.004 |
| `profile` | the operator values | | |

`wuji_status.smooth_profile` reports nominal and effective gains and
`cap_saturation_error_rad = cap / (kp * gain_scale)`. For example, 0.8 A at kp 8 saturates
at 0.05 rad, and 0.8 A at kp 5.05 at 0.08 rad.

### Fixed constants

The IPC frame is 1 MiB (trajectories of up to 256 waypoints). The GIL switch interval in the
hand process is 0.5 ms (the Python default is 5 ms). The stream gap that makes the next
target a one-off is 0.25 s.

## Trace format and recording

One directory per trial (`services/wuji/trace.py`; consumed by `services/dex_eval`):

* `hand_<side>.npz`, with one row per command published at the writer rate:
  * the contract arrays: `t` (T,) float64 monotonic s, plus `q_ref` (as sent, deadband
    included), `q_target` (latest caller target, or a blocking move's goal), `q_meas`,
    `dq_meas`, `current` (joint_states effort, A), `i_ff` (bias plus friction sent), `kp`
    and `kd`, each (T, 20) float64;
  * extras: `dq_ref`, `q_goal` (the interpolated goal), `current_diag`, `temperature_c`
    (MCU, °C) and `current_limit_a`, each (T, 20) float32; and `state_seq` and
    `state_time_us`, each (T,) int64, for de-duplicating native frames;
  * disabled joints are NaN in the measured arrays.
* `meta.json` holds the shared keys (`sides`, `joint_names`, `nominal_rate_hz`, `controller`,
  `skill`, `variant`, `params`, `success`, `started_at`, `backend`, `notes`). It also holds
  `current_limit_a` (a scalar, a 20-list, or `{side: ...}`), `trace_format`, and `hands`
  (serial, firmware, profile revision, active profile, stream settings, command timing).
  `backend` is `mock` for the test fakes, `sim` when the SDK module sets
  `RAX_BACKEND_KIND='sim'`, and `real` otherwise.

Ways to record:
* per session: `trace_start` / `trace_stop`, or `hand.recording()`;
* per call: `wuji_trajectory(record={...})`;
* in the battery: automatic, one trial per run.

When `trace_start` receives a directory outside the agent workspace, it writes to
`runtime/traces/<label>` and reports the actual `directory`. The harness copies the files
from there. A workspace path is resolved first, so a planted symlink cannot redirect host
writes, and symlinked trial, traces and runtime directories are refused.

## Command rate and host timing

* The writer was spending most of each tick re-reading the arming file, up to three JSON
  reads per tick. `arming.CachedReader` re-parses the file only when its inode, mtime or
  size changes. It evaluates expiry on every call and re-reads at least every 0.1 s.
  With the fake driver, holding costs about 40 µs per tick, down from 160 µs. A whole-hand
  30 Hz stream with friction costs about 75 µs per tick. Ruckig with a moving goal costs
  about 15–55 µs for 20 joints.
* `sys.setswitchinterval(0.5 ms)` in the hand process bounds how long IPC, snapshot or SDK
  callback threads can hold the writer off.
* The examples still request 200 Hz. In November, 1 kHz on two hands reached only about
  340 Hz under the old tick cost. Raise `command_hz` per hand (500 first, then 1000) only
  after checking `command_timing.observed_hz` and `max_interval_s`, and the trace's
  achieved rate. `stream_config(command_hz=...)` can lower the rate at runtime, which the
  battery uses for M0.
* MCP overhead is about 3.3 ms per call on NVMe, mostly audit fsyncs, so stream at
  10–50 Hz per hand. Use `wuji_stream_both` for two hands, and `wuji_trajectory` chunks
  for denser motion.
* The pinned Hand 2 SDK 2026.10.12 has no real-time controller; the publisher is the only
  command path. Moving the writer out of the GIL-shared process, or onto the C API, is the
  next step if traces still show gaps.

## Motion battery (`scripts/wuji_motion_battery.py`)

The battery runs Phase A of the research protocol on one or both hands. It writes one trace
trial directory per run, and `meta.controller` is the condition name.

| Condition (`meta.controller`) | Delivery | Gains / cap | Friction | Rate |
|---|---|---|---|---|
| `M0_baseline` | blocking rest-to-rest S-curve per waypoint | operator profile | off | 200 Hz |
| `M1_stream_1khz` | one stream target per waypoint (minimum-jerk, no settle dwell) | profile | off | profile `command_hz` |
| `M2_stream_friction_ff` | as M1 | profile | on | profile |
| `M3_soft_gains_cap` | as M1 | `default` (~5/0.1), cap 1.0 A clamped to the ceiling, thermal guard | on | profile |
| `M4_online_replan` | 30 Hz stream of the waypoint spline (online replanning, no stops) | as M3 | on | profile |

Optional extras are `M4t_trajectory` (one trajectory call), `M2s_soft_preset` and
`M2h_hold_preset`.

Items:
* single-joint moves on thumb S2, index S1/S3 and ring S1, ±0.05 and ±0.4 rad, out and back;
* a 0.03 rad/s stick-slip ramp over 0.2 rad, plus a 0.2 Hz, 0.1 rad sinusoid (always a
  continuous reference);
* open → fist → open and open → pinch → open, using commissioned or `--postures` targets;
* finger tapping: thumb opposition with `tap_*` postures, or else sequential S1 flexion of
  index to pinky;
* 30 s holds at home and at the fist;
* five waypoints. M0 runs them as separate calls, M4 as one stream, which gives the A6
  comparison.

Speeds are `--speeds` (protocol: `0.1,0.3,0.6`), clipped to 0.9x each hand's
`max_velocity_rad_s`.

```bash
# 1. Plan only (default): validates every waypoint against each hand profile; touches nothing.
python scripts/wuji_motion_battery.py --session <session dir> [--hands right] [--speeds 0.1,0.3,0.6]
# 2. Hardware: RAX must not be running. The battery copies the session, starts only the hand
#    owners (arms are never connected), waits for the operator to arm the copy with
#    scripts/arm_robot.py <printed root> arm, asks for READY, prepares, runs, then disarms and stops.
python scripts/wuji_motion_battery.py --session <dir> --execute [--hand-patch patch.json] [--shuffle --seed 1]
```

Between trials the battery waits until the hottest active joint is below `--start-temp-c`
(50 °C). In M3/M4 a thermal guard requests an emergency stop above `--thermal-guard-c`
(58 °C), below the profile's own stop. Any error stops the battery and the hands.

`--hand-patch` (for example `{"all": {"command_hz": 1000, "stream_max_jerk_rad_s3": 30}}`)
changes only the battery's own copy of the session. It is limited to rate, gain and cap
ceilings and stream limits, and it is validated by the normal profile rules. Raising
`current_limit_a` above 0.8 A is an operator decision: heating was recorded at 0.8 A in
November. `--attach` uses running hand owners instead, and cannot patch them.

## Tuning on hardware (operator order)

1. **Dry run.** Run the battery dry run and review the plan, the excursions and the notes.
   On the current profiles it reports a 200 Hz rate, a 0.8 A ceiling, and S-curve jerk
   limits for streams.
2. **Idle check.** With the hand prepared and in free space, call `wuji_status` and confirm
   that `stream.ready` is true and `command_timing` is unchanged.
3. **Smoke test.** Run a small-amplitude streaming test, at least 0.02 rad on one joint.
   Use `wuji_stream(joints={...}, duration_s=1)`, then `wuji_wait`. Check the trace for
   q_ref against q_meas.
4. **Stream limits.** The current jerk limits (2 and 0.5 rad/s³) make streams lag by 0.1 rad
   or more. For the right hand, try `stream_max_acceleration_rad_s2` 4–6 and
   `stream_max_jerk_rad_s3` 20–40. Use about half that for the partial left hand. Keep
   `max_velocity_rad_s` unchanged.
5. **Friction.** Start at `friction_current_a` 0.04 A and raise it toward 0.08 A on the
   stick-slip ramp. Stop when bursts in that item disappear or the joint leads its
   reference. Stay below breakaway, which is about 0.075 A reported (0.04 A commanded at
   1.98x). Tune S1 joints separately (per-joint list).
6. **Gains.** Compare `firm`, `default` and `soft` on single-joint items and on holds. Use
   `hold` (deadband 0.004) for long holds and check that the 30 s hold item no longer
   twitches. Watch `cap_saturation_error_rad` against observed tracking errors.
7. **Rate.** Raise `command_hz` to 500, then 1000, and compare achieved rate and gaps (the
   trace `t` and `command_timing`), and acoustic data, against 200 Hz.
8. **Full battery.** Run the full battery with `--shuffle`, at least 10 repeats per
   condition per the protocol. Report the left hand separately.

## Firmware 3.0.0 (not installed; do not upgrade without re-qualification)

Firmware 3.0.0 with SDK 2026.11.9 adds a beta Position mode. It applies only `position`,
with `effort` limits in N·m. It has a whole-hand position-command low-pass filter
(`hand.position_command_low_pass_cutoff_frequency()`, default 40 Hz; ≥1000 bypasses it) with
derived velocity and acceleration feed-forward. It also adds 500 µs overcurrent filtering,
a `SoftLimitHit` warning, and thermal derating above 85 °C.

The filter would remove staircase ripple in the firmware, at the cost of MIT compliance.
That would make a research condition M5. Upgrading invalidates RAX's firmware-keyed fault
exceptions (`isolated_joint_faults`, `allow_init_current_calib_failed`), and both hands
would need full re-qualification. This layer works in MIT mode on firmware 2.13.1.

## SDK boundary (for the simulator)

The module docstring of `services/wuji/sdk.py` lists every SDK call. The smooth layer adds no
new SDK calls:

* streaming uses the existing `joint_command().publish().send()` at the writer rate;
* friction is the `JointCommand` effort field;
* presets use `effort_limit()` and `mit_params()` set and get, with readback, through
  `WujiDriver.set_control_parameters`. That method also supports partial hands, with
  disabled joints held at zero.

A simulated SDK module may set `RAX_BACKEND_KIND = 'sim'` so that traces say `backend: sim`.

## Tests

The tests are hardware-free:
* `test_wuji_smooth.py`: engine math with real Ruckig;
* `test_wuji_stream_controller.py`: the controller with fake hands, including partial hands,
  hold, disarm, the tracking guard, profiles and traces;
* `test_wuji_stream_service.py`: the real owner process and IPC, plus a battery run
  validated by `services.dex_eval.trace.validate_trial_dir`;
* `test_wuji_stream_mcp.py`: MCP tools, trace tools and the ctx helper over the real
  socket, for single-hand and bimanual sessions;
* `test_wuji_motion_battery.py`: dry-run planning.

## Open risks

* No part of this has moved a real hand. Real friction, encoder noise, the 1.98x scale and
  host timing may need different defaults.
* The stream limits come from the profile. Until the operator sets the stream jerk and
  acceleration ceilings, streams are smooth but sluggish.
* Feed-forward can push a joint ahead of its reference if the friction magnitude exceeds
  breakaway. It is bounded (at most 0.3 A, at most half the cap) and withheld above 1.5x
  the velocity ceiling, but not verified on hardware.
* The hold deadband makes the command follow filtered measurements by up to 4 mrad. Under
  heavy encoder glitches this is bounded but untested.
* MCP audit fsyncs limit stream rates through the agent path.
* The writer remains Python in a GIL-shared process, so 1 kHz on two hands under camera load
  is unverified.
