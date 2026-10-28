---
name: live-stream
description: Inspect the configured rolling camera/robot recording before acting after a pause, and review motion or script outcomes. Discover available rates and history from live status and index.json.
---

# Live stream: the world keeps changing

This is the real world. Between your observations people may move or remove objects,
things settle, slide or fall, and seconds to minutes pass while you think or write code.
A frame you looked at earlier describes the past, not the present.

`runtime/stream/` (read-only) holds a configured rolling window. Read
`{{ robot.status }}` → `stream` and `runtime/stream/index.json` for the available
window, camera names, RGB rate and depth rate; do not assume 30 FPS or 10 minutes.
If streaming is unavailable, use fresh observations instead.
The files are:

- `<camera>/<UTC minute>/<epoch_ms>.jpg`: the configured cameras at the configured
  stream rate; inspect `index.json` for their names and reported rates.
  `<epoch_ms>_depth.png` at the configured depth rate (uint16 millimetres, 0 = invalid).
- `robot/<UTC minute>.jsonl`: TCP pose, arm joints and end-effector feedback at the
  configured robot logging rate (typically 10 Hz); use record timestamps for timing.
{% if robot.has_hand %}
  `hand` contains measured/target finger joints, measured and commanded effort in A,
  active and baseline gains/current caps, diagnostics, source
  timestamps and freshness. These are sampled alongside the arm state, not exactly
  synchronized with each camera exposure. Missing or stale state is not a zero pose.
{% else %}
  The scalar `gripper_width` is in metres.
{% endif %}
- `index.json`: retention/rate settings and per-camera stream health/timestamps.

Older data is deleted as time goes by; copy what you need to keep into `output/`.

## Two ways in

- **Directly, as a tool:** `{{ robot.stream }} {seconds_ago: [0, 30, 120]}` returns the frames from
  those moments (all cameras, or `cameras: [...]`), labelled with their age, plus the arm
  state then; `{{ robot.stream }} {sheet: {seconds: 120, count: 12}}` returns one contact
  sheet per camera over the last two minutes, if that much history is available.
  Shorten requested ages/windows to the session's retained history. No motion, no arming needed.
- **As files,** for code and bulk analysis: `python -m api.stream ...` below, or read the
  images under `runtime/stream/` directly.

## When to look

- **Before any motion that follows a pause**, check that the scene still matches what
  you planned against: `{{ robot.stream }}` with `seconds_ago: [0]` (or call
  `{{ robot.observe }}`, which also saves depth for localization). If something
  moved, re-localize before moving; never act on stale positions.
- **What changed while you were away**: `{{ robot.stream }}` with a `sheet` over the time you
  were away, or `python -m api.stream changes {{ robot.overview_camera }} --seconds 30` to list the moments the
  image changed and then `{{ robot.stream }} {seconds_ago: [...]}` at those moments.
  Compare with `python -m api.stream robot --seconds 120`: changes while the arm was
  still can come from a person, object motion or lighting.
{% if robot.has_hand %}
  A stationary arm does not imply a stationary hand: inspect the finger state and
  current commands too, including persistent feed-forward effort.
{% endif %}
- **After a motion or a script run**, review the frames during it (grasp slip, a
  knocked object, the moment a stack toppled) instead of guessing from the end state.
- In controllers, use `from api import stream` to check that the scene is quiet before
  committing to a grasp, and to keep evidence frames of each step.

{% if camera_geometry %}
Metric geometry comes from `{{ robot.observe }}` frames and `{{ robot.pixel }}`
([object-localization](../object-localization/SKILL.md)).
{% else %}
Metric geometry requires the calibrated RGB methods in
[object-localization](../object-localization/SKILL.md); depth lookup is unavailable.
{% endif %}
The stream is for timing, change and review. Look at a few selected frames or sheets,
never whole minutes of images.

{% if robot.has_hand %}
When enabled, mutating arm/hand tools return a `motion_clip` reference and the
operator retains the clip under the session's `motion-archive/`, including configured
pre/post padding. This is distinct from the rolling stream and from optional
continuous full-session archiving; a saved motion clip does not mean all idle time
is permanently archived. The private archive is operator-owned. Save accessible
evidence frames in `output/` and record returned clip references in memory.
{% endif %}
