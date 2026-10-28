---
name: live-stream
description: Review the right Piper session's RGB evidence and robot history.
---

# Piper RGB history

`piper_observe {}` saves a fresh overview RGB image under `runtime/frames/`.
`piper_stream {seconds_ago: [0, 2, 10]}` reviews selected recorded images and
nearby arm state. Missing frames are explicitly unavailable, never synthesized.
The default recorder retains ten minutes at up to 5 Hz, not the Franka 30 Hz
three-camera stream. There is no depth or calibrated projection.

The read-only recording is `runtime/stream/overview/<UTC minute>/<epoch_ms>.jpg`;
`runtime/stream/robot/<UTC minute>.jsonl` contains model-based arm poses and
the last hand feedback (which may predate the camera image). Camera timestamps
are host receive times, not synchronized hardware exposure times.

For bulk review use `python -m api.stream sheet overview --seconds 30 --count 8`
or `python -m api.stream robot --seconds 30`. Use `python -m api.stream latest`
to list the latest images. Refresh with
`piper_observe` before acting; never command from stale recording frames.
