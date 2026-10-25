# Regression tests

Run from the repository root in the robot Python environment:

```bash
$RAX_PYTHON -m pytest -q
```

The suite uses temporary artifact roots, the `mock` robot backend, simulated bridges and
fake model providers. It never moves hardware, opens cameras, calls a model or needs
credentials. Tests marked `hardware` are excluded by default; the archive tests skip
without `ffmpeg`.

The Wuji tests also use fake drivers behind the real private process/socket boundary
and MCP frontend. They cover joint feedback mapping, trajectory bounds, cancellation,
watchdogs, stop confirmation, workspace skill selection and G2 compatibility. No
test imports a real device connection or starts a vendor motion demo. Local socket
tests need a host environment that permits loopback and Unix sockets.
