# xArm + Wuji Hand 2

The `real/xarmwuji` branch retains the xArm native motion planner, camera service,
recordings and session MCP. Select `end_effector.type: "wuji_hand2"` to replace the
G2 tool with twenty-joint hand control. Existing G2 configurations continue to work.

The implementation has hardware-free unit and process tests and an initial
[real-hand MCP test record](wuji-live-test-2026-10-25.md). Basic joint motion,
target holding, a recorded posture and confirmed disable were exercised. Full
manipulation and fault qualification remain pending. The local October 22 vendor
self-check is separate historical evidence. The research and design rationale are in
[xarm-wuji-backend-plan.md](xarm-wuji-backend-plan.md).

## Environment and configuration

Use a Linux Python environment containing the existing xArm/RealSense dependencies
(`xarm-python-sdk`, `pyrealsense2`, `numpy`, `scipy`, `opencv-python` or its headless
variant, `pyyaml`, `jinja2`, `pillow`). Install the pinned hand SDK into that same
environment:

```bash
export RAX_PYTHON=/path/to/robot-env/bin/python
"$RAX_PYTHON" -m pip install -r requirements-wuji.txt
```

Python 3.12 with `wuji-sdk==2026.10.12` matches the successful local hand deployment.
The backend checks the SDK version at connection. It does not import or connect
the SDK when generating workspaces or validating configuration.

On this host, `/home/jerry/rax/.venv/bin/python` is prepared with that SDK,
`xarm-python-sdk==1.18.4`, `pyrealsense2==2.59.1.11053` and the test dependencies.
Use it as `RAX_PYTHON` for the commands below. Installing these packages did not
connect to either device.

Copy [examples/xarm-wuji.json](../examples/xarm-wuji.json) into your operator
configuration location and review it before running:

- The example retains RAX's existing arm IP and camera calibration paths. Verify
  the current arm address and the physical camera configuration.
- It selects right hand `WH2KA01260823017` at `192.168.1.111:7447`. Connection verifies
  serial and side even when the endpoint is supplied. Set `address` to null to
  discover by serial. The port matches the recorded firmware 2.6.0 deployment;
  do not replace it with a newer SDK example's default without checking the device.
- Its joint bounds follow the published Hand 2 SDK table. They are a commissioning
  profile, not a measured SDK-to-model mapping or self-collision model. Check actual
  joint directions, origin, mounting orientation, TCP, payload and swept clearance.
- The example applies 0.5 A, kp 1.5, kd 0.1 only on explicit `prepare`. These values
  need real-world qualification. Current is not fingertip force. Gains/current
  limits are read back for every joint before enabling; partial failure refuses
  preparation and requests stop. It does not silently restore unknown old settings.
- `postures` is initially empty: there is no guessed "open" or "grasp" posture.
  Add named twenty-angle arrays after commissioning them, and change
  `profile_revision` when changing limits, gains, origin conventions or postures.
- The optional `lock_paths` entry coordinates with the local teleop checkout.
  Remove or update it on another host. Parent directories must exist. Also stop
  vendor demos/Studio and the separate arm-hand adapter before RAX control:
  advisory locks cannot exclude programs that ignore them.

The hand uses its own Ethernet/power connection. The example does not change
network routes, firmware, calibration, xArm TCP, collision settings or payload.
The wrist camera's current extrinsics are TCP-relative; a changed TCP needs an
updated transform even if the camera itself stayed fixed. Fixed camera transforms
remain valid only if the base and cameras stayed fixed.

## Start a session for MCP testing

Choose the artifact root as in the main README, then configure a session:

```bash
export RAX_BACKEND_OPTIONS=/absolute/path/to/your/xarm-wuji.json
"$RAX_PYTHON" scripts/configure_session.py \
  --robot xarm --task wuji_commission \
  --task-text 'Operator-led Wuji MCP testing' \
  --backend-options "$RAX_BACKEND_OPTIONS"
```

The command prints an `operator-input.json` path. To test the MCP directly, without
starting an agent, run the frontend in a terminal using that path:

```bash
export CONFIG=/absolute/path/printed/by/configure_session/operator-input.json
"$RAX_PYTHON" -m services.controller.frontend \
  --config "$CONFIG" --socket /tmp/rax-wuji-mcp-test.sock
```

The frontend starts a private hand process. Connecting reads identity and feedback;
it does not enable motors, home fingers or clear faults. A connection failure is
reported by `wuji_status`, and hand-dependent motion is refused. The log is
`<session>/wuji-controller.log`. With streaming enabled, frontend startup also
starts the existing camera service. `stream_enabled: false` is useful for initial
hand-only checks; camera observations still connect cameras when requested.

Alternatively launch the normal agent using `scripts/start_agent.sh --config ...`.
Workspace generation automatically selects the Wuji tool contract and hand-control
skill. Do not start both frontends for the same session/device.

From another terminal at the repository root, inspect the tools and state:

```python
from auto_research_agent.api.runtime import Context

with Context('/tmp/rax-wuji-mcp-test.sock') as ctx:
    print(ctx.request('tools/list', {}))
    print(ctx.value('wuji_status'))
    print(ctx.value('xarm_status'))
```

These reads do not prepare or move the hand. For real movement, after checking
clearance and the stop setup, the operator explicitly readies the arm, arms the
session, and prepares the hand:

```bash
scripts/xarm ready
scripts/xarm arm 10
scripts/wuji prepare --session /absolute/path/to/session
scripts/wuji status --session /absolute/path/to/session
```

`scripts/xarm` targets the newest session and the configured backend options;
verify the printed session name. `scripts/wuji --session` always targets an explicit
existing hand owner. Preparation requires fresh fault-free feedback with all joints
disabled, applies/verifies the profile, seeds measured positions, enables, and waits
for all twenty joints to report Enabled. It does not open or zero the hand.

Arming expires after the requested duration. The separate process checks arming
and parent liveness while the agent is thinking or waiting for an operator.

## MCP contract

| Tool | Arguments | Result/behavior |
| --- | --- | --- |
| `wuji_status` | none | Measured/target positions, rad/s velocities, effort in A, diagnostics, independent stream ages, profile, limits, posture names, mode and current action. |
| `wuji_move_joints` | `positions`: 20 finite radians; optional `duration_s` | Smooth bounded move and measured settling. Holds the final target afterward. |
| `wuji_pose` | `name`; optional `duration_s` | Same motion path using an operator-configured posture. Unknown names refuse before dispatch. |
| `wuji_hold` | none | Retain the current bounded commanded posture when dispatched and healthy. Falls back to emergency stop when disarmed/unreliable. |
| `xarm_hold` | none | Independently request arm hold and hand emergency stop; report both outcomes. |

Joint order is thumb S1–S4, index S1–S4, middle S1–S4, ring S1–S4, pinky S1–S4.
There is no scalar width or `is_grasped` field for the hand. Wuji sessions do not
publish `xarm_gripper`. The arm tools retain their existing arguments, configured
TCP and quaternion order `[x,y,z,w]`.

Commands enforce configured angle limits, maximum step, velocity, acceleration,
tracking deviation and deadlines. An omitted duration is computed from the bounds;
an explicit duration that is too short is rejected. State and diagnostic streams
must both remain fresh. Missing or duplicated NIDs, nonfinite values, stale
sequences, reported faults and an unexpected enabled state prevent continued
execution. The hand keeps its target during arm motion; calls are sequential,
not a synchronized arm/hand trajectory.

During your tool tests, check invalid shapes/limits, disabled and disarmed admission,
small single-joint moves in clear space, posture execution, holding between calls,
fault/status reporting, and the two independent stop paths. Fresh observed
convergence is not proof of grasping; validate retention and release visually.

## Stop and recover

```bash
scripts/wuji stop --session /absolute/path/to/session
scripts/wuji disable --session /absolute/path/to/session
scripts/xarm disarm
```

The operator hand commands reach the private hand service directly, bypassing a
long-running MCP dispatcher call. `scripts/xarm disarm` changes the arming files,
queues hand emergency stops and requests arm hold. Dashboard Disarm/Disarm All also
queue hand stops. Dashboard hand state comes from the owner's timestamped snapshot,
without opening another SDK connection.

All MCP calls are serialized, including `wuji_status`, `wuji_hold` and `xarm_hold`.
A hold call from a second MCP client therefore waits behind an active motion
call; it is not a preemptive stop interface. Use the operator's direct stop or
disarm path for interruption. The live concurrency test measured a 2.28 s wait
behind a 2 s stationary motion command; idle disarm independently disabled the
hand in about 42 ms in that test, not a guaranteed worst-case latency.

An emergency/disable result separates `sdk_returned` from `confirmed`. Confirmation
requires new complete feedback, disabled/stopped joint states and low measured
velocity. On the tested firmware 2.6.0 hand, the SDK emergency-stop call left joints
Enabled. The RAX emergency path therefore requests SDK emergency stop followed by
explicit disable, including an attempt to disable if the emergency RPC raises.
A normal `wuji_hold` reports its stationary command with
`confirmed: false`; it does not claim that the hand has physically settled.
If stop delivery or confirmation fails, the service reports uncertainty and does
not resume. A late native SDK return cannot resume an old trajectory.

Support held objects before disarming, disabling or ending a session: grasp
retention is not guaranteed. Software cannot deliver a stop over a failed link or
guarantee a deadline while native code is blocked. Hardware stop/watchdog behavior
and the independent hand supply require the planned physical tests.

After a resolved connection problem, `scripts/wuji reconnect --session ...` explicitly
reconnects without enabling, provided this owner has no unconfirmed active control.
Read state and perform explicit preparation again. Firmware updates, origin changes
and fault clearing remain outside the agent interface. If a stale service socket
remains after a crash, verify its process has exited before removing that socket;
the device-level lease prevents a second service from acquiring the same hand.

## Validation and recording

Run the hardware-free suite in an environment with pytest and the base dependencies:

```bash
"$RAX_PYTHON" -m pytest -q
```

Wuji tests inject fake drivers, including in the actual child process and Unix
socket/MCP tests. They cover malformed/missing feedback, arming and heartbeat loss,
delayed native calls, command cancellation, stop failure, target retention,
workspace generation and G2 compatibility. No SDK demo or motion test runs
automatically. Some tests need permission to create local sockets.

`xarm_observe` records a hand snapshot in each frame's `meta.json`; replies include
it too. Robot stream records carry `hand` instead of `gripper_width`. Hand motions
also produce the existing camera motion clips when recording is enabled. Inspect
sample ages and device timestamps; camera, arm and hand samples are not exactly
synchronized. Fingertip online flags are exposed, but raw tactile decoding and
force-controlled grasping are not implemented in this first port.
