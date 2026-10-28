# RAX: one AgileX Piper + BrainCo Revo 2

This port takes its RAX architecture from `real/xarmwuji` at `9e34222` (fetched
2026-10-28), not its xArm geometry, Wuji units, calibration or firmware settings.
It retains the RAX frontend, session arming, audit trail, sandbox and workspace
builder. The robot boundary is a **direct SDK integration**, not a client of an
old evaluation process.

## Architecture

```
Claude agent / scripts -> RAX session frontend (serialized MCP)
                           |-- PiperArm -> pyAgxArm -> SocketCAN -> right Piper
                           |     `-- independent 100 Hz command/watchdog thread
                           |-- RevoHand -> isolated SDK worker
                           |     `-- bc-stark-sdk -> RAX DaMiao adapter -> right Revo 2
                           `-- RGB recorder -> existing Spyglass MJPEG camera service
```

The frontend is the RAX arm owner and the sole parent of the hand worker. It
opens its own `pyAgxArm` driver. There is **no dependency** on the old
`arm_server.py`, its Unix socket, `piper_sdk`, `hand_ctl.py`, or an evaluation
checkout. A native hand-library crash cannot kill the arm watchdog. No ROS,
Polymetis or realtime process from the Franka branch is needed.

### Why not pyAgxArm for both devices?

The vendor's `pyAgxArm` Revo2 Pro/Touch API wraps the official `bc-stark-sdk`, but
its transport requires an **AgileX conversion board on the arm's CAN bus**. This
rig instead connects the hand through a **separate DaMiao USB-CAN-FD adapter**.
Using that wrapper unchanged would send to the wrong physical transport.

The chosen boundary is therefore `pyAgxArm` for Piper and the official BrainCo
SDK for Revo2 Touch. `services/piper/damiao.py` supplies only native USB transport,
CAN-FD framing and checked SDK callbacks. It does not reimplement the hand SDK
or import application code from another robot project. Only the exact authorized
finite six-motor position request can be written; configuration, calibration,
current mode, turbo, firmware writes and retries are rejected at the transport.

Vendor references: `agilexrobotics/pyAgxArm`,
`docs/piper/piper_api.md` and
`docs/effector/revo2_pro_touch/revo2_pro_touch_api.md`; BrainCo `bc-stark-sdk`.
Dependencies are pinned in `requirements-piper.txt`: pyAgxArm commit `e7aef17`,
BrainCo SDK `2.0.7`. A vendor DM_Device `libdm_device.so` is required; point
`damiao_library` at a trusted local installation. Its current location under
CenterOS is just the installed native library, **not a CenterOS Python/runtime
dependency**. Keep the library and any required companion shared libraries
together if relocating them. Native context/callback lifetime is process-owned;
USB teardown is isolated from the arm process.

## Selected hardware

Only the **right pair** is controlled:

| Component | Identity / connection |
| --- | --- |
| Piper USB-CAN serial | `002C001B4759530920353131` |
| Piper firmware | `S-V1.8-2`, verified SDK profile `default` |
| Revo2 Touch serial / firmware | `BCXTR2362J2500015` / `1.0.22.U` |
| Revo address | DaMiao channel **1**, hand ID **127** |
| DaMiao firmware | `dual_app v1.0.0.7` |
| Overview camera | D435 `263322070113`, Spyglass port 8090 |

The arm interface is resolved by USB serial, currently **can1**, not the stale
can0 label in older profiles. Independent receive-only CAN checks freshness of
all three joint feedback frames, status and all six driver frames; SDK angles
must agree with the raw measurements. The SDK firmware query must match the
operator's chosen profile before motion. There is no feedback clamping.

Hand discovery probes the selected right-hand address on candidate DaMiao
adapters and checks its serial, firmware, normalized units, protected currents
(500 mA) and disabled turbo mode. Discovery may read an identity on another
adapter; it never moves the left hand. The left arm is not connected by RAX.
A pair lease and per-adapter lock prevent duplicate RAX owners. As with any CAN
controller, do not run an independent teleop/controller that also sends commands
while RAX owns the selected robot; existing software is not an architectural
requirement and should not dictate this port's design.

## Install and read-only bring-up

```bash
export RAX_PYTHON=/home/roboticscenter/rax-envs/robot/bin/python
uv pip install --python "$RAX_PYTHON" -r requirements-piper.txt
uv venv --python 3.11 ~/rax-agent-env
uv pip install --python ~/rax-agent-env/bin/python numpy scipy opencv-python pillow
$RAX_PYTHON scripts/piper.py check --output /tmp/rax-piper-direct-check.json
```

The arm CAN interface must already be up at 1 Mbit/s and the DaMiao channel set
to CAN-FD 1/5 Mbit/s. RAX checks configuration; it does not silently reconfigure
another device. Preflight reads firmware, arm/CAN feedback, hand identity,
protections and state, and a fresh RGB frame. Communication-channel activation
is not motor enable. No arm trajectory, finger target or enable is sent.
`ok` means **communication passed**, not motion readiness or grasp success.

The [read-only bring-up report](piper-brainco-bringup-2026-10-28.md) records the
verified direct connections. The right arm remains disabled, with raw J2/J3
outside commanded limits. RAX neither clamps those readings nor attempts a
recovery. A person must safely support/recover the arm before enabling it.

## Configure and run a session

Use `examples/piper-brainco.json` or an operator-owned copy. Workspace bounds,
table height, hand extent and DH offset variant are starting assumptions, not
measured workcell calibration. `motion_commissioned` and `tool_calibrated`
default to false. Configuring starts nothing.

```bash
export RAX_ARTIFACT_ROOT=/path/to/disk/rax-piper-runtime
$RAX_PYTHON scripts/piper.py configure --task pick_object --task-file task.md
bash scripts/start_agent.sh --config <printed-operator-input.json>
```

Use an artifact directory outside the repository and protected robot-stack
paths. `scripts/configure_session.py --robot piper` also works and defaults to
`rgb-only`; RGB-D is refused until geometry is actually commissioned. The
existing dashboard's direct robot/hold widgets are still Franka-specific: use
the CLI for Piper bring-up and session configuration, not those widgets.

Generated skills advertise only Piper/Revo tools and the actual overview camera.
The agent cannot read either SDK, the native-library directory, raw USB nodes,
operator settings/arming, or the private hand worker log. Old arm sockets are
also hidden as bypass paths even though the port does not depend on them.
The example uses `~/rax-agent-env` for agent Python, separate from the protected
robot SDK environment. Do not place that interpreter under a denied robot-stack
directory; it needs only ordinary scientific/image packages and the workspace API.

## Motion and safety contract

The agent receives small joint/Cartesian moves, six normalized motor targets,
status, observation, stream review and best-effort hold. There is no agent motor
enable, disable, reset, homing, current mode, stored hand action or calibration API.
Cartesian motion additionally requires the measured flange-to-grasp-TCP transform.
Joint arguments are radians; pose arguments are base-frame metres and xyzw unit
quaternions; Revo targets are integers on a 0–1000 normalized scale, not radians
or parallel-gripper widths.

- The entire planned joint-interpolated path is checked at <=0.002 rad spacing.
  It is not claimed to be a straight Cartesian path.
- The backend and independent arm thread check arming, legal joints, measured
  FK/workspace/table clearance, hand envelope and following error. Only one
  <=100 ms segment is accepted at once. An expired motion lease cancels the
  segment and changes to measured-position hold, without needing the MCP
  dispatcher to return. Disarm is checked during the 100 Hz stream.
- Stale/failed feedback, unsafe state or a stream exception requests the SDK's
  damped electronic stop and latches a fault. This does not reset faults or
  disable/drop the arm. Closing an active stream requests a damped stop;
  closing a read-only connection sends no stop or motion.
- Finger moves are finite <=200 ms commands and <=100 normalized units per call.
  Writes require a CRC-checked matching register-range ACK and are never retried.
  Hand stop is best-effort measured-position hold. Missing replies/native worker
  failure stop further dispatch and leave the result uncertain, not "safe".
- Software threads, USB and the OS are **not a certified realtime safety system**.
  The spherical hand/table bound is not arbitrary-obstacle or self-collision
  detection. The physical emergency stop and supervised clearance remain required.

Before setting `motion_commissioned`, qualify the DH variant, hand envelope,
table/workspace, joint directions, finger targets/obstruction, failure/watchdog
paths, and disarm under supervision. Measure the grasp TCP before setting
`tool_calibrated`. Enable hardware using a trusted operator procedure with a
person beside the robot; RAX never does it automatically. Then configure a new
session and separately arm it using `scripts/arm_robot.py <session> arm --minutes 5`.
No physical motion qualification is claimed by this change.

## Observation and evidence

Spyglass owns the fixed D435; RAX records its existing local RGB stream. There
is no wrist camera, depth image, metric pixel tool or inherited Franka/xArm
extrinsic calibration. Camera timestamps are host receipt times, not synchronized
exposure. JPEG/JSONL recordings use the existing RAX `api.stream` format (default
5 Hz / 10 minutes). Background hand values are last-known, not necessarily
simultaneous with the image. Explicit status calls read current hand feedback.
Grasp success requires independent visual/physical evidence; motor target
arrival, obstruction or current is not calibrated force or a verified grasp.
