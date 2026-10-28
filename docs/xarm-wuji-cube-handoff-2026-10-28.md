# Cube-into-bowl operator handoff — October 28, 2026

The operator starts Claude Code. The first operator-launched session reached
MCP status but could not read its workspace because it was inside the protected
repository. A fresh session is now prepared outside the checkout and has not
been started. The completed hand test is described in the
[qualification report](wuji-rax-qualification-2026-10-28.md).

Prepared session:
`/home/jerry/rax-runtime/sessions/20261028T103931Z_cube_into_bowl_v13_1a66a7/`.
Its `operator-input.json` snapshots v13, both calibrated cameras, 15 mm/s
Cartesian speed and a 50 mm per-call translation limit. The selected provider
is `openai`, with model `openai/gpt-6-astra` in Claude Code via the inherited
RAX Responses relay. Task source:
[`examples/xarm-wuji-cube-into-bowl.md`](../examples/xarm-wuji-cube-into-bowl.md).
`workspace-preview/` is an offline rendering for inspection; the launcher will
build its separate `agent-workspace/` when the operator starts it.

The former session remains under
`.local-runtime/sessions/20261028T101833Z_cube_into_bowl_v13_bf9647/` for its
conversation and audit history. Exit that Claude session with `/exit` before
launching the new one so its backend releases both cameras and the hand lease.
Do not resume the old session: its location is inside the protected checkout.

Agent Python is `/home/jerry/rax-runtime/agent-env/bin/python`, with the same
NumPy, SciPy, OpenCV and Pillow versions used by commissioning, without the robot
SDKs. Robot services still use `/home/jerry/rax/.venv/bin/python`. A Bubblewrap
preflight read `AGENTS.md`, `MCP_SESSION.md` and `TASK.md`, wrote a scratch file
and imported the image-processing libraries while keeping the repository and
private session configuration hidden. It made no model or hardware calls.

The arm is stopped in state 4 at [0.25310, -0.00134, 0.31412] m. All twenty
hand joints were confirmed disabled, partway through the interrupted stop-test
path. Both commissioning frontends and their hardware owners were closed.
Preparation will hold the measured posture; it will not home the hand.

## Launch prerequisites

The operator selected GPT Astra. This provider uses an OpenAI API key; no Claude
experiment login or token is required. The bundled Claude Code executable is
linked at `~/.local/bin/claude` (version 2.1.312 verified). Add that directory
to PATH below. Use the repository's official
`scripts/start_agent.sh` directly. Set the environment in your launch terminal
and separate operator terminal:

```bash
cd /home/jerry/rax
export RAX_PYTHON=/home/jerry/rax/.venv/bin/python
export RAX_ARTIFACT_ROOT=/home/jerry/rax-runtime
export RAX_BACKEND_OPTIONS=/home/jerry/rax-runtime/cube-into-bowl-v13-options.json
export PATH="$HOME/.local/bin:$PATH"
export CUBE_SESSION=/home/jerry/rax-runtime/sessions/20261028T103931Z_cube_into_bowl_v13_1a66a7
```

Use an independently installed `claude` if preferred. In the launch terminal,
supply the key through the environment (skip this if already exported):

```bash
read -r -s -p 'OpenAI API key: ' OPENAI_API_KEY
printf '\n'
export OPENAI_API_KEY
```

The hidden prompt keeps the key out of shell history and does not save it to
disk. The host relay reads it; the agent process gets only the local relay
placeholder. The operator's first session reached Astra and MCP status; the
remaining launch correction is the workspace location.

Alternatively, pass `--key-file /absolute/path/to/openai.key` to the launcher;
the file must be owned by you with mode 0600. The existing `openai` provider
routes the configured model to `gpt-6-astra` on `/v1/responses`. Its Claude Code
model override is labelled `Astra via RAX`.

Verify the hand route before launch:

```bash
ip route get 192.168.1.111
ip route get 192.168.100.16
```

The hand route should use `enx6c1ff7618956`, source `192.168.1.11`. It was
restored for this test but is temporary. If adapter reconnection removes it:

```bash
nmcli device modify enx6c1ff7618956 \
  +ipv4.routes '192.168.1.111/32 0.0.0.0 50 src=192.168.1.11'
```

## Start and prepare

After supplying the OpenAI key, start the prepared session yourself:

```bash
bash scripts/start_agent.sh --config "$CUBE_SESSION/operator-input.json"
```

The official launcher enables the optional Gemini tool when its separate
OpenRouter key is available at `~/.config/rax/openrouter.key` or supplied with
`--gemini-key-file`. With no such key, the tool is omitted automatically. Astra
uses the OpenAI key independently; no Gemini override is needed.

It starts disarmed. In a second terminal with the same environment, inspect the
scene/status and complete the physical checks below. When ready and at the
emergency stop, use the operator controls:

```bash
scripts/xarm status
scripts/xarm-wuji-ready
scripts/wuji status --session "$CUBE_SESSION"
scripts/xarm requests
```

`xarm-wuji-ready` uses `$CUBE_SESSION` (or `--session PATH`) and prompts once
with `READY` for physical clearance covering the arm and hand. It readies the
arm using that session's saved options, arms for 10 minutes (override with
`--minutes N`), then prepares the hand through its running owner to hold the
measured posture. If preparation fails or is cancelled, it revokes session
arming and requests a hand stop. It never selects a different recent session.
Answer the agent's
pending request with `scripts/xarm reply 'your answer'` when its specific
prerequisite is satisfied. For an independent software stop:

```bash
"$RAX_PYTHON" scripts/arm_robot.py "$CUBE_SESSION" disarm
scripts/wuji stop --session "$CUBE_SESSION"
```

In the operator dashboard, Wuji requests have a **Ready & send reply** button
(**Ready & reset done** for scene resets). One browser confirmation runs the
same preparation sequence for that request's session, arms for 60 minutes,
and sends the reply only after success. Failures leave the request unanswered
and disarm the session. **DISARM** interrupts an ongoing dashboard preparation.
Use **Send reply only** for a message that should not prepare the robot.
The separate **ARM motion** button also defaults to 60 minutes.

## Remaining physical commissioning

- Verify assembly payload and CoG. Current controller values are 0.82 kg and
  [0, 0, 48] mm; these were not measured for this Wuji assembly.
- Establish a palm/fingertip contact reference and inspect full hand sweep.
  The retained 172 mm TCP offset is not a calibrated contact point. Changing
  the controller TCP requires updating the TCP-relative wrist extrinsics.
- Qualify cube-specific opening, opposing-finger placement, retention after a
  short lift, transport and release. Home/fist are empty-hand joint postures.
- Verify the task's arm envelope and reset from the resulting task poses.

The prepared task explicitly requests operator help for missing geometry or
contact prerequisites and stops after one verified placement. Camera mounts
and robot base were confirmed unchanged. A fresh read-only cube-top check
agreed across cameras within about 2.34 mm; the agent still needs fresh images
after any scene changes. Encoder-warning root cause, communication/process
loss and physical emergency-stop qualification remain open checklist items.
