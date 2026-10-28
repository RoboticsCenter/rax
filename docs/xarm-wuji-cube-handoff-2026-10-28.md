# Cube-into-bowl operator handoff — October 28, 2026

The operator will start Claude Code. No Claude Code session or model was
started during this commissioning work. The completed hand test is described
in the [qualification report](wuji-rax-qualification-2026-10-28.md).

Prepared session:
`.local-runtime/sessions/20261028T101833Z_cube_into_bowl_v13_bf9647/`.
Its `operator-input.json` snapshots v13, both calibrated cameras, 15 mm/s
Cartesian speed and a 50 mm per-call translation limit. Task source:
[`examples/xarm-wuji-cube-into-bowl.md`](../examples/xarm-wuji-cube-into-bowl.md).
`workspace-preview/` is an offline rendering for inspection; the launcher will
build its separate `agent-workspace/` when the operator starts it.

The arm is stopped in state 4 at [0.25310, -0.00134, 0.31412] m. All twenty
hand joints were confirmed disabled, partway through the interrupted stop-test
path. Both commissioning frontends and their hardware owners were closed.
Preparation will hold the measured posture; it will not home the hand.

## Launch prerequisites

The Claude Code executable bundled with the installed VS Code extension is
available, but `claude` is not on this shell's PATH. The RAX launcher's dedicated
experiment token is also absent. In your terminal:

```bash
cd /home/jerry/rax
export RAX_PYTHON=/home/jerry/rax/.venv/bin/python
export RAX_ARTIFACT_ROOT=/home/jerry/rax/.local-runtime
export RAX_BACKEND_OPTIONS=/home/jerry/rax/.local-runtime/cube-into-bowl-v13-options.json
export PATH=/home/jerry/.vscode/extensions/anthropic.claude-code-2.1.312-linux-x64/resources/native-binary:$PATH
export CUBE_SESSION=/home/jerry/rax/.local-runtime/sessions/20261028T101833Z_cube_into_bowl_v13_bf9647
```

Use an independently installed `claude` if preferred. Configure the launcher's
account interactively using the repository's existing setup command:

```bash
bash scripts/setup_claude_experiment_login.sh
```

That flow uses a separate experiment account. To deliberately bill your current
personal Claude account instead, the existing script supports `--personal`.
Complete authentication in your terminal/browser; the setup script keeps the
token outside the agent workspace.

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

After login setup, start the prepared session yourself:

```bash
bash scripts/start_agent.sh --config "$CUBE_SESSION/operator-input.json"
```

It starts disarmed. In a second terminal with the same environment, inspect the
scene/status and complete the physical checks below. When ready and at the
emergency stop, use the operator controls:

```bash
scripts/xarm status
scripts/xarm ready
"$RAX_PYTHON" scripts/arm_robot.py "$CUBE_SESSION" arm --minutes 10
scripts/wuji prepare --session "$CUBE_SESSION"
scripts/wuji status --session "$CUBE_SESSION"
scripts/xarm requests
```

`ready` and `arm` prompt for physical clearance. The explicit session argument
avoids accidentally arming a different recent session. Answer the agent's
pending request with `scripts/xarm reply 'your answer'` when its specific
prerequisite is satisfied. For an independent software stop:

```bash
"$RAX_PYTHON" scripts/arm_robot.py "$CUBE_SESSION" disarm
scripts/wuji stop --session "$CUBE_SESSION"
```

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
