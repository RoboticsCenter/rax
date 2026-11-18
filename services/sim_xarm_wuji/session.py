"""Start a robot service on the twin, for running skills without an agent.

    python -m services.sim_xarm_wuji.session [--scene bottle] [--seed 3] [--privileged]
    # prints RAX_MCP_SOCKET=...; then, in another shell:
    RAX_MCP_SOCKET=... python -c "from api.runtime import Context; ..."   # or a skill's main(ctx)

It writes a session configuration under the artifact root (robot `xarm_bimanual` with
the `simulation` block of examples/xarm-wuji-bimanual-sim.json, so the session is the
real one's in every respect but its devices), starts RAX's own robot service
(services/controller/frontend.py) on a private socket, and waits. The backend readies,
arms and prepares the simulated rig itself (simulation.auto_prepare). Ctrl-C stops all.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time

PROJECT = Path(__file__).resolve().parents[2]


def configure(scene='cube', seed=0, *, privileged=False, cameras=True, task='sim', overrides=None):
    from services.controller.config import Configuration
    from services.storage_root import artifact_root
    options = json.loads((PROJECT/'examples'/'xarm-wuji-bimanual-sim.json').read_text())
    options['simulation'].update(scene=scene, seed=seed, privileged_tools=privileged, cameras=cameras,
                                 **(overrides or {}))
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    root = artifact_root()/'sessions'/f'{stamp}_{task}_{secrets.token_hex(3)}'
    config = Configuration(root=root, task=task, task_description=f'Simulated rig, scene {scene}.',
                           robot='xarm_bimanual', backend_options=options)
    root.mkdir(parents=True, mode=0o700)
    path = root/'operator-input.json'
    path.write_text(json.dumps(config.to_dict(), indent=2) + '\n')
    path.chmod(0o600)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scene', default='cube')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--privileged', action='store_true', help='expose the PRIVILEGED sim_* MCP tools')
    parser.add_argument('--no-cameras', action='store_true')
    args = parser.parse_args(argv)
    config = configure(args.scene, args.seed, privileged=args.privileged, cameras=not args.no_cameras)
    socket_path = Path(f'/tmp/rax-sim-{os.getuid()}')/f'mcp-{config.parent.name[-6:]}.sock'
    socket_path.parent.mkdir(mode=0o700, exist_ok=True)
    process = subprocess.Popen([sys.executable, '-m', 'services.controller.frontend', '--config', str(config),
                                '--socket', str(socket_path)], cwd=PROJECT,
                               env={**os.environ, 'PYTHONPATH': str(PROJECT)})

    def stop(*_):
        raise KeyboardInterrupt  # SIGTERM too: never leave the robot service running orphaned
    signal.signal(signal.SIGTERM, stop)
    try:
        while not socket_path.exists():
            if process.poll() is not None:
                raise SystemExit(f'robot service exited ({process.returncode})')
            time.sleep(0.1)
        print(f'session {config.parent}\nRAX_MCP_SOCKET={socket_path}', flush=True)
        process.wait()
    except KeyboardInterrupt:
        pass
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=30)


if __name__ == '__main__':
    main()
