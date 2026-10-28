"""Refresh an existing, stopped Wuji session's tool contract and all provided skills.

Preserves the conversation, task, memory, code, reference files and recordings.
Starts no robot service or agent. Resume with scripts/start_agent.sh afterward.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from services.controller.config import Configuration  # noqa: E402
from services.workspace import Setup, build, recipe_for  # noqa: E402

CONTRACT_FILES = ('MCP_CONTRACT.json', 'MCP_SESSION.md')


def refresh(root):
    root = Path(root).resolve()
    if root.is_file():
        root = root.parent
    config = Configuration.load(root/'operator-input.json')
    if config.root != root:
        raise ValueError('configuration root does not match the selected session')
    setup = Setup.from_config(config, gemini=bool(json.loads((root/'model-relay.json').read_text()).get('gemini_key_file')))
    if setup.end_effector != 'wuji_hand2':
        raise ValueError('expected an existing Wuji session')
    workspace = root/'agent-workspace'
    if not workspace.is_dir():
        raise ValueError('session has no deployed agent workspace')
    with (root/'launch.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('exit the running RAX session before refreshing its tools') from None
        with tempfile.TemporaryDirectory(prefix='rax-wuji-tools-') as temporary:
            staging = Path(temporary)/'workspace'
            build(recipe_for(), setup, staging)
            files = [*CONTRACT_FILES, *(p.relative_to(staging).as_posix()
                       for p in sorted((staging/'.agents/skills').rglob('*')) if p.is_file())]
            # Check every destination before writing any; do not follow agent links.
            for relative in files:
                path = workspace/relative
                for candidate in (path, *path.parents):
                    if candidate == root:
                        break
                    if candidate.is_symlink():
                        raise ValueError(f'refusing symlinked workspace path: {candidate}')
                if path.exists() and not path.is_file():
                    raise ValueError(f'expected a file: {path}')
            operator = root/'operator'
            operator.mkdir(exist_ok=True)
            if operator.is_symlink():
                raise ValueError('refusing symlinked operator directory')
            backup = Path(tempfile.mkdtemp(prefix='tool-refresh-backup-', dir=operator))
            for relative in files:
                destination = workspace/relative
                if destination.exists():
                    old = backup/relative
                    old.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(destination, old)
                destination.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as handle:
                    pending = Path(handle.name)
                try:
                    shutil.copyfile(staging/relative, pending)
                    os.replace(pending, destination)
                finally:
                    pending.unlink(missing_ok=True)
            (backup/'refresh-manifest.json').write_text(json.dumps({
                'session': str(root), 'updated_files': {
                    relative: hashlib.sha256((workspace/relative).read_bytes()).hexdigest()
                    for relative in files}}, indent=2) + '\n')
            return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', default=os.environ.get('CUBE_SESSION'),
                        help='Existing session directory (default: $CUBE_SESSION)')
    args = parser.parse_args()
    if not args.session:
        parser.error('set CUBE_SESSION or pass --session')
    backup = refresh(args.session)
    print(f'Updated Wuji tool contract and all provided skills. Previous files and refresh manifest: {backup}')
    print('No services started. Resume using scripts/start_agent.sh --config SESSION/operator-input.json --resume.')


if __name__ == '__main__':
    main()
