"""Clone the external Mooncake libraries into a session, like Mooncake's launcher.

RAX tracks only source refs and the adaptation skill. Each session gets its own
Git checkout; source commits are pinned, and existing session work is preserved.
"""
import json
import os
from pathlib import Path
import subprocess
import tempfile

PROJECT = Path(__file__).resolve().parents[1]
SOURCES = PROJECT / 'agent_skills/end_effectors/wuji_hand2/screw-lightbulb/references/sources.json'


def _git(path, *args):
    result = subprocess.run(['git', '-C', str(path), *args], capture_output=True,
                            text=True, timeout=180)
    if result.returncode:
        raise RuntimeError(f'Mooncake skill library: {result.stderr.strip()}')
    return result.stdout.strip()


def checkout(workspace):
    workspace = Path(workspace)
    if not (workspace / '.agents/skills/screw-lightbulb/SKILL.md').is_file():
        return {}
    sources = json.loads(SOURCES.read_text())
    sibling = PROJECT.parent / 'mooncake'
    repository = os.environ.get('RAX_MOONCAKE_REPO') or (
        str(sibling) if sibling.exists() else sources['repository'])
    root = workspace / 'skill-library'
    if root.is_symlink():
        raise ValueError(f'Refusing symlinked skill library: {root}')
    root.mkdir(exist_ok=True)
    result = {}
    for robot, source in sources['sources'].items():
        destination = root / robot
        if destination.is_symlink():
            raise ValueError(f'Refusing symlinked skill library: {destination}')
        if not destination.exists():
            with tempfile.TemporaryDirectory(prefix=f'.{robot}-', dir=root) as temporary:
                pending = Path(temporary) / 'library'
                pending.mkdir()
                _git(pending, 'init', '-q')
                _git(pending, 'remote', 'add', 'origin', repository)
                _git(pending, 'fetch', '-q', '--depth=1', 'origin', source['commit'])
                _git(pending, 'checkout', '-q', '-b', 'session', source['commit'])
                pending.rename(destination)
        if not (destination / '.git').is_dir():
            raise ValueError(f'Expected a session Git library: {destination}')
        result[robot] = {'path': str(destination), 'source_ref': source['ref'],
                         'source_commit': source['commit'], 'head': _git(destination, 'rev-parse', 'HEAD')}
    return result
