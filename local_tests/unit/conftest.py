import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))


@pytest.fixture(autouse=True)
def artifact_root(tmp_path, monkeypatch):
    """Every test session lives under its own temporary artifact root."""
    monkeypatch.setenv('RAX_ARTIFACT_ROOT', str(tmp_path))
    return tmp_path


@pytest.fixture
def mooncake_source(tmp_path, monkeypatch):
    """Local Git source with a pinned revision behind HEAD; no private repo/network."""
    import json
    import subprocess
    from services import skill_library
    repo = tmp_path / 'mooncake'
    repo.mkdir()
    def git(*args):
        return subprocess.check_output(['git', '-C', str(repo), *args], text=True).strip()
    original = 'Original source {{ untouched }}\n'
    git('init', '-q')
    (repo / 'SKILL.md').write_text(original)
    git('add', '.')
    git('-c', 'user.name=Test', '-c', 'user.email=test@example.com', 'commit', '-qm', 'source')
    commit = git('rev-parse', 'HEAD')
    (repo / 'SKILL.md').write_text('Newer unpinned version\n')
    git('add', '.')
    git('-c', 'user.name=Test', '-c', 'user.email=test@example.com', 'commit', '-qm', 'newer')
    lock = tmp_path / 'sources.json'
    lock.write_text(json.dumps({'repository': str(repo), 'sources': {
        name: {'ref': f'skill-library/{name}/main', 'commit': commit} for name in ('dex', 'dexmate')}}))
    monkeypatch.setattr(skill_library, 'SOURCES', lock)
    monkeypatch.setenv('RAX_MOONCAKE_REPO', str(repo))
    return commit, original


def configuration(base, *, provider='anthropic', **overrides):
    """A mock-robot session configuration under `base` (which must be in the artifact root)."""
    from services.controller.config import Configuration
    return Configuration(**{'root': Path(base)/'session', 'task': 'stack_cups', 'robot': 'mock',
                            'task_description': 'Stack the red cup on the blue cup.',
                            'agent_provider': provider, **overrides})


def frontend(base, gemini=None):
    """A mock-robot frontend with its own session ledger."""
    from services.controller.frontend import Frontend
    from services.controller.session import Session
    from services.mock_backend import MockBackend
    config = configuration(base)
    return Frontend(Session(config), MockBackend(config), gemini)
