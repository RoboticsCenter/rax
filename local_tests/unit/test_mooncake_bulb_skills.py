"""External libraries load on the real deployment path, outside tracked templates."""
import json
from pathlib import Path

import pytest

from harness.claude_cli.launcher import populate_workspace
from services import skill_library
from services.controller.config import Configuration
from services.workspace import Setup, build, recipe_for

ROOT = Path(__file__).resolve().parents[2]


def test_deployment_clones_pinned_libraries(tmp_path, mooncake_source):
    commit, original = mooncake_source
    options = json.loads((ROOT / 'examples/xarm-wuji-bimanual.json').read_text())
    config = Configuration(root=tmp_path/'session', robot='xarm_bimanual',
                           task='screw_lightbulb', backend_options=options)
    workspace = config.root / 'agent-workspace'
    built = populate_workspace(config, workspace)
    assert 'screw-lightbulb' in (workspace/'AGENTS.md').read_text()
    for robot in ('dex', 'dexmate'):
        library = workspace/'skill-library'/robot
        assert (library/'SKILL.md').read_text() == original
        assert built['skill_library'][robot]['head'] == commit
        assert skill_library._git(library, 'branch', '--show-current') == 'session'
    assert not (workspace/'.agents/skills/screw-lightbulb/references/mooncake').exists()
    # A refresh must preserve uncommitted task learning.
    (workspace/'skill-library/dex/SKILL.md').write_text('session edits')
    skill_library.checkout(workspace)
    assert (workspace/'skill-library/dex/SKILL.md').read_text() == 'session edits'
    contract = json.loads((workspace/'MCP_CONTRACT.json').read_text())
    assert not any(name.startswith(('dex_', 'dexmate_')) for name in contract['tools'])


def test_missing_source_fails_without_publishing_partial_checkout(tmp_path, mooncake_source, monkeypatch):
    skill = tmp_path/'.agents/skills/screw-lightbulb'
    skill.mkdir(parents=True)
    (skill/'SKILL.md').touch()
    monkeypatch.setenv('RAX_MOONCAKE_REPO', str(tmp_path/'missing'))
    with pytest.raises(RuntimeError, match='Mooncake skill library'):
        skill_library.checkout(tmp_path)
    assert not list((tmp_path/'skill-library').iterdir())


def test_no_library_without_wuji(tmp_path, monkeypatch):
    build(recipe_for(), Setup('franka'), tmp_path)
    monkeypatch.setenv('RAX_MOONCAKE_REPO', '/nonexistent')
    assert skill_library.checkout(tmp_path) == {}
    assert not (tmp_path/'skill-library').exists()
