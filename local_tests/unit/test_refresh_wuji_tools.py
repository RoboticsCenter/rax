import fcntl
import importlib.util
import json
from pathlib import Path

import pytest

from services.controller.config import Configuration
from services.workspace import Setup, build, recipe_for
from wuji_fakes import hand_config


@pytest.fixture
def deployed(tmp_path):
    path = Path(__file__).resolve().parents[2]/'scripts/refresh_wuji_tools.py'
    spec = importlib.util.spec_from_file_location('refresh_wuji_tools', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root = tmp_path/'session'
    root.mkdir()
    config = Configuration(root=root, task='test', backend_options={'end_effector': hand_config()})
    (root/'operator-input.json').write_text(json.dumps(config.to_dict()))
    (root/'model-relay.json').write_text(json.dumps({'gemini_key_file': None}))
    build(recipe_for(), Setup.from_config(config), root/'agent-workspace')
    return module, root


def test_refresh_backs_up_old_contract_and_preserves_learning(deployed):
    module, root = deployed
    ws = root/'agent-workspace'
    (ws/'MCP_SESSION.md').write_text('old tools')
    old_skill = ws/'.agents/skills/live-stream/SKILL.md'
    old_skill.write_text('old stream skill')
    for name in ('memory/PROGRESS.md', 'code/grasp.py', 'reference/offsets.txt', '.agents/skills/custom/SKILL.md'):
        path = ws/name
        path.parent.mkdir(exist_ok=True)
        path.write_text('keep this')
    backup = module.refresh(root)
    assert (backup/'MCP_SESSION.md').read_text() == 'old tools'
    assert (backup/'.agents/skills/live-stream/SKILL.md').read_text() == 'old stream skill'
    assert old_skill.read_text() != 'old stream skill'
    manifest = json.loads((backup/'refresh-manifest.json').read_text())['updated_files']
    assert '.agents/skills/live-stream/SKILL.md' in manifest
    assert '.agents/skills/experience-memory/references/format.md' in manifest
    tools = json.loads((ws/'MCP_CONTRACT.json').read_text())['tools']
    assert {'wuji_set_effort', 'wuji_set_impedance'} <= set(tools)
    assert 'wuji_set_effort' in (ws/'.agents/skills/hand-control/SKILL.md').read_text()
    for name in ('memory/PROGRESS.md', 'code/grasp.py', 'reference/offsets.txt', '.agents/skills/custom/SKILL.md'):
        assert (ws/name).read_text() == 'keep this'


def test_refresh_refuses_running_session(deployed):
    module, root = deployed
    with (root/'launch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(RuntimeError, match='exit the running'):
            module.refresh(root)


def test_refresh_does_not_follow_agent_symlink(deployed, tmp_path):
    module, root = deployed
    outside = tmp_path/'outside.txt'
    outside.write_text('keep this')
    contract = root/'agent-workspace/MCP_SESSION.md'
    contract.unlink()
    contract.symlink_to(outside)
    with pytest.raises(ValueError, match='symlink'):
        module.refresh(root)
    assert outside.read_text() == 'keep this'
