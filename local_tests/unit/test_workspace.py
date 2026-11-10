"""Workspace generation for the real-robot agent: template, skills, blocks and contract."""
import json
from pathlib import Path
import re

import pytest

from services.workspace import ROBOT_FACTS, Setup, build, recipe_for, render
from services.workspace.templates import registry

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('profile', ['rgbd', 'rgb-only'])
def test_xarm_workspace_renders_without_leftover_markup(tmp_path, profile):
    workspace = tmp_path/'ws'
    manifest = build(recipe_for(), Setup('xarm', 'stack_cups', 'claude', profile), workspace)
    for path in workspace.rglob('*.md'):
        text = path.read_text()
        assert '{{' not in text and '{%' not in text, path
        assert not re.search(r'[Rr]obo[Dd]ojo|BEHAVIOR|rehears|formal batch|container', text), path
    skills = sorted(p.name for p in (workspace/'.agents/skills').iterdir())
    assert {'manual-robot-control', 'experience-memory', 'wrist-camera-inspection', 'gemini'} <= set(skills)
    contract = json.loads((workspace/'MCP_CONTRACT.json').read_text())
    assert contract['robot'] == 'xarm' and 'status' in contract['tools']
    assert set(ROBOT_FACTS['xarm']['tools']) <= set(contract['tools'])
    assert manifest['contract'] == contract
    assert 'real hardware' in (workspace/'MCP_SESSION.md').read_text()
    assert 'UFactory xArm 6' in (workspace/'AGENTS.md').read_text()


def test_subagent_text_only_with_subagents(tmp_path):
    build(recipe_for(), Setup('franka', subagents=0), tmp_path/'none')
    build(recipe_for(), Setup('franka', subagents=2), tmp_path/'two')
    assert 'subagents' not in (tmp_path/'none/AGENTS.md').read_text()
    assert 'up to 2 subagents' in (tmp_path/'two/AGENTS.md').read_text()


@pytest.mark.parametrize('profile', ['rgbd', 'rgb-only'])
def test_configured_wuji_cameras_reach_agent_instructions(tmp_path, profile):
    options = json.loads((ROOT/'examples/xarm-wuji.json').read_text())
    setup = Setup('xarm', observation_profile=profile, backend_options=options)
    assert setup.robot_facts['overview_camera'] == 'side'
    build(recipe_for(), setup, tmp_path/'ws')
    for name in ('robot-interface', 'object-localization'):
        text = (tmp_path/'ws'/'.agents'/'skills'/name/'SKILL.md').read_text()
        assert '`side`' in text and '`wrist`' in text
        assert '`left`' not in text and '`right`' not in text


def test_blocks_registry_and_unknown_names_fail():
    assert set(registry()) == {'session.contract-pointer', 'images.forwarding'}
    with pytest.raises(ValueError, match='Unknown workspace template name'):
        render('{{ nonexistent }}', Setup('franka'))
    with pytest.raises(ValueError, match='Unknown workspace block'):
        render('{{ block("missing") }}', Setup('franka'))


def test_every_template_variable_is_a_setup_fact():
    """Sources only use names Setup.context() and vocab provide (StrictUndefined renders all)."""
    for path in [*ROOT.glob('auto_research_agent/*.md'), *ROOT.glob('agent_skills/**/*.md'),
                 *ROOT.glob('agent_blocks/**/*.md')]:
        if path.name == 'README.md' and path.parent.name == 'agent_skills':
            continue
        render(path.read_text(), Setup('xarm'))


@pytest.mark.parametrize('example', ['xarm-wuji-bimanual.json',
                                    'xarm-wuji-bimanual-three-camera.json',
                                    'xarm-wuji-bimanual-five-camera.json'])
def test_bimanual_workspace_uses_selected_camera_inventory(tmp_path, example):
    options = json.loads((ROOT/'examples'/example).read_text())
    setup = Setup('xarm_bimanual', observation_profile='rgb-only', backend_options=options)
    names = tuple(c['name'] for c in options['cameras'])
    assert setup.robot_facts['cameras'] == names
    expected_overview = next(c['name'] for c in options['cameras'] if c['mount'] == 'fixed')
    assert setup.robot_facts['overview_camera'] == expected_overview
    ws = tmp_path/'workspace'
    build(recipe_for(), setup, ws)
    stream = (ws/'.agents/skills/live-stream/SKILL.md').read_text()
    perception = (ws/'.agents/skills/object-localization/SKILL.md').read_text()
    assert f'at most {12 // len(names)} ages' in stream
    assert f'api.stream changes {expected_overview}' in stream
    for name in names:
        assert f'`{name}`' in stream and f'`{name}`' in perception
    if len(names) == 3:
        assert expected_overview == 'front'
        for path in ws.rglob('*.md'):
            assert 'side_a' not in path.read_text() and 'side_b' not in path.read_text()
    assert 'xarm_pixel_to_position' not in json.loads((ws/'MCP_CONTRACT.json').read_text())['tools']
