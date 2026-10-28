"""Combined operator preparation with fake commands; never contacts hardware."""
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from services.xarm import arming


@pytest.fixture
def shortcut(tmp_path, monkeypatch):
    path = Path(__file__).resolve().parents[2] / 'scripts' / 'prepare_xarm_wuji.py'
    spec = importlib.util.spec_from_file_location('prepare_xarm_wuji', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root = tmp_path / 'selected-session'
    root.mkdir()
    options = {'xarm_ip': '192.0.2.16', 'end_effector': {'type': 'wuji_hand2'}}
    (root / 'operator-input.json').write_text(json.dumps({'backend_options': options}))
    monkeypatch.setenv('CUBE_SESSION', str(root))
    monkeypatch.setenv('RAX_BACKEND_OPTIONS', '/wrong-session-options.json')
    calls = []
    state = {'mode': 'disabled'}
    failures = set()

    def run(command, **kwargs):
        command = list(command)
        action = command[2] if command[1] != 'scripts/arm_robot.py' else command[3]
        calls.append((action, command))
        if action in failures:
            raise subprocess.CalledProcessError(1, command)
        if action == 'ready':
            assert json.loads(Path(kwargs['env']['RAX_BACKEND_OPTIONS']).read_text()) == options
        elif action == 'arm':
            assert command[2] == str(root)
            assert '--yes' in command
            arming.write(root, True, minutes=float(command[command.index('--minutes') + 1]))
        if command[1] == 'scripts/wuji':
            assert command[-2:] == ['--session', str(root)]
        return SimpleNamespace(stdout=json.dumps(state))

    monkeypatch.setattr(module.subprocess, 'run', run)
    return SimpleNamespace(module=module, root=root, calls=calls, state=state, failures=failures)


def test_success_uses_one_session_and_its_options(shortcut):
    shortcut.module.main(['--minutes', '7'])
    assert [action for action, _ in shortcut.calls] == ['status', 'ready', 'arm', 'prepare']
    state = arming.read(shortcut.root)
    assert state['armed']
    assert state['until'] - state['since'] == pytest.approx(420)


def test_dashboard_confirmation_avoids_terminal_prompt(shortcut):
    shortcut.module.main(['--yes'])
    ready = next(command for action, command in shortcut.calls if action == 'ready')
    assert ready[-1] == '--yes'


@pytest.mark.parametrize('failed', ['ready', 'arm', 'prepare'])
def test_failure_revokes_existing_arming_and_stops_hand(shortcut, failed):
    arming.write(shortcut.root, True, minutes=10)
    shortcut.failures.add(failed)
    with pytest.raises(subprocess.CalledProcessError):
        shortcut.module.main([])
    assert not arming.read(shortcut.root)['armed']
    actions = [action for action, _ in shortcut.calls]
    assert actions[-1] == 'stop'
    assert actions.index(failed) == len(actions) - 2


def test_stop_failure_still_revokes_arming(shortcut):
    shortcut.failures.update(['prepare', 'stop'])
    with pytest.raises(subprocess.CalledProcessError) as error:
        shortcut.module.main([])
    assert 'prepare' in error.value.cmd
    assert not arming.read(shortcut.root)['armed']


def test_busy_hand_does_not_enable_arm(shortcut):
    shortcut.state['mode'] = 'executing'
    with pytest.raises(SystemExit):
        shortcut.module.main([])
    assert [action for action, _ in shortcut.calls] == ['status']
    assert not arming.arm_file(shortcut.root).exists()


@pytest.mark.parametrize('minutes', ['0', '-1', 'nan', 'inf'])
def test_invalid_duration_rejected_before_hardware_access(shortcut, minutes):
    with pytest.raises(SystemExit):
        shortcut.module.main(['--minutes', minutes])
    assert not shortcut.calls


def test_no_implicit_latest_session(shortcut, monkeypatch):
    monkeypatch.delenv('CUBE_SESSION')
    with pytest.raises(SystemExit):
        shortcut.module.main([])
    assert not shortcut.calls


def test_explicit_config_path_selects_session(shortcut, monkeypatch):
    monkeypatch.setenv('CUBE_SESSION', '/wrong-session')
    shortcut.module.main(['--session', str(shortcut.root / 'operator-input.json')])
    assert arming.read(shortcut.root)['armed']
