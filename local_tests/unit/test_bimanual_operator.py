"""Operator actions stay pinned to one session and cover both assemblies."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from test_xarm_bimanual import options, Arm
from services.xarm import arming
from services.xarm_bimanual import operator as controls
from services.wuji.preparation import PreparationBlocked


@pytest.fixture
def setup(tmp_path, monkeypatch):
    root = tmp_path/'session'
    root.mkdir()
    cfg = options()
    states = {s: {'fresh': True, 'enabled': False, 'owns_control': False,
                   'mode': 'disabled', 'fault': None, 'faults': [], 'target_positions': [0]*20} for s in controls.SIDES}
    calls = []
    def request(path, op):
        side = 'left' if str(path).endswith('-left.sock') else 'right'
        return deepcopy(states[side])
    def operation(root_arg, op, *, side):
        assert root_arg == root
        calls.append((op, side))
        if op == 'prepare':
            assert arming.read(root)['armed']
            states[side].update(enabled=True, owns_control=True, mode='holding')
        return {'confirmed': True}
    def run(command, *, env, **kwargs):
        pinned = json.loads(Path(env['RAX_BACKEND_OPTIONS']).read_text())
        calls.append(('ready', pinned['xarm_ip']))
    monkeypatch.setattr(controls, 'request', request)
    monkeypatch.setattr(controls, 'operation', operation)
    monkeypatch.setattr(controls.subprocess, 'run', run)
    return SimpleNamespace(root=root, options=cfg, states=states, calls=calls)


def test_preparation_readies_both_pinned_ips_and_preserves_healthy_hold(setup):
    setup.states['right'].update(enabled=True, owns_control=True, mode='holding')
    controls.prepare(setup.root, setup.options, minutes=5, confirmed=True)
    assert setup.calls == [('ready', '192.0.2.1'), ('ready', '192.0.2.2'), ('prepare', 'left')]
    assert arming.read(setup.root)['armed']


def test_hand_fault_prevents_readiness_of_either_arm(setup):
    setup.states['left']['faults'] = [{'joint': 'middle_S1', 'name': 'Overcurrent',
                                      'code': 8450, 'severity': 'Fault', 'clear_policy': 'ManualClear'}]
    with pytest.raises(PreparationBlocked):
        controls.prepare(setup.root, setup.options, minutes=5, confirmed=True)
    assert not setup.calls
    assert not arming.read(setup.root)['armed']


def test_partial_preparation_failure_stops_both_arms_and_hands(setup, monkeypatch):
    original = controls.operation
    def fail_right(root, op, *, side):
        if op == 'prepare' and side == 'right':raise RuntimeError('right preparation failed')
        return original(root, op, side=side)
    arms = []
    def factory(cfg):
        arm = Arm()
        arms.append((cfg['xarm_ip'], arm))
        return arm
    stop = controls.stop
    monkeypatch.setattr(controls, 'operation', fail_right)
    monkeypatch.setattr(controls, 'stop', lambda root, opts: stop(root, opts, driver_factory=factory))
    with pytest.raises(RuntimeError, match='right preparation failed'):
        controls.prepare(setup.root, setup.options, minutes=5, confirmed=True)
    assert not arming.read(setup.root)['armed']
    assert {ip for ip, _ in arms} == {'192.0.2.1', '192.0.2.2'}
    assert all(arm.holds and arm.closed for _, arm in arms)
    assert ('emergency', 'left') in setup.calls and ('emergency', 'right') in setup.calls


def test_stop_attempts_other_devices_when_one_arm_is_unreachable(setup):
    arms = []
    def factory(cfg):
        if cfg['xarm_ip'] == '192.0.2.1':raise ConnectionError('left unavailable')
        arm = Arm()
        arms.append(arm)
        return arm
    arming.write(setup.root, True)
    result = controls.stop(setup.root, setup.options, driver_factory=factory)
    assert 'left unavailable' in result['left_arm']['error']
    assert result['right_arm']['sdk_returned']
    assert result['left_hand']['confirmed'] and result['right_hand']['confirmed']
    assert arms[0].holds and arms[0].closed and not arming.read(setup.root)['armed']


def test_status_reads_both_profiles_and_reports_partial_connectivity(setup, monkeypatch):
    monkeypatch.setattr(controls, 'recorded_status', lambda root, side:
                        {'fresh': True, 'serial': side, 'mode': 'disabled'})
    def factory(cfg):
        if cfg['xarm_ip'] == '192.0.2.1':raise ConnectionError('offline')
        return Arm()
    result = controls.status(setup.root, setup.options, driver_factory=factory)
    assert result['bimanual']
    assert not result['arms']['left']['arm']['ok']
    assert result['arms']['right']['arm']['ok']
    assert result['arms']['left']['hand']['serial'] == 'left'
    assert result['arms']['right']['hand']['serial'] == 'right'


def test_status_keeps_both_readings_if_one_disconnect_fails(setup, monkeypatch):
    class DisconnectErrorArm(Arm):
        def close(self):raise OSError('disconnect failed')
    result = controls.status(None, setup.options, driver_factory=lambda cfg:
                             DisconnectErrorArm() if cfg['xarm_ip'] == '192.0.2.1' else Arm())
    assert result['arms']['left']['arm']['close_error'] == 'disconnect failed'
    assert all(result['arms'][s]['arm']['ok'] for s in controls.SIDES)
