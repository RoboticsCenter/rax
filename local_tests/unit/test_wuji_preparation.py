"""Explicit thermal recovery uses fake owner IPC and never commands movement."""
from copy import deepcopy
import json
import math
from types import SimpleNamespace

import pytest

from services.wuji import preparation
from services.wuji.preparation import PreparationBlocked, preparation_action
from services.xarm import arming


def thermal_state(temperature=64.9):
    return {'fresh': True, 'enabled': False, 'owns_control': False, 'mode': 'faulted',
            'fault': 'fresh fault-free positions and diagnostics required',
            'faults': [{'joint': 'thumb_S2', 'reason': 'temperature limit exceeded', 'temperature_c': 65.005}],
            'velocity_rad_s': [0.]*20,
            'diagnostics': {'mcu_temp_c_fb': [temperature]*20, 'error_code_current': [0]*20,
                            'state_names': ['Ready']*20},
            'diagnostics_policy': {'max_temperature_c': 65.}}


@pytest.mark.parametrize('temperature', [59., 60.6, 64.999])
def test_below_cutoff_can_clear_retained_thermal_fault_without_extra_margin(temperature):
    assert preparation_action(thermal_state(temperature), {'max_temperature_c': 65.}) == 'reconnect'


@pytest.mark.parametrize('temperature', [65., 65.005, 70.])
def test_at_or_above_cutoff_shows_actual_temperature_and_blocks(temperature):
    with pytest.raises(PreparationBlocked, match='below 65 C') as exc:
        preparation_action(thermal_state(temperature), {})
    assert f'{temperature:.1f} C' in str(exc.value)


@pytest.mark.parametrize('change', [
    {'fresh': False}, {'enabled': True}, {'owns_control': True}, {'mode': 'executing'},
    {'velocity_rad_s': [.06]*20}, {'velocity_rad_s': [math.nan]*20},
    {'faults': [{'joint': 'thumb_S2', 'code': 3, 'name': 'Enc1BitRate'}]},
])
def test_unsafe_or_nonthermal_state_does_not_reconnect(change):
    state = {**thermal_state(), **change}
    with pytest.raises(PreparationBlocked):
        preparation_action(state, {})


@pytest.mark.parametrize('change', [
    {'error_code_current': [0]*19+[3]}, {'error_code_current': [0]*19},
    {'state_names': ['Enabled']*20}, {'mcu_temp_c_fb': [math.nan]*20},
    {'mcu_temp_c_fb': [math.inf]*20}, {'mcu_temp_c_fb': [True]*20},
])
def test_partial_or_unhealthy_diagnostics_are_not_cleared(change):
    state = thermal_state()
    state['diagnostics'].update(change)
    with pytest.raises(PreparationBlocked):
        preparation_action(state, {})


def test_mixed_faults_are_not_cleared_as_thermal_recovery():
    state = thermal_state()
    state['faults'].append({'joint': 'index_S1', 'code': 9, 'name': 'motor error'})
    with pytest.raises(PreparationBlocked, match='inspection'):
        preparation_action(state, {})


@pytest.mark.parametrize('limit', [None, math.nan, math.inf, True, 100])
def test_invalid_temperature_policy_is_not_used_for_recovery(limit):
    with pytest.raises(PreparationBlocked, match='temperature limit'):
        preparation_action(thermal_state(), {'max_temperature_c': limit})


def test_lower_active_owner_limit_wins_over_changed_config():
    state = thermal_state(61.)
    state['diagnostics_policy']['max_temperature_c'] = 60.
    with pytest.raises(PreparationBlocked, match='below 60 C'):
        preparation_action(state, {'max_temperature_c': 65.})


@pytest.fixture
def recovery(tmp_path, monkeypatch):
    root = tmp_path/'session'
    root.mkdir()
    (root/'operator-input.json').write_text(json.dumps({'backend_options': {
        'end_effector': {'max_temperature_c': 65.}}}))
    arming.write(root, False)
    case = SimpleNamespace(root=root, state=thermal_state(), operations=[], reads=0, now=100., after=lambda: None)

    def read(path, op):
        assert op == 'status'
        case.reads += 1
        return deepcopy(case.state)

    def operate(root, op):
        case.operations.append(op)
        assert not arming.read(root)['armed'] and not case.state['enabled']
        case.state.update(faults=[], fault=None, mode='disabled')
        case.after()
        return {'outcome': 'connected', 'ok': True}

    def sleep(seconds):
        case.now += seconds

    monkeypatch.setattr(preparation, 'request', read)
    monkeypatch.setattr(preparation, 'operation', operate)
    monkeypatch.setattr(preparation, 'time', SimpleNamespace(monotonic=lambda: case.now, sleep=sleep,
                                                           time=lambda: case.now, time_ns=lambda: 123456))
    return case


def test_one_reconnect_preserves_evidence_and_never_arms_or_moves(recovery):
    result = preparation.recover_cooled_thermal(recovery.root)
    assert result['outcome'] == 'thermal_reconnected'
    assert recovery.operations == ['connect']
    assert recovery.now >= 102 and recovery.reads > 10
    assert not arming.read(recovery.root)['armed'] and not recovery.state['enabled']
    record = json.loads((recovery.root/'operator/thermal-recovery-123456.json').read_text())
    assert record['before']['faults'][0]['temperature_c'] == 65.005
    assert record['after']['faults'] == []


def test_armed_session_is_not_reconnected(recovery):
    arming.write(recovery.root, True)
    with pytest.raises(PreparationBlocked, match='Disarm'):
        preparation.recover_cooled_thermal(recovery.root)
    assert recovery.operations == []


def test_temperature_rebound_during_confirmation_blocks_reconnect(recovery, monkeypatch):
    original = preparation.request

    def read(*args):
        if recovery.reads >= 3:
            recovery.state['diagnostics']['mcu_temp_c_fb'][1] = 65.
        return original(*args)

    monkeypatch.setattr(preparation, 'request', read)
    with pytest.raises(PreparationBlocked, match='cooling'):
        preparation.recover_cooled_thermal(recovery.root)
    assert recovery.operations == []


def test_fault_after_reconnect_is_not_retried(recovery):
    recovery.after = lambda: recovery.state.update(faults=thermal_state()['faults'])
    with pytest.raises(PreparationBlocked, match='fault-free'):
        preparation.recover_cooled_thermal(recovery.root)
    assert recovery.operations == ['connect']
    assert not arming.read(recovery.root)['armed']


def test_missing_feedback_after_reconnect_is_not_retried(recovery):
    recovery.after = lambda: recovery.state.update(fresh=False)
    with pytest.raises(PreparationBlocked, match='Fresh diagnostics'):
        preparation.recover_cooled_thermal(recovery.root)
    assert recovery.operations == ['connect']
