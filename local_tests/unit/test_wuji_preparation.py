"""Explicit retained-fault recovery uses fake owner IPC and never commands movement."""
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


def stall_state(temperature=58.):
    state = thermal_state(temperature)
    state['faults'] = [{'joint': 'thumb_S2', 'code': 7, 'name': 'Stall',
                        'severity': 'Warning', 'clear_policy': 'AutoClear'}]
    return state


def test_cleared_device_stall_can_recover_retained_software_fault():
    assert preparation_action(stall_state(), {}) == 'reconnect-stall'


@pytest.mark.parametrize('change', [
    {'code': 3}, {'code': 7.0}, {'name': 'Other'}, {'joint': 'unknown'},
    {'severity': 'Fault'}, {'severity': None}, {'clear_policy': 'ManualClear'}, {'clear_policy': None},
])
def test_stall_recovery_requires_exact_native_fault_metadata(change):
    state = stall_state()
    state['faults'][0].update(change)
    with pytest.raises(PreparationBlocked, match='inspection'):
        preparation_action(state, {})


def test_cleared_stall_and_cooled_thermal_faults_can_recover_together():
    state = stall_state()
    state['faults'] += thermal_state()['faults']
    assert preparation_action(state, {}) == 'reconnect-stall'


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
@pytest.mark.parametrize('state_factory', [thermal_state, stall_state])
def test_unsafe_or_unrecognized_state_does_not_reconnect(change, state_factory):
    state = {**state_factory(), **change}
    with pytest.raises(PreparationBlocked):
        preparation_action(state, {})


@pytest.mark.parametrize('change', [
    {'error_code_current': [0, 7]+[0]*18}, {'error_code_current': [0]*19+[3]},
    {'error_code_current': [0]*19},
    {'state_names': ['Enabled']*20}, {'mcu_temp_c_fb': [math.nan]*20},
    {'mcu_temp_c_fb': [math.inf]*20}, {'mcu_temp_c_fb': [True]*20},
])
@pytest.mark.parametrize('state_factory', [thermal_state, stall_state])
def test_partial_or_unhealthy_diagnostics_are_not_cleared(change, state_factory):
    state = state_factory()
    state['diagnostics'].update(change)
    with pytest.raises(PreparationBlocked):
        preparation_action(state, {})


@pytest.mark.parametrize('state_factory', [thermal_state, stall_state])
def test_unrelated_faults_are_not_cleared_during_recovery(state_factory):
    state = state_factory()
    state['faults'].append({'joint': 'index_S1', 'code': 9, 'name': 'motor error'})
    with pytest.raises(PreparationBlocked, match='inspection'):
        preparation_action(state, {})


@pytest.mark.parametrize('limit', [None, math.nan, math.inf, True, 100])
def test_invalid_temperature_policy_is_not_used_for_recovery(limit):
    with pytest.raises(PreparationBlocked, match='temperature limit'):
        preparation_action(thermal_state(), {'max_temperature_c': limit})


@pytest.mark.parametrize('state_factory', [thermal_state, stall_state])
def test_lower_active_owner_limit_wins_over_changed_config(state_factory):
    state = state_factory(61.)
    state['diagnostics_policy']['max_temperature_c'] = 60.
    with pytest.raises(PreparationBlocked, match='below 60 C'):
        preparation_action(state, {'max_temperature_c': 65.})


@pytest.fixture(params=['thermal', 'stall'])
def recovery(tmp_path, monkeypatch, request):
    root = tmp_path/'session'
    root.mkdir()
    (root/'operator-input.json').write_text(json.dumps({'backend_options': {
        'end_effector': {'max_temperature_c': 65.}}}))
    arming.write(root, False)
    kind = request.param
    case = SimpleNamespace(root=root, state=thermal_state() if kind == 'thermal' else stall_state(),
                           kind=kind, recover=(preparation.recover_cooled_thermal if kind == 'thermal'
                                               else preparation.recover_cleared_stall),
                           operations=[], reads=0, now=100., after=lambda: None)

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
    before_faults = deepcopy(recovery.state['faults'])
    result = recovery.recover(recovery.root)
    assert result['outcome'] == f'{recovery.kind}_reconnected'
    assert recovery.operations == ['connect']
    assert recovery.now >= 102 and recovery.reads > 10
    assert not arming.read(recovery.root)['armed'] and not recovery.state['enabled']
    record = json.loads((recovery.root/f'operator/{recovery.kind}-recovery-123456.json').read_text())
    assert record['before']['faults'] == before_faults
    assert record['after']['faults'] == []


def test_armed_session_is_not_reconnected(recovery):
    arming.write(recovery.root, True)
    with pytest.raises(PreparationBlocked, match='Disarm'):
        recovery.recover(recovery.root)
    assert recovery.operations == []


def test_temperature_rebound_during_confirmation_blocks_reconnect(recovery, monkeypatch):
    original = preparation.request

    def read(*args):
        if recovery.reads >= 3:
            recovery.state['diagnostics']['mcu_temp_c_fb'][1] = 65.
        return original(*args)

    monkeypatch.setattr(preparation, 'request', read)
    with pytest.raises(PreparationBlocked, match='cooling'):
        recovery.recover(recovery.root)
    assert recovery.operations == []


def test_fault_after_reconnect_is_not_retried(recovery):
    recovery.after = lambda: recovery.state.update(faults=thermal_state()['faults'])
    with pytest.raises(PreparationBlocked, match='fault-free'):
        recovery.recover(recovery.root)
    assert recovery.operations == ['connect']
    assert not arming.read(recovery.root)['armed']


def test_missing_feedback_after_reconnect_is_not_retried(recovery):
    recovery.after = lambda: recovery.state.update(fresh=False)
    with pytest.raises(PreparationBlocked, match='Fresh diagnostics'):
        recovery.recover(recovery.root)
    assert recovery.operations == ['connect']


def test_device_fault_returning_during_checks_prevents_reconnect(recovery, monkeypatch):
    original = preparation.request

    def read(*args):
        if recovery.reads >= 3:
            recovery.state['diagnostics']['error_code_current'][1] = 7
        return original(*args)

    monkeypatch.setattr(preparation, 'request', read)
    with pytest.raises(PreparationBlocked, match='zero native error codes'):
        recovery.recover(recovery.root)
    assert recovery.operations == []


@pytest.mark.parametrize('change', [
    {'diagnostics': {'error_code_current': [0, 7]+[0]*18}},
    {'diagnostics': {'mcu_temp_c_fb': [66.]*20}},
    {'diagnostics': {'state_names': ['Enabled']*20}},
    {'velocity_rad_s': [.06]*20}, {'owns_control': True}, {'enabled': True},
])
def test_unhealthy_feedback_after_reconnect_is_not_accepted_or_retried(recovery, change):
    def after():
        for key, value in change.items():
            if key == 'diagnostics':
                recovery.state[key].update(value)
            else:
                recovery.state[key] = value
    recovery.after = after
    with pytest.raises(PreparationBlocked):
        recovery.recover(recovery.root)
    assert recovery.operations == ['connect']
    record = json.loads((recovery.root/f'operator/{recovery.kind}-recovery-123456.json').read_text())
    assert record['error'] and record['before']['faults']


def test_thermal_command_cannot_clear_a_stall_fault(recovery):
    recovery.state = stall_state()
    with pytest.raises(PreparationBlocked, match='No recoverable thermal fault'):
        preparation.recover_cooled_thermal(recovery.root)
    assert recovery.operations == []
