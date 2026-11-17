"""Parent supervision without a vendor process or device connection."""
import threading
import time
from copy import deepcopy
from pathlib import Path

import pytest

from wuji_fakes import hand_config

from services.wuji.client import WujiClient, operation


def test_scurve_client_wait_includes_stretched_execution_and_settling(monkeypatch):
    value = WujiClient.__new__(WujiClient)
    value.cfg = hand_config(motion_mode='scurve', reference_timeout_scale=3.)
    captured = {}
    def operation(op, args, **kwargs):
        captured.update(kwargs)
        return {'ok': True}
    monkeypatch.setattr(value, '_operation', operation)
    value.move([0.]*20)
    assert captured['timeout'] == 3*value.cfg['max_duration_s']+value.cfg['settle_timeout_s']+2


def client():
    value = WujiClient.__new__(WujiClient)
    value.cfg = hand_config()
    value.path = '/unused-fake-socket'
    value.root = Path('/unused-fake-session')
    value.stop_event = threading.Event()
    value.received = time.monotonic()
    value.cached = {'fresh': False, 'enabled': False, 'mode': 'starting'}
    return value


def supervise(monkeypatch, snapshots, *, config=None):
    value = client()
    if config is not None:
        value.cfg = config
    arm_stops, hand_stops = [], []
    value.on_fault = lambda: arm_stops.append(True)
    states = iter(snapshots)
    def request(path, op, **kwargs):
        if op == 'emergency':
            hand_stops.append(kwargs['id'])
            return {}
        state = next(states, None)
        if state is None:
            value.stop_event.set()
            return value.cached
        if isinstance(state, Exception):
            raise state
        return deepcopy(state)
    monkeypatch.setattr('services.wuji.client.request', request)
    value._monitor()
    return value, arm_stops, hand_stops


def passive_init_after_stop():
    names = ['Ready'] * 20
    names[2] = 'Init'
    return {'mode': 'disabled', 'fresh': True, 'owns_control': False, 'fault': None,
            'diagnostics': {'state_names': names},
            'faults': [{'joint': 'thumb_S3', 'native_state': 'Init',
                        'reason': 'excluded joint is not disabled'}]}


def test_confirmed_disarmed_stop_does_not_alert_for_passive_init(monkeypatch):
    monkeypatch.setattr('services.wuji.client.arming.read', lambda root: {'armed': False})
    healthy = {'mode': 'holding', 'fresh': True, 'owns_control': True}
    value, arm, hand = supervise(monkeypatch, [healthy, passive_init_after_stop()],
                                config={'disabled_joints': ['thumb_S3']})
    assert not arm and not hand
    assert value.cached['faults']  # Still visible and still blocks preparation.


def pending_passive_stop():
    return {**passive_init_after_stop(), 'mode': 'stopping', 'owns_control': True,
            'diagnostics': {'state_names': ['Init'] * 20},
            'stop': {'op': 'emergency', 'behavior': 'emergency', 'since': 99., 'deadline': 101.}}


def test_disarmed_passive_init_during_bounded_stop_does_not_alert(monkeypatch):
    monkeypatch.setattr('services.wuji.client.arming.read', lambda root: {'armed': False})
    monkeypatch.setattr('services.wuji.client.time.monotonic', lambda: 100.)
    _, arm, hand = supervise(monkeypatch, [
        {'mode': 'holding', 'fresh': True, 'owns_control': True},
        pending_passive_stop(), passive_init_after_stop()],
        config={'disabled_joints': ['thumb_S3'], 'stop_timeout_s': 2.})
    assert not arm and not hand


def test_passive_pending_stop_still_alerts_while_armed(monkeypatch):
    monkeypatch.setattr('services.wuji.client.arming.read', lambda root: {'armed': True})
    monkeypatch.setattr('services.wuji.client.time.monotonic', lambda: 100.)
    _, arm, _ = supervise(monkeypatch, [
        {'mode': 'holding', 'fresh': True, 'owns_control': True}, pending_passive_stop()],
        config={'disabled_joints': ['thumb_S3'], 'stop_timeout_s': 2.})
    assert len(arm) == 1


@pytest.mark.parametrize('change', [
    {'fresh': False}, {'fault': 'stop delivery failed'}, {'mode': 'uncertain'},
    {'faults': [{'joint': 'thumb_S3', 'reason': 'temperature limit exceeded'}]},
    {'stop': None},
    {'stop': {'op': 'emergency', 'behavior': 'emergency', 'since': 97., 'deadline': 99.}},
    {'stop': {'op': 'emergency', 'behavior': 'emergency', 'since': 99., 'deadline': 110.}},
])
def test_pending_stop_failures_still_alert(monkeypatch, change):
    monkeypatch.setattr('services.wuji.client.arming.read', lambda root: {'armed': False})
    monkeypatch.setattr('services.wuji.client.time.monotonic', lambda: 100.)
    _, arm, _ = supervise(monkeypatch, [
        {'mode': 'holding', 'fresh': True, 'owns_control': True},
        {**pending_passive_stop(), **change}],
        config={'disabled_joints': ['thumb_S3'], 'stop_timeout_s': 2.})
    assert len(arm) == 1


@pytest.mark.parametrize('change', [
    {'fresh': False}, {'owns_control': True}, {'mode': 'stopping'},
    {'fault': 'tracking error'},
    {'faults': [{'joint': 'thumb_S3', 'reason': 'temperature limit exceeded'}]},
    {'faults': [{'joint': 'index_S1', 'native_state': 'Init',
                'reason': 'excluded joint is not disabled'}]},
    {'diagnostics': {'state_names': ['Enabled'] * 20}},
])
def test_real_or_unconfirmed_stop_fault_still_alerts(monkeypatch, change):
    monkeypatch.setattr('services.wuji.client.arming.read', lambda root: {'armed': False})
    healthy = {'mode': 'holding', 'fresh': True, 'owns_control': True}
    _, arm, _ = supervise(monkeypatch, [healthy, {**passive_init_after_stop(), **change}],
                          config={'disabled_joints': ['thumb_S3']})
    assert len(arm) == 1


def test_passive_init_is_not_expected_stop_while_armed(monkeypatch):
    monkeypatch.setattr('services.wuji.client.arming.read', lambda root: {'armed': True})
    _, arm, _ = supervise(monkeypatch, [
        {'mode': 'holding', 'fresh': True, 'owns_control': True}, passive_init_after_stop()],
        config={'disabled_joints': ['thumb_S3']})
    assert len(arm) == 1


def test_missing_hand_during_initial_connection_does_not_stop_arm(monkeypatch):
    _, arm, hand = supervise(monkeypatch, [
        {'mode': 'disconnected', 'fresh': False, 'owns_control': False},
        ConnectionError('service unavailable'),
    ])
    assert not arm and not hand


def test_owned_hand_loss_stops_arm_once_until_explicit_repreparation(monkeypatch):
    healthy = {'mode': 'holding', 'fresh': True, 'enabled': True, 'owns_control': True}
    lost = {'mode': 'uncertain', 'fresh': False, 'enabled': False, 'owns_control': True}
    _, arm, hand = supervise(monkeypatch, [healthy, lost, lost, healthy, ConnectionError('child lost')])
    assert len(arm) == len(hand) == 2


def test_parent_snapshot_ages_both_feedback_streams():
    value = client()
    value.cached = {'fresh': True, 'enabled': True, 'age_s': {'state': .01, 'diagnostics': .2}}
    value.received = time.monotonic()-.1
    status = value.status()
    assert not status['fresh'] and not status['enabled']
    assert status['age_s']['diagnostics'] >= .3
    assert value.cached['fresh']  # Reading aged data must not mutate the source snapshot.


def test_next_move_admission_reads_owner_after_previous_move_completed(monkeypatch):
    value = client()
    value.cached = {'fresh': True, 'enabled': True, 'mode': 'executing'}
    calls = []

    def request(path, op):
        calls.append((path, op))
        return {'fresh': True, 'enabled': True, 'mode': 'holding', 'fault': None}

    monkeypatch.setattr('services.wuji.client.request', request)
    assert value.require_ready()['mode'] == 'holding'
    assert calls == [(value.path, 'status')]
    assert value.cached['mode'] == 'executing'  # Do not race the supervisor's cache writes.


@pytest.mark.parametrize('changed', [
    {'mode': 'executing'}, {'mode': 'disabled'}, {'fresh': False},
    {'enabled': False}, {'fault': 'tracking error'},
])
def test_admission_does_not_trust_healthy_parent_cache(monkeypatch, changed):
    value = client()
    value.cached = {'fresh': True, 'enabled': True, 'mode': 'holding', 'fault': None}
    monkeypatch.setattr('services.wuji.client.request', lambda *a: {**value.cached, **changed})
    with pytest.raises(RuntimeError, match='not prepared and holding'):
        value.require_ready()


def test_admission_refuses_unreachable_owner_despite_healthy_cache(monkeypatch):
    value = client()
    value.cached = {'fresh': True, 'enabled': True, 'mode': 'holding', 'fault': None}

    def unavailable(*args):
        raise ConnectionError('owner unavailable')

    monkeypatch.setattr('services.wuji.client.request', unavailable)
    with pytest.raises(ConnectionError, match='owner unavailable'):
        value.require_ready()


def test_delayed_result_is_polled_without_resending_the_hardware_operation(monkeypatch):
    calls = []
    def request(path, op, **kwargs):
        calls.append((op, kwargs['id']))
        if op == 'connect':
            return {'command_id': kwargs['id']}
        if len(calls) == 2:
            raise TimeoutError('native SDK delayed the IPC reply')
        return {'done': True, 'ok': True, 'outcome': 'connected'}
    monkeypatch.setattr('services.wuji.client.request', request)
    assert operation('/unused-session', 'connect', timeout=1)['outcome'] == 'connected'
    assert [op for op, _ in calls] == ['connect', 'result', 'result']
    assert len({ident for _, ident in calls}) == 1
