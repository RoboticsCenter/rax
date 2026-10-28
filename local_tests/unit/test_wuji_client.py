"""Parent supervision without a vendor process or device connection."""
import threading
import time
from copy import deepcopy

import pytest

from wuji_fakes import hand_config

from services.wuji.client import WujiClient, operation


def client():
    value = WujiClient.__new__(WujiClient)
    value.cfg = hand_config()
    value.path = '/unused-fake-socket'
    value.stop_event = threading.Event()
    value.received = time.monotonic()
    value.cached = {'fresh': False, 'enabled': False, 'mode': 'starting'}
    return value


def supervise(monkeypatch, snapshots):
    value = client()
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
