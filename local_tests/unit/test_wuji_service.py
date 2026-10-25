"""Actual process and Unix IPC, using a fake driver exclusively."""
import multiprocessing
import threading
import time

import pytest
from wuji_fakes import FakeHand, hand_config

from services.wuji.client import operation, request
from services.wuji.controller import serve
from services.xarm import arming


def child(config, root, path, events, block, entered, release):
    class RecordingHand(FakeHand):
        def send(self, q):
            super().send(q)
            events.put(('send', q))
            if block.is_set():
                entered.set()
                release.wait(5)

        def stop(self, behavior='emergency'):
            super().stop(behavior)
            events.put((behavior,))

    serve(config, root, path, driver=RecordingHand(config, continuous=True))


def until(check, timeout=3):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if value := check():
            return value
        time.sleep(.01)
    raise AssertionError('timed out awaiting fake service')


@pytest.fixture
def service(tmp_path, monkeypatch):
    context = multiprocessing.get_context('fork')
    events = context.Queue()
    block, entered, release = context.Event(), context.Event(), context.Event()
    path = tmp_path/'hand.sock'
    cfg = hand_config(parent_timeout_s=.3, max_scheduler_gap_s=.2, stop_timeout_s=.3)
    process = context.Process(target=child, args=(cfg, tmp_path, path, events, block, entered, release))
    process.start()
    stop_heartbeat = threading.Event()
    heartbeat = None
    try:
        until(lambda: path.exists())
        until(lambda: request(path, 'status').get('fresh'))
        monkeypatch.setattr('services.wuji.client.socket_path', lambda root: path)
        def keep_alive():
            while not stop_heartbeat.is_set():
                try:
                    request(path, 'heartbeat')
                except OSError:
                    return
                stop_heartbeat.wait(.03)
        heartbeat = threading.Thread(target=keep_alive, daemon=True)
        heartbeat.start()
        yield tmp_path, path, events, stop_heartbeat, block, entered, release
    finally:
        release.set()
        stop_heartbeat.set()
        if heartbeat:
            heartbeat.join(1)
        process.terminate()
        process.join(3)
        if process.is_alive():
            process.kill()
            process.join(1)
        events.close()


def test_process_moves_holds_and_stops_on_disarm(service):
    root, path, events, _, _, _, _ = service
    assert events.empty()  # Discovery/subscriptions did not write motor commands.
    arming.write(root, True)
    assert operation(root, 'prepare')['outcome'] == 'prepared'
    result = operation(root, 'move', {'positions': [.1]*20})
    assert result['reached']
    assert request(path, 'status')['target_positions'] == pytest.approx([.1]*20)
    arming.write(root, False)
    until(lambda: (s := request(path, 'status'))['mode'] in ('faulted', 'disabled') and not s['enabled'])
    assert not request(path, 'status')['enabled']
    assert (root/'operator'/'wuji-state.json').is_file()


def test_process_watchdog_does_not_need_an_agent_tool_call(service):
    root, path, _, heartbeat_stop, _, _, _ = service
    arming.write(root, True)
    operation(root, 'prepare')
    heartbeat_stop.set()
    until(lambda: request(path, 'status')['mode'] == 'faulted')
    assert 'heartbeat' in request(path, 'status')['fault']


def test_status_and_priority_stop_remain_available_during_native_call(service):
    root, path, _, _, block, entered, release = service
    arming.write(root, True)
    operation(root, 'prepare')
    block.set()
    assert entered.wait(1)
    before = time.monotonic()
    request(path, 'emergency', id='operator-stop')
    assert time.monotonic()-before < .5
    assert not request(path, 'result', id='operator-stop')['done']
    time.sleep(.25)
    assert not request(path, 'status')['fresh']
    block.clear()
    release.set()
    result = until(lambda: (r if (r := request(path, 'result', id='operator-stop'))['done'] else None))
    assert result['confirmed']
    assert request(path, 'status')['mode'] in ('faulted', 'disabled')
