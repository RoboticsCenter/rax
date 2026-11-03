"""Two actual Unix IPC owners share arming, but not commands or snapshots."""
import multiprocessing
import threading
import time

from wuji_fakes import FakeHand, hand_config
from services.wuji.client import operation, request, recorded_status
from services.wuji.controller import serve
from services.xarm import arming


def run_owner(cfg, root, path, side):
    serve(cfg, root, path, side=side, driver=FakeHand(cfg, continuous=True))


def until(check, seconds=4):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        try:
            value = check()
            if value:return value
        except (OSError, RuntimeError):
            pass
        time.sleep(.02)
    raise AssertionError('fake hand owners did not reach the expected state')


def test_two_owners_route_independently_and_both_obey_one_disarm(tmp_path, monkeypatch):
    # Keep actual sockets short enough for AF_UNIX even with long pytest paths.
    import tempfile
    context = multiprocessing.get_context('fork')
    processes, threads = [], []
    stop = threading.Event()
    with tempfile.TemporaryDirectory(prefix='rax-pair-test-') as directory:
        from pathlib import Path
        paths = {side: Path(directory)/f'{side}.sock' for side in ('left', 'right')}
        monkeypatch.setattr('services.wuji.client.socket_path', lambda root, side=None: paths[side])
        try:
            for side in paths:
                cfg = hand_config(serial=f'FAKE-{side}', handedness=side, max_scheduler_gap_s=.2)
                process = context.Process(target=run_owner, args=(cfg, tmp_path, paths[side], side))
                process.start()
                processes.append(process)
            for path in paths.values():
                until(lambda path=path: request(path, 'status').get('fresh'))
                def heartbeat(path=path):
                    while not stop.is_set():
                        try:request(path, 'heartbeat')
                        except OSError:return
                        stop.wait(.03)
                thread = threading.Thread(target=heartbeat, daemon=True)
                thread.start()
                threads.append(thread)
            arming.write(tmp_path, True)
            for side in paths:
                assert operation(tmp_path, 'prepare', side=side)['outcome'] == 'prepared'
            operation(tmp_path, 'move', {'positions': [.05]*20}, side='left')
            operation(tmp_path, 'move', {'positions': [-.05]*20}, side='right')
            left = until(lambda: (s if (s := recorded_status(tmp_path, side='left'))
                                   .get('target_positions') == [.05]*20 else None))
            right = until(lambda: (s if (s := recorded_status(tmp_path, side='right'))
                                    .get('target_positions') == [-.05]*20 else None))
            assert left['serial'] == 'FAKE-left' and right['serial'] == 'FAKE-right'
            arming.write(tmp_path, False)
            for path in paths.values():
                until(lambda path=path: request(path, 'status')['mode'] in ('disabled', 'faulted')
                      and not request(path, 'status')['enabled'])
                assert not request(path, 'status')['enabled']
        finally:
            stop.set()
            for thread in threads:thread.join(1)
            for process in processes:
                process.terminate()
                process.join(3)
                if process.is_alive():
                    process.kill()
                    process.join(1)
