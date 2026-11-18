"""Smooth layer through the real hand-owner process and Unix IPC, with a fake driver only."""
import importlib.util
import json
import multiprocessing
from pathlib import Path
import threading
import time

import pytest
from wuji_fakes import FakeHand, hand_config

from services.wuji import client
from services.wuji.config import JOINT_NAMES
from services.wuji.controller import serve
from services.wuji.trace import load_trial
from services.xarm import arming

I = JOINT_NAMES.index('index_S1')
HOME = [.1, .2, 0., .3]+[.05]*16


def child(config, root, path):
    serve(config, root, path, driver=FakeHand(config, continuous=True))


@pytest.fixture
def service(tmp_path, monkeypatch):
    context = multiprocessing.get_context('fork')
    path = tmp_path/'hand.sock'
    cfg = hand_config(parent_timeout_s=.5, max_scheduler_gap_s=.25, stop_timeout_s=.5, motion_mode='scurve',
                      command_hz=1000, max_velocity_rad_s=.6, max_acceleration_rad_s2=3., max_jerk_rad_s3=2.,
                      stream_max_acceleration_rad_s2=6., stream_max_jerk_rad_s3=40., max_step_rad=2.1,
                      current_limit_a=.8, kp=8., kd=.15, settle_timeout_s=3., tolerance_rad=.01,
                      postures={'home': HOME})
    process = context.Process(target=child, args=(cfg, tmp_path, path))
    process.start()
    stop = threading.Event()
    heartbeat = None
    try:
        deadline = time.monotonic()+5
        while not path.exists():
            assert time.monotonic() < deadline
            time.sleep(.01)
        monkeypatch.setattr(client, 'socket_path', lambda root, side=None: path)

        def keep_alive():
            while not stop.is_set():
                try:
                    client.request(path, 'heartbeat')
                except OSError:
                    return
                stop.wait(.03)
        heartbeat = threading.Thread(target=keep_alive, daemon=True)
        heartbeat.start()
        arming.write(tmp_path, True)
        assert client.operation(tmp_path, 'prepare')['outcome'] == 'prepared'
        yield tmp_path, path, cfg
    finally:
        stop.set()
        if heartbeat:
            heartbeat.join(1)
        process.terminate()
        process.join(3)
        if process.is_alive():
            process.kill()
            process.join(1)


def test_stream_trajectory_profile_and_trace_over_ipc(service):
    root, path, cfg = service
    started = client.direct(root, 'record', {'action': 'start', 'directory': str(root/'trial'), 'controller': 'C2'})
    assert started['nominal_rate_hz'] == 1000
    with pytest.raises(ValueError, match='outside configured limits'):
        client.direct(root, 'stream', {'joints': {'index_S1': 3.}})
    for k in range(25):  # 50 Hz policy stream
        value = client.direct(root, 'stream', {'joints': {'index_S1': .008*k}})
        assert value['accepted']
        time.sleep(.02)
    result = client.wait_settled(root, timeout_s=3.)
    assert result['settled'] and result['target_positions'][I] == pytest.approx(.192)
    accepted = client.direct(root, 'trajectory', {'waypoints': [{'t': .3, 'joints': {'index_S1': .3}},
                                                                {'t': .6, 'joints': {'index_S1': .1}}]})
    result = client.wait_settled(root, timeout_s=5., after=accepted['sequence'])
    assert result['settled'] and result['positions'][I] == pytest.approx(.1, abs=.01)
    saved = client.record_stop(root)
    assert saved['saved']['samples'] > 300 and saved['saved']['observed_hz'] > 200
    with __import__('numpy').load(root/'trial'/'hand_right.npz') as data:
        assert data['q_ref'].shape[1] == 20 and data['t'].shape[0] == saved['saved']['samples']
        assert data['q_ref'][:, I].max() <= .3+1e-9
    assert client.operation(root, 'set_profile', {'name': 'soft'})['outcome'] == 'profile_set'
    status = client.request(path, 'status')
    assert status['smooth_profile']['name'] == 'soft' and status['command_timing']['runtime_hz'] == 1000
    assert client.operation(root, 'hold')['outcome'] == 'held'
    arming.write(root, False)
    deadline = time.monotonic()+3
    while client.request(path, 'status')['mode'] not in ('disabled',) and time.monotonic() < deadline:
        time.sleep(.02)
    assert client.request(path, 'status')['mode'] == 'disabled'
    with pytest.raises(RuntimeError):
        client.direct(root, 'stream', {'joints': {'index_S1': .1}})


def load_battery():
    path = Path(__file__).resolve().parents[2]/'scripts/wuji_motion_battery.py'
    spec = importlib.util.spec_from_file_location('wuji_motion_battery', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_battery_runner_writes_one_trial_per_condition(service, tmp_path):
    root, path, cfg = service
    battery = load_battery()
    link = battery.HandLink(root, 'right', cfg, socket_side=None)
    target = list(HOME)
    target[I] += .15
    frames = [(0., list(HOME)), (.4, target), (.6, target), (1., list(HOME))]
    plan = {'trials': [{'hand': 'right', 'condition': name, 'item': 'single_joint', 'variant': 'index_S1',
                        'repeat': 0, 'params': {'joint': 'index_S1'}, 'keyframes': frames, 'continuous': False,
                        'summary': {}} for name in ('M0_baseline', 'M1_stream_1khz', 'M3_soft_gains_cap',
                                                    'M4_online_replan', 'M4t_trajectory')]}
    runner = battery.Runner({'right': link}, tmp_path/'out', stream_hz=30., rest_s=0., log=lambda *_: None)
    results = runner.run(plan)
    assert [r["error"] for r in results] == [None]*5 and all(r["settled"] for r in results), results
    for result in results:
        trial = load_trial(tmp_path/'out'/'trials'/result['trial'])
        meta, hand = trial['meta'], trial['hands']['right']
        assert meta['controller'] == result['condition'] and meta['skill'] == 'single_joint'
        assert meta['success'] is True and meta['backend'] == 'mock'
        assert hand['q_ref'][:, I].max() == pytest.approx(HOME[I]+.15, abs=.01)
        assert hand['t'].shape[0] > 100
    rates = {r['condition']: load_trial(tmp_path/'out'/'trials'/r['trial'])['meta']['nominal_rate_hz']
             for r in results}
    assert rates['M0_baseline'] == 200 and rates['M1_stream_1khz'] == 1000
    m3 = load_trial(tmp_path/'out'/'trials'/results[2]['trial'])
    assert m3['hands']['right']['kp'][0, 0] == pytest.approx(10/1.98)  # default preset, ~5/0.1 nominal
    assert m3['meta']['current_limit_a'] == .8  # 1.0 A requested, clamped to the 0.8 A operator ceiling.
    assert float(m3['hands']['right']['current_limit_a'][0, 0]) == pytest.approx(.8)
    from services.dex_eval.trace import validate_trial_dir
    for result in results:
        errors, warnings = validate_trial_dir(tmp_path/'out'/'trials'/result['trial'])
        assert not errors, (errors, warnings)  # The evaluation harness accepts the trial.
    assert json.loads((tmp_path/'out'/'trials'/results[0]['trial']/'meta.json').read_text())['params']['condition'][
        'delivery'] == 'blocking'
