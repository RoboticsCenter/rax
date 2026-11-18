"""Streaming inside the single-writer controller: fake hand, real Ruckig, simulated clock."""
import math
import time

import pytest
from test_wuji_controller import Rig

from services.wuji.config import JOINT_NAMES
from services.wuji.trace import TrialRecorder, load_trial

I = JOINT_NAMES.index('index_S1')
PROFILE = dict(motion_mode='scurve', command_hz=1000, max_velocity_rad_s=.6, max_acceleration_rad_s2=3.,
               max_jerk_rad_s3=2., max_step_rad=2.1, stream_max_acceleration_rad_s2=6., stream_max_jerk_rad_s3=40.,
               current_limit_a=.8, kp=8., kd=.15)


def rig(**options):
    r = Rig(**{**PROFILE, **options})
    r.prepare()
    return r


def partial_rig(**options):
    from test_wuji_partial import PartialFake
    r = Rig(**{**PROFILE, 'disabled_joints': ['thumb_S3', 'middle_S1'], **options})
    r.driver = PartialFake(r.cfg, r.clock)
    r.driver.connect()
    r.engine.driver = r.driver
    r.prepare()
    return r


def run(r, seconds, dt=.001):
    for _ in range(round(seconds/dt)):
        r.step(dt)


def sent(r, start=0):
    return [w for w in r.driver.writes[start:] if w[0] == 'send']


def test_idle_stream_sends_the_previous_hold_command():
    r = rig()
    start = len(r.driver.writes)
    run(r, .5)  # The settle monitor runs briefly after preparation, then the engine idles.
    assert all(w[1] == r.engine.target for w in sent(r, start))
    assert r.driver.command_velocity == [0.]*20 and r.driver.command_effort == [0.]*20
    status = r.engine.status()['stream']
    assert not r.engine.stream.engaged and status['ready'] and status['settled']


def test_stream_admission_requires_a_prepared_armed_holding_hand():
    r = Rig(**PROFILE)
    with pytest.raises(RuntimeError, match='holding'):
        r.engine.direct('stream', {'joints': {'index_S1': .1}})
    r.prepare()
    with pytest.raises(ValueError, match='outside configured limits'):
        r.engine.direct('stream', {'joints': {'index_S1': 2.}})
    assert r.engine.direct('stream', {'joints': {'index_S1': .1}, 'dry_run': True})['valid']
    assert r.engine.stream_accepted == 0  # Dry runs accept nothing.
    r.armed = False
    with pytest.raises(RuntimeError, match='armed'):
        r.engine.direct('stream', {'joints': {'index_S1': .1}})


def test_policy_rate_stream_moves_smoothly_and_keeps_other_joints():
    r = rig()
    start = len(r.driver.writes)
    for k in range(31):
        r.engine.direct('stream', {'joints': {'index_S1': .01*k}})
        run(r, 1/30)
    run(r, 1.)
    q = [w[1][I] for w in sent(r, start)]
    steps = [b-a for a, b in zip(q, q[1:])]
    assert max(map(abs, steps)) <= .6*.001+1e-9                      # No staircase: <= v_max per tick.
    assert all(s >= -1e-12 for s in steps)                           # Monotone: never stops or reverses.
    assert all(w[1][0] == 0. for w in sent(r, start))                 # Untouched joints keep their targets.
    assert r.engine.target[I] == pytest.approx(.3)
    status = r.engine.status()['stream']
    assert status['reference_converged'] and status['settled'] and status['processed'] == status['accepted'] == 31


def test_blocking_moves_wait_for_the_stream_and_continue_from_it():
    r = rig()
    r.engine.direct('stream', {'joints': {'index_S1': .4}, 'duration_s': 1.})
    run(r, .2)
    r.engine.submit('m', 'move', {'positions': [0.]*20})
    run(r, .01)
    assert r.engine.result('m')['outcome'] == 'rejected' and 'stream' in r.engine.result('m')['reason']
    assert r.engine.mode == 'holding' and r.engine.fault is None
    run(r, 2.)
    r.engine.submit('m2', 'move', {'positions': [.1]*20})
    for _ in range(8000):
        r.step(.001)
        if r.engine.result('m2')['done']:
            break
    assert r.engine.result('m2')['outcome'] == 'reached'
    assert r.engine.stream_target == [.1]*20  # The stream adopted the blocking move's final target.
    r.engine.direct('stream', {'joints': {'thumb_S1': .2}})
    run(r, 1.5)
    assert r.engine.target[0] == pytest.approx(.2) and r.engine.target[I] == pytest.approx(.1)


def test_hold_brakes_the_stream_and_drops_queued_targets():
    r = rig()
    r.engine.direct('stream', {'joints': {'index_S1': .8}, 'duration_s': 2.})
    run(r, .6)
    assert r.engine.command_velocity[I] > .2
    r.engine.submit('h', 'hold')
    r.engine.stream_inbox.append({'kind': 'target', 'epoch': r.engine.epoch-1, 'accepted': 99})  # Pre-hold queue.
    for _ in range(3000):
        r.step(.001)
        if r.engine.result('h')['done']:
            break
    result = r.engine.result('h')
    assert result['outcome'] == 'held' and r.engine.mode == 'holding'
    assert r.engine.command_velocity == [0.]*20 and r.engine.target[I] < .8
    held = r.engine.target[I]
    run(r, .5)
    assert r.engine.target[I] == held  # Nothing resumed after the hold.


def test_disarm_during_stream_stops_and_releases_feed_forward():
    r = rig()
    r.engine.direct('stream_config', {'friction': True})
    run(r, .01)
    r.engine.direct('stream', {'joints': {'index_S1': .5}, 'duration_s': 1.})
    run(r, .3)
    assert r.engine.stream_effort[I] > 0
    r.armed = False
    run(r, .05)
    assert ('emergency',) in r.driver.writes and r.engine.stream_target is None
    assert r.engine.stream_effort == [0.]*20 and r.engine.command_offset == [0.]*20
    with pytest.raises(RuntimeError):
        r.engine.direct('stream', {'joints': {'index_S1': .1}})


def test_friction_feedforward_is_sent_and_bounded_by_the_cap():
    r = rig()
    r.engine.direct('stream_config', {'friction': True, 'friction_current_a': .3})
    run(r, .01)
    r.engine.direct('stream', {'joints': {'index_S1': .5}, 'duration_s': 1.})
    peak = 0.
    for _ in range(1500):
        r.step(.001)
        peak = max(peak, abs(r.driver.command_effort[I]))
    assert peak == pytest.approx(.3, rel=.01)  # <= 0.5 x 0.8 A cap fraction.
    assert r.driver.command_effort == [0.]*20  # Zero at rest.
    r.engine.direct('stream_config', {'friction_current_a': .3, 'friction': True})
    r2 = rig(current_limit_a=.4)
    r2.engine.direct('stream_config', {'friction': True, 'friction_current_a': .3})
    run(r2, .01)
    r2.engine.direct('stream', {'joints': {'index_S1': .5}, 'duration_s': 1.})
    peak = 0.
    for _ in range(1500):
        r2.step(.001)
        peak = max(peak, abs(r2.driver.command_effort[I]))
    assert peak == pytest.approx(.2, rel=.01)  # Clamped to 0.5 x 0.4 A.


def test_settings_apply_only_at_rest_and_runtime_rate_scales_scurve():
    r = rig()
    r.engine.direct('stream', {'joints': {'index_S1': .3}, 'duration_s': 1.})
    run(r, .1)
    assert r.engine.direct('stream_config', {'max_velocity_rad_s': .1})['applies'] == 'at_rest'
    assert r.engine.stream.settings['max_velocity_rad_s'][I] == .6  # Not mid-motion.
    run(r, 2.)
    assert r.engine.stream.settings['max_velocity_rad_s'][I] == .1
    r.engine.direct('stream_config', {'reset': True, 'command_hz': 200})
    run(r, .01)
    assert r.engine.runtime_hz == 200
    r.engine.submit('m', 'move', {'positions': [.2]*20})
    for _ in range(2000):
        r.step(.005)
        if r.engine.result('m')['done']:
            break
    result = r.engine.result('m')
    assert result['outcome'] == 'reached' and result['seconds'] < 2*result['nominal_duration_s']+2


def test_set_profile_writes_presets_and_requires_rest():
    r = rig()
    r.engine.submit('p', 'set_profile', {'name': 'soft', 'current_limit_a': .6})
    run(r, .01)
    result = r.engine.result('p')
    assert result['outcome'] == 'profile_set' and result['ok']
    assert r.driver.control_parameters['kp'] == pytest.approx([6./1.98]*20)
    assert r.driver.control_parameters['current_limit_a'] == [.6]*20
    assert r.engine.status()['smooth_profile']['name'] == 'soft'
    r.engine.submit('p2', 'set_profile', {'name': 'soft', 'current_limit_a': .9})
    run(r, .01)
    assert r.engine.result('p2')['outcome'] == 'rejected' and r.engine.fault is None
    r.engine.submit('h', 'set_profile', {'name': 'hold'})
    run(r, .01)
    assert r.engine.stream.settings['hold_deadband_rad'] == .004
    r.engine.direct('stream', {'joints': {'index_S1': .3}, 'duration_s': 1.})
    run(r, .1)
    r.engine.submit('p3', 'set_profile', {'name': 'firm'})
    run(r, .01)
    assert r.engine.result('p3')['outcome'] == 'rejected'
    r.engine.submit('prep', 'hold')  # A prepare restores the operator profile and stream defaults.


def test_partial_hand_profiles_keep_disabled_joints_at_zero():
    r = partial_rig()
    r.engine.submit('p', 'set_profile', {'name': 'soft'})
    run(r, .01)
    assert r.engine.result('p')['outcome'] == 'profile_set'
    params = r.driver.control_parameters
    assert all(params[k][i] == 0. for k in ('kp', 'kd', 'current_limit_a') for i in (2, 8))
    r.engine.direct('stream_config', {'friction': True})
    run(r, .01)
    before = list(r.engine.target)
    r.engine.direct('stream', {'positions': [before[i] if i in (2, 8) else .2 for i in range(20)]})
    run(r, 1.)
    assert r.engine.target[2] == before[2] and r.engine.target[8] == before[8]
    with pytest.raises(ValueError, match='disabled'):
        r.engine.direct('stream', {'joints': {'thumb_S3': .3}})


def test_tracking_error_guard_still_stops_a_stream():
    r = rig(max_tracking_error_rad=.05)
    r.driver.follow = False  # The finger does not move.
    r.engine.direct('stream', {'joints': {'index_S1': .5}, 'duration_s': .5})
    run(r, 1.)
    assert r.engine.mode in ('faulted', 'stopping', 'disabled', 'uncertain')
    assert 'tracking error' in (r.engine.fault or '') and ('emergency',) in r.driver.writes


def test_trajectory_and_recording_write_the_shared_trace_format(tmp_path):
    r = partial_rig()
    trials = TrialRecorder(tmp_path/'traces')

    class Link:  # The controller's own direct() ops, as the backend/battery reach them over IPC.
        def record_start(self, directory, **info):
            return r.engine.direct('record', {'action': 'start', 'directory': str(directory), **info})

        def record_stop(self):
            r.engine.direct('record', {'action': 'stop'})
            for _ in range(200):
                value = r.engine.direct('record', {'action': 'status'})
                if value['saved'] or value['error']:
                    return value
                time.sleep(.01)
            raise AssertionError('trace was not saved')

        def status(self):
            return r.engine.status()
    started = trials.start({'right': Link()}, controller='C4_replan', skill='waypoints5', variant='v1',
                           params={'speed': .3})
    accepted = r.engine.direct('trajectory', {'waypoints': [
        {'t': .4, 'joints': {'index_S1': .2}}, {'t': .8, 'fingers': {'ring': [.1, 0., .2, .1]}},
        {'t': 1.2, 'joints': {'index_S1': 0.}}]})
    assert accepted['estimated_duration_s'] >= 1.2
    run(r, 2.5)
    saved = trials.stop(success=True)
    trial = load_trial(saved['directory'])
    meta, hand = trial['meta'], trial['hands']['right']
    for key in ('sides', 'joint_names', 'nominal_rate_hz', 'controller', 'skill', 'variant', 'params', 'success',
                'started_at', 'backend', 'notes'):
        assert key in meta
    assert meta['sides'] == ['right'] and meta['backend'] == 'mock' and meta['success'] is True
    assert meta['joint_names'] == list(JOINT_NAMES) and meta['nominal_rate_hz'] == 1000
    t = hand['t']
    assert t.dtype.name == 'float64' and t.shape[0] == 2500 and (t[1:] > t[:-1]).all()
    for key in ('q_ref', 'q_target', 'q_meas', 'dq_meas', 'current', 'i_ff', 'kp', 'kd'):
        assert hand[key].shape == (2500, 20)
    assert math.isnan(hand['q_meas'][0, 2]) and math.isnan(hand['current'][-1, 8])
    assert hand['q_target'][-1, I] == 0. and hand['q_ref'][:, I].max() == pytest.approx(.2, abs=1e-6)
    assert hand['kp'][0, 0] == 8. and hand['kp'][0, 2] == 0.
    assert started['sides'] == ['right']


def test_cached_arming_reader_sees_every_change_and_expiry(tmp_path):
    from services.xarm import arming
    read = arming.CachedReader(tmp_path)
    assert not read()['armed']
    arming.write(tmp_path, True, minutes=1)
    assert read()['armed']
    arming.write(tmp_path, False, reason='operator stop')
    assert not read()['armed'] and read()['reason'] == 'operator stop'
    arming.write(tmp_path, True, minutes=1)
    assert read()['armed'] and not read(now=time.time()+61)['armed']  # Expiry needs no file change.
    arming.arm_file(tmp_path).unlink()
    assert not read()['armed']
    arming.arm_file(tmp_path).write_text('not json')
    assert not read()['armed']


def test_sdk_parameter_writes_verify_readback_and_keep_disabled_joints_at_zero(monkeypatch):
    from types import SimpleNamespace as NS
    from test_wuji_partial import sdk_setup
    driver, hand, clock, value, emit = sdk_setup(monkeypatch)
    hand.gains.set = lambda values: setattr(hand.gains, 'value', [NS(kp=p, kd=d) for p, d in values])
    driver.apply_profile()
    desired = {'kp': [1.]*20, 'kd': [.05]*20, 'current_limit_a': [.3]*20}
    for key in desired:
        desired[key][2] = desired[key][8] = 0.
    driver.set_control_parameters(desired)
    assert driver.profile == desired and hand.efforts.value == desired['current_limit_a']
    for key, joint, number in (('kp', 2, 1.), ('kp', 4, 9.), ('current_limit_a', 4, 0.)):
        bad = {k: list(v) for k, v in desired.items()}
        bad[key][joint] = number
        with pytest.raises(ValueError):
            driver.set_control_parameters(bad)
    hand.gains.set = lambda values: None  # Readback mismatch leaves no verified profile.
    with pytest.raises(RuntimeError, match='readback'):
        driver.set_control_parameters({**desired, 'kp': [0. if i in (2, 8) else .5 for i in range(20)]})
    assert driver.profile is None
