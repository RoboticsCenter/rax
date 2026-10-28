"""Effort/impedance control uses fake hardware and the actual tickable controller."""
import math

import pytest

from test_wuji_controller import Rig
from services.wuji.config import validate_tool
from wuji_fakes import hand_config


def finish(rig, ident, op, args):
    rig.engine.submit(ident, op, args)
    for _ in range(1500):
        rig.step()
        if rig.engine.result(ident)['done']:
            return rig.engine.result(ident)
    raise AssertionError('operation did not finish')


@pytest.mark.parametrize('values', [[0]*19, [0]*21, [math.nan]*20, [math.inf]*20,
                                   [True]*20, ['0.1']*20, [.6]*20, [-.6]*20])
def test_bad_effort_rejected_before_dispatch(values):
    with pytest.raises(ValueError):
        validate_tool('wuji_set_effort', {'effort_a': values}, hand_config())


@pytest.mark.parametrize('args', [{}, {'force_n': [1]*20}, {'kp': [0]*20}, {'kd': [-.1]*20},
                                  {'kp': [2]*20}, {'current_limit_a': [.6]*20},
                                  {'current_limit_a': [0]*20}, {'kp': [math.nan]*20}])
def test_impedance_cannot_escape_operator_profile(args):
    with pytest.raises(ValueError):
        validate_tool('wuji_set_impedance', args, hand_config())


@pytest.mark.parametrize('motion_mode', ['timed', 'feedback', 'scurve'])
def test_signed_effort_ramps_persists_through_move_and_hold_and_clears(motion_mode):
    r = Rig(motion_mode=motion_mode)
    r.prepare()
    goal = [.2, -.2] * 10
    result = finish(r, 'effort', 'set_effort', {'effort_a': goal})
    assert result['ok'] and result['units'] == 'A' and not result['contact_force_verified']
    assert r.driver.goal == [0.]*20 and r.driver.command_effort == goal
    samples = r.driver.effort_writes
    assert any(0 < row[1][0] < .2 for row in samples)
    for (t0, a), (t1, b) in zip(samples, samples[1:]):
        if t1 > t0:
            assert max(abs(x-y)/(t1-t0) for x, y in zip(a, b)) <= r.cfg['max_effort_slew_a_s']+1e-8
    assert finish(r, 'move', 'move', {'positions': [.1]*20})['ok']
    assert r.driver.command_effort == goal
    assert finish(r, 'hold', 'hold', {})['ok']
    assert r.driver.command_effort == goal
    assert finish(r, 'clear', 'set_effort', {'effort_a': [0.]*20})['ok']
    assert r.driver.command_effort == [0.]*20
    status = r.engine.status()
    assert status['effort_control']['calibrated_contact_force'] is False
    assert status['command_effort_a'] == [0.]*20


def test_impedance_preserves_target_and_active_cap_is_enforced_inside_owner():
    r = Rig()
    r.driver.goal = [.12]*20
    r.prepare()
    result = finish(r, 'impedance', 'set_impedance', {'kp': [.8]*20, 'current_limit_a': [.15]*20})
    assert result['ok'] and result['readback_verified']
    assert r.driver.goal == [.12]*20
    state = r.engine.status()
    assert state['control_parameters']['kp'] == [.8]*20
    assert state['control_parameters']['kd'] == r.cfg['kd']
    assert state['profile_control_parameters']['kp'] == r.cfg['kp']
    rejected = finish(r, 'over_cap', 'set_effort', {'effort_a': [.2]*20})
    assert not rejected['ok'] and 'current limit' in rejected['reason']
    assert r.driver.command_effort == [0.]*20
    assert finish(r, 'valid', 'set_effort', {'effort_a': [.1]*20})['ok']


def test_prepare_restores_operator_profile_and_zero_effort():
    r = Rig()
    r.prepare()
    assert finish(r, 'soften', 'set_impedance', {'kp': [.8]*20})['ok']
    assert finish(r, 'effort', 'set_effort', {'effort_a': [.1]*20})['ok']
    assert finish(r, 'stop', 'emergency', {})['ok']
    assert finish(r, 'prepare_again', 'prepare', {})['ok']
    assert r.engine.status()['control_parameters'] == {k: r.cfg[k] for k in ('kp', 'kd', 'current_limit_a')}
    assert r.driver.command_effort == [0.]*20


def test_impedance_requires_zero_feedforward_and_measured_stationary_hand():
    r = Rig()
    r.prepare()
    assert finish(r, 'effort', 'set_effort', {'effort_a': [.1]*20})['ok']
    assert not finish(r, 'under_load', 'set_impedance', {'kp': [.8]*20})['ok']
    assert finish(r, 'clear', 'set_effort', {'effort_a': [0.]*20})['ok']
    r.driver.velocity = [.1]*20
    result = finish(r, 'moving', 'set_impedance', {'kp': [.8]*20})
    assert not result['ok'] and 'stationary' in result['reason']
    assert not any(w[0] == 'impedance' for w in r.driver.writes)


@pytest.mark.parametrize('failure', ['readback', 'stop', 'deadline'])
def test_partial_impedance_write_cannot_report_success_or_continue_after_stop(failure):
    r = Rig()
    r.prepare()
    original = r.driver.set_current_limits

    def broken(values):
        original(values)
        if failure == 'readback':
            raise RuntimeError('current limit readback mismatch')
        if failure == 'stop':
            r.engine.submit('priority_stop', 'emergency')
        else:
            r.clock.now += r.cfg['max_scheduler_gap_s']*2

    r.driver.set_current_limits = broken
    result = finish(r, 'adjust', 'set_impedance', {'kp': [.8]*20, 'current_limit_a': [.2]*20})
    assert not result['ok']
    r.step()
    assert not any(w[0] == 'impedance' for w in r.driver.writes)
    assert ('emergency',) in r.driver.writes


@pytest.mark.parametrize('failure', ['disarm', 'parent', 'heartbeat', 'fault', 'stale'])
@pytest.mark.parametrize('ramping', [False, True])
def test_effort_is_cleared_by_watchdogs_during_ramp_and_persistent_hold(failure, ramping):
    r = Rig()
    r.prepare()
    if ramping:
        r.engine.submit('effort', 'set_effort', {'effort_a': [.2]*20})
        for _ in range(15):
            r.step()
    else:
        assert finish(r, 'effort', 'set_effort', {'effort_a': [.2]*20})['ok']
    assert r.driver.command_effort[0] > 0
    if failure == 'disarm':
        r.armed = False
    elif failure == 'parent':
        r.alive = False
    elif failure == 'fault':
        r.driver.faults = ['motor fault']
    elif failure == 'stale':
        r.clock.now += r.cfg['max_feedback_age_s']+.01
    else:
        r.clock.now += r.cfg['parent_timeout_s']+.01
    r.step(heartbeat=failure != 'heartbeat', state=failure != 'stale', diagnostics=failure != 'stale')
    r.step()
    assert ('emergency',) in r.driver.writes
    assert r.engine.command_effort == r.driver.command_effort == [0.]*20


@pytest.mark.parametrize('guard', ['position', 'velocity'])
def test_effort_checks_measured_joint_limits_and_speed(guard):
    r = Rig()
    r.driver.goal = [.95]*20
    r.prepare()
    assert finish(r, 'effort', 'set_effort', {'effort_a': [.1]*20})['ok']
    r.driver.follow = False
    if guard == 'position':
        r.driver.q = [1.05]*20  # Under tracking-error bound, but outside joint limits.
    else:
        r.driver.velocity = [r.cfg['max_velocity_rad_s']+.1]*20
    r.step()
    assert r.engine.fault
    r.step()
    assert ('emergency',) in r.driver.writes


def test_effort_does_not_report_success_when_final_command_fails():
    r = Rig()
    r.prepare()

    def fail_send():
        if r.driver.command_effort[0] == .1:
            raise RuntimeError('final current write failed')

    r.driver.on_send = fail_send
    result = finish(r, 'effort', 'set_effort', {'effort_a': [.1]*20})
    assert not result['ok']
    r.step()
    assert ('emergency',) in r.driver.writes


def test_private_effort_endpoint_checks_duration_and_profile_ceiling():
    r = Rig()
    r.prepare()
    for i, args in enumerate([{'effort_a': [.6]*20}, {'effort_a': [.2]*20, 'duration_s': .1},
                              {'effort_a': [.1]*20, 'duration_s': None}]):
        assert not finish(r, f'invalid{i}', 'set_effort', args)['ok']
        assert r.driver.command_effort == [0.]*20
