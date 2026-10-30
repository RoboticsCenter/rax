"""Transient tolerance never bypasses sustained speed or other hand guards."""
import math

import pytest

from test_wuji_controller import Rig
from test_wuji_effort import finish
from wuji_fakes import hand_config


def biased_rig(**overrides):
    r = Rig(command_hz=1000, effort_velocity_grace_s=.02, **overrides)
    r.prepare()
    assert finish(r, 'effort', 'set_effort', {'effort_a': [.1]*20})['ok']
    return r


@pytest.mark.parametrize('value', [-.001, .021, True, '0.02', None, math.nan, math.inf])
def test_grace_configuration_is_bounded(value):
    with pytest.raises(ValueError, match='effort_velocity_grace_s'):
        hand_config(effort_velocity_grace_s=value)


def test_default_remains_immediate_and_records_the_trigger():
    r = Rig()
    r.prepare()
    assert finish(r, 'effort', 'set_effort', {'effort_a': [.1]*20})['ok']
    r.driver.velocity[4] = -.7
    r.step()
    assert r.engine.fault
    event = r.engine.status()['effort_control']['velocity_guard']['last_trip']
    assert event['reason'] == 'immediate' and event['joint'] == 'index_S1'
    assert event['velocity_rad_s'] == -.7
    r.step()
    assert r.engine.command_effort == r.driver.command_effort == [0.]*20


@pytest.mark.parametrize('motion_mode', ['timed', 'feedback', 'scurve'])
def test_short_spike_preserves_hold_target_and_bias(motion_mode):
    r = biased_rig(motion_mode=motion_mode)
    target = list(r.engine.target)
    parameters = r.engine.status()['control_parameters']
    r.driver.velocity[4] = -.7
    for _ in range(5):
        r.step(.001)
    r.driver.velocity[4] = 0
    r.step(.001)
    assert r.engine.fault is None and r.engine.mode == 'holding'
    assert r.driver.goal == target and r.driver.command_effort == [.1]*20
    status = r.engine.status()
    assert status['control_parameters'] == parameters
    guard = status['effort_control']['velocity_guard']
    assert guard['tolerated_events'] == 1 and guard['last_trip'] is None
    assert guard['last_event']['peak_joint'] == 'index_S1'
    assert guard['active'] is None


@pytest.mark.parametrize('new_samples', [True, False])
def test_sustained_speed_stops_even_when_the_frame_does_not_advance(new_samples):
    r = biased_rig()
    r.driver.velocity[1] = .7
    r.step(.001)
    for _ in range(19):
        r.step(.001, state=new_samples)
        assert r.engine.fault is None
    r.step(.001, state=new_samples)
    assert 'duration' in r.engine.fault
    r.step(.001)
    assert ('emergency',) in r.driver.writes
    event = r.engine.status()['effort_control']['velocity_guard']['last_trip']
    assert event['peak_joint'] == 'thumb_S2'
    assert event['elapsed_s'] == pytest.approx(.02)
    assert r.driver.command_effort == [0.]*20


@pytest.mark.parametrize('joint,speed', [(1, 1.), (4, -1.01), (19, 1.2)])
def test_hard_speed_cutoff_is_immediate_for_any_joint(joint, speed):
    r = biased_rig()
    r.driver.velocity[joint] = speed
    r.step(.001)
    assert 'hard_limit' in r.engine.fault
    assert r.engine.command_effort == [0.]*20


def test_measured_displacement_ends_grace_before_the_deadline():
    r = biased_rig()
    r.driver.follow = False
    r.driver.velocity[4] = .7
    r.step(.001)
    r.driver.q[4] = .011  # .5 rad/s * .02 s = .01 rad displacement budget.
    r.step(.001)
    assert 'displacement' in r.engine.fault


def test_late_low_speed_feedback_does_not_erase_the_expired_deadline():
    r = biased_rig()
    r.driver.velocity[4] = .7
    r.step(.001)
    r.driver.velocity[4] = 0
    r.step(.02)
    assert 'duration' in r.engine.fault


def test_changing_joints_does_not_restart_the_grace_period():
    r = biased_rig()
    for i in range(21):
        r.driver.velocity = [0.]*20
        r.driver.velocity[i % 20] = .7
        r.step(.001)
    assert 'duration' in r.engine.fault


@pytest.mark.parametrize('failure', ['joint_limit', 'tracking', 'hardware', 'stale', 'disarm', 'parent'])
def test_other_guards_still_abort_during_grace(failure):
    r = biased_rig()
    r.driver.velocity[4] = .7
    r.step(.001)
    if failure == 'joint_limit':
        r.engine.target = [.99]*20
        r.driver.follow = False
        r.driver.q = [1.01]*20
    elif failure == 'tracking':
        r.driver.follow = False
        r.driver.q[0] = .4
    elif failure == 'hardware':
        r.driver.faults = ['encoder fault']
    elif failure == 'stale':
        r.driver.received['diagnostics'] -= 1
    elif failure == 'disarm':
        r.armed = False
    else:
        r.alive = False
    r.step(.001, diagnostics=failure != 'stale')
    assert r.engine.fault
    r.step(.001)
    assert ('emergency',) in r.driver.writes
    assert r.driver.command_effort == [0.]*20


def test_prepare_resets_the_timer_but_retains_historical_fault_evidence():
    r = biased_rig()
    r.driver.velocity[4] = 1.1
    r.step(.001)
    event = r.engine.effort_velocity_guard.status()['last_trip']
    r.driver.velocity = [0.]*20
    for _ in range(4):
        r.step(.01)
    assert finish(r, 'reprepare', 'prepare', {})['ok']
    status = r.engine.status()
    assert status['fault'] is None
    assert status['effort_control']['velocity_guard']['last_trip'] == event
    assert status['effort_control']['velocity_guard']['active'] is None
    assert status['command_effort_a'] == [0.]*20


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_nonfinite_velocity_is_never_tolerated(value):
    r = biased_rig()
    r.driver.velocity[4] = value
    r.step(.001)
    assert 'invalid measured feedback' in r.engine.fault
