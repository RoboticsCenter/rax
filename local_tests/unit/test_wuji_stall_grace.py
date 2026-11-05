from copy import deepcopy

import pytest

from test_wuji_sdk import driver_setup, frame, Hand


DETAIL = {'name': 'Stall', 'severity': 'Warning', 'clear_policy': 'AutoClear'}


def emit(hand, clock, seq, *, code=7, state=2):
    value = frame(clock, seq)
    value.joints[17].error_code_current = code
    value.joints[17].status_word.ext_state = state
    value.joints[17].status_word.ext_state_name = 'Enabled' if state == 2 else 'Stopped'
    hand.states.callback(value)
    hand.diagnostics.callback(value)


def test_brief_stall_is_visible_warning_but_sustained_stall_latches(monkeypatch):
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(lambda _: deepcopy(DETAIL)), raising=False)
    driver, hand, clock, _ = driver_setup(stall_warning_grace_s=.1)
    driver.connect()
    emit(hand, clock, 1)
    assert not driver.snapshot()['faults']
    assert driver.snapshot()['warnings'][0]['name'] == 'Stall'
    clock.now += .05
    emit(hand, clock, 1, code=0)  # Duplicate cannot reset the running grace period.
    clock.now += .06
    emit(hand, clock, 2)
    assert driver.snapshot()['faults'][0]['name'] == 'Stall'
    clock.now += .01
    emit(hand, clock, 3, code=0)
    assert driver.snapshot()['faults']  # Native AutoClear does not clear the software fault latch.
    driver.close()


def test_genuinely_cleared_transient_can_have_a_new_episode(monkeypatch):
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(lambda _: deepcopy(DETAIL)), raising=False)
    driver, hand, clock, _ = driver_setup(stall_warning_grace_s=.1)
    driver.connect()
    emit(hand, clock, 1)
    clock.now += .05
    emit(hand, clock, 2, code=0)
    clock.now += .07
    emit(hand, clock, 3)
    assert not driver.snapshot()['faults']
    assert driver.snapshot()['warning_history'][0]['frames'] == 2
    driver.close()


@pytest.mark.parametrize('options,state,detail', [
    ({}, 2, DETAIL),
    ({'stall_warning_grace_s': .1}, 3, DETAIL),
    ({'stall_warning_grace_s': .1}, 2, {**DETAIL, 'severity': 'Fatal'}),
    ({'stall_warning_grace_s': .1}, 2, {**DETAIL, 'clear_policy': 'ManualClear'}),
    ({'stall_warning_grace_s': .1}, 2, {**DETAIL, 'name': 'Other'}),
])
def test_no_grace_for_disabled_joints_other_faults_or_default_policy(monkeypatch, options, state, detail):
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(lambda _: deepcopy(detail)), raising=False)
    driver, hand, clock, _ = driver_setup(**options)
    driver.connect()
    emit(hand, clock, 1, state=state)
    assert driver.snapshot()['faults']
    driver.close()
