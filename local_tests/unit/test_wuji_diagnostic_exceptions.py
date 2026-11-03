"""Exact operator exceptions preserve native diagnostics and all other guards."""
from copy import deepcopy

import pytest

from test_wuji_sdk import driver_setup, frame, Hand
from test_wuji_controller import Rig
from wuji_fakes import hand_config
from services.wuji.controller import Controller
from services.wuji.preparation import preparation_action

DETAIL = {'name': 'InitCurrentCalibFailed', 'severity': 'Fatal', 'clear_policy': 'NonClearable'}


@pytest.mark.parametrize('value', [True, ['middle_S1'], {'all': '2.13.1'},
                                 {'middle_S1': True}, {'middle_S1': 'latest'}, {'middle_S1': '*'}])
def test_exception_requires_exact_joint_and_firmware(value):
    with pytest.raises(ValueError, match='exact joint names'):
        hand_config(allow_init_current_calib_failed=value)


@pytest.mark.parametrize('policy,firmware,joint,code,detail,accepted', [
    ({}, '2.13.1', 8, 0x3208, DETAIL, False),
    ({'middle_S1': '2.13.1'}, '2.13.1', 8, 0x3208, DETAIL, True),
    ({'middle_S1': '2.13.1'}, '2.12.0', 8, 0x3208, DETAIL, False),
    ({'middle_S1': '2.13.1'}, '2.13.1', 9, 0x3208, DETAIL, False),
    ({'middle_S1': '2.13.1'}, '2.13.1', 8, 0x2102, DETAIL, False),
    ({'middle_S1': '2.13.1'}, '2.13.1', 8, 0x3208, {**DETAIL, 'name': 'Other'}, False),
    ({'middle_S1': '2.13.1'}, '2.13.1', 8, 0x3208, {**DETAIL, 'severity': 'Warning'}, False),
    ({'middle_S1': '2.13.1'}, '2.13.1', 8, 0x3208, {**DETAIL, 'clear_policy': 'ManualClear'}, False),
    ({'middle_S1': '2.13.1'}, '2.13.1', 8, 0x3208, None, False),
])
def test_only_exact_approved_diagnostic_is_nonblocking(monkeypatch, policy, firmware, joint, code, detail, accepted):
    driver, hand, clock, _ = driver_setup(allow_init_current_calib_failed=policy)
    hand.info.firmware_version = firmware
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(lambda _: detail), raising=False)
    driver.connect()
    value = frame(clock)
    value.joints[joint].error_code_current = code
    value.joints[joint].status_word.ext_state = 3
    value.joints[joint].status_word.ext_state_name = 'Stopped'
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    status = driver.snapshot()
    assert bool(status['diagnostic_exceptions']) is accepted
    assert bool(status['faults']) is not accepted
    assert status['diagnostics']['error_code_current'][joint] == code
    assert status['diagnostics']['state_names'][joint] == 'Stopped'
    assert status['fresh'] and not status['enabled'] and not hand.writes
    if accepted:
        assert status['diagnostic_exceptions'][0]['severity'] == 'Fatal'
        assert not status['warnings']  # A fatal exception is not relabelled as an encoder warning.
        assert preparation_action(status, driver.cfg) == 'prepare'
        engine = Controller(driver.cfg, driver, lambda: True, clock=clock)
        engine._publish()
        assert engine.status()['diagnostics_policy']['allow_init_current_calib_failed'] == policy


def test_exception_history_staleness_and_other_faults_remain_visible(monkeypatch):
    driver, hand, clock, _ = driver_setup(allow_init_current_calib_failed={'middle_S1': '2.6.0'})
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(lambda code: DETAIL if code == 0x3208 else
                        {'name': 'Overcurrent', 'severity': 'ImmediateStop', 'clear_policy': 'ManualClear'}), raising=False)
    driver.connect()
    value = frame(clock)
    value.joints[8].error_code_current = 0x3208
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    hand.diagnostics.callback(value)
    status = driver.snapshot()
    assert status['diagnostic_exception_history'][0]['frames'] == 1
    status['diagnostic_exceptions'][0]['severity'] = 'changed'
    assert driver.snapshot()['diagnostic_exceptions'][0]['severity'] == 'Fatal'
    clock.now += 1
    assert not driver.snapshot()['fresh']
    value = frame(clock, 2)
    value.joints[8].error_code_current = 0x3208
    value.joints[9].error_code_current = 0x2102
    value.joints[8].mcu_temp_c_fb = 70
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    status = driver.snapshot()
    assert any(f.get('code') == 0x2102 for f in status['faults'])
    assert any(f.get('reason') == 'temperature limit exceeded' for f in status['faults'])
    clock.now += .01
    hand.states.callback(frame(clock, 3))
    hand.diagnostics.callback(frame(clock, 3))
    status = driver.snapshot()
    assert not status['diagnostic_exceptions'] and status['diagnostic_exception_history'][0]['frames'] == 2
    assert status['faults']  # Unapproved transient faults remain latched.
    driver.close()
    driver.connect()
    assert not driver.snapshot()['diagnostic_exception_history']


def test_exception_does_not_waive_native_enable_confirmation():
    rig = Rig(stop_timeout_s=.15, allow_init_current_calib_failed={'middle_S1': '2.13.1'})
    rig.armed = True
    rig.engine.submit('p', 'prepare')
    rig.step()
    assert ('enable',) in rig.driver.writes
    rig.driver.states[8], rig.driver.names[8] = 3, 'Stopped'
    for _ in range(25):
        rig.step()
    assert not rig.engine.result('p')['ok']
    assert ('emergency',) in rig.driver.writes
    assert rig.engine.mode != 'holding'


def test_runtime_policy_change_does_not_erase_a_latched_fault(monkeypatch):
    driver, hand, clock, _ = driver_setup()
    monkeypatch.setattr(Hand, 'describe_error', staticmethod(lambda _: deepcopy(DETAIL)), raising=False)
    driver.connect()
    value = frame(clock)
    value.joints[8].error_code_current = 0x3208
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    driver.cfg['allow_init_current_calib_failed'] = {'middle_S1': '2.6.0'}
    clock.now += .01
    value.header.seq += 1
    value.header.timestamp_us = int(clock()*1e6)
    hand.states.callback(value)
    hand.diagnostics.callback(value)
    status = driver.snapshot()
    assert status['diagnostic_exceptions'] and status['faults']
