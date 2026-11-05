import pytest

from test_wuji_sdk import driver_setup, frame
from services.wuji.preparation import preparation_action, PreparationBlocked
from services.wuji.partial import effective_parameters


def emit(hand, clock, seq, *, initializing=True, other_enabled=False):
    f = frame(clock, seq)
    if initializing:
        for index in [2, 8]:
            f.joints[index].status_word.ext_state = 0
            f.joints[index].status_word.ext_state_name = 'Init'
    if other_enabled:
        f.joints[0].status_word.ext_state = 2
        f.joints[0].status_word.ext_state_name = 'Enabled'
    hand.states.callback(f)
    hand.diagnostics.callback(f)


def test_powerup_init_blocks_preparation_but_does_not_poison_later_ready_state():
    driver, hand, clock, _ = driver_setup(disabled_joints=['thumb_S3', 'middle_S1'])
    driver.connect()
    emit(hand, clock, 1)
    state = driver.snapshot()
    assert not state['faults'] and len(state['warnings']) == 2
    with pytest.raises(PreparationBlocked, match='confirmed disabled'):
        preparation_action(state, driver.cfg)
    clock.now += .01
    emit(hand, clock, 2, initializing=False)
    state = driver.snapshot()
    assert not state['faults'] and not state['warnings']
    assert preparation_action(state, driver.cfg) == 'prepare'
    assert len(state['warning_history']) == 2
    driver.close()


@pytest.mark.parametrize('profile_loaded,other_enabled', [(True, False), (False, True)])
def test_init_during_control_or_with_any_enabled_joint_still_latches(profile_loaded, other_enabled):
    driver, hand, clock, _ = driver_setup(disabled_joints=['thumb_S3', 'middle_S1'])
    driver.connect()
    if profile_loaded:
        driver.profile = effective_parameters(driver.cfg)
    emit(hand, clock, 1, other_enabled=other_enabled)
    assert len(driver.snapshot()['faults']) == 2
    clock.now += .01
    emit(hand, clock, 2, initializing=False)
    assert driver.snapshot()['faults']  # Never clear an existing latch automatically.
    driver.close()


def test_requested_stop_init_is_warning_but_never_disabled_confirmation():
    driver, hand, clock, _ = driver_setup(disabled_joints=['thumb_S3', 'middle_S1'])
    driver.connect(); driver.profile = effective_parameters(driver.cfg)
    hand.emergency_stop = lambda: None
    hand.disable = lambda: None
    driver.stop()
    emit(hand, clock, 1)
    state = driver.snapshot()
    assert not state['faults'] and len(state['warnings']) == 2
    assert all(w['during_requested_stop'] for w in state['warnings'])
    with pytest.raises(PreparationBlocked, match='confirmed disabled'):
        preparation_action(state, driver.cfg)
    clock.now += .01; emit(hand, clock, 2, initializing=False)
    assert preparation_action(driver.snapshot(), driver.cfg) == 'prepare'
    driver.close()


def test_requested_stop_does_not_hide_persistent_init_or_temperature_fault():
    driver, hand, clock, _ = driver_setup(disabled_joints=['thumb_S3', 'middle_S1'])
    driver.connect(); driver.profile = effective_parameters(driver.cfg)
    hand.emergency_stop = lambda: None
    hand.disable = lambda: None
    driver.stop()
    f = frame(clock, 1)
    f.joints[2].status_word.ext_state = 0; f.joints[2].status_word.ext_state_name = 'Init'
    f.joints[0].mcu_temp_c_fb = driver.cfg['max_temperature_c']
    hand.states.callback(f); hand.diagnostics.callback(f)
    assert any(x.get('reason') == 'temperature limit exceeded' for x in driver.snapshot()['faults'])
    clock.now += driver.cfg['stop_timeout_s']+.01; emit(hand, clock, 2)
    assert any(x.get('reason') == 'excluded joint is not disabled' for x in driver.snapshot()['faults'])
    driver.close()
