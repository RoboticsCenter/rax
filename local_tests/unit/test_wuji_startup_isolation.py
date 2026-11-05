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
