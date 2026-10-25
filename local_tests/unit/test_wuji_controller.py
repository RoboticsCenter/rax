import math
import threading

import pytest
from wuji_fakes import Clock, FakeHand, hand_config

from services.wuji.config import validate_tool
from services.wuji.controller import Controller, Trajectory


class Rig:
    def __init__(self, **options):
        self.cfg = hand_config(**options)
        self.clock = Clock()
        self.armed = False
        self.alive = True
        self.driver = FakeHand(self.cfg, self.clock)
        self.driver.connect()
        self.engine = Controller(self.cfg, self.driver, lambda: self.armed, clock=self.clock,
                                 parent_alive=lambda: self.alive)
        self.engine.mode = 'disabled'

    def step(self, dt=.01, *, heartbeat=True, state=True, diagnostics=True):
        self.clock.now += dt
        if heartbeat:
            self.engine.heartbeat()
        self.driver.refresh(state, diagnostics)
        self.engine.tick()

    def prepare(self):
        self.armed = True
        self.engine.submit('prepare', 'prepare')
        self.step()
        self.step()
        assert self.engine.result('prepare')['outcome'] == 'prepared'
        assert self.engine.mode == 'holding'


@pytest.mark.parametrize('value', [[0]*19, [0]*21, [[0]*4]*5, [True]*20, ['0']*20,
                                   [math.nan]*20, [math.inf]*20, [2.]*20])
def test_bad_targets_rejected(value):
    with pytest.raises(ValueError):
        validate_tool('wuji_move_joints', {'positions': value}, hand_config())


@pytest.mark.parametrize('args', [{'positions': [0]*20, 'duration_s': None},
                                 {'positions': [0]*20, 'duration_s': True},
                                 {'positions': [0]*20, 'force': 10}])
def test_unknown_and_invalid_options_rejected(args):
    with pytest.raises(ValueError):
        validate_tool('wuji_move_joints', args, hand_config())


@pytest.mark.parametrize('duration', [True, '1', float('nan'), -1.])
def test_private_trajectory_path_also_rejects_invalid_duration(duration):
    with pytest.raises(ValueError):
        Trajectory([0.]*20, [.1]*20, hand_config(), duration)


def test_connection_does_not_enable_or_seed_and_unarmed_prepare_does_nothing():
    rig = Rig()
    rig.step()
    assert not rig.driver.writes
    rig.engine.submit('p', 'prepare')
    rig.step()
    assert not rig.driver.writes
    assert rig.engine.result('p')['outcome'] == 'rejected'


def test_seed_precedes_enable_and_all_twenty_must_confirm():
    rig = Rig(stop_timeout_s=.15)
    rig.driver.goal = [.08]*20
    rig.armed = True
    rig.engine.submit('p', 'prepare')
    rig.step()
    assert [w[0] for w in rig.driver.writes[:3]] == ['profile', 'send', 'enable']
    assert rig.driver.writes[1][1] == [.08]*20
    assert not rig.engine.result('p')['done']
    rig.driver.states[-1], rig.driver.names[-1] = 1, 'Ready'
    for _ in range(25):
        rig.step()
    assert not rig.engine.result('p')['ok']
    assert ('emergency',) in rig.driver.writes


def test_trajectory_respects_velocity_acceleration_and_never_overshoots():
    cfg = hand_config()
    path = Trajectory([0.]*20, [.3]*20, cfg)
    dt = path.duration/5000
    values = [path.sample(i*dt)[0] for i in range(5001)]
    velocities = [(b-a)/dt for a, b in zip(values, values[1:])]
    accelerations = [(b-a)/dt for a, b in zip(velocities, velocities[1:])]
    assert min(values) >= 0 and max(values) <= .3+1e-12
    assert max(map(abs, velocities)) <= cfg['max_velocity_rad_s']+1e-6
    assert max(map(abs, accelerations)) <= cfg['max_acceleration_rad_s2']+1e-6
    with pytest.raises(ValueError, match='at least'):
        Trajectory([0.]*20, [.3]*20, cfg, .1)
    with pytest.raises(ValueError, match='max_step'):
        Trajectory([0.]*20, [.5]*20, cfg)


def test_move_settles_and_target_persists_between_calls_and_hold():
    rig = Rig()
    rig.prepare()
    rig.engine.submit('move', 'move', {'positions': [.1]*20})
    for _ in range(200):
        rig.step()
    result = rig.engine.result('move')
    assert result['outcome'] == 'reached' and result['max_joint_error_rad'] < .001
    rig.driver.follow = False
    rig.driver.q = [.08]*20  # Object contact: retain target, not measured angles.
    rig.engine.submit('hold', 'hold')
    rig.step()
    assert rig.engine.mode == 'holding'
    assert rig.driver.goal == pytest.approx([.1]*20)
    assert not rig.engine.result('hold')['confirmed']
    assert rig.engine.submit('move', 'move', {'positions': [-.1]*20}) == {'command_id': 'move'}
    rig.step()
    assert rig.driver.goal == pytest.approx([.1]*20)


@pytest.mark.parametrize('failure', ['disarm', 'diagnostics', 'positions', 'parent', 'heartbeat', 'fault', 'enabled', 'scheduler'])
@pytest.mark.parametrize('moving', [False, True])
def test_idle_and_active_watchdogs_stop_without_another_tool_call(failure, moving):
    rig = Rig(stop_timeout_s=.15)
    rig.prepare()
    if moving:
        rig.engine.submit('motion', 'move', {'positions': [.2]*20})
    rig.step()
    if failure == 'disarm':
        rig.armed = False
    elif failure == 'parent':
        rig.alive = False
    elif failure == 'fault':
        rig.driver.faults = [{'joint': 'thumb_S2', 'code': 3}]
    elif failure == 'enabled':
        rig.driver.states[-1] = 1
    for _ in range(130):
        rig.step(.2 if failure == 'scheduler' else .01,
                 heartbeat=failure != 'heartbeat', state=failure != 'positions', diagnostics=failure != 'diagnostics')
    assert ('emergency',) in rig.driver.writes
    assert rig.engine.mode in ('faulted', 'uncertain')
    if moving:
        assert not rig.engine.result('motion')['ok']
    # Returning health does not enable or replay anything.
    rig.armed, rig.alive, rig.driver.faults = True, True, []
    enable_count = sum(w[0] == 'enable' for w in rig.driver.writes)
    for _ in range(5):
        rig.step()
    assert sum(w[0] == 'enable' for w in rig.driver.writes) == enable_count


def test_stop_delivery_error_is_uncertain_not_confirmed():
    rig = Rig()
    rig.prepare()
    rig.driver.stop_error = ConnectionError('lost cable')
    rig.engine.submit('stop', 'emergency')
    rig.step()
    result = rig.engine.result('stop')
    assert result['outcome'] == 'uncertain' and not result['confirmed'] and not result['sdk_returned']


def test_stop_needs_new_feedback_and_low_measured_velocity():
    rig = Rig(stop_timeout_s=.15)
    rig.prepare()
    rig.driver.velocity = [.5]*20
    rig.engine.submit('stop', 'emergency')
    rig.step()
    assert not rig.engine.result('stop')['done']
    for _ in range(20):
        rig.step()
    result = rig.engine.result('stop')
    assert result['sdk_returned'] and not result['confirmed']


def test_stop_during_native_call_fences_late_targets_and_can_be_queued_without_lock():
    rig = Rig()
    rig.prepare()
    entered, release = threading.Event(), threading.Event()
    def blocked():
        entered.set()
        assert release.wait(2)
    rig.driver.on_send = blocked
    rig.engine.submit('move', 'move', {'positions': [.2]*20})
    thread = threading.Thread(target=rig.step)
    thread.start()
    assert entered.wait(1)
    rig.engine.submit('stop', 'emergency')
    release.set()
    thread.join(2)
    assert not thread.is_alive()
    rig.driver.on_send = None
    before = len(rig.driver.writes)
    rig.step()
    rig.step()
    assert all(row[0] != 'send' for row in rig.driver.writes[before:])
    assert rig.engine.result('stop')['confirmed']
    assert not rig.engine.result('move')['ok']


def test_stalled_native_send_trips_watchdog_after_return():
    rig = Rig()
    rig.prepare()
    rig.driver.on_send = lambda: setattr(rig.clock, 'now', rig.clock.now+.2)
    rig.step()
    rig.driver.on_send = None
    rig.step()
    assert ('emergency',) in rig.driver.writes


def test_blocked_target_reports_failure_not_grasp_success():
    rig = Rig(settle_timeout_s=.1)
    rig.prepare()
    rig.driver.follow = False
    rig.engine.submit('move', 'move', {'positions': [.1]*20})
    for _ in range(120):
        rig.step()
    result = rig.engine.result('move')
    assert result['outcome'] == 'target_not_reached' and not result['ok'] and not result['reached']


def test_hold_while_disarmed_falls_back_to_emergency():
    rig = Rig()
    rig.prepare()
    rig.armed = False
    rig.engine.submit('hold', 'hold')
    rig.step()
    rig.step()
    assert rig.engine.result('hold')['requested'] == 'emergency'
    assert rig.engine.result('hold')['confirmed']
