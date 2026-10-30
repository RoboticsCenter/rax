"""Real local Ruckig planner and simulated hand; never connects to hardware."""
import json
import math
from pathlib import Path

import pytest
from test_wuji_controller import Rig
from wuji_fakes import hand_config

from services.wuji.config import validate_config, validate_tool
from services.wuji.scurve import SCurvePath, next_deadline


def config(**options):
    return hand_config(motion_mode='scurve', command_hz=1000, max_velocity_rad_s=.6,
                       max_acceleration_rad_s2=3., max_jerk_rad_s3=2., max_step_rad=2.1, **options)


def rig(**options):
    return Rig(**config(**options))


def finish(r, ident):
    for _ in range(12000):
        r.step(.001)
        if r.engine.result(ident)['done']:
            return r.engine.result(ident)
    pytest.fail(f'command did not finish: {r.engine.status()}')


def test_selected_profile_and_config_validation():
    path = Path(__file__).resolve().parents[2]/'examples'/'xarm-wuji.json'
    cfg = validate_config(json.loads(path.read_text())['end_effector'])
    assert cfg['motion_mode'] == 'scurve' and cfg['command_hz'] == 1000
    assert cfg['kp'] == [8.]*20 and cfg['kd'] == [.15]*20 and cfg['current_limit_a'] == [1.5]*20
    assert (cfg['max_velocity_rad_s'], cfg['max_acceleration_rad_s2'], cfg['max_jerk_rad_s3']) == (.6, 3., 2.)
    # Preserve the commissioned RAX home instead of replacing it with zero.
    assert cfg['postures']['home'][0] == .099798
    for options in ({'command_hz': 1001}, {'allow_encoder_warning': 1}, {'motion_mode': 'unknown'}):
        with pytest.raises(ValueError):
            hand_config(**options)
    assert not hand_config()['allow_encoder_warning']


def test_full_hand_phase_sync_derivatives_and_exact_rest_endpoints():
    start = [(-1)**i*.1 for i in range(20)]
    goal = [(-1)**i*(.2+i*.03) for i in range(20)]
    cfg = config()
    path = SCurvePath(start, goal, cfg)
    assert path.sample_state(0) == (start, [0.]*20, [0.]*20)
    old_q, old_v, old_a = start, [0.]*20, [0.]*20
    while not path.complete:
        q, v, a = path.advance()
        progress = [(p-s)/(g-s) for p, s, g in zip(q, start, goal)]
        assert max(progress)-min(progress) < 1e-10
        for p, last, speed, last_speed, acc, last_acc in zip(q, old_q, v, old_v, a, old_a):
            assert math.isfinite(p) and abs(speed) <= .6+1e-9 and abs(acc) <= 3.+1e-9
            assert abs(acc-last_acc) <= 2.*path.dt+1e-9
            # Matching position/velocity, including intervals straddling a jerk switch.
            assert abs((p-last)/path.dt-(speed+last_speed)/2) < 1e-6
            assert abs(speed-last_speed) <= 3.*path.dt+1e-9
        old_q, old_v, old_a = q, v, a
    assert q == goal and v == a == [0.]*20
    assert path.status()['peak_nominal_reference_jerk_rad_s3'] == pytest.approx(2.)


def test_explicit_duration_rounding_noop_and_invalid_paths():
    cfg = config()
    short = SCurvePath([0.]*20, [.2]*20, cfg)
    with pytest.raises(ValueError, match='at least'):
        SCurvePath([0.]*20, [.2]*20, cfg, .1)
    long = SCurvePath([0.]*20, [.2]*20, cfg, short.duration+1.0004)
    assert long.duration == pytest.approx(short.duration+1.001)
    same = SCurvePath([0.]*20, [0.]*20, cfg)
    for _ in range(100):
        assert same.advance() == ([0.]*20, [0.]*20, [0.]*20)
    assert same.complete
    with pytest.raises(ValueError, match='max_step'):
        SCurvePath([0.]*20, [.5]*20, hand_config(motion_mode='scurve'))
    for bad in [[float('nan')]*20, [True]*20, [0.]*19, [1.1]*20]:
        with pytest.raises(ValueError):
            SCurvePath([0.]*20, bad, cfg)


def test_replan_and_hold_brake_preserve_initial_reference_state_and_limits():
    cfg = config()
    initial = SCurvePath([0.]*20, [.8]*20, cfg)
    for _ in range(400):
        q, v, a = initial.advance()
    for brake in (False, True):
        path = SCurvePath(q, q if brake else [-.2]*20, cfg, velocity=v, acceleration=a, brake=brake)
        assert path.sample_state(0) == (q, v, a)
        while not path.complete:
            path.advance()  # Checks every q/v/a and acceleration-difference jerk bound.
        assert path.velocity == path.acceleration == [0.]*20
        if brake:
            assert path.goal[0] > q[0] and path.duration <= cfg['stop_timeout_s']
    with pytest.raises(ValueError, match='joint limits'):
        SCurvePath([.999]*20, [.999]*20, cfg, velocity=[.5]*20, brake=True)


def test_initialize_from_measured_pose_and_refuse_moving_initialization():
    r = rig()
    r.driver.goal = [.123]*20
    r.prepare()
    assert r.driver.writes[1] == ('send', [.123]*20)
    assert r.engine.command_velocity == r.engine.command_acceleration == [0.]*20
    r = rig()
    r.driver.velocity = [.1]*20
    r.armed = True
    r.engine.submit('prepare', 'prepare')
    r.step(.001)
    assert r.engine.result('prepare')['outcome'] == 'rejected' and not r.driver.writes


def test_missing_planner_refuses_prepare_before_any_motor_write(monkeypatch):
    def missing():
        raise RuntimeError('Install ruckig==0.19.4')
    monkeypatch.setattr('services.wuji.controller.require_ruckig', missing)
    r = rig()
    r.armed = True
    r.engine.submit('prepare', 'prepare')
    r.step(.001)
    assert 'ruckig' in r.engine.result('prepare')['reason'] and not r.driver.writes


def test_prepare_remeasures_after_profile_and_enable_rpc_is_not_a_stream_gap():
    r = rig(max_scheduler_gap_s=.05)
    original_profile, original_enable = r.driver.apply_profile, r.driver.enable
    def profile():
        original_profile()
        r.clock.now += .1
        r.driver.goal = [.2]*20
        r.driver.refresh()
    def enable():
        original_enable()
        r.clock.now += .1
        r.driver.refresh()
    r.driver.apply_profile, r.driver.enable = profile, enable
    r.prepare()
    assert r.driver.writes[1] == ('send', [.2]*20)
    assert r.engine.max_command_interval < .05 and not r.engine.fault


def test_full_hand_sequence_reaches_measured_target_and_keeps_holding():
    r = rig()
    r.prepare()
    for index, goal in enumerate(([(-1)**i*(.05+i*.01) for i in range(20)], [0.]*20, [.3]*20)):
        ident = f'move{index}'
        r.engine.submit(ident, 'move', {'positions': goal})
        result = finish(r, ident)
        assert result['outcome'] == 'reached' and result['trajectory_complete']
        assert result['max_joint_error_rad'] == pytest.approx(0.)
        assert result['peak_nominal_reference_jerk_rad_s3'] <= 2.+1e-7
        assert r.driver.goal == goal and r.driver.command_velocity == [0.]*20
        r.step(.001)
        assert r.driver.goal == goal
    assert r.engine.status()['command_timing']['commands'] > 1000


def test_home_uses_custom_scurve_and_keeps_prepared_gains_and_damping():
    home = [(-1)**i*(.1+i*.01) for i in range(20)]
    r = rig(kp=8., kd=.15, current_limit_a=1.5, postures={'home': home})
    r.prepare()
    before = {key: list(value) for key, value in r.driver.control_parameters.items()}
    # The MCP alias resolves to the same private move operation as arbitrary targets.
    goal = validate_tool('wuji_home', {}, r.cfg)
    r.engine.submit('home', 'move', goal)
    result = finish(r, 'home')
    assert result['reached'] and result['trajectory_complete']
    assert result['peak_nominal_reference_jerk_rad_s3'] <= 2.+1e-7
    assert r.driver.goal == home and r.driver.command_velocity == [0.]*20
    assert r.driver.control_parameters == before
    assert [w[0] for w in r.driver.writes].count('profile') == 1  # Only operator prepare.
    assert r.engine.mode == 'holding'


def test_hold_brakes_smoothly_and_cancels_old_move():
    r = rig()
    r.prepare()
    r.engine.submit('move', 'move', {'positions': [.8]*20})
    for _ in range(400):
        r.step(.001)
    q, v, a = r.engine.target[:], r.engine.command_velocity[:], r.engine.command_acceleration[:]
    r.engine.submit('hold', 'hold')
    r.step(.001)
    assert not r.engine.result('move')['ok'] and not r.engine.result('hold')['done']
    assert r.engine.command_velocity[0] > 0
    assert abs(r.engine.command_velocity[0]-v[0]) <= 3.*.001
    assert abs(r.engine.command_acceleration[0]-a[0]) <= 2.*.001+1e-9
    result = finish(r, 'hold')
    assert result['outcome'] == 'held' and not result['confirmed']
    assert r.driver.goal[0] > q[0] and r.engine.command_velocity == [0.]*20
    assert ('emergency',) not in r.driver.writes


def test_unbounded_braking_falls_back_to_emergency():
    r = rig()
    r.prepare()
    r.engine.target = r.driver.q = r.driver.goal = [.999]*20
    r.engine.command_velocity = [.5]*20
    r.engine.submit('hold', 'hold')
    r.step(.001)
    r.step(.001)
    assert ('emergency',) in r.driver.writes and r.engine.result('hold')['confirmed']
    assert 'joint limits' in r.engine.result('hold')['hold_fallback_reason']


def test_last_braking_send_failure_is_not_reported_held():
    r = rig()
    r.prepare()
    r.engine.submit('move', 'move', {'positions': [.8]*20})
    for _ in range(100):
        r.step(.001)
    r.engine.submit('hold', 'hold')
    r.step(.001)
    def fail_last_send():
        if r.engine.active['scurve'].complete:
            raise ConnectionError('last brake write failed')
    r.driver.on_send = fail_last_send
    result = finish(r, 'hold')
    assert result['outcome'] == 'fault' and not result['ok']
    r.driver.on_send = None
    r.step(.001)
    assert ('emergency',) in r.driver.writes


def test_hold_cannot_resume_control_after_a_controller_fault():
    r = rig()
    r.prepare()
    r.engine.fault = 'reference deadline exceeded'
    r.engine.submit('hold', 'hold')
    before = len(r.driver.writes)
    r.step(.001)
    r.step(.001)
    assert r.engine.result('hold')['confirmed']
    assert r.driver.writes[before:] == [('emergency',)]


def test_short_stall_does_not_jump_ahead_and_long_stall_stops():
    r = rig(max_scheduler_gap_s=.05)
    r.prepare()
    r.engine.submit('move', 'move', {'positions': [.2]*20})
    r.step(.001)
    path = r.engine.active['scurve']
    r.step(.02)
    assert path.clock_s == pytest.approx(.002)  # One tick, not twenty catch-up steps.
    r.step(.06)
    r.step(.001)
    assert ('emergency',) in r.driver.writes
    assert next_deadline(10., 10.0005, .001) == (10.001, 0)
    due, missed = next_deadline(10., 10.02, .001)
    assert due == pytest.approx(10.021) and missed >= 19


def test_obstructed_target_is_not_reported_reached():
    r = rig(settle_timeout_s=.1)
    r.prepare()
    r.driver.follow = False
    r.engine.submit('move', 'move', {'positions': [.1]*20})
    result = finish(r, 'move')
    assert result['outcome'] == 'target_not_reached' and not result['reached']
    assert result['trajectory_complete'] and r.driver.goal == [.1]*20
    assert r.driver.command_velocity == [0.]*20


def test_stop_during_planning_fences_reference_before_sdk_write(monkeypatch):
    r = rig()
    r.prepare()
    def stopped_plan(*args, **kwargs):
        path = SCurvePath(*args, **kwargs)
        r.engine.submit('stop', 'emergency')
        return path
    monkeypatch.setattr('services.wuji.controller.SCurvePath', stopped_plan)
    before = len(r.driver.writes)
    r.engine.submit('move', 'move', {'positions': [.2]*20})
    r.step(.001)
    r.step(.001)
    r.step(.001)
    assert not any(row[0] == 'send' for row in r.driver.writes[before:])
    assert r.engine.result('stop')['confirmed'] and not r.engine.result('move')['ok']
