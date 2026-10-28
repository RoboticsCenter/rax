import math

import numpy as np
import pytest

from test_wuji_controller import Rig
from wuji_fakes import hand_config
from services.wuji.controller import Trajectory
from services.wuji.motion import FeedbackPath, SettlingWindow


def feedback_config(**options):
    return hand_config(**{'motion_mode': 'feedback', 'max_velocity_rad_s': .12,
                         'max_acceleration_rad_s2': .2, 'max_jerk_rad_s3': .8,
                         'tolerance_rad': .06, 'settle_s': .4, 'settle_timeout_s': 8., **options})


@pytest.mark.parametrize('distance', [.25, 1.9])
def test_feedback_reference_stays_on_path_and_bounds_velocity_acceleration_jerk(distance):
    cfg = feedback_config(max_step_rad=2.1)
    goal = [.5*distance, -.4*distance, .2*distance, 0.]*5
    start = [-q for q in goal]
    trajectory = Trajectory(start, goal, cfg)
    path = FeedbackPath(trajectory, cfg, 0.)
    samples, velocities, scales = [], [], []
    dt = .002
    for i in range(math.ceil((trajectory.duration+12)/dt)+1):
        now = i*dt
        # Alternate perfect following and a large lag, forcing full speed-scale transitions.
        lag = .2 if 1.5 < now < 4.5 or 7 < now < 10 else 0.
        measured = [q-lag for q in path.positions]
        q, v = path.advance(now, measured)
        samples.append(q); velocities.append(v); scales.append(path.scale)
    samples, velocities = np.array(samples), np.array(velocities)
    acceleration = np.diff(velocities, axis=0)/dt
    jerk = np.diff(acceleration, axis=0)/dt
    assert np.max(abs(velocities)) <= cfg['max_velocity_rad_s']+1e-9
    assert np.max(abs(acceleration)) <= cfg['max_acceleration_rad_s2']+1e-3
    assert np.max(abs(jerk)) <= cfg['max_jerk_rad_s3']+.01
    assert np.max(abs(np.diff(samples, axis=0)/dt-velocities[1:])) < .001
    assert min(scales) < .01 and path.complete
    assert all(np.min(samples[:, j]) >= min(start[j], target)-1e-12 and
               np.max(samples[:, j]) <= max(start[j], target)+1e-12 for j, target in enumerate(goal))
    assert np.min(np.diff(samples[:, 0])) >= -1e-12
    assert samples[:, 1] == pytest.approx(samples[:, 0]*(-.2/.25))
    assert samples[-1] == pytest.approx(goal)
    assert velocities[0] == pytest.approx([0.]*20)
    assert velocities[-1] == pytest.approx([0.]*20)


def test_unreachable_joint_pauses_progress_and_times_out_without_claiming_success():
    rig = Rig(**feedback_config(feedback_slow_error_rad=.01, feedback_pause_error_rad=.03,
                                settle_timeout_s=2.))
    rig.prepare(); rig.driver.follow = False
    rig.engine.submit('move', 'move', {'positions': [.3]*20})
    for _ in range(2000):
        rig.step()
        if rig.engine.result('move')['done']:
            break
    result = rig.engine.result('move')
    assert result['outcome'] == 'target_not_reached' and not result['reached']
    assert not result['trajectory_complete']
    assert result['minimum_speed_scale'] < .01
    assert result['target_positions'] == [.3]*20
    assert max(result['command_positions']) < .3
    assert rig.driver.command_velocity == [0.]*20
    assert result['seconds'] <= result['nominal_duration_s']+2.02


def test_slow_actuator_gets_more_time_with_feedback_and_reaches_stable_goal():
    rig = Rig(**feedback_config())
    rig.prepare(); rig.driver.follow = False
    rig.engine.submit('move', 'move', {'positions': [.2]*20})
    for _ in range(2500):
        rig.driver.q = [q+min(.00035, max(-.00035, target-q))
                        for q, target in zip(rig.driver.q, rig.driver.goal)]
        rig.step()
        if rig.engine.result('move')['done']:
            break
    result = rig.engine.result('move')
    assert result['reached'] and result['minimum_speed_scale'] < .99
    assert result['seconds'] > result['nominal_duration_s']+rig.cfg['settle_s']
    assert result['max_joint_error_rad'] <= .06
    assert rig.driver.command_velocity == [0.]*20


def test_stable_residual_is_reported_honestly_inside_configured_tolerance():
    rig = Rig(**feedback_config())
    rig.prepare(); rig.driver.follow = False
    rig.engine.submit('move', 'move', {'positions': [.2]*20})
    for _ in range(2000):
        rig.driver.q = [q-.0544 for q in rig.driver.goal]
        rig.step()
        if rig.engine.result('move')['done']:
            break
    result = rig.engine.result('move')
    assert result['reached']
    assert result['max_joint_error_rad'] == pytest.approx(.0544)
    assert result['accepted_tolerance_rad'] == .06
    assert result['target_positions'] == [.2]*20
    assert result['positions'] == pytest.approx([.1456]*20)
    assert max(w[1][0] for w in rig.driver.writes if w[0]=='send') <= .2


def test_settling_rejects_motion_through_tolerance_and_duplicate_feedback():
    window = SettlingWindow([0.]*20, .06, .4, .004)
    for i in range(100):
        assert not window.update(i*.01, [.03*math.sin(i*.12)]*20)
    for _ in range(100):
        assert not window.update(.99, [0.]*20)
    reached = False
    for i in range(100, 151):
        reached = window.update(i*.01, [.02]*20)
    assert reached
    assert not window.update(1.51, [.061]*20)
    assert not window.update(1.52, [.02]*20)


@pytest.mark.parametrize('options', [
    {'motion_mode': 'anything'}, {'feedback_slow_error_rad': .1},
    {'feedback_pause_error_rad': .34, 'max_tracking_error_rad': .3},
    {'max_jerk_rad_s3': 0}, {'feedback_transition_s': .01},
    {'settle_position_range_rad': .05, 'tolerance_rad': .04},
])
def test_invalid_feedback_limits_are_rejected_before_motion(options):
    with pytest.raises(ValueError):
        feedback_config(**options)


def test_feedback_nominal_duration_is_validated_before_motion():
    cfg = feedback_config()
    minimum = Trajectory([0.]*20, [.2]*20, cfg).duration
    with pytest.raises(ValueError, match='at least'):
        Trajectory([0.]*20, [.2]*20, cfg, minimum-.01)
    assert Trajectory([0.]*20, [.2]*20, cfg, minimum+1).duration == minimum+1
