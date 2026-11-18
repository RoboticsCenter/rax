"""Smooth streaming layer: pure engine math with the real local Ruckig; no hardware."""
import math

import pytest
from wuji_fakes import hand_config

from services.wuji.config import JOINT_NAMES
from services.wuji.smooth import (PRESETS, StreamEngine, check_tool_args, friction_feedforward, parse_stream,
                                  parse_targets, parse_trajectory, profile_parameters, profile_summary,
                                  resolve_settings)

I = JOINT_NAMES.index('index_S1')


def config(**options):
    return hand_config(motion_mode='scurve', command_hz=1000, max_velocity_rad_s=.6, max_acceleration_rad_s2=3.,
                       max_jerk_rad_s3=2., max_step_rad=2.1, stream_max_acceleration_rad_s2=6.,
                       stream_max_jerk_rad_s3=40., **options)


class Sim:
    """Engine at 1 kHz with perfect tracking feedback."""
    def __init__(self, cfg=None, q=None, **settings):
        self.cfg = cfg or config()
        self.engine = StreamEngine(self.cfg)
        self.engine.reset(q or [0.]*20)
        if settings:
            self.engine.apply({'kind': 'settings', 'settings': resolve_settings(settings, self.cfg)}, 0.)
        self.now, self.rows = 0., []
        self.state = {'positions': list(q or [0.]*20), 'velocity_rad_s': [0.]*20}
        self.measured = {}  # Joint index -> fixed measured position (otherwise perfect tracking).

    def target(self, **args):
        self.engine.apply(parse_stream(args, self.cfg, self.engine.latest_target), self.now)

    def step(self, n=1, bias=None, caps=None):
        for _ in range(n):
            self.now += .001
            q, v, a, offset, friction = self.engine.step(self.now, self.state, bias or [0.]*20, caps or [.8]*20)
            self.rows.append((self.now, list(q), list(v), list(a), list(offset), list(friction)))
            self.state = {'positions': list(q), 'velocity_rad_s': list(v)}
            for i, value in self.measured.items():
                self.state['positions'][i] = value
        return self.rows[-1]


def test_targets_merge_subsets_relative_and_reject_bad_requests():
    cfg = config()
    base = [.1]*20
    indices, merged = parse_targets({'fingers': {'index': [.2, .1, .3, .4]}}, cfg, base)
    assert indices == (4, 5, 6, 7) and merged[4:8] == [.2, .1, .3, .4] and merged[0] == .1
    indices, merged = parse_targets({'joints': {'thumb_S2': -.05}, 'relative': True}, cfg, base)
    assert indices == (1,) and merged[1] == pytest.approx(.05)
    for args in ({'positions': [0.]*19}, {'joints': {'index_S9': 0.}}, {'fingers': {'index': [0.]*3}},
                 {'positions': [2.]*20}, {'joints': {'index_S1': math.nan}}, {}, {'positions': [0.]*20, 'joints': {}}):
        with pytest.raises(ValueError):
            parse_targets(args, cfg, base)
    with pytest.raises(ValueError, match='max_step'):
        parse_targets({'joints': {'index_S1': .9}}, hand_config(max_step_rad=.2), [0.]*20)
    partial = hand_config(disabled_joints=['thumb_S3', 'middle_S1'])
    with pytest.raises(ValueError, match='disabled'):
        parse_targets({'joints': {'thumb_S3': .2}}, partial, [0.]*20)
    delta = [0. if i in (2, 8) else .05 for i in range(20)]
    indices, merged = parse_targets({'positions': delta, 'relative': True}, partial, [0.]*20)
    assert 2 not in indices and merged[2] == 0. and merged[3] == .05  # Disabled entries retained.


def test_streamed_ramp_tracks_without_stops_and_rests_at_the_last_target():
    sim = Sim()
    for k in range(37):  # 30 Hz targets on a 0.3 rad/s ramp, then the caller stops.
        sim.target(joints={'index_S1': min(.36, .01*k)})
        sim.step(33)
    sim.step(800)
    t, q, v, a = zip(*[(r[0], r[1][I], r[2][I], r[3][I]) for r in sim.rows])
    steady = [x for tt, x in zip(t, v) if .4 < tt < 1.15]
    assert min(steady) > .28 and max(steady) < .33          # No stop-go between targets.
    assert max(q) <= .36+1e-12 and q[-1] == pytest.approx(.36)  # No overshoot of the final target.
    assert max(map(abs, v)) <= .6+1e-9 and max(map(abs, a)) <= 6.+1e-9
    jerk = max(abs(a2-a1)/.001 for a1, a2 in zip(a, a[1:]))
    assert jerk <= 40.+1e-6
    assert sim.engine.converged and sim.engine.settle.settled


def test_profile_limits_bound_the_reference_and_cannot_be_raised():
    sim = Sim(max_velocity_rad_s=.2, max_acceleration_rad_s2=1., max_jerk_rad_s3=5.)
    sim.target(joints={'index_S1': .5})
    sim.step(4000)
    v = [abs(r[2][I]) for r in sim.rows]
    a = [abs(r[3][I]) for r in sim.rows]
    assert max(v) <= .2+1e-9 and max(a) <= 1.+1e-9 and sim.rows[-1][1][I] == pytest.approx(.5)
    for update in ({'max_velocity_rad_s': .7}, {'max_jerk_rad_s3': 41.}, {'command_hz': 2000},
                   {'synchronization': 'arm'}, {'friction_current_a': .31}, {'unknown': 1}):
        with pytest.raises(ValueError):
            resolve_settings(update, sim.cfg)
    per_joint = resolve_settings({'max_velocity_rad_s': [.1]*4+[.5]*16}, sim.cfg)
    assert per_joint['max_velocity_rad_s'][0] == .1 and per_joint['max_velocity_rad_s'][4] == .5


def test_minimum_jerk_duration_and_first_order_filter():
    sim = Sim()
    sim.target(joints={'index_S1': .3}, duration_s=1.)
    sim.step(1500)
    q = [r[1][I] for r in sim.rows]
    assert q[499] == pytest.approx(.15, abs=.015) and q[-1] == pytest.approx(.3)  # Lookahead: little lag.
    assert max(q) <= .3+1e-12
    filtered = Sim(filter_tau_s=.1, interpolation_s=0., lookahead=False)
    filtered.target(joints={'index_S1': .2})
    filtered.step(100)
    assert filtered.engine.filtered[I] == pytest.approx(.2*(1-math.exp(-1)), rel=.02)
    filtered.step(2000)
    assert filtered.engine.converged and filtered.rows[-1][1][I] == pytest.approx(.2)


def test_trajectory_passes_waypoints_monotonically_and_stretches_time():
    sim = Sim()
    command = parse_trajectory({'waypoints': [{'t': 1., 'joints': {'index_S1': .2}}, {'t': 2., 'joints': {'index_S1': .4}},
                                              {'t': 3., 'joints': {'index_S1': .1}}]}, sim.cfg, sim.engine.latest_target)
    sim.engine.apply(command, sim.now)
    assert command['time_scale'] == pytest.approx(1.)
    goals, refs = [], []
    for _ in range(4000):
        sim.step()
        goals.append(sim.engine.goal[I])
        refs.append(sim.rows[-1][1][I])
    assert max(goals) == pytest.approx(.4, abs=1e-9) and min(goals[1100:]) >= .1-1e-9  # No overshoot.
    assert goals[999] == pytest.approx(.2, abs=1e-3) and goals[1999] == pytest.approx(.4, abs=1e-3)
    assert max(refs) <= .4+1e-9 and min(refs[2100:]) >= .1-1e-9
    assert sim.engine.converged and sim.rows[-1][1][I] == pytest.approx(.1)
    fast = Sim()
    command = parse_trajectory({'waypoints': [{'t': .1, 'joints': {'index_S1': .5}}]}, fast.cfg, [0.]*20)
    fast.engine.apply(command, 0.)
    assert command['time_scale'] > 1.5  # 5 rad/s requested; stretched to the 0.6 rad/s ceiling.
    with pytest.raises(ValueError, match='max_duration'):
        Sim(config(max_duration_s=1.)).engine.apply(
            parse_trajectory({'waypoints': [{'t': .1, 'joints': {'index_S1': .9}},
                                            {'t': .2, 'joints': {'index_S1': -.9}}]}, config(), [0.]*20), 0.)
    with pytest.raises(ValueError, match='increase'):
        parse_trajectory({'waypoints': [{'t': .5, 'positions': [0.]*20}, {'t': .5, 'positions': [0.]*20}]},
                         sim.cfg, [0.]*20)


def test_new_trajectory_continues_from_the_moving_goal_without_a_stop():
    sim = Sim()
    sim.engine.apply(parse_trajectory({'waypoints': [{'t': 1., 'joints': {'index_S1': .4}}]}, sim.cfg, [0.]*20), 0.)
    sim.step(500)
    moving = sim.engine.goal_velocity(sim.now, I)
    assert moving > .2
    sim.engine.apply(parse_trajectory({'waypoints': [{'t': 1., 'joints': {'index_S1': .6}}]}, sim.cfg,
                                      sim.engine.latest_target), sim.now)
    assert sim.engine.goal_velocity(sim.now+.001, I) == pytest.approx(moving, rel=.1)  # Slope carried over.
    sim.step(2000)
    v = [r[2][I] for r in sim.rows[400:1100]]
    assert min(v) > .1  # The reference never stopped between chunks.


def test_friction_feedforward_is_bounded_and_counts_toward_the_cap():
    cfg = config(current_limit_a=.8)
    settings = resolve_settings({'friction': True}, cfg)
    v = [0.]*20
    v[4], v[5], v[6] = .3, -.001, 0.
    f = friction_feedforward(v, [0.]*20, settings, cfg, [0.]*20, [.8]*20)
    assert f[4] == pytest.approx(.08*math.tanh(.3/.03)) and f[5] < 0 and abs(f[5]) < .003 and f[6] == 0.
    bias = [0.]*20
    bias[4] = .38
    capped = friction_feedforward(v, [0.]*20, settings, cfg, bias, [.8]*20)
    assert bias[4]+capped[4] == pytest.approx(.4)  # max_feedforward_fraction 0.5 x 0.8 A.
    runaway = [0.]*20
    runaway[4] = 1.
    assert friction_feedforward(v, runaway, settings, cfg, [0.]*20, [.8]*20)[4] == 0.
    off = resolve_settings({}, cfg)
    assert not any(friction_feedforward(v, None, off, cfg, [0.]*20, [.8]*20))
    partial = hand_config(disabled_joints=['index_S1'], current_limit_a=.8)
    assert friction_feedforward(v, None, resolve_settings({'friction': True}, partial), partial,
                                [0.]*20, [.8]*20)[4] == 0.


def test_hold_deadband_engages_after_delay_and_is_bounded():
    sim = Sim(hold_deadband_rad=.004)
    sim.measured[I] = .003  # Measured inside the band.
    sim.step(150)
    assert sim.engine.weight == 0.  # hold_delay_s = 0.2 s
    sim.step(500)
    assert sim.engine.weight == 1. and sim.rows[-1][4][I] == pytest.approx(.003, abs=2e-4)
    sim.measured[I] = .05  # A glitch far outside moves the command at most the band.
    sim.step(100)
    assert sim.rows[-1][4][I] == pytest.approx(.004)
    sim.target(joints={'index_S1': .2})
    sim.step(80)
    assert sim.engine.weight == 0.  # Released while moving.


def test_brake_reaches_rest_and_clamps_to_joint_limits():
    sim = Sim()
    sim.target(joints={'index_S1': .9}, duration_s=1.)
    sim.step(400)
    sim.engine.brake()
    while sim.engine.braking:
        sim.step()
    assert sim.engine.converged and sim.rows[-1][2][I] == 0. and sim.rows[-1][1][I] < .9
    edge = Sim(q=[.99]+[0.]*19)
    edge.engine._v[0] = .6  # Would overshoot the 1.0 rad limit while braking.
    edge.engine.sync([.99]+[0.]*19, edge.engine._v)
    edge.engine.brake()
    while edge.engine.braking:
        edge.step()
    assert max(r[1][0] for r in edge.rows) <= 1.0 and edge.engine.limit_clamps > 0


def test_partial_hand_engine_never_moves_disabled_joints():
    cfg = config(disabled_joints=['thumb_S3', 'middle_S1'])
    sim = Sim(cfg, q=[.1]*20, friction=True)
    sim.target(positions=[.3]*2+[.1]+[.3]*5+[.1]+[.3]*11)
    sim.step(2000)
    for row in sim.rows:
        assert row[1][2] == row[1][8] == .1 and row[2][2] == row[2][8] == 0. and row[5][2] == row[5][8] == 0.


def test_profile_presets_use_gain_scale_and_operator_ceilings():
    cfg = hand_config(kp=8., kd=.15, current_limit_a=.8)
    desired, info = profile_parameters({'name': 'soft'}, cfg, {'current_limit_a': [.8]*20})
    assert desired['kp'][0] == pytest.approx(PRESETS['soft']['stiffness_a_per_rad']/1.98)
    assert desired['kd'][0] == pytest.approx(PRESETS['soft']['damping_a_s_per_rad']/1.98)
    firm, _ = profile_parameters({'name': 'firm'}, cfg, {'current_limit_a': [.8]*20})
    assert firm['kp'] == [8.]*20 and firm['kd'] == [.15]*20  # Clamped to the profile.
    hold, info = profile_parameters({'name': 'hold', 'current_limit_a': .5}, cfg, {'current_limit_a': [.8]*20})
    assert info['hold_deadband_rad'] == .004 and hold['current_limit_a'] == [.5]*20
    for args in ({'name': 'soft', 'current_limit_a': .9}, {'name': 'soft', 'kp': 9.}, {'name': 'max'},
                 {'name': 'soft', 'kp': 0.}):
        with pytest.raises(ValueError):
            profile_parameters(args, cfg, {'current_limit_a': [.8]*20})
    partial = hand_config(kp=8., kd=.15, current_limit_a=.8, disabled_joints=['thumb_S3'])
    desired, _ = profile_parameters({'name': 'default'}, partial, {'current_limit_a': [.8]*20})
    assert desired['kp'][2] == desired['kd'][2] == desired['current_limit_a'][2] == 0.
    summary = profile_summary(firm, cfg, 'firm')
    assert summary['effective_stiffness_a_per_rad'] == pytest.approx(15.84)
    assert summary['cap_saturation_error_rad'] == pytest.approx(.8/15.84, rel=1e-4)


def test_config_bounds_and_structural_tool_checks():
    for options in ({'friction_current_a': .31}, {'max_feedforward_fraction': 0.}, {'gain_scale': 10.},
                    {'stream_max_jerk_rad_s3': 0.}, {'max_record_s': 1000.}):
        with pytest.raises(ValueError):
            hand_config(**options)
    cfg = hand_config()
    assert cfg['friction_current_a'] == [.08]*20 and cfg['stream_max_jerk_rad_s3'] is None
    check_tool_args('wuji_stream', {'joints': {'index_S1': .2}, 'relative': True}, cfg)
    for name, args in [('wuji_stream', {'joints': {'index_S1': .2}, 'dry_run': True}),
                       ('wuji_trajectory', {'waypoints': []}),
                       ('wuji_trajectory', {'waypoints': [{'t': 1, 'joints': {'index_S1': .1}}],
                                            'record': {'params': {'x': math.inf}}}),
                       ('wuji_wait', {'timeout_s': 61}), ('wuji_set_profile', {'name': 'turbo'}),
                       ('wuji_stream_config', {'command_hz': 5000})]:
        with pytest.raises(ValueError):
            check_tool_args(name, args, cfg)
