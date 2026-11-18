"""Motion battery planning: dry run by default, never touching hardware."""
import importlib.util
import json
from pathlib import Path

import pytest

from services.controller.config import Configuration

PROJECT = Path(__file__).resolve().parents[2]


def battery():
    spec = importlib.util.spec_from_file_location('wuji_motion_battery', PROJECT/'scripts/wuji_motion_battery.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def session(tmp_path):
    options = json.loads((PROJECT/'examples/xarm-wuji-bimanual.json').read_text())
    options.pop('calibration_manifest', None)
    options.pop('unverified_camera_extrinsics', None)
    root = tmp_path/'sessions'/'bimanual'
    root.mkdir(parents=True)
    config = Configuration(root=root, task='battery', robot='xarm_bimanual', backend_options=options)
    (root/'operator-input.json').write_text(json.dumps(config.to_dict()))
    return root


def test_default_dry_run_plans_both_hands_and_starts_nothing(session, monkeypatch, capsys):
    module = battery()

    def forbidden(*args, **kwargs):
        raise AssertionError('a dry run must not start a hand owner or arm anything')
    monkeypatch.setattr('services.wuji.client.WujiClient', forbidden)
    monkeypatch.setattr('services.xarm.arming.write', forbidden)
    before = sorted(p.name for p in session.parent.iterdir())
    plan = module.main(['--session', str(session)])
    out = capsys.readouterr().out
    assert 'Dry run: nothing was started, armed or moved' in out
    assert sorted(p.name for p in session.parent.iterdir()) == before
    assert {t['hand'] for t in plan['trials']} == {'left', 'right'}
    assert {t['condition'] for t in plan['trials']} == set(module.DEFAULT_CONDITIONS)
    assert {t['item'] for t in plan['trials']} >= {'single_joint', 'stick_slip', 'tapping', 'hold', 'waypoints5'}
    left = [t for t in plan['trials'] if t['hand'] == 'left']
    for trial in left:  # Partial hand: disabled joints never move; speeds respect its 0.15 rad/s profile.
        home = trial['keyframes'][0][1]
        assert all(q[2] == home[2] and q[8] == home[8] for _, q in trial['keyframes'])
        assert trial['summary']['peak_spline_speed_rad_s'] <= .15+1e-9 or trial['continuous']
    assert any(s['item'] == 'fist_pinch' and s['hand'] == 'left' for s in plan['skipped'])
    assert 'cap ceiling is 0.8 A' in out and 'command_hz is 200' in out


def test_plan_validation_patch_limits_and_condition_names(session, tmp_path):
    module = battery()
    options = json.loads((session/'operator-input.json').read_text())['backend_options']
    patched = module.patched_options(options, {'all': {'command_hz': 1000, 'current_limit_a': 1.0,
                                                       'stream_max_jerk_rad_s3': 30.}})
    profiles = module.hand_profiles(patched)
    assert profiles['right']['command_hz'] == 1000 and profiles['left']['current_limit_a'][0] == 1.
    assert options['arms']['right']['end_effector']['command_hz'] == 200  # The source is untouched.
    for patch in ({'all': {'current_limit_a': 3.}}, {'all': {'disabled_joints': []}}, {'arm': {}},
                  {'right': {'max_temperature_c': 90}}):
        with pytest.raises(ValueError):
            module.patched_options(options, patch)
    plan = module.make_plan(options, hands=['right'], conditions=['M0_baseline', 'M4_online_replan'],
                            items=['waypoints5'], repeats=2, speeds=[.3, .6], hold_s=30., extra_postures={},
                            shuffle=True, seed=3)
    assert len(plan['trials']) == 2*2*2 and len({t['variant'] for t in plan['trials']}) == 2
    with pytest.raises(ValueError):
        module.make_plan(options, hands=['right'], conditions=['M0_baseline'], items=['fist_pinch'], repeats=1,
                         speeds=[.3], hold_s=1., extra_postures={'right': {'pinch': [3.]*20}}, shuffle=False, seed=0)
    with pytest.raises(SystemExit):
        module.main(['--session', str(session), '--conditions', 'C9_unknown'])
