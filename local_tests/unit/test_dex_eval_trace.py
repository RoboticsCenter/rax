"""Trace format I/O and validation, and whole-trial evaluation on synthetic trace directories."""
import json

import numpy as np
import pytest

from services.dex_eval import trace
from services.dex_eval.evaluate import EvalConfig, evaluate_trial
from services.dex_eval.outcome import outcome
from services.dex_eval.synthetic import (breakaway, demo_waypoints, hand_arrays, second_order, waypoint_reference,
                                         write_trial)

FS = 200.0


def smooth_hand(seed=0, disabled=()):
    reference = waypoint_reference(demo_waypoints(np.random.default_rng(seed)), FS, move_s=1.5, hold_s=1.5)
    measured = second_order(reference, FS) + np.random.default_rng(seed).normal(0, 1e-4, reference.shape)
    return hand_arrays(reference, measured, FS, disabled=disabled)


def sticky_hand(seed=0):
    reference = waypoint_reference(demo_waypoints(np.random.default_rng(seed)), FS, move_s=6.0, hold_s=1.5)
    measured, _ = breakaway(reference, FS, break_rad=0.012)
    return hand_arrays(reference, measured, FS)


def test_joint_names_match_the_wuji_service():
    from services.wuji.config import JOINT_NAMES
    assert trace.JOINT_NAMES == JOINT_NAMES


def test_round_trip_and_disabled_joints(tmp_path):
    arrays = smooth_hand(disabled=['thumb_S3', 'middle_S1'])
    arrays['temperature_c'] = np.full((arrays['t'].size, 20), 40.0)
    write_trial(tmp_path / 'trial', {'left': arrays}, meta={'skill': 'grasp', 'variant': 'v1', 'backend': 'sim'})
    errors, warnings = trace.validate_trial_dir(tmp_path / 'trial')
    assert errors == [] and warnings == []
    loaded = trace.load_trial(tmp_path / 'trial')
    hand = loaded.hands['left']
    assert np.array_equal(hand.t, arrays['t'])
    assert np.isnan(hand.q_meas[:, trace.JOINT_NAMES.index('thumb_S3')]).all()
    active = hand.active()
    assert active.sum() == 18 and not active[trace.JOINT_NAMES.index('middle_S1')]
    assert 'temperature_c' in hand.extra
    assert loaded.meta['backend'] == 'sim' and loaded.t0 == pytest.approx(arrays['t'][0])
    assert trace.find_trial_dirs(tmp_path) == [tmp_path / 'trial']


def test_validation_reports_contract_violations(tmp_path):
    arrays = smooth_hand()
    errors, _ = trace.validate_hand_arrays({**arrays, 'q_ref': arrays['q_ref'][:, :19]})
    assert any('q_ref must have shape' in e for e in errors)
    t = arrays['t'].copy()
    t[10] = t[5]
    errors, _ = trace.validate_hand_arrays({**arrays, 't': t})
    assert any('not monotonic' in e for e in errors)
    t = arrays['t'].copy()
    t[11] = t[10]
    errors, warnings = trace.validate_hand_arrays({**arrays, 't': t})
    assert errors == [] and any('duplicate' in w for w in warnings)
    errors, _ = trace.validate_hand_arrays({k: v for k, v in arrays.items() if k != 'kd'})
    assert errors and 'missing' in errors[0]
    errors, _ = trace.validate_meta({'sides': ['middle'], 'backend': 'robot', 'nominal_rate_hz': 0, 'success': 'y'})
    text = ' '.join(errors)
    for fragment in ('missing keys', 'sides', 'backend', 'nominal_rate_hz', 'success'):
        assert fragment in text
    write_trial(tmp_path / 'bad', {'right': arrays})
    meta = json.loads((tmp_path / 'bad' / 'meta.json').read_text())
    meta['sides'] = ['left', 'right']
    (tmp_path / 'bad' / 'meta.json').write_text(json.dumps(meta))
    with pytest.raises(trace.TraceError, match='hand_left.npz missing'):
        trace.load_trial(tmp_path / 'bad')
    with pytest.raises(trace.TraceError):
        trace.save_hand(tmp_path / 'x.npz', t=arrays['t'], q_ref=arrays['q_ref'], q_target=arrays['q_ref'],
                        q_meas=arrays['q_meas'][:-1])


def test_outcome_precedence_and_result_normalization():
    ok = {'ok': True, 'measured': {'status': 'success'}, 'why': ''}
    assert outcome(ok)['success'] is True
    assert outcome({'ok': True, 'measured': {'status': 'dropped'}, 'why': 'fell'})['success'] is False
    assert outcome(ok, {'success': False}) == {**outcome(ok), 'success': False, 'success_source': 'meta'}
    assert outcome(None, {'success': None})['success'] is None
    weird = outcome({'ok': 1, 'measured': {'status': 'great'}})
    assert weird['success'] is False and any('not one of' in p for p in weird['problems'])
    assert outcome('nope')['status'] == 'aborted'


def test_evaluate_trial_separates_smooth_from_stick_slip(tmp_path):
    write_trial(tmp_path / 'smooth', {'right': smooth_hand(), 'left': smooth_hand(1, ['thumb_S3', 'middle_S1'])})
    write_trial(tmp_path / 'sticky', {'right': sticky_hand()})
    good = evaluate_trial(tmp_path / 'smooth', EvalConfig(fingertip=False),
                          {'ok': True, 'measured': {'status': 'success'}, 'why': ''})
    bad = evaluate_trial(tmp_path / 'sticky', EvalConfig(fingertip=False))
    g, b = good['summary'], bad['summary']
    assert g['success'] is True and b['success'] is None
    assert g['sparc_ratio'] < 1.15 < b['sparc_ratio']
    assert g['sparc'] > b['sparc']
    assert g['stick_slip_count'] == 0 and b['stick_slip_count'] > 50
    assert g['tracking_rms_motion_rad'] < 0.03
    assert g['timing_rate_ratio'] == pytest.approx(1.0)
    assert set(good['hands']['left']['disabled_joints']) == {'thumb_S3', 'middle_S1'}
    assert 'thumb_S3' not in good['hands']['left']['tracking']['rms_per_joint']
    assert good['hands']['right']['fingertip']['available'] is False
    json.dumps(good, allow_nan=False)  # JSON-safe.


def test_evaluate_trial_with_arm_audio_cache_and_jittered_clock(tmp_path):
    rng = np.random.default_rng(3)
    hand = smooth_hand()
    hand['t'] = hand['t'] + rng.uniform(0, 0.0015, hand['t'].size)
    hand['t'] = np.maximum.accumulate(hand['t'])
    arm_reference = waypoint_reference([np.zeros(6), np.full(6, 0.3)], FS, move_s=2.0)
    arm = {'t': 100.0 + np.arange(arm_reference.shape[0]) / FS, 'q_ref': arm_reference,
           'q_meas': second_order(arm_reference, FS)}
    fs_audio = 16000
    samples = rng.normal(0, 1e-3, int(4 * fs_audio))
    write_trial(tmp_path / 'trial', {'right': hand}, arms={'right': arm}, audio=(samples, fs_audio),
                meta={'audio': {'start_t': 100.0}})
    first = evaluate_trial(tmp_path / 'trial', EvalConfig(fingertip=False), cache=True)
    s = first['summary']
    assert s['arm_sparc_ratio'] == pytest.approx(1.0, abs=0.1)
    assert s['arm_tracking_lag_s'] > 0
    assert s['audio_laeq_db'] is not None and first['audio']['alignment'] == 'meta.audio.start_t'
    assert s['timing_jitter_ms'] > 0.1
    assert (tmp_path / 'trial' / 'metrics.json').is_file()
    second = evaluate_trial(tmp_path / 'trial', EvalConfig(fingertip=False), cache=True)
    assert second['summary'] == s
