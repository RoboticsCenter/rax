"""Synthetic-signal tests for services.dex_eval metrics (known answers and orderings)."""
import builtins
import math

import numpy as np
import pytest

from services.dex_eval import audio, metrics, stats, thresholds
from services.dex_eval.metrics import MetricConfig
from services.dex_eval.synthetic import breakaway, min_jerk, second_order, waypoint_reference

FS = 1000.0
CFG = MetricConfig()


def minjerk_profile(T=1.0, A=1.0, fs=FS):
    t = np.arange(0.0, T + 1e-9, 1 / fs)
    u = t / T
    position = A * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5)
    speed = A / T * (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4)
    return position, speed


# --------------------------------------------------------------------------- SPARC / LDLJ / jerk

def test_sparc_matches_the_analytic_arc_length_of_a_gaussian_speed_profile():
    sigma = 0.1
    t = np.arange(-1.0, 1.0, 1 / FS)
    speed = np.exp(-t ** 2 / (2 * sigma ** 2))
    # |F(f)| / |F(0)| = exp(-2 pi^2 sigma^2 f^2); adaptive cut-off where it falls to 0.05 (< 10 Hz here).
    fc = math.sqrt(math.log(1 / 0.05) / (2 * math.pi ** 2 * sigma ** 2))
    f = np.linspace(0.0, fc, 200001)
    m = np.exp(-2 * math.pi ** 2 * sigma ** 2 * f ** 2)
    expected = -np.sum(np.sqrt((np.diff(f) / fc) ** 2 + np.diff(m) ** 2))
    assert metrics.sparc(speed, FS) == pytest.approx(expected, abs=0.01)


def test_sparc_of_minimum_jerk_is_about_minus_1_4_and_invariant_to_scale_duration_and_idle():
    _, speed = minjerk_profile()
    value = metrics.sparc(speed, FS)
    assert value == pytest.approx(-1.40, abs=0.02)
    _, slow = minjerk_profile(T=2.0, A=3.0)
    assert metrics.sparc(slow, FS) == pytest.approx(value, abs=0.01)
    padded = np.concatenate([np.zeros(500), speed, np.zeros(1500)])
    assert metrics.sparc(padded, FS) == pytest.approx(value, abs=0.01)


def test_minimum_jerk_scores_smoother_than_submovements_step_and_noisy_profiles():
    rng = np.random.default_rng(0)
    _, smooth = minjerk_profile()
    two = np.convolve(np.concatenate([smooth, np.zeros(700)]), np.r_[1.0, np.zeros(699), 1.0])[:1700]  # two bumps
    step = np.r_[np.zeros(100), np.full(800, 1.0), np.zeros(100)]   # rectangular speed: abrupt start/stop
    from scipy import signal
    tremor = signal.sosfiltfilt(signal.butter(2, 8, fs=FS, output='sos'), rng.normal(0, 1, smooth.size))
    irregular = np.abs(smooth + 0.4 * smooth.max() * tremor / tremor.std())   # band-limited speed irregularity
    white = np.abs(smooth + rng.normal(0, 0.5, smooth.size))                  # broadband noise, SNR ~4
    s = metrics.sparc(smooth, FS)
    assert s > metrics.sparc(two, FS) + 0.3
    assert s > metrics.sparc(step, FS) + 0.5
    assert s > metrics.sparc(irregular, FS) + 0.3
    # SPARC is robust to broadband noise (Balasubramanian et al. 2015); LDLJ is not.
    assert metrics.sparc(white, FS) == pytest.approx(s, abs=0.05)
    l = metrics.ldlj(smooth, FS)
    assert l > metrics.ldlj(two, FS)
    assert l > metrics.ldlj(irregular, FS) + 1
    assert l > metrics.ldlj(white, FS) + 3


def test_ldlj_and_rms_jerk_of_minimum_jerk_match_closed_form():
    position, speed = minjerk_profile(T=1.0, A=1.0)
    # integral jerk^2 = 720 A^2 / T^5 and v_peak = 1.875 A / T  =>  LDLJ = -ln(720 / 1.875^2).
    assert metrics.ldlj(speed, FS) == pytest.approx(-math.log(720 / 1.875 ** 2), abs=0.02)
    assert metrics.rms_jerk(position, FS) == pytest.approx(math.sqrt(720), rel=0.02)


def test_sparc_handles_degenerate_input():
    assert math.isnan(metrics.sparc(np.zeros(100), FS))
    assert math.isnan(metrics.sparc([1.0, np.nan, 2.0, 3.0], FS))
    assert math.isnan(metrics.ldlj(np.ones(3), FS))


def test_smoothness_pipeline_ranks_tracked_minimum_jerk_above_stick_slip_staircase():
    fs = 200.0
    reference = waypoint_reference([np.zeros(2), np.full(2, 0.4)], fs, move_s=4.0, hold_s=0.5)
    smooth = second_order(reference, fs)
    sticky, _ = breakaway(reference, fs, break_rad=0.012)
    a = metrics.smoothness(smooth, fs, CFG)
    b = metrics.smoothness(sticky, fs, CFG)
    assert a['sparc'] > b['sparc'] + 1.0
    assert a['ldlj'] > b['ldlj']
    assert set(a['per_joint']) == {'0', '1'}


# --------------------------------------------------------------------------- tracking

def test_cross_correlation_lag_recovers_a_known_delay_with_sign():
    reference = waypoint_reference([np.zeros(3), np.array([0.3, -0.2, 0.5]), np.zeros(3)], FS, move_s=0.8)
    delay = 37
    delayed = np.vstack([np.repeat(reference[:1], delay, axis=0), reference[:-delay]])
    lag, per_joint = metrics.xcorr_lag(reference, delayed, FS, 0.5)
    assert lag == pytest.approx(0.037, abs=0.0015)
    assert all(v == pytest.approx(0.037, abs=0.0015) for v in per_joint)
    lead, _ = metrics.xcorr_lag(delayed, reference, FS, 0.5)
    assert lead == pytest.approx(-0.037, abs=0.0015)


def test_tracking_error_statistics():
    reference = waypoint_reference([np.zeros(2), np.full(2, 0.2)], FS, move_s=1.0)
    measured = reference - 0.01
    motion = np.zeros(reference.shape[0], dtype=bool)
    motion[600:1400] = True
    out = metrics.tracking(reference, measured, FS, CFG, motion, [0, 1], ['a', 'b'])
    assert out['rms'] == pytest.approx(0.01)
    assert out['max'] == pytest.approx(0.01)
    assert out['rms_motion'] == pytest.approx(0.01)
    assert out['final_max'] == pytest.approx(0.01)
    assert out['lag_s'] == pytest.approx(0.0, abs=0.002)
    assert set(out['rms_per_joint']) == {'a', 'b'}


def test_overshoot_of_an_underdamped_step_matches_theory():
    reference = np.zeros((2000, 1))
    reference[500:] = 0.2
    zeta = 0.3
    measured = second_order(reference, FS, omega=40.0, zeta=zeta)
    out = metrics.overshoot(reference, measured, FS, CFG)
    expected = math.exp(-math.pi * zeta / math.sqrt(1 - zeta ** 2))
    assert out['moves'] == 1
    assert out['max_rel'] == pytest.approx(expected, abs=0.02)
    assert metrics.overshoot(reference, reference, FS, CFG)['max_rad'] == 0.0


# --------------------------------------------------------------------------- stick-slip

def test_injected_stick_slip_is_detected_and_counted():
    for fs in (200.0, 1000.0):
        reference = waypoint_reference([np.zeros(3), np.full(3, 0.3)], fs, move_s=5.0, hold_s=1.0)
        sticky, onsets = breakaway(reference, fs, break_rad=0.012, slip_s=0.008)
        injected = sum(len(v) for v in onsets.values())
        out = metrics.stick_slip(reference, sticky, fs, CFG, names=['a', 'b', 'c'], t0=10.0)
        assert out['count'] + out['burst_count'] == injected
        assert out['count'] >= 0.9 * injected
        assert out['peak_speed_max'] > 0.25
        event = out['events'][0]
        assert event['t'] >= 10.0 and event['joint'] in {'a', 'b', 'c'}
        assert event['stuck'] and event['error_growing'] and event['toward_reference']
        assert abs(event['error_before_rad']) >= CFG.stick_min_error_rad


def test_smooth_tracking_has_no_stick_slip_even_when_fast():
    reference = waypoint_reference([np.zeros(2), np.array([1.0, -1.0]), np.zeros(2)], FS, move_s=0.6)
    rng = np.random.default_rng(1)
    measured = second_order(reference, FS) + rng.normal(0, 1e-4, reference.shape)
    out = metrics.stick_slip(reference, measured, FS, CFG)
    assert out['count'] == 0 and out['burst_count'] == 0


def test_a_glitch_while_moving_is_a_burst_not_stick_slip():
    t = np.arange(0, 3, 1 / FS)
    reference = (0.2 * t)[:, None]
    measured = reference - 0.005
    measured[1500, 0] += 0.01
    out = metrics.stick_slip(reference, measured, FS, CFG)
    assert out['count'] == 0
    assert out['burst_count'] >= 1


# --------------------------------------------------------------------------- holding

def test_hold_oscillation_amplitude_reversals_and_events():
    t = np.arange(0, 4, 1 / FS)
    reference = np.full((t.size, 2), 0.3)
    measured = reference.copy()
    measured[:, 0] += 0.004 * np.sin(2 * np.pi * 8 * t)
    measured[:, 1] += np.random.default_rng(0).normal(0, 1e-4, t.size)
    out = metrics.hold_oscillation(reference, measured, FS, CFG, names=['osc', 'quiet'])
    assert out['available']
    assert out['per_joint_p2p_rad']['osc'] == pytest.approx(0.008, rel=0.02)
    assert out['per_joint_p2p_rad']['quiet'] < 0.001
    assert out['oscillation_events'] >= 1
    assert {e['joint'] for e in out['events']} == {'osc'}
    quiet = metrics.hold_oscillation(reference[:, 1:], measured[:, 1:], FS, CFG)
    assert quiet['oscillation_events'] == 0 and quiet['p2p_max_rad'] < 0.001


def test_hold_requires_a_constant_reference_and_skips_the_settling_window():
    reference = waypoint_reference([np.zeros(1), np.full(1, 0.5)], FS, move_s=1.0, hold_s=2.0, lead_s=0.0)
    measured = second_order(reference, FS, zeta=0.3)  # Rings after the move.
    out = metrics.hold_oscillation(reference, measured, FS, CFG)
    assert out['holds'] == 1
    assert out['p2p_max_rad'] < 0.004   # Ringing decays within hold_settle_s.
    moving = metrics.hold_oscillation(reference[:1000], measured[:1000], FS, CFG)
    assert not moving['available']


def test_reversal_indices_on_a_sine():
    x = np.sin(np.linspace(0, 6 * np.pi, 3000))
    assert len(metrics.reversal_indices(x, 0.1)) == 6
    assert metrics.reversal_indices(0.01 * x, 0.1) == []


# --------------------------------------------------------------------------- current, temperature, timing

def test_current_statistics_and_fraction_near_cap():
    current = np.zeros((1000, 3))
    current[:, 0] = 0.5
    current[:100, 1] = -0.79
    current[:, 2] = np.nan
    out = metrics.current_stats(current, 0.8, 0.05, ['a', 'b', 'c'])
    assert out['peak_a'] == pytest.approx(0.79)
    assert out['frac_near_cap'] == pytest.approx(0.1)
    assert out['per_joint']['a']['rms_a'] == pytest.approx(0.5)
    assert 'c' not in out['per_joint']
    assert out['rms_a'] == pytest.approx(math.sqrt((1000 * 0.25 + 100 * 0.79 ** 2) / 2000))
    assert not metrics.current_stats(np.full((10, 2), np.nan), 0.8)['available']


def test_temperature_rise():
    t = np.arange(0, 60, 0.1)
    temperature = np.column_stack([40 + 0.05 * t, np.full(t.size, 45.0)])
    out = metrics.temperature_stats(temperature, t)
    assert out['max_c'] == pytest.approx(45.0)
    assert out['rise_c'] == pytest.approx(0.05 * 59, abs=0.06)
    assert out['rise_c_per_min'] == pytest.approx(3.0, abs=0.1)


def test_command_timing_jitter_and_gaps():
    rng = np.random.default_rng(0)
    dt = 0.005 + rng.uniform(-0.0005, 0.0005, 1999)
    dt[1000] = 0.030
    t = np.concatenate([[0.0], np.cumsum(dt)])
    out = metrics.command_timing(t, 200.0, 1.5)
    assert out['gaps'] == 1
    assert out['max_gap_periods'] == pytest.approx(6.0)
    assert out['skipped_slots'] == 5
    assert out['rate_ratio'] == pytest.approx(1999 * 0.005 / t[-1], rel=1e-9)
    assert 0.2 < out['jitter_ms'] < 1.0
    assert out['max_ms'] == pytest.approx(30.0)


# --------------------------------------------------------------------------- audio

def tone(freq, seconds=1.0, fs=48000, amplitude=0.1):
    t = np.arange(int(seconds * fs)) / fs
    return amplitude * np.sin(2 * np.pi * freq * t)


def test_a_weighting_matches_iec_reference_points():
    fs = 48000
    for freq, expected in ((100, -19.1), (1000, 0.0), (31.5, -39.4), (4000, 1.0)):
        x = tone(freq, 2.0, fs)
        gain = audio.level_db(audio.a_weight(x, fs)[fs // 2:]) - audio.level_db(x[fs // 2:])
        assert gain == pytest.approx(expected, abs=0.3), freq
    assert audio.level_db(2 * tone(1000)) - audio.level_db(tone(1000)) == pytest.approx(6.02, abs=0.01)


def clicks_signal(times, fs=48000, seconds=3.0, seed=0):
    rng = np.random.default_rng(seed)
    x = rng.normal(0, 1e-3, int(seconds * fs))
    n = np.arange(int(0.003 * fs))
    for t in times:
        i = int(t * fs)
        x[i:i + n.size] += 0.05 * np.sin(2 * np.pi * 5000 * n / fs) * np.exp(-n / (0.0005 * fs))
    return x


def test_click_detection_counts_injected_transients_at_the_right_times():
    times = [0.5, 1.1, 1.7, 2.4]
    found, prominence = audio.detect_clicks(clicks_signal(times), 48000)
    assert len(found) == len(times)
    assert np.max(np.abs(found - np.asarray(times))) < 0.003
    assert np.all(prominence > 10)
    quiet, _ = audio.detect_clicks(clicks_signal([]), 48000)
    assert len(quiet) == 0


def test_audio_analysis_aligns_clicks_to_the_trace_clock(tmp_path):
    from scipy.io import wavfile
    fs = 48000
    x = clicks_signal([0.5, 1.1, 1.7, 2.4], fs) + tone(200, 3.0, fs, 0.002)
    path = tmp_path / 'audio.wav'
    wavfile.write(str(path), fs, (x * 32767).astype(np.int16))
    out = audio.analyze(path, start_t=100.0, motion=[(101.0, 102.0)], stick_slip_times=[101.101, 103.0])
    assert out['clicks'] == 4
    assert out['click_times'][1] == pytest.approx(101.1, abs=0.003)
    assert out['clicks_motion'] == 2 and out['clicks_hold'] == 2
    assert out['excess_clicks_motion'] == pytest.approx(2 - 2 / 2.0 * 1.0, abs=0.05)
    assert out['stick_slip_with_click'] == 1
    assert out['dominant_hz'] == pytest.approx(200, abs=10)
    assert out['motion_above_background_db'] >= 0


def test_audio_start_prefers_meta_and_applies_offset():
    assert audio.audio_start({'audio': {'start_t': 5.0, 'offset_s': 0.25}}, 1.0) == (5.25, 'meta.audio.start_t')
    assert audio.audio_start({}, 7.0) == (7.0, 'assumed trace start')


# --------------------------------------------------------------------------- fingertip (optional)

def test_fingertip_metrics_skip_gracefully_without_mujoco(monkeypatch):
    from services.dex_eval import fingertip
    real_import = builtins.__import__

    def blocked(name, *args, **kwargs):
        if name == 'mujoco':
            raise ImportError('blocked for test')
        return real_import(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', blocked)
    q = np.zeros((100, 20))
    out = fingertip.fingertip_sparc(q, q, 200.0, 'right', CFG)
    assert out == {'available': False, 'reason': 'mujoco not importable (ImportError)'}


def test_fingertip_metrics_skip_without_the_model(monkeypatch, tmp_path):
    pytest.importorskip('mujoco')
    from services.dex_eval import fingertip
    monkeypatch.setenv(fingertip.ENVIRONMENT, str(tmp_path))
    monkeypatch.setattr(fingertip, 'candidate_roots', lambda: [tmp_path])
    out = fingertip.fingertip_sparc(np.zeros((100, 20)), np.zeros((100, 20)), 200.0, 'right', CFG)
    assert not out['available'] and 'wuji-description' in out['reason']


def test_fingertip_sparc_with_the_official_model():
    pytest.importorskip('mujoco')
    from services.dex_eval import fingertip
    if fingertip.find_model('right') is None:
        pytest.skip('wuji-description not available')
    fs = 200.0
    # A slow move: its stick-slip staircase lies inside the SPARC band (fast slips are filtered out).
    reference = waypoint_reference([np.zeros(20), np.r_[np.zeros(4), np.full(16, 0.6)]], fs, move_s=4.0)
    sticky, _ = breakaway(reference, fs, break_rad=0.012)
    smooth = fingertip.fingertip_sparc(reference, second_order(reference, fs), fs, 'right', CFG)
    rough = fingertip.fingertip_sparc(reference, sticky, fs, 'right', CFG)
    assert smooth['available'] and len(smooth['per_finger']) == 5
    assert smooth['sparc_meas'] > rough['sparc_meas'] + 1.0
    assert smooth['sparc_ref'] == pytest.approx(smooth['sparc_meas'], abs=0.05)


# --------------------------------------------------------------------------- statistics and thresholds

def test_wilson_interval_known_values():
    rate, low, high = stats.wilson(8, 10)
    assert rate == 0.8
    assert low == pytest.approx(0.4902, abs=1e-3) and high == pytest.approx(0.9433, abs=1e-3)
    assert stats.wilson(0, 10)[1] == 0.0
    assert all(math.isnan(v) for v in stats.wilson(0, 0))


def test_bootstrap_interval_contains_the_median_and_ignores_missing():
    point, low, high = stats.bootstrap_ci([1.0, 2.0, 3.0, 4.0, None, float('nan'), 5.0])
    assert point == 3.0 and low <= 3.0 <= high
    assert math.isnan(stats.bootstrap_ci([1.0])[1])


def test_holm_adjustment():
    assert stats.holm([0.01, 0.04, 0.03]) == pytest.approx([0.03, 0.06, 0.06])
    adjusted = stats.holm([0.01, float('nan')])
    assert adjusted[0] == pytest.approx(0.01) and math.isnan(adjusted[1])


def test_bradley_terry_recovers_the_true_order():
    rng = np.random.default_rng(0)
    true = {'a': 1.0, 'b': 0.0, 'c': -1.0}
    comparisons = []
    for _ in range(300):
        x, y = rng.choice(list(true), 2, replace=False)
        p = 1 / (1 + math.exp(-(true[x] - true[y])))
        comparisons.append((x, y, 'A' if rng.random() < p else 'B'))
    comparisons += [('a', 'c', 'tie')] * 3
    scores = stats.bradley_terry(comparisons)
    assert scores['a'] > scores['b'] > scores['c']
    assert scores['a'] - scores['c'] == pytest.approx(2.0, abs=0.5)
    unbeaten = stats.bradley_terry([('x', 'y', 'A')] * 5)
    assert math.isfinite(unbeaten['x']) and unbeaten['x'] > unbeaten['y']


def test_thresholds_check_calibrate_and_override(tmp_path):
    result = thresholds.check({'sparc_ratio': 1.1, 'stick_slip_count': 3, 'timing_rate_ratio': None})
    assert result['sparc_ratio']['pass'] is True
    assert result['stick_slip_count']['pass'] is False
    assert result['timing_rate_ratio']['pass'] is None
    reference = [{'hold_p2p_max_rad': v, 'tracking_rms_motion_rad': 0.001} for v in (0.005, 0.006, 0.007)]
    calibrated = {t.key: t for t in thresholds.calibrate(thresholds.DEFAULTS, reference, label='ideal')}
    assert calibrated['hold_p2p_max_rad'].value > 0.004 and 'calibrated' in calibrated['hold_p2p_max_rad'].source
    assert calibrated['tracking_rms_motion_rad'].value == 0.03   # Never tightened.
    path = tmp_path / 'thresholds.json'
    path.write_text('{"sparc_ratio": 1.3, "stick_slip_count": null, "custom": {"op": "<", "value": 2}}')
    loaded = {t.key: t for t in thresholds.load(path)}
    assert loaded['sparc_ratio'].value == 1.3 and loaded['sparc_ratio'].op == '<='
    assert 'stick_slip_count' not in loaded and loaded['custom'].passes(1.0)


def test_min_jerk_helper_is_rest_to_rest():
    path = min_jerk(np.zeros(2), np.ones(2), 101)
    assert np.allclose(path[0], 0) and np.allclose(path[-1], 1)
    velocity = np.gradient(path[:, 0])
    assert abs(velocity[0]) < 1e-3 and abs(velocity[-1]) < 1e-3
