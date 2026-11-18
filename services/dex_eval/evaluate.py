"""Evaluate one trial directory: per-hand and per-arm metrics, audio, task outcome, summary.

``evaluate_trial`` returns a JSON-safe dict (NaN becomes null) with:

* ``hands[side]`` / ``arms[side]``: detailed metrics (see metrics.py);
* ``audio``: acoustic metrics (see audio.py), when audio.wav exists;
* ``outcome``: ok / status / success (see outcome.py);
* ``summary``: one flat dict of the headline values listed in SUMMARY_METRICS;
* ``motion_intervals``: trace-clock intervals with commanded motion.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from . import METRICS_VERSION
from .audio import AudioConfig, analyze as analyze_audio, audio_start
from .metrics import (MetricConfig, command_timing, current_stats, hold_oscillation, motion_window, overshoot,
                      smoothness, stick_slip, temperature_stats, tracking)
from .outcome import outcome as task_outcome
from .signals import derivative, lowpass, regions, sample_rate, uniform
from .trace import JOINT_NAMES, Trial, load_trial

NAN = float('nan')
ARM_NAMES = tuple(f'joint{i}' for i in range(1, 7))
TEMPERATURE_KEYS = ('temperature_c', 'temperature', 'mcu_temp_c')

# key: (label, unit, better). "better" orders rankings and plots.
SUMMARY_METRICS = {
    'sparc': ('SPARC, hand joint speed', '', 'higher'),
    'sparc_ref': ('SPARC of the commanded reference', '', 'higher'),
    'sparc_ratio': ('SPARC / reference SPARC (worst hand)', '', 'lower'),
    'sparc_joint_median': ('Median per-joint SPARC', '', 'higher'),
    'ldlj': ('LDLJ, hand joint velocity', '', 'higher'),
    'ldlj_ref': ('LDLJ of the reference', '', 'higher'),
    'rms_jerk': ('RMS jerk', 'rad/s^3', 'lower'),
    'fingertip_sparc': ('SPARC, fingertip speed', '', 'higher'),
    'fingertip_sparc_ref': ('Fingertip SPARC of the reference', '', 'higher'),
    'tracking_rms_rad': ('RMS tracking error', 'rad', 'lower'),
    'tracking_rms_motion_rad': ('RMS tracking error in motion', 'rad', 'lower'),
    'tracking_max_rad': ('Max tracking error', 'rad', 'lower'),
    'tracking_final_max_rad': ('Final error (worst joint)', 'rad', 'lower'),
    'tracking_lag_s': ('Lag behind reference', 's', 'lower'),
    'overshoot_max_rad': ('Max overshoot', 'rad', 'lower'),
    'overshoot_max_rel': ('Max overshoot / move amplitude', '', 'lower'),
    'stick_slip_count': ('Stick-slip events', '', 'lower'),
    'stick_slip_peak_speed': ('Stick-slip peak speed', 'rad/s', 'lower'),
    'burst_count': ('Other speed bursts', '', 'lower'),
    'hold_p2p_max_rad': ('Hold peak-to-peak (1 s)', 'rad', 'lower'),
    'hold_rms_max_rad': ('Hold RMS', 'rad', 'lower'),
    'hold_reversals_per_s': ('Hold reversals per joint-second', '1/s', 'lower'),
    'hold_oscillation_events': ('Hold oscillation events', '', 'lower'),
    'current_rms_a': ('RMS current', 'A', 'lower'),
    'current_peak_a': ('Peak current', 'A', 'lower'),
    'current_frac_near_cap': ('Fraction of time near current cap', '', 'lower'),
    'temperature_max_c': ('Max temperature', 'C', 'lower'),
    'temperature_rise_c': ('Temperature rise', 'C', 'lower'),
    'timing_achieved_hz': ('Achieved command rate', 'Hz', 'higher'),
    'timing_rate_ratio': ('Achieved / nominal rate', '', 'higher'),
    'timing_jitter_ms': ('Command interval jitter (SD)', 'ms', 'lower'),
    'timing_p99_ms': ('Command interval p99', 'ms', 'lower'),
    'timing_max_gap_periods': ('Max command gap', 'periods', 'lower'),
    'timing_gaps': ('Command gaps', '', 'lower'),
    'timing_skipped_slots': ('Skipped command slots', '', 'lower'),
    'arm_sparc': ('SPARC, arm joint speed', '', 'higher'),
    'arm_sparc_ref': ('Arm reference SPARC', '', 'higher'),
    'arm_sparc_ratio': ('Arm SPARC / reference SPARC', '', 'lower'),
    'arm_ldlj': ('LDLJ, arm', '', 'higher'),
    'arm_tracking_rms_rad': ('Arm RMS tracking error', 'rad', 'lower'),
    'arm_tracking_lag_s': ('Arm lag', 's', 'lower'),
    'audio_laeq_db': ('LAeq (relative)', 'dB', 'lower'),
    'audio_laeq_motion_db': ('LAeq during motion (relative)', 'dB', 'lower'),
    'audio_background_db': ('Background level (relative)', 'dB', 'lower'),
    'audio_motion_above_background_db': ('Motion level above background', 'dB', 'lower'),
    'audio_clicks': ('Click transients', '', 'lower'),
    'audio_clicks_motion': ('Clicks during motion', '', 'lower'),
    'audio_click_rate_motion_hz': ('Click rate during motion', '1/s', 'lower'),
    'audio_excess_clicks_motion': ('Clicks beyond hold rate', '', 'lower'),
    'audio_dominant_hz': ('Dominant tone', 'Hz', None),
    'duration_s': ('Trial duration', 's', 'lower'),
}


@dataclass
class EvalConfig:
    metrics: MetricConfig = field(default_factory=MetricConfig)
    audio: AudioConfig = field(default_factory=AudioConfig)
    resample_hz: float | None = None     # default: the trace's median sample rate
    fingertip: bool = True               # try FK fingertip SPARC (skipped without mujoco/model)
    wuji_description: str | None = None  # wuji-description checkout for FK
    audio_background: str | None = None  # optional background recording for all trials

    def to_dict(self):
        return {'metrics': self.metrics.to_dict(), 'audio': self.audio.to_dict(), 'resample_hz': self.resample_hz,
                'fingertip': self.fingertip, 'wuji_description': self.wuji_description,
                'audio_background': self.audio_background}

    def digest(self):
        text = json.dumps({'version': METRICS_VERSION, **self.to_dict()}, sort_keys=True, default=str)
        return hashlib.sha256(text.encode()).hexdigest()[:16]


def clean(value):
    """JSON-safe copy: numpy to Python, NaN/inf to None, tuples to lists."""
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        value = float(value)
        return value if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    return value


def current_cap(meta, side, default):
    """Per-joint current cap (A) from meta (current_limit_a, top level or in params), else default."""
    for source in (meta, meta.get('params') or {}):
        cap = source.get('current_limit_a') if isinstance(source, dict) else None
        if isinstance(cap, dict):
            cap = cap.get(side)
        if isinstance(cap, (int, float)) and not isinstance(cap, bool) and cap > 0:
            return float(cap)
        if isinstance(cap, list) and len(cap) == 20:
            return np.asarray(cap, dtype=float)
    return default


def _excursion(x):
    finite = np.isfinite(x)
    out = np.zeros(x.shape[1])
    for j in range(x.shape[1]):
        if finite[:, j].any():
            out[j] = np.nanmax(x[:, j]) - np.nanmin(x[:, j])
    return out


def _reference_motion(q_ref, fs, cfg):
    """(speed, motion mask, analysis window) of the joint-space reference speed."""
    if q_ref.shape[1] == 0:
        return np.zeros(q_ref.shape[0]), np.zeros(q_ref.shape[0], dtype=bool), None
    speed = np.linalg.norm(derivative(lowpass(q_ref, fs, cfg.smooth_cutoff_hz), fs), axis=1)
    peak = float(np.max(speed)) if speed.size else 0.0
    mask = speed > max(cfg.motion_speed_frac * peak, cfg.motion_speed_min)
    window = motion_window(speed, fs, frac=cfg.motion_speed_frac, floor=cfg.motion_speed_min,
                           pad_s=cfg.motion_pad_s)
    return speed, mask, window


def _ratio(measured, reference):
    """measured / reference SPARC (both negative): 1 is as smooth as commanded, larger is worse."""
    if measured is None or reference is None or not (np.isfinite(measured) and np.isfinite(reference)):
        return NAN
    return float(measured / reference) if reference < 0 else NAN


def evaluate_hand(hand, meta, config: EvalConfig):
    cfg = config.metrics
    names = JOINT_NAMES
    active = hand.active()
    out = {'side': hand.side, 'samples': hand.n, 'duration_s': hand.duration,
           'active_joints': [names[j] for j in np.flatnonzero(active)],
           'disabled_joints': [names[j] for j in np.flatnonzero(~active)],
           'timing': command_timing(hand.t, float(meta.get('nominal_rate_hz') or 0) or sample_rate(hand.t),
                                    cfg.gap_factor)}
    if not active.any():
        out['available'] = False
        out['reason'] = 'no joint has measured positions'
        return out
    fs = config.resample_hz or round(sample_rate(hand.t), 3)
    stacked = np.concatenate([hand.q_ref, hand.q_meas, hand.current], axis=1)
    tu, X, fs = uniform(hand.t, stacked, fs)
    q_ref, q_meas, current = X[:, :20], X[:, 20:40], X[:, 40:60]
    t0 = float(tu[0])
    act = [int(j) for j in np.flatnonzero(active & np.isfinite(q_ref).all(axis=0))]
    excursion = _excursion(q_ref)
    moving = [j for j in act if excursion[j] >= cfg.min_excursion_rad]
    speed, mask, window = _reference_motion(q_ref[:, moving], fs, cfg)
    out.update(available=True, fs=fs, moving_joints=[names[j] for j in moving],
               motion_intervals=[(t0 + a / fs, t0 + b / fs) for a, b in regions(mask)],
               window_s=None if window is None else [t0 + window[0] / fs, t0 + window[1] / fs])
    moving_names = [names[j] for j in moving]
    if moving and window is not None:
        measured = smoothness(q_meas[:, moving], fs, cfg, window, moving_names)
        reference = smoothness(q_ref[:, moving], fs, cfg, window, moving_names)
        ratio = _ratio(measured['sparc'], reference['sparc'])
        for name in moving_names:
            measured['per_joint'][name]['sparc_ratio'] = _ratio(measured['per_joint'][name]['sparc'],
                                                                reference['per_joint'][name]['sparc'])
    else:
        measured = reference = {'sparc': NAN, 'ldlj': NAN, 'rms_jerk': NAN, 'peak_speed': NAN, 'per_joint': {}}
        ratio = NAN
    out['smoothness'] = {'meas': measured, 'ref': reference, 'sparc_ratio': ratio}
    lag_joints = [act.index(j) for j in moving]
    out['tracking'] = tracking(q_ref[:, act], q_meas[:, act], fs, cfg, mask, lag_joints, [names[j] for j in act])
    out['overshoot'] = overshoot(q_ref, q_meas, fs, cfg, moving, names)
    out['stick_slip'] = stick_slip(q_ref, q_meas, fs, cfg, act, names, t0)
    out['hold'] = hold_oscillation(q_ref, q_meas, fs, cfg, act, names, t0)
    cap = current_cap(meta, hand.side, cfg.current_cap_a)
    cap = cap[act] if isinstance(cap, np.ndarray) else cap
    out['current'] = current_stats(current[:, act], cap, cfg.near_cap_frac, [names[j] for j in act])
    temperature = next((hand.extra[k] for k in TEMPERATURE_KEYS if k in hand.extra), None)
    out['temperature'] = (temperature_stats(np.asarray(temperature, dtype=float), hand.t)
                          if temperature is not None and np.shape(temperature)[0] == hand.n
                          else {'available': False})
    if config.fingertip and moving and window is not None:
        from .fingertip import fingertip_sparc
        out['fingertip'] = fingertip_sparc(q_ref, q_meas, fs, hand.side, cfg, window=window,
                                           root=config.wuji_description)
    else:
        out['fingertip'] = {'available': False, 'reason': 'disabled' if not config.fingertip else 'no motion'}
    return out


def evaluate_arm(arm, config: EvalConfig):
    cfg = config.metrics
    finite = np.isfinite(arm.q_meas).mean(axis=0) >= 0.9
    if not finite.any():
        return {'side': arm.side, 'available': False, 'reason': 'no measured arm joints'}
    tu, X, fs = uniform(arm.t, np.concatenate([arm.q_ref, arm.q_meas], axis=1),
                        config.resample_hz or round(sample_rate(arm.t), 3))
    q_ref, q_meas = X[:, :6], X[:, 6:]
    act = [int(j) for j in np.flatnonzero(finite & np.isfinite(q_ref).all(axis=0))]
    excursion = _excursion(q_ref)
    moving = [j for j in act if excursion[j] >= cfg.min_excursion_rad]
    _, mask, window = _reference_motion(q_ref[:, moving], fs, cfg)
    out = {'side': arm.side, 'available': True, 'fs': fs, 'samples': arm.n,
           'moving_joints': [ARM_NAMES[j] for j in moving]}
    if moving and window is not None:
        measured = smoothness(q_meas[:, moving], fs, cfg, window, [ARM_NAMES[j] for j in moving])
        reference = smoothness(q_ref[:, moving], fs, cfg, window, [ARM_NAMES[j] for j in moving])
    else:
        measured = reference = {'sparc': NAN, 'ldlj': NAN, 'rms_jerk': NAN, 'peak_speed': NAN, 'per_joint': {}}
    out['smoothness'] = {'meas': measured, 'ref': reference, 'sparc_ratio': _ratio(measured['sparc'],
                                                                                    reference['sparc'])}
    out['tracking'] = tracking(q_ref[:, act], q_meas[:, act], fs, cfg, mask,
                               [act.index(j) for j in moving], [ARM_NAMES[j] for j in act])
    return out


def _values(items, *path):
    out = []
    for item in items:
        value = item
        for key in path:
            value = value.get(key) if isinstance(value, dict) else None
        if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
            out.append(float(value))
    return out


def _agg(values, how):
    if not values:
        return NAN
    return {'mean': float(np.mean(values)), 'max': float(np.max(values)), 'min': float(np.min(values)),
            'sum': float(np.sum(values)), 'median': float(np.median(values))}[how]


def summarize(hands, arms, audio, result_outcome, duration):
    """Flat headline summary. Across hands: SPARC/LDLJ are averaged; ratios, errors, holds and
    currents take the worst hand; event counts are summed; timing takes the worst hand."""
    h = [x for x in hands.values() if x.get('available')]
    s = {'success': result_outcome.get('success'), 'status': result_outcome.get('status'),
         'ok': result_outcome.get('ok'), 'duration_s': duration}
    s['sparc'] = _agg(_values(h, 'smoothness', 'meas', 'sparc'), 'mean')
    s['sparc_ref'] = _agg(_values(h, 'smoothness', 'ref', 'sparc'), 'mean')
    s['sparc_ratio'] = _agg(_values(h, 'smoothness', 'sparc_ratio'), 'max')
    per_joint = [v['sparc'] for x in h for v in x['smoothness']['meas']['per_joint'].values()
                 if v.get('sparc') is not None and math.isfinite(v['sparc'])]
    s['sparc_joint_median'] = _agg(per_joint, 'median')
    s['ldlj'] = _agg(_values(h, 'smoothness', 'meas', 'ldlj'), 'mean')
    s['ldlj_ref'] = _agg(_values(h, 'smoothness', 'ref', 'ldlj'), 'mean')
    s['rms_jerk'] = _agg(_values(h, 'smoothness', 'meas', 'rms_jerk'), 'max')
    s['fingertip_sparc'] = _agg(_values(h, 'fingertip', 'sparc_meas'), 'mean')
    s['fingertip_sparc_ref'] = _agg(_values(h, 'fingertip', 'sparc_ref'), 'mean')
    for key, path, how in (('tracking_rms_rad', ('tracking', 'rms'), 'max'),
                           ('tracking_rms_motion_rad', ('tracking', 'rms_motion'), 'max'),
                           ('tracking_max_rad', ('tracking', 'max'), 'max'),
                           ('tracking_final_max_rad', ('tracking', 'final_max'), 'max'),
                           ('tracking_lag_s', ('tracking', 'lag_s'), 'max'),
                           ('overshoot_max_rad', ('overshoot', 'max_rad'), 'max'),
                           ('overshoot_max_rel', ('overshoot', 'max_rel'), 'max'),
                           ('stick_slip_count', ('stick_slip', 'count'), 'sum'),
                           ('stick_slip_peak_speed', ('stick_slip', 'peak_speed_max'), 'max'),
                           ('burst_count', ('stick_slip', 'burst_count'), 'sum'),
                           ('hold_p2p_max_rad', ('hold', 'p2p_max_rad'), 'max'),
                           ('hold_rms_max_rad', ('hold', 'rms_max_rad'), 'max'),
                           ('hold_reversals_per_s', ('hold', 'reversals_per_s'), 'max'),
                           ('hold_oscillation_events', ('hold', 'oscillation_events'), 'sum'),
                           ('current_rms_a', ('current', 'rms_a'), 'max'),
                           ('current_peak_a', ('current', 'peak_a'), 'max'),
                           ('current_frac_near_cap', ('current', 'frac_near_cap'), 'max'),
                           ('temperature_max_c', ('temperature', 'max_c'), 'max'),
                           ('temperature_rise_c', ('temperature', 'rise_c'), 'max')):
        s[key] = _agg(_values(h, *path), how)
    if not _values(h, 'stick_slip', 'count'):
        s['stick_slip_count'] = s['burst_count'] = NAN
    timing = list(hands.values())
    for key, field_name, how in (('timing_achieved_hz', 'achieved_hz', 'min'),
                                 ('timing_rate_ratio', 'rate_ratio', 'min'),
                                 ('timing_jitter_ms', 'jitter_ms', 'max'), ('timing_p99_ms', 'p99_ms', 'max'),
                                 ('timing_max_gap_periods', 'max_gap_periods', 'max'),
                                 ('timing_gaps', 'gaps', 'sum'), ('timing_skipped_slots', 'skipped_slots', 'sum')):
        s[key] = _agg(_values(timing, 'timing', field_name), how)
    a = [x for x in arms.values() if x.get('available')]
    s['arm_sparc'] = _agg(_values(a, 'smoothness', 'meas', 'sparc'), 'mean')
    s['arm_sparc_ref'] = _agg(_values(a, 'smoothness', 'ref', 'sparc'), 'mean')
    s['arm_sparc_ratio'] = _agg(_values(a, 'smoothness', 'sparc_ratio'), 'max')
    s['arm_ldlj'] = _agg(_values(a, 'smoothness', 'meas', 'ldlj'), 'mean')
    s['arm_tracking_rms_rad'] = _agg(_values(a, 'tracking', 'rms'), 'max')
    s['arm_tracking_lag_s'] = _agg(_values(a, 'tracking', 'lag_s'), 'max')
    audio = audio if audio and audio.get('available') else {}
    for key in ('laeq_db', 'laeq_motion_db', 'background_db', 'motion_above_background_db', 'clicks',
                'clicks_motion', 'click_rate_motion_hz', 'excess_clicks_motion', 'dominant_hz'):
        value = audio.get(key, NAN)
        s['audio_' + key] = NAN if value is None else value
    return s


def _union(intervals):
    merged = []
    for a, b in sorted(intervals):
        if merged and a <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], b))
        else:
            merged.append((a, b))
    return merged


def trace_metrics(trial: Trial, config: EvalConfig):
    """Metrics that depend only on the recorded trace (cacheable)."""
    hands = {side: evaluate_hand(hand, trial.meta, config) for side, hand in trial.hands.items()}
    arms = {side: evaluate_arm(arm, config) for side, arm in trial.arms.items()}
    motion = _union([iv for h in hands.values() for iv in h.get('motion_intervals', [])])
    audio = None
    if trial.audio is not None:
        start, source = audio_start(trial.meta, trial.t0)
        slips = [e['t'] for h in hands.values() for e in h.get('stick_slip', {}).get('events', [])]
        try:
            audio = analyze_audio(trial.audio, start_t=start, motion=motion, stick_slip_times=slips,
                                  background=config.audio_background, cfg=config.audio)
            audio['alignment'] = source
        except Exception as exc:  # A damaged recording must not hide the motion metrics.
            audio = {'available': False, 'reason': f'{type(exc).__name__}: {exc}'}
    ends = [h.t[-1] for h in trial.hands.values()] + [a.t[-1] for a in trial.arms.values()]
    duration = float(max(ends) - trial.t0) if ends else NAN
    return {'hands': hands, 'arms': arms, 'audio': audio, 'motion_intervals': motion, 'duration_s': duration}


def evaluate_trial(trial, config: EvalConfig | None = None, result=None, *, cache=False, recompute=False):
    """Evaluate a trial directory (path or loaded Trial). result: the skill's return value, if known.

    With cache, trace metrics are stored in <trial>/metrics.json keyed by the config digest.
    """
    config = config or EvalConfig()
    if not isinstance(trial, Trial):
        trial = load_trial(trial)
    cache_path = trial.path / 'metrics.json'
    digest = config.digest()
    stamp = _stamp(trial.path)
    metrics = None
    if cache and not recompute and cache_path.is_file():
        try:
            stored = json.loads(cache_path.read_text())
            if stored.get('config_digest') == digest and stored.get('trace_stamp') == stamp:
                metrics = stored['trace_metrics']
        except (OSError, ValueError, KeyError):
            metrics = None
    if metrics is None:
        metrics = clean(trace_metrics(trial, config))
        if cache:
            try:
                cache_path.write_text(json.dumps({'version': METRICS_VERSION, 'config_digest': digest,
                                                  'trace_stamp': stamp, 'trace_metrics': metrics},
                                                 allow_nan=False) + '\n')
            except OSError:
                pass
    result_outcome = task_outcome(result, trial.meta)
    summary = summarize(metrics['hands'], metrics['arms'], metrics['audio'], result_outcome,
                        metrics['duration_s'] if metrics['duration_s'] is not None else NAN)
    return clean({'trial': str(trial.path), 'meta': trial.meta, 'warnings': trial.warnings,
                  'config_digest': digest, **metrics, 'outcome': result_outcome, 'summary': summary})


def _stamp(path):
    files = sorted(p for p in Path(path).iterdir()
                   if p.suffix in ('.npz', '.wav') or p.name == 'meta.json')
    return [[p.name, p.stat().st_size, p.stat().st_mtime_ns] for p in files]
