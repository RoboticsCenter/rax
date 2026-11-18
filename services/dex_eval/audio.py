"""Acoustic metrics from a trial's audio.wav: relative A-weighted level and click/crackle counts.

Levels are relative (dB re digital full scale, A-weighted) because the microphone is
not calibrated; compare them only within one rig and session, or against the trial's
own background. Click times are reported on the trace clock (see ``align``).
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from scipy import signal
from scipy.io import wavfile
from scipy.ndimage import median_filter

from .signals import interval_mask, merge_regions, regions

NAN = float('nan')
EPS = 1e-12


@dataclass(frozen=True)
class AudioConfig:
    level_frame_s: float = 0.125        # "fast" time weighting frame
    background_percentile: float = 10.0  # self-calibrated noise floor from the recording itself
    click_band_hz: tuple = (2000.0, None)  # high band for transient detection (upper None = 0.45 fs)
    click_frame_s: float = 0.001
    click_hop_s: float = 0.0005
    click_median_s: float = 0.1         # local-median window for the transient threshold
    click_threshold_db: float = 10.0    # transient: frame energy this far above the local median
    click_refractory_s: float = 0.01
    tonal_min_hz: float = 50.0
    stick_slip_match_s: float = 0.025   # a click within this of a stick-slip event "matches" it

    def to_dict(self):
        return asdict(self)


def read_wav(path):
    """(mono float64 samples in [-1, 1], sample rate)."""
    fs, data = wavfile.read(str(path))
    data = np.asarray(data)
    if np.issubdtype(data.dtype, np.integer):
        info = np.iinfo(data.dtype)
        if info.min == 0:  # unsigned 8-bit
            data = (data.astype(float) - (info.max + 1) / 2) / ((info.max + 1) / 2)
        else:
            data = data.astype(float) / max(abs(info.min), info.max)
    else:
        data = data.astype(float)
    if data.ndim == 2:
        data = data.mean(axis=1)
    return data, float(fs)


def a_weighting_sos(fs):
    """IEC 61672 A-weighting as a digital filter (bilinear transform of the analog prototype)."""
    f1, f2, f3, f4 = 20.598997, 107.65265, 737.86223, 12194.217
    a1000 = 1.9997
    zeros = [0.0, 0.0, 0.0, 0.0]
    poles = [-2 * np.pi * f1, -2 * np.pi * f1, -2 * np.pi * f4, -2 * np.pi * f4,
             -2 * np.pi * f2, -2 * np.pi * f3]
    gain = (2 * np.pi * f4) ** 2 * 10 ** (a1000 / 20)
    z, p, k = signal.bilinear_zpk(zeros, poles, gain, fs)
    return signal.zpk2sos(z, p, k)


def a_weight(x, fs):
    return signal.sosfilt(a_weighting_sos(fs), np.asarray(x, dtype=float))


def level_db(x):
    """Equivalent level 10 log10(mean x^2), dB re full scale."""
    x = np.asarray(x, dtype=float)
    return float(10 * np.log10(np.mean(x ** 2) + EPS)) if x.size else NAN


def frame_levels(x, fs, frame_s):
    """(frame start times, dB per non-overlapping frame)."""
    n = max(1, int(round(frame_s * fs)))
    frames = x.size // n
    if frames == 0:
        return np.zeros(0), np.zeros(0)
    power = np.mean(x[:frames * n].reshape(frames, n) ** 2, axis=1)
    return np.arange(frames) * n / fs, 10 * np.log10(power + EPS)


def detect_clicks(x, fs, cfg: AudioConfig = AudioConfig()):
    """Transient (click/crackle) times in seconds from the start of x, and their prominence (dB).

    The signal is band-passed to cfg.click_band_hz; short-frame energy (click_frame_s,
    hop click_hop_s) is compared with its running median over click_median_s. Frames more
    than click_threshold_db above it form a click; clicks closer than click_refractory_s merge.
    """
    x = np.asarray(x, dtype=float)
    low, high = cfg.click_band_hz
    high = min(high or 0.45 * fs, 0.45 * fs)
    if low >= high or x.size < int(fs * 0.01):
        return np.zeros(0), np.zeros(0)
    sos = signal.butter(4, [low, high], btype='band', fs=fs, output='sos')
    y = signal.sosfiltfilt(sos, x)
    frame = max(1, int(round(cfg.click_frame_s * fs)))
    hop = max(1, int(round(cfg.click_hop_s * fs)))
    if y.size < frame:
        return np.zeros(0), np.zeros(0)
    starts = np.arange(0, y.size - frame + 1, hop)
    cumulative = np.concatenate(([0.0], np.cumsum(y ** 2)))
    energy = (cumulative[starts + frame] - cumulative[starts]) / frame
    db = 10 * np.log10(energy + EPS)
    width = max(3, int(round(cfg.click_median_s / (hop / fs))) | 1)
    baseline = median_filter(db, size=width, mode='nearest')
    above = db > baseline + cfg.click_threshold_db
    times, prominence = [], []
    for a, b in merge_regions(regions(above), max(1, int(round(cfg.click_refractory_s * fs / hop)))):
        i = a + int(np.argmax(db[a:b] - baseline[a:b]))
        times.append((starts[i] + frame / 2) / fs)
        prominence.append(float(db[i] - baseline[i]))
    return np.asarray(times), np.asarray(prominence)


def dominant_frequency(x, fs, fmin=50.0):
    """Frequency (Hz) of the largest Welch PSD peak above fmin."""
    x = np.asarray(x, dtype=float)
    if x.size < 256:
        return NAN
    f, p = signal.welch(x, fs=fs, nperseg=min(8192, x.size))
    keep = f >= fmin
    return float(f[keep][np.argmax(p[keep])]) if np.any(keep) else NAN


def audio_start(meta, trace_t0):
    """(trace-clock time of the first audio sample, how it was obtained).

    meta['audio'] may give 'start_t' (trace clock) and 'offset_s' (manual correction,
    e.g. from a sync clap); otherwise the audio is assumed to start at the trace start.
    """
    info = meta.get('audio') if isinstance(meta.get('audio'), dict) else {}
    offset = float(info.get('offset_s', 0.0) or 0.0)
    if info.get('start_t') is not None:
        return float(info['start_t']) + offset, 'meta.audio.start_t'
    if meta.get('audio_start_t') is not None:
        return float(meta['audio_start_t']) + offset, 'meta.audio_start_t'
    return float(trace_t0) + offset, 'assumed trace start'


def analyze(path, *, start_t=0.0, motion=(), stick_slip_times=(), background=None, cfg: AudioConfig = AudioConfig()):
    """Acoustic summary of one recording.

    start_t: trace-clock time of the first sample. motion: [(t_start, t_end)] trace-clock
    intervals with commanded motion (the rest counts as hold). background: optional path
    of a background recording; otherwise the background is the background_percentile of
    the recording's own frame levels.
    """
    x, fs = read_wav(path)
    if x.size == 0:
        return {'available': False, 'reason': 'empty audio'}
    weighted = a_weight(x, fs)
    frame_t, frame_db = frame_levels(weighted, fs, cfg.level_frame_s)
    sample_t = start_t + np.arange(x.size) / fs
    in_motion = interval_mask(sample_t, motion)
    if background is not None:
        bx, bfs = read_wav(background)
        background_db, background_source = level_db(a_weight(bx, bfs)), 'recording'
    else:
        background_db = float(np.percentile(frame_db, cfg.background_percentile)) if frame_db.size else NAN
        background_source = f'p{cfg.background_percentile:g} of frame levels'
    click_t, prominence = detect_clicks(x, fs, cfg)
    click_trace_t = start_t + click_t
    click_motion = interval_mask(click_trace_t, motion)
    motion_s = float(in_motion.sum() / fs)
    hold_s = float((~in_motion).sum() / fs)
    laeq_motion = level_db(weighted[in_motion]) if in_motion.any() else NAN
    laeq_hold = level_db(weighted[~in_motion]) if (~in_motion).any() else NAN
    clicks_motion = int(click_motion.sum())
    clicks_hold = int((~click_motion).sum())
    hold_rate = clicks_hold / hold_s if hold_s > 0 else NAN
    expected = hold_rate * motion_s if np.isfinite(hold_rate) else 0.0
    matched = 0
    for t in stick_slip_times:
        if click_trace_t.size and np.min(np.abs(click_trace_t - t)) <= cfg.stick_slip_match_s:
            matched += 1
    return {
        'available': True, 'fs': fs, 'duration_s': x.size / fs, 'start_t': start_t,
        'laeq_db': level_db(weighted), 'laeq_motion_db': laeq_motion, 'laeq_hold_db': laeq_hold,
        'peak_fast_db': float(np.max(frame_db)) if frame_db.size else NAN,
        'background_db': background_db, 'background_source': background_source,
        'motion_above_background_db': laeq_motion - background_db if np.isfinite(laeq_motion) else NAN,
        'clicks': int(click_t.size), 'clicks_motion': clicks_motion, 'clicks_hold': clicks_hold,
        'click_rate_motion_hz': clicks_motion / motion_s if motion_s > 0 else NAN,
        'click_rate_hold_hz': hold_rate,
        # Self-calibrated: clicks in motion beyond what the trial's own hold rate predicts.
        'excess_clicks_motion': float(clicks_motion - expected),
        'click_times': [float(v) for v in click_trace_t], 'click_prominence_db': [float(v) for v in prominence],
        'stick_slip_with_click': matched, 'stick_slip_events': len(stick_slip_times),
        'dominant_hz': dominant_frequency(x, fs, cfg.tonal_min_hz),
    }
