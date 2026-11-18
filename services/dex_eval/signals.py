"""Signal helpers shared by the metrics: resampling, filtering, differentiation, regions."""
from __future__ import annotations

import numpy as np
from scipy import signal


def dedupe(t, *arrays):
    """Drop samples whose timestamp does not increase (keeps the first of each run)."""
    t = np.asarray(t, dtype=float)
    keep = np.ones(t.shape[0], dtype=bool)
    if t.shape[0] > 1:
        running = np.maximum.accumulate(t)
        keep[1:] = t[1:] > running[:-1]
    return (t[keep], *[np.asarray(a)[keep] for a in arrays])


def sample_rate(t):
    """Median sampling rate of a timestamp vector (Hz)."""
    dt = np.diff(np.asarray(t, dtype=float))
    dt = dt[dt > 0]
    return float(1.0 / np.median(dt)) if dt.size else float('nan')


def uniform(t, x, fs=None):
    """Linearly resample x(t) (T,) or (T, J) onto a uniform grid.

    Columns with fewer than two finite samples stay NaN; interior NaN gaps are
    bridged by interpolation between finite neighbours. Returns (tu, xu, fs).
    """
    t, x = dedupe(t, x)
    x = np.asarray(x, dtype=float)
    squeeze = x.ndim == 1
    if squeeze:
        x = x[:, None]
    fs = float(fs) if fs else sample_rate(t)
    n = int(np.floor((t[-1] - t[0]) * fs + 1e-9)) + 1
    tu = t[0] + np.arange(n) / fs
    xu = np.full((n, x.shape[1]), np.nan)
    for j in range(x.shape[1]):
        ok = np.isfinite(x[:, j])
        if ok.sum() >= 2:
            xu[:, j] = np.interp(tu, t[ok], x[ok, j])
    return tu, (xu[:, 0] if squeeze else xu), fs


def lowpass(x, fs, cutoff_hz, order=2):
    """Zero-phase Butterworth low-pass along axis 0; a no-op when the cutoff is unusable."""
    x = np.asarray(x, dtype=float)
    if not cutoff_hz or cutoff_hz >= 0.45 * fs:
        return x.copy()
    sos = signal.butter(order, cutoff_hz, btype='low', fs=fs, output='sos')
    padlen = 3 * (2 * len(sos) + 1)
    if x.shape[0] <= padlen:
        return x.copy()
    out = x.copy()
    if out.ndim == 1:
        return signal.sosfiltfilt(sos, out) if np.all(np.isfinite(out)) else out
    for j in range(out.shape[1]):
        if np.all(np.isfinite(out[:, j])):
            out[:, j] = signal.sosfiltfilt(sos, out[:, j])
    return out


def derivative(x, fs, order=1):
    """order-th time derivative along axis 0 by repeated central differences."""
    out = np.asarray(x, dtype=float)
    for _ in range(order):
        if out.shape[0] < 2:
            return np.zeros_like(out)
        out = np.gradient(out, 1.0 / fs, axis=0)
    return out


def central_speed(x, fs, window_s):
    """Velocity by a centred difference spanning about window_s (edge samples use shorter spans)."""
    x = np.asarray(x, dtype=float)
    k = max(1, int(round(window_s * fs / 2)))
    n = x.shape[0]
    idx = np.arange(n)
    hi = np.minimum(idx + k, n - 1)
    lo = np.maximum(idx - k, 0)
    span = (hi - lo) / fs
    span[span == 0] = np.nan
    if x.ndim == 1:
        return (x[hi] - x[lo]) / span
    return (x[hi] - x[lo]) / span[:, None]


def regions(mask):
    """[(start, stop)] index ranges (stop exclusive) where mask is True."""
    mask = np.asarray(mask, dtype=bool)
    if not mask.size:
        return []
    padded = np.concatenate(([False], mask, [False]))
    edges = np.flatnonzero(np.diff(padded.astype(np.int8)))
    return [(int(a), int(b)) for a, b in zip(edges[::2], edges[1::2])]


def merge_regions(spans, gap):
    """Merge [(start, stop)] spans separated by fewer than gap samples."""
    merged = []
    for start, stop in spans:
        if merged and start - merged[-1][1] < gap:
            merged[-1] = (merged[-1][0], stop)
        else:
            merged.append((start, stop))
    return merged


def running_max(x, width):
    """Centred running maximum of x over width samples."""
    from scipy.ndimage import maximum_filter1d
    width = max(1, int(width))
    return maximum_filter1d(np.asarray(x, dtype=float), size=width, mode='nearest')


def interval_mask(t, intervals):
    """Boolean mask of times t inside any [a, b) interval."""
    t = np.asarray(t, dtype=float)
    mask = np.zeros(t.shape, dtype=bool)
    for a, b in intervals:
        mask |= (t >= a) & (t < b)
    return mask
