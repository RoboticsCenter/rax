"""Motion-quality metrics for joint traces. Pure numpy/scipy.

Units: radians, seconds, amperes. Inputs are uniformly sampled at ``fs`` unless a
function says otherwise (see ``signals.uniform``). Matrices are (T, J).

Smoothness follows Balasubramanian, Melendez-Calderon, Roby-Brami & Burdet (2015),
"On the analysis of movement smoothness", J NeuroEng Rehabil 12:112:

* SPARC: negative arc length of the normalised Fourier magnitude spectrum of the
  speed profile, over [0, wc] where wc is the highest frequency below the 10 Hz
  cap whose magnitude is still >= 0.05 (adaptive cut-off). Closer to 0 is smoother.
* LDLJ: -ln((t2-t1)^3 / v_peak^2 * integral |d^2 v/dt^2|^2 dt), the
  dimensionless, velocity-normalised jerk. Closer to 0 is smoother.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np
from scipy import signal
from scipy.integrate import trapezoid
from scipy.ndimage import maximum_filter1d, minimum_filter1d

from .signals import central_speed, derivative, lowpass, merge_regions, regions, running_max

NAN = float('nan')


@dataclass(frozen=True)
class MetricConfig:
    # Smoothness
    smooth_cutoff_hz: float = 20.0       # position low-pass before differentiation (protocol: 20-30 Hz)
    sparc_fc_hz: float = 10.0            # SPARC maximum cut-off (wc,max = 20*pi rad/s)
    sparc_amp_th: float = 0.05           # SPARC adaptive-cut-off amplitude threshold
    sparc_padlevel: int = 4              # zero padding: nfft = 2^(ceil(log2 T) + padlevel)
    min_excursion_rad: float = 0.01      # a joint "moves" if its reference spans at least this
    motion_speed_frac: float = 0.02      # motion = reference speed above this fraction of its peak
    motion_speed_min: float = 0.005      # ... and above this floor (rad/s)
    motion_pad_s: float = 0.5            # analysis window padding around reference motion
    # Tracking
    max_lag_s: float = 0.5
    final_window_s: float = 0.2
    overshoot_min_amplitude_rad: float = 0.02
    # Stick-slip and bursts (thresholds from docs/wuji-smoothness-2026-10-27.md)
    event_diff_window_s: float = 0.016   # centred-difference span for event speed
    spike_ratio: float = 2.0             # spike: speed > max(ratio * local ref peak, ...
    spike_margin_rad_s: float = 0.25     # ... |ref speed| + margin)
    ref_window_s: float = 0.1            # half-width of the local reference-peak window
    stick_window_s: float = 0.15         # pre-event window that must look stuck
    stick_speed_rad_s: float = 0.05      # stuck: median |speed| in the pre-window at most this
    stick_min_error_rad: float = 0.003   # tracking error at slip onset at least this
    stick_error_growth_rad: float = 0.0005  # and grew by at least this across the pre-window
    event_merge_s: float = 0.02
    # Holding oscillation
    hold_ref_speed_rad_s: float = 1e-3   # reference counts as constant below this speed
    hold_min_s: float = 0.5
    hold_settle_s: float = 0.3           # ignore this long after the reference stops
    hold_window_s: float = 1.0           # peak-to-peak window
    osc_window_s: float = 0.15           # oscillation detector (existing RAX test detector)
    osc_span_rad: float = 0.006
    osc_min_reversals: int = 3
    osc_deadband_rad: float = 0.0005
    # Current
    current_cap_a: float = 0.8           # used when the trace meta does not give a cap
    near_cap_frac: float = 0.05          # "near the cap": |I| >= (1 - frac) * cap
    # Timing
    gap_factor: float = 1.5              # an interval longer than this many periods is a gap

    def to_dict(self):
        return asdict(self)


# --------------------------------------------------------------------------- smoothness

def sparc(speed, fs, *, fc=10.0, amp_th=0.05, padlevel=4, return_spectrum=False):
    """Spectral arc length of a speed profile (Balasubramanian et al. 2015).

    speed: (T,) non-negative speed samples at fs Hz. Returns NaN for empty, constant-zero
    or non-finite input. With return_spectrum, also returns (f_sel, M_sel).
    """
    v = np.asarray(speed, dtype=float).ravel()
    if v.size < 3 or not np.all(np.isfinite(v)) or np.max(np.abs(v)) <= 0:
        return (NAN, None) if return_spectrum else NAN
    nfft = int(2 ** (np.ceil(np.log2(v.size)) + padlevel))
    magnitude = np.abs(np.fft.rfft(v, nfft))
    magnitude = magnitude / magnitude.max()
    f = np.arange(magnitude.size) * fs / nfft
    keep = f <= fc
    f_sel, m_sel = f[keep], magnitude[keep]
    above = np.flatnonzero(m_sel >= amp_th)
    if above.size == 0:
        return (NAN, None) if return_spectrum else NAN
    f_sel, m_sel = f_sel[above[0]:above[-1] + 1], m_sel[above[0]:above[-1] + 1]
    if f_sel.size < 2 or f_sel[-1] == f_sel[0]:
        value = NAN
    else:
        value = -float(np.sum(np.sqrt((np.diff(f_sel) / (f_sel[-1] - f_sel[0])) ** 2 + np.diff(m_sel) ** 2)))
    return (value, (f_sel, m_sel)) if return_spectrum else value


def ldlj(velocity, fs):
    """Log dimensionless jerk of a velocity profile: (T,) or (T, D) vectors.

    LDLJ = -ln((t2 - t1)^3 / v_peak^2 * integral ||d^2 v/dt^2||^2 dt), where v_peak is
    the peak speed ||v||. Dimensionless; closer to 0 is smoother.
    """
    v = np.asarray(velocity, dtype=float)
    if v.ndim == 1:
        v = v[:, None]
    if v.shape[0] < 5 or not np.all(np.isfinite(v)):
        return NAN
    duration = (v.shape[0] - 1) / fs
    peak = float(np.max(np.linalg.norm(v, axis=1)))
    if peak <= 0 or duration <= 0:
        return NAN
    jerk = derivative(v, fs, order=2)
    integral = float(trapezoid(np.sum(jerk ** 2, axis=1), dx=1.0 / fs))
    if integral <= 0:
        return NAN
    return -float(np.log(duration ** 3 / peak ** 2 * integral))


def rms_jerk(position, fs):
    """Root-mean-square of the jerk vector norm, from positions (T,) or (T, D)."""
    q = np.asarray(position, dtype=float)
    if q.ndim == 1:
        q = q[:, None]
    if q.shape[0] < 5 or not np.all(np.isfinite(q)):
        return NAN
    jerk = derivative(q, fs, order=3)
    return float(np.sqrt(np.mean(np.sum(jerk ** 2, axis=1))))


def motion_window(speed, fs, *, frac=0.02, floor=0.005, pad_s=0.5):
    """(start, stop) sample window covering all motion (speed above max(frac*peak, floor)), padded."""
    s = np.asarray(speed, dtype=float)
    if not s.size or not np.any(np.isfinite(s)):
        return None
    peak = np.nanmax(s)
    threshold = max(frac * peak, floor)
    moving = np.flatnonzero(s > threshold)
    if moving.size == 0:
        return None
    pad = int(round(pad_s * fs))
    return max(0, int(moving[0]) - pad), min(s.size, int(moving[-1]) + 1 + pad)


def smoothness(q, fs, cfg: MetricConfig, window=None, names=None):
    """SPARC, LDLJ and RMS jerk of a joint-space trajectory q (T, J), aggregated and per joint.

    The aggregate uses the joint-space speed ||dq/dt|| across the given joints.
    """
    q = np.asarray(q, dtype=float)
    if q.ndim == 1:
        q = q[:, None]
    qf = lowpass(q, fs, cfg.smooth_cutoff_hz)
    if window is not None:
        qf = qf[window[0]:window[1]]
    if qf.shape[0] < 5 or not np.all(np.isfinite(qf)):
        return {'sparc': NAN, 'ldlj': NAN, 'rms_jerk': NAN, 'peak_speed': NAN, 'per_joint': {}}
    v = derivative(qf, fs, 1)
    speed = np.linalg.norm(v, axis=1)
    kwargs = dict(fc=cfg.sparc_fc_hz, amp_th=cfg.sparc_amp_th, padlevel=cfg.sparc_padlevel)
    per_joint = {}
    for j in range(q.shape[1]):
        name = names[j] if names is not None else str(j)
        per_joint[name] = {'sparc': sparc(np.abs(v[:, j]), fs, **kwargs), 'ldlj': ldlj(v[:, j], fs),
                           'rms_jerk': rms_jerk(qf[:, j], fs), 'peak_speed': float(np.max(np.abs(v[:, j])))}
    return {'sparc': sparc(speed, fs, **kwargs), 'ldlj': ldlj(v, fs), 'rms_jerk': rms_jerk(qf, fs),
            'peak_speed': float(np.max(speed)), 'per_joint': per_joint}


# --------------------------------------------------------------------------- tracking

def xcorr_lag(ref, meas, fs, max_lag_s=0.5):
    """Delay of meas behind ref (s) from the cross-correlation of their velocities.

    ref, meas: (T,) or (T, J) positions. Per-joint normalised correlations are summed
    before the peak is located, with parabolic sub-sample refinement. Positive means
    the measurement lags the reference. Returns (lag_s, per_joint_lag_s list).
    """
    ref = np.asarray(ref, dtype=float)
    meas = np.asarray(meas, dtype=float)
    if ref.ndim == 1:
        ref, meas = ref[:, None], meas[:, None]
    n = ref.shape[0]
    K = int(min(round(max_lag_s * fs), n - 1))
    if K < 1:
        return NAN, []
    vr, vm = derivative(ref, fs), derivative(meas, fs)
    total = np.zeros(2 * K + 1)
    per_joint = []
    used = 0
    for j in range(ref.shape[1]):
        a, b = vr[:, j], vm[:, j]
        if not (np.all(np.isfinite(a)) and np.all(np.isfinite(b))):
            per_joint.append(NAN)
            continue
        a = a - a.mean()
        b = b - b.mean()
        norm = np.linalg.norm(a) * np.linalg.norm(b)
        if norm <= 0:
            per_joint.append(NAN)
            continue
        c = signal.correlate(b, a, mode='full', method='auto')[n - 1 - K:n + K] / norm
        total += c
        used += 1
        per_joint.append(_peak_lag(c, K, fs))
    return (_peak_lag(total, K, fs) if used else NAN), per_joint


def _peak_lag(c, K, fs):
    k = int(np.argmax(c))
    delta = 0.0
    if 0 < k < len(c) - 1:
        y0, y1, y2 = c[k - 1], c[k], c[k + 1]
        denominator = y0 - 2 * y1 + y2
        if denominator != 0:
            delta = 0.5 * (y0 - y2) / denominator
    return float((k - K + delta) / fs)


def tracking(q_ref, q_meas, fs, cfg: MetricConfig, motion_mask=None, lag_joints=None, names=None):
    """Tracking error e = q_ref - q_meas: pooled RMS, max, motion-only RMS, final error, lag."""
    e = np.asarray(q_ref, dtype=float) - np.asarray(q_meas, dtype=float)
    if e.ndim == 1:
        e = e[:, None]
    if not np.any(np.isfinite(e)):
        return {'available': False}
    abs_e = np.abs(e)
    n_final = max(1, int(round(cfg.final_window_s * fs)))
    final = np.nanmean(abs_e[-n_final:], axis=0)
    out = {'available': True,
           'rms': float(np.sqrt(np.nanmean(e ** 2))),
           'max': float(np.nanmax(abs_e)),
           'final_max': float(np.nanmax(final)),
           'rms_motion': NAN, 'lag_s': NAN}
    if motion_mask is not None and np.any(motion_mask):
        out['rms_motion'] = float(np.sqrt(np.nanmean(e[np.asarray(motion_mask)] ** 2)))
    if lag_joints is not None and len(lag_joints):
        lag, per = xcorr_lag(np.asarray(q_ref)[:, lag_joints], np.asarray(q_meas)[:, lag_joints], fs, cfg.max_lag_s)
        out['lag_s'] = lag
        out['lag_per_joint_s'] = {(names[j] if names else str(j)): v for j, v in zip(lag_joints, per)}
    out['rms_per_joint'] = {(names[j] if names else str(j)): float(np.sqrt(np.nanmean(e[:, j] ** 2)))
                            for j in range(e.shape[1]) if np.any(np.isfinite(e[:, j]))}
    return out


def overshoot(q_ref, q_meas, fs, cfg: MetricConfig, joints=None, names=None):
    """Excursion of q_meas beyond each reference move's final value, in the direction of motion.

    Moves are contiguous reference-motion segments with amplitude at least
    cfg.overshoot_min_amplitude_rad. Each move's window runs to the next move's start.
    """
    q_ref = np.asarray(q_ref, dtype=float)
    q_meas = np.asarray(q_meas, dtype=float)
    T = q_ref.shape[0]
    joints = range(q_ref.shape[1]) if joints is None else joints
    v_ref = derivative(q_ref, fs)
    worst, worst_rel, values, per_joint = 0.0, 0.0, [], {}
    for j in joints:
        speed = np.abs(v_ref[:, j])
        peak = float(np.nanmax(speed)) if np.any(np.isfinite(speed)) else 0.0
        if peak <= 0:
            continue
        threshold = max(cfg.motion_speed_min, cfg.motion_speed_frac * peak)
        spans = merge_regions(regions(speed > threshold), max(1, int(0.05 * fs)))
        for i, (a, _) in enumerate(spans):
            end = spans[i + 1][0] if i + 1 < len(spans) else T
            final = q_ref[end - 1, j]
            amplitude = final - q_ref[a, j]
            if abs(amplitude) < cfg.overshoot_min_amplitude_rad:
                continue
            excess = np.sign(amplitude) * (q_meas[a:end, j] - final)
            if not np.any(np.isfinite(excess)):
                continue
            value = max(0.0, float(np.nanmax(excess)))
            values.append(value)
            key = names[j] if names else str(j)
            per_joint[key] = max(per_joint.get(key, 0.0), value)
            worst = max(worst, value)
            worst_rel = max(worst_rel, value / abs(amplitude))
    return {'moves': len(values), 'max_rad': worst if values else NAN,
            'max_rel': worst_rel if values else NAN,
            'mean_rad': float(np.mean(values)) if values else NAN, 'per_joint_max_rad': per_joint}


# --------------------------------------------------------------------------- stick-slip

def stick_slip(q_ref, q_meas, fs, cfg: MetricConfig, joints=None, names=None, t0=0.0):
    """Stick-slip events: speed spikes far above the reference after a stuck, error-building phase.

    A spike is |v_meas| > max(spike_ratio * local reference peak speed, |v_ref| + margin),
    with v_meas a centred difference over event_diff_window_s. It is a stick-slip event
    when, over the stick_window_s before the spike, the joint was nearly still (median
    |v_meas| <= stick_speed), the tracking error reached stick_min_error and grew by at
    least stick_error_growth, and the spike moves toward the reference. Other spikes are
    counted as bursts. Event times are t0 + sample / fs. Peak speeds are centred-window
    speeds, so slips shorter than event_diff_window_s are reported below their true speed.
    """
    q_ref = np.asarray(q_ref, dtype=float)
    q_meas = np.asarray(q_meas, dtype=float)
    T = q_ref.shape[0]
    joints = range(q_ref.shape[1]) if joints is None else joints
    v_meas = central_speed(q_meas, fs, cfg.event_diff_window_s)
    v_ref = derivative(q_ref, fs)
    k = max(1, int(round(cfg.event_diff_window_s * fs / 2)))
    pre_n = max(3, int(round(cfg.stick_window_s * fs)))
    width = 2 * int(round(cfg.ref_window_s * fs)) + 1
    merge = max(1, int(round(cfg.event_merge_s * fs)))
    events, bursts, per_joint = [], [], {}
    for j in joints:
        name = names[j] if names else str(j)
        vm = v_meas[:, j]
        if not np.any(np.isfinite(vm)):
            continue
        vr = np.nan_to_num(v_ref[:, j])
        local = running_max(np.abs(vr), width)
        threshold = np.maximum(cfg.spike_ratio * local, np.abs(vr) + cfg.spike_margin_rad_s)
        spike = np.nan_to_num(np.abs(vm)) > threshold
        error = q_ref[:, j] - q_meas[:, j]
        for a, b in merge_regions(regions(spike), merge):
            p = a + int(np.nanargmax(np.abs(vm[a:b])))
            peak = float(vm[p])
            onset = max(0, a - k)
            start = onset - pre_n
            record = {'joint': name, 't': float(t0 + a / fs), 't_peak': float(t0 + p / fs),
                      'peak_speed': abs(peak), 'ref_speed': float(local[p]),
                      'jump_rad': float(q_meas[min(T - 1, b - 1 + k), j] - q_meas[onset, j])}
            stuck = growing = toward = False
            if start >= 0:
                pre = vm[start:onset]
                before = error[onset]
                smallest = np.nanmin(np.abs(error[start:onset + 1]))
                record['error_before_rad'] = float(before)
                stuck = bool(np.nanmedian(np.abs(pre)) <= cfg.stick_speed_rad_s) if pre.size else False
                growing = bool(abs(before) >= cfg.stick_min_error_rad and
                               abs(before) - smallest >= cfg.stick_error_growth_rad)
                toward = bool(np.sign(peak) == np.sign(before))
            record.update(stuck=stuck, error_growing=growing, toward_reference=toward)
            if stuck and growing and toward:
                events.append(record)
                per_joint[name] = per_joint.get(name, 0) + 1
            else:
                bursts.append(record)
    peaks = [e['peak_speed'] for e in events]
    return {'count': len(events), 'burst_count': len(bursts), 'per_joint': per_joint,
            'peak_speed_max': max(peaks) if peaks else NAN,
            'peak_speed_median': float(np.median(peaks)) if peaks else NAN,
            'events': events, 'bursts': bursts}


# --------------------------------------------------------------------------- holding

def reversal_indices(x, deadband):
    """Indices of direction reversals of x with hysteresis deadband (turning-point extremes)."""
    x = np.asarray(x, dtype=float)
    out = []
    n = x.size
    if n < 3:
        return out
    lo = hi = x[0]
    lo_i = hi_i = 0
    direction, i = 0, 1
    while i < n:  # Establish the initial direction once the range exceeds the deadband.
        if x[i] > hi:
            hi, hi_i = x[i], i
        if x[i] < lo:
            lo, lo_i = x[i], i
        if hi - lo > deadband:
            direction = 1 if hi_i > lo_i else -1
            break
        i += 1
    if direction == 0:
        return out
    extreme, extreme_i = (hi, hi_i) if direction == 1 else (lo, lo_i)
    for i in range(i + 1, n):
        v = x[i]
        if direction == 1:
            if v > extreme:
                extreme, extreme_i = v, i
            elif extreme - v > deadband:
                out.append(extreme_i)
                direction, extreme, extreme_i = -1, v, i
        else:
            if v < extreme:
                extreme, extreme_i = v, i
            elif v - extreme > deadband:
                out.append(extreme_i)
                direction, extreme, extreme_i = 1, v, i
    return out


def hold_oscillation(q_ref, q_meas, fs, cfg: MetricConfig, joints=None, names=None, t0=0.0):
    """Oscillation while the reference is constant.

    Hold periods are reference-still spans (|v_ref| < hold_ref_speed) at least
    hold_min_s long after discarding the first hold_settle_s. Reports the largest
    peak-to-peak of q_meas over hold_window_s windows, the RMS about the hold mean,
    direction reversals per second (deadband osc_deadband_rad), and oscillation events:
    osc_window_s windows with span > osc_span_rad and >= osc_min_reversals reversals.
    """
    q_ref = np.asarray(q_ref, dtype=float)
    q_meas = np.asarray(q_meas, dtype=float)
    joints = range(q_ref.shape[1]) if joints is None else joints
    v_ref = derivative(q_ref, fs)
    settle = int(round(cfg.hold_settle_s * fs))
    minimum = max(2, int(round(cfg.hold_min_s * fs)))
    window = max(2, int(round(cfg.hold_window_s * fs)))
    osc = max(2, int(round(cfg.osc_window_s * fs)))
    p2p_max, rms_max, reversals, joint_seconds, events = 0.0, 0.0, 0, 0.0, []
    holds = 0
    per_joint = {}
    for j in joints:
        name = names[j] if names else str(j)
        still = np.abs(np.nan_to_num(v_ref[:, j], nan=np.inf)) < cfg.hold_ref_speed_rad_s
        for a, b in regions(still):
            a += settle
            if b - a < minimum:
                continue
            segment = q_meas[a:b, j]
            if not np.all(np.isfinite(segment)):
                segment = segment[np.isfinite(segment)]
                if segment.size < minimum:
                    continue
            holds += 1
            size = min(window, segment.size)
            p2p = float(np.max(maximum_filter1d(segment, size, mode='nearest') -
                               minimum_filter1d(segment, size, mode='nearest')))
            rms = float(np.std(segment))
            turns = reversal_indices(segment, cfg.osc_deadband_rad)
            reversals += len(turns)
            joint_seconds += segment.size / fs
            p2p_max, rms_max = max(p2p_max, p2p), max(rms_max, rms)
            per_joint[name] = max(per_joint.get(name, 0.0), p2p)
            span = maximum_filter1d(segment, osc, mode='nearest') - minimum_filter1d(segment, osc, mode='nearest')
            ref_span = np.ptp(q_ref[a:b, j])
            if ref_span >= 0.001 or not np.any(span > cfg.osc_span_rad):
                continue
            turns = np.asarray(turns)
            centres = np.arange(segment.size)
            counts = (np.searchsorted(turns, centres + osc // 2, side='left') -
                      np.searchsorted(turns, centres - osc // 2, side='left')) if turns.size else np.zeros(segment.size)
            flagged = (span > cfg.osc_span_rad) & (counts >= cfg.osc_min_reversals)
            for s, e in merge_regions(regions(flagged), osc):
                events.append({'joint': name, 't': float(t0 + (a + s) / fs), 'duration_s': (e - s) / fs,
                               'span_rad': float(np.max(span[s:e]))})
    if not holds:
        return {'available': False, 'holds': 0, 'p2p_max_rad': NAN, 'rms_max_rad': NAN,
                'reversals_per_s': NAN, 'oscillation_events': 0, 'events': [], 'hold_joint_seconds': 0.0}
    return {'available': True, 'holds': holds, 'p2p_max_rad': p2p_max, 'rms_max_rad': rms_max,
            'reversals_per_s': reversals / joint_seconds if joint_seconds else NAN,
            'oscillation_events': len(events), 'events': events, 'hold_joint_seconds': joint_seconds,
            'per_joint_p2p_rad': per_joint}


# --------------------------------------------------------------------------- effort, thermal, timing

def current_stats(current, cap, near_frac=0.05, names=None):
    """RMS, peak and the fraction of samples with any joint within near_frac of the cap.

    current: (T, J) amperes (NaN where unavailable). cap: scalar or (J,) amperes.
    """
    current = np.asarray(current, dtype=float)
    if current.ndim == 1:
        current = current[:, None]
    finite = np.isfinite(current)
    if not finite.any():
        return {'available': False}
    cap = np.broadcast_to(np.asarray(cap, dtype=float), (current.shape[1],))
    magnitude = np.abs(current)
    near = np.where(finite, magnitude >= (1 - near_frac) * cap[None, :], False)
    rows = finite.any(axis=1)
    per_joint = {}
    for j in range(current.shape[1]):
        if finite[:, j].any():
            per_joint[names[j] if names else str(j)] = {
                'rms_a': float(np.sqrt(np.nanmean(current[:, j] ** 2))),
                'frac_near_cap': float(near[finite[:, j], j].mean())}
    return {'available': True, 'rms_a': float(np.sqrt(np.nanmean(current ** 2))),
            'peak_a': float(np.nanmax(magnitude)),
            'frac_near_cap': float(near.any(axis=1)[rows].mean()),
            'cap_a': [float(c) for c in cap], 'per_joint': per_joint}


def temperature_stats(temperature, t):
    """Maximum, and the largest per-joint rise between the first and last second."""
    temperature = np.asarray(temperature, dtype=float)
    if temperature.ndim == 1:
        temperature = temperature[:, None]
    if not np.any(np.isfinite(temperature)):
        return {'available': False}
    t = np.asarray(t, dtype=float)
    first = t <= t[0] + 1.0
    last = t >= t[-1] - 1.0
    with np.errstate(all='ignore'):
        rise = np.nanmean(temperature[last], axis=0) - np.nanmean(temperature[first], axis=0)
    rise_max = float(np.nanmax(rise)) if np.any(np.isfinite(rise)) else NAN
    duration = float(t[-1] - t[0])
    return {'available': True, 'max_c': float(np.nanmax(temperature)), 'rise_c': rise_max,
            'rise_c_per_min': rise_max / duration * 60 if duration > 0 else NAN}


def command_timing(t, nominal_rate_hz, gap_factor=1.5):
    """Achieved rate, interval jitter and gaps of the command timestamps t (raw, not resampled)."""
    t = np.asarray(t, dtype=float)
    dt = np.diff(t)
    if dt.size == 0 or t[-1] <= t[0]:
        return {'available': False}
    period = 1.0 / nominal_rate_hz
    achieved = (t.size - 1) / (t[-1] - t[0])
    gaps = dt > gap_factor * period
    skipped = int(np.sum(np.maximum(0, np.round(dt[gaps] / period) - 1)))
    return {'available': True, 'nominal_hz': float(nominal_rate_hz), 'achieved_hz': float(achieved),
            'rate_ratio': float(achieved / nominal_rate_hz),
            'jitter_ms': float(np.std(dt) * 1e3), 'p99_ms': float(np.percentile(dt, 99) * 1e3),
            'max_ms': float(np.max(dt) * 1e3), 'max_gap_periods': float(np.max(dt) / period),
            'gaps': int(np.sum(gaps)), 'skipped_slots': skipped}
