"""Synthetic references and plants with known properties, for tests, fixtures and the mock rig."""
from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from .trace import JOINT_NAMES, N_HAND, default_meta, save_arm, save_hand, write_meta


def min_jerk(q0, q1, n):
    """n samples of the quintic minimum-jerk blend from q0 to q1 (inclusive)."""
    u = np.linspace(0.0, 1.0, n)[:, None]
    s = u ** 3 * (10 - 15 * u + 6 * u ** 2)
    return np.asarray(q0, dtype=float)[None, :] + s * (np.asarray(q1, dtype=float) - np.asarray(q0, dtype=float))[None, :]


def waypoint_reference(waypoints, fs, move_s=1.0, hold_s=0.5, lead_s=0.5):
    """Rest-to-rest min-jerk moves through waypoints (each (J,)), with holds between and around."""
    waypoints = [np.asarray(w, dtype=float) for w in waypoints]
    pieces = [np.repeat(waypoints[0][None, :], int(round(lead_s * fs)), axis=0)]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        pieces.append(min_jerk(a, b, int(round(move_s * fs)) + 1)[1:])
        pieces.append(np.repeat(b[None, :], int(round(hold_s * fs)), axis=0))
    return np.concatenate(pieces, axis=0)


def second_order(q_ref, fs, omega=40.0, zeta=0.9, q0=None):
    """Measured positions of a damped second-order tracker of q_ref (semi-implicit Euler, 4x substeps)."""
    q_ref = np.asarray(q_ref, dtype=float)
    q = np.array(q_ref[0] if q0 is None else q0, dtype=float)
    v = np.zeros_like(q)
    dt = 1.0 / fs / 4
    out = np.empty_like(q_ref)
    for i, r in enumerate(q_ref):
        for _ in range(4):
            a = omega ** 2 * (r - q) - 2 * zeta * omega * v
            v = v + a * dt
            q = q + v * dt
        out[i] = q
    return out


def breakaway(q_ref, fs, break_rad=0.012, slip_s=0.008, joints=None):
    """Stick-slip plant: a joint stays put until |q_ref - q| exceeds break_rad, then slips to q_ref
    within slip_s. Returns (q_meas, slip onset sample indices per joint)."""
    q_ref = np.asarray(q_ref, dtype=float)
    out = q_ref.copy()
    joints = range(q_ref.shape[1]) if joints is None else joints
    n_slip = max(1, int(round(slip_s * fs)))
    onsets = {}
    for j in joints:
        q = q_ref[0, j]
        i = 0
        T = q_ref.shape[0]
        events = []
        while i < T:
            if abs(q_ref[i, j] - q) > break_rad:
                start, goal = q, q_ref[min(T - 1, i + n_slip), j]
                events.append(i)
                for k in range(n_slip):
                    if i + k < T:
                        out[i + k, j] = start + (goal - start) * (k + 1) / n_slip
                q = goal
                i += n_slip
                continue
            out[i, j] = q
            i += 1
        onsets[j] = events
    return out, onsets


def hand_arrays(q_ref, q_meas, fs, *, t0=100.0, jitter_s=0.0, disabled=(), current=None, kp=8.0, kd=0.15,
                rng=None):
    """Complete hand_<side>.npz arrays for a reference and measurement on a (possibly jittered) clock."""
    rng = rng or np.random.default_rng(0)
    T = q_ref.shape[0]
    t = t0 + np.arange(T) / fs
    if jitter_s:
        t = t + rng.uniform(0, jitter_s, T)
        t = np.maximum.accumulate(t)
        t[1:] = np.maximum(t[1:], t[:-1] + 1e-6)
    q_meas = np.array(q_meas, dtype=float)
    dq = np.gradient(q_meas, 1.0 / fs, axis=0)
    if current is None:
        current = np.clip(2 * kp * (q_ref - q_meas) - 2 * kd * dq, -0.8, 0.8)
    current = np.array(current, dtype=float)
    for j in disabled:
        index = JOINT_NAMES.index(j) if isinstance(j, str) else j
        q_meas[:, index] = dq[:, index] = current[:, index] = np.nan
    return {'t': t, 'q_ref': q_ref, 'q_target': q_ref, 'q_meas': q_meas, 'dq_meas': dq, 'current': current,
            'i_ff': np.zeros((T, N_HAND)), 'kp': np.full((T, N_HAND), kp), 'kd': np.full((T, N_HAND), kd)}


def write_trial(directory, hands, *, arms=None, meta=None, audio=None):
    """Write a trial directory. hands/arms: {side: arrays}; audio: (samples, fs) or None."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for side, arrays in hands.items():
        save_hand(directory / f'hand_{side}.npz', **arrays)
    for side, arrays in (arms or {}).items():
        save_arm(directory / f'arm_{side}.npz', **arrays)
    if audio is not None:
        from scipy.io import wavfile
        samples, fs = audio
        wavfile.write(str(directory / 'audio.wav'), int(fs),
                      (np.clip(samples, -1, 1) * 32767).astype(np.int16))
    full = default_meta(sides=sorted(hands), started_at=datetime.now(timezone.utc).isoformat())
    full.update(meta or {})
    write_meta(directory / 'meta.json', full)
    return directory


def demo_waypoints(rng=None, joints=(4, 5, 6, 8, 9, 10), amplitude=0.4):
    """A small open-close-open sequence on a few finger joints."""
    rng = rng or np.random.default_rng(0)
    open_pose = np.zeros(N_HAND)
    closed = open_pose.copy()
    closed[list(joints)] = amplitude * (0.8 + 0.4 * rng.random(len(joints)))
    return [open_pose, closed, open_pose]
