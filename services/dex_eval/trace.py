"""Shared per-trial trace format: reading, writing and validation.

One directory per trial::

    hand_<side>.npz   t (T,), q_ref q_target q_meas dq_meas current i_ff kp kd (T, 20)
    arm_<side>.npz    t (T,), q_ref q_meas (T, 6)                       optional
    meta.json         sides, joint_names, nominal_rate_hz, controller, skill, variant,
                      params, success, started_at, backend, notes
    audio.wav         optional, 48 kHz
    video_<cam>.mp4   optional

Units: seconds (monotonic), radians, amperes. Disabled joints are NaN in the
measured arrays (q_meas, dq_meas, current). Extra arrays (for example
``temperature_c``) are kept and ignored by validation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

SIDES = ('left', 'right')
N_HAND = 20
N_ARM = 6
JOINT_NAMES = tuple(f'{finger}_S{joint}' for finger in ('thumb', 'index', 'middle', 'ring', 'pinky')
                    for joint in range(1, 5))
HAND_MATRICES = ('q_ref', 'q_target', 'q_meas', 'dq_meas', 'current', 'i_ff', 'kp', 'kd')
MEASURED = ('q_meas', 'dq_meas', 'current')
ARM_MATRICES = ('q_ref', 'q_meas')
META_KEYS = ('sides', 'joint_names', 'nominal_rate_hz', 'controller', 'skill', 'variant', 'params',
             'success', 'started_at', 'backend', 'notes')
BACKENDS = ('real', 'sim', 'mock')
STATUSES = ('success', 'failed_precondition', 'dropped', 'timeout', 'aborted')
# Joints whose q_meas is finite in at least this fraction of samples count as active.
ACTIVE_FRACTION = 0.9


class TraceError(ValueError):
    """A trial directory or array set violates the trace contract."""

    def __init__(self, problems):
        self.problems = list(problems) if not isinstance(problems, str) else [problems]
        super().__init__('; '.join(self.problems))


@dataclass
class HandTrace:
    side: str
    t: np.ndarray
    q_ref: np.ndarray
    q_target: np.ndarray
    q_meas: np.ndarray
    dq_meas: np.ndarray
    current: np.ndarray
    i_ff: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    extra: dict = field(default_factory=dict)

    @property
    def n(self):
        return int(self.t.shape[0])

    @property
    def duration(self):
        return float(self.t[-1] - self.t[0]) if self.n > 1 else 0.0

    def active(self):
        """Boolean (20,) mask of joints with measured positions (disabled joints are NaN)."""
        finite = np.isfinite(self.q_meas).mean(axis=0) if self.n else np.zeros(N_HAND)
        return finite >= ACTIVE_FRACTION

    def arrays(self):
        return {'t': self.t, **{k: getattr(self, k) for k in HAND_MATRICES}, **self.extra}


@dataclass
class ArmTrace:
    side: str
    t: np.ndarray
    q_ref: np.ndarray
    q_meas: np.ndarray
    extra: dict = field(default_factory=dict)

    @property
    def n(self):
        return int(self.t.shape[0])

    def arrays(self):
        return {'t': self.t, 'q_ref': self.q_ref, 'q_meas': self.q_meas, **self.extra}


@dataclass
class Trial:
    path: Path
    meta: dict
    hands: dict
    arms: dict
    audio: Path | None = None
    videos: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)

    @property
    def t0(self):
        """Earliest trace timestamp (the trial's clock origin)."""
        starts = [h.t[0] for h in self.hands.values() if h.n] + [a.t[0] for a in self.arms.values() if a.n]
        return float(min(starts)) if starts else 0.0


def _time_problems(t, name):
    problems, warnings = [], []
    if t.ndim != 1:
        problems.append(f'{name}: t must be 1-D, got shape {t.shape}')
        return problems, warnings
    if t.shape[0] < 2:
        problems.append(f'{name}: at least 2 samples required, got {t.shape[0]}')
        return problems, warnings
    if not np.all(np.isfinite(t)):
        problems.append(f'{name}: t contains non-finite values')
        return problems, warnings
    dt = np.diff(t)
    if np.any(dt < 0):
        problems.append(f'{name}: t is not monotonic ({int(np.sum(dt < 0))} decreasing steps)')
    elif np.any(dt == 0):
        warnings.append(f'{name}: {int(np.sum(dt == 0))} duplicate timestamps (dropped for analysis)')
    return problems, warnings


def validate_hand_arrays(arrays: dict, name='hand'):
    """(errors, warnings) for one hand's arrays."""
    errors, warnings = [], []
    missing = [k for k in ('t', *HAND_MATRICES) if k not in arrays]
    if missing:
        return [f'{name}: missing arrays {missing}'], warnings
    t = np.asarray(arrays['t'])
    if not np.issubdtype(t.dtype, np.floating):
        errors.append(f'{name}: t must be floating point, got {t.dtype}')
    e, w = _time_problems(t.astype(float, copy=False), name)
    errors += e
    warnings += w
    T = t.shape[0] if t.ndim == 1 else None
    for key in HAND_MATRICES:
        value = np.asarray(arrays[key])
        if value.shape != (T, N_HAND):
            errors.append(f'{name}: {key} must have shape ({T}, {N_HAND}), got {value.shape}')
            continue
        if not np.issubdtype(value.dtype, np.number):
            errors.append(f'{name}: {key} must be numeric, got {value.dtype}')
            continue
        if np.any(np.isinf(value)):
            errors.append(f'{name}: {key} contains infinities')
        if key not in MEASURED:
            partial = [JOINT_NAMES[j] for j in range(N_HAND)
                       if np.isnan(value[:, j]).any() and not np.isnan(value[:, j]).all()]
            if partial:
                warnings.append(f'{name}: {key} has partial NaN in {partial}')
    if not errors:
        q_meas = np.asarray(arrays['q_meas'])
        if not np.isfinite(q_meas).any():
            warnings.append(f'{name}: q_meas is entirely NaN')
    return errors, warnings


def validate_arm_arrays(arrays: dict, name='arm'):
    errors, warnings = [], []
    missing = [k for k in ('t', *ARM_MATRICES) if k not in arrays]
    if missing:
        return [f'{name}: missing arrays {missing}'], warnings
    t = np.asarray(arrays['t'], dtype=float)
    e, w = _time_problems(t, name)
    errors += e
    warnings += w
    T = t.shape[0] if t.ndim == 1 else None
    for key in ARM_MATRICES:
        value = np.asarray(arrays[key])
        if value.shape != (T, N_ARM):
            errors.append(f'{name}: {key} must have shape ({T}, {N_ARM}), got {value.shape}')
    return errors, warnings


def validate_meta(meta: Any):
    errors, warnings = [], []
    if not isinstance(meta, dict):
        return ['meta.json must contain an object'], warnings
    missing = [k for k in META_KEYS if k not in meta]
    if missing:
        errors.append(f'meta.json: missing keys {missing}')
    sides = meta.get('sides')
    if 'sides' in meta and (not isinstance(sides, list) or not sides or
                            any(s not in SIDES for s in sides) or len(set(sides)) != len(sides)):
        errors.append(f'meta.json: sides must be a non-empty list of distinct {SIDES}, got {sides!r}')
    names = meta.get('joint_names')
    if 'joint_names' in meta and (not isinstance(names, list) or len(names) != N_HAND or
                                  not all(isinstance(n, str) for n in names)):
        errors.append('meta.json: joint_names must be a list of 20 strings')
    elif isinstance(names, list) and tuple(names) != JOINT_NAMES:
        warnings.append('meta.json: joint_names differ from the Wuji SDK order')
    rate = meta.get('nominal_rate_hz')
    if 'nominal_rate_hz' in meta and (type(rate) not in (int, float) or not math.isfinite(rate) or rate <= 0):
        errors.append(f'meta.json: nominal_rate_hz must be a positive number, got {rate!r}')
    if 'backend' in meta and meta.get('backend') not in BACKENDS:
        errors.append(f"meta.json: backend must be one of {BACKENDS}, got {meta.get('backend')!r}")
    if 'success' in meta and meta.get('success') not in (True, False, None):
        errors.append('meta.json: success must be true, false or null')
    if 'params' in meta and not isinstance(meta.get('params'), dict):
        errors.append('meta.json: params must be an object')
    for key in ('controller', 'skill', 'variant', 'notes'):
        if key in meta and meta[key] is not None and not isinstance(meta[key], str):
            errors.append(f'meta.json: {key} must be a string or null')
    started = meta.get('started_at')
    if 'started_at' in meta and started is not None and not isinstance(started, (str, int, float)):
        errors.append('meta.json: started_at must be an ISO-8601 string or epoch seconds')
    return errors, warnings


def _load_npz(path):
    with np.load(path, allow_pickle=False) as data:
        return {k: np.array(data[k]) for k in data.files}


def validate_trial_dir(path):
    """(errors, warnings) for a trial directory, without raising."""
    path = Path(path)
    errors, warnings = [], []
    meta_path = path / 'meta.json'
    if not meta_path.is_file():
        return [f'{path}: meta.json missing'], warnings
    try:
        meta = json.loads(meta_path.read_text())
    except ValueError as exc:
        return [f'{path}: meta.json is not valid JSON ({exc})'], warnings
    e, w = validate_meta(meta)
    errors += e
    warnings += w
    sides = meta.get('sides') if isinstance(meta.get('sides'), list) else []
    for side in sides:
        if side not in SIDES:
            continue
        hand = path / f'hand_{side}.npz'
        if not hand.is_file():
            errors.append(f'hand_{side}.npz missing (listed in meta.sides)')
            continue
        try:
            e, w = validate_hand_arrays(_load_npz(hand), f'hand_{side}')
        except Exception as exc:  # Corrupt archive.
            e, w = [f'hand_{side}.npz unreadable: {exc}'], []
        errors += e
        warnings += w
    for side in SIDES:
        if side not in sides and (path / f'hand_{side}.npz').is_file():
            warnings.append(f'hand_{side}.npz present but not listed in meta.sides (ignored)')
        arm = path / f'arm_{side}.npz'
        if arm.is_file():
            try:
                e, w = validate_arm_arrays(_load_npz(arm), f'arm_{side}')
            except Exception as exc:
                e, w = [f'arm_{side}.npz unreadable: {exc}'], []
            errors += e
            warnings += w
    return errors, warnings


def load_trial(path, *, validate=True) -> Trial:
    """Load a trial directory; raises TraceError listing every contract violation."""
    path = Path(path)
    warnings = []
    if validate:
        errors, warnings = validate_trial_dir(path)
        if errors:
            raise TraceError([f'{path}: {e}' for e in errors])
    meta = json.loads((path / 'meta.json').read_text())
    hands, arms = {}, {}
    for side in meta.get('sides') or []:
        data = _load_npz(path / f'hand_{side}.npz')
        core = {k: np.asarray(data.pop(k), dtype=float) for k in ('t', *HAND_MATRICES)}
        hands[side] = HandTrace(side=side, extra=data, **core)
    for side in SIDES:
        arm_path = path / f'arm_{side}.npz'
        if arm_path.is_file():
            data = _load_npz(arm_path)
            core = {k: np.asarray(data.pop(k), dtype=float) for k in ('t', *ARM_MATRICES)}
            arms[side] = ArmTrace(side=side, extra=data, **core)
    audio = path / 'audio.wav'
    videos = {p.stem[len('video_'):]: p for p in sorted(path.glob('video_*.mp4'))}
    return Trial(path=path, meta=meta, hands=hands, arms=arms,
                 audio=audio if audio.is_file() else None, videos=videos, warnings=warnings)


def is_trial_dir(path):
    path = Path(path)
    return (path / 'meta.json').is_file() and any(path.glob('hand_*.npz'))


def find_trial_dirs(root):
    """Every trial directory at or below root, sorted by path."""
    root = Path(root)
    if is_trial_dir(root):
        return [root]
    return sorted({p.parent for p in root.rglob('meta.json') if is_trial_dir(p.parent)})


def default_meta(**overrides):
    meta = {'sides': [], 'joint_names': list(JOINT_NAMES), 'nominal_rate_hz': 200.0, 'controller': None,
            'skill': None, 'variant': None, 'params': {}, 'success': None, 'started_at': None,
            'backend': 'mock', 'notes': None}
    meta.update(overrides)
    return meta


def save_hand(path, *, t, q_ref, q_target, q_meas, dq_meas=None, current=None, i_ff=None, kp=None, kd=None,
              **extra):
    """Write hand_<side>.npz; optional arrays default to NaN (measured) or zeros (commands)."""
    t = np.asarray(t, dtype=float)
    T = t.shape[0]
    nan = np.full((T, N_HAND), np.nan)
    zeros = np.zeros((T, N_HAND))
    arrays = {'t': t, 'q_ref': q_ref, 'q_target': q_target, 'q_meas': q_meas,
              'dq_meas': nan if dq_meas is None else dq_meas,
              'current': nan if current is None else current,
              'i_ff': zeros if i_ff is None else i_ff,
              'kp': zeros if kp is None else kp, 'kd': zeros if kd is None else kd}
    arrays = {k: np.asarray(v, dtype=float) for k, v in arrays.items()}
    errors, _ = validate_hand_arrays(arrays, Path(path).stem)
    if errors:
        raise TraceError(errors)
    np.savez_compressed(path, **arrays, **{k: np.asarray(v) for k, v in extra.items()})


def save_arm(path, *, t, q_ref, q_meas, **extra):
    arrays = {'t': np.asarray(t, dtype=float), 'q_ref': np.asarray(q_ref, dtype=float),
              'q_meas': np.asarray(q_meas, dtype=float)}
    errors, _ = validate_arm_arrays(arrays, Path(path).stem)
    if errors:
        raise TraceError(errors)
    np.savez_compressed(path, **arrays, **{k: np.asarray(v) for k, v in extra.items()})


def write_meta(path, meta):
    """Write meta.json after validation (a trial directory's last step)."""
    errors, _ = validate_meta(meta)
    if errors:
        raise TraceError(errors)
    Path(path).write_text(json.dumps(meta, indent=2, sort_keys=True, allow_nan=False, default=str) + '\n')
