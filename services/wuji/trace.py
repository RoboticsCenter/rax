"""Shared hand trace format: one directory per trial.

    <trial>/hand_<side>.npz   t (T,) float64 monotonic s; q_ref, q_target, q_meas, dq_meas,
                              current, i_ff, kp, kd (T, 20) float64. Disabled joints are NaN
                              in measured arrays; unavailable measurements are NaN.
                              Extras: dq_ref, q_goal, current_diag, temperature_c (MCU, C),
                              current_limit_a (T, 20) float32; state_seq, state_time_us (T,)
                              int64 (-1 if unknown).
    <trial>/meta.json         sides, joint_names, nominal_rate_hz, controller, skill, variant,
                              params, success, started_at, backend, notes, plus current_limit_a
                              (active cap: scalar, 20-list, or {side: ...}) and extra keys.
    <trial>/arm_<side>.npz    optional: t, q_ref (T, 6), q_meas (T, 6).

The recorder appends one row per command published by the hand writer, so ``t`` is the
inner command rate. numpy is imported only when a recording starts or is loaded.
"""
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import threading
import uuid

from .config import JOINT_NAMES

FORMAT = 'rax-hand-trace/1'
CONTRACT = ('q_ref', 'q_target', 'q_meas', 'dq_meas', 'current', 'i_ff', 'kp', 'kd')
EXTRAS = ('dq_ref', 'q_goal', 'current_diag', 'temperature_c', 'current_limit_a')
CHUNK = 4096
META_KEYS = ('sides', 'joint_names', 'nominal_rate_hz', 'controller', 'skill', 'variant', 'params',
             'success', 'started_at', 'backend', 'notes')
BACKENDS = ('real', 'sim', 'mock')


def trial_id(label='trial'):
    label = re.sub(r'[^A-Za-z0-9_-]+', '-', str(label or 'trial'))[:48].strip('-') or 'trial'
    return f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')}_{label}_{uuid.uuid4().hex[:6]}"


def iso_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def safe_directory(path, depth=3):
    """Absolute trial directory whose last components are not symlinks.

    Trace roots can live in the agent-writable workspace; refusing symlinked trial,
    traces and runtime directories keeps host writes inside the intended tree.
    """
    path = Path(path)
    if not path.is_absolute() or '..' in path.parts:
        raise ValueError('trace directory must be an absolute path without ..')
    for candidate in [path, *path.parents][:depth+1]:
        if candidate.is_symlink():
            raise ValueError(f'refusing symlinked trace path {candidate}')
    return path


class TraceRecorder:
    """Chunked in-memory rows for one hand; save() writes hand_<side>.npz atomically."""
    def __init__(self, directory, side, *, max_seconds, nominal_rate_hz, cfg, backend='real', info=None):
        import numpy as np
        self.np = np
        self.directory = safe_directory(directory)
        if side not in ('left', 'right'):
            raise ValueError('trace side must be left or right')
        self.side, self.max_seconds, self.nominal_rate_hz = side, float(max_seconds), float(nominal_rate_hz)
        self.backend = backend if backend in BACKENDS else 'real'
        self.info = dict(info or {})
        self.excluded = [i for i, name in enumerate(JOINT_NAMES) if name in cfg.get('disabled_joints', ())]
        self.lock = threading.Lock()
        self.chunks, self.rows, self.count = [], 0, 0
        self.started_at, self.first_t, self.truncated = iso_now(), None, False
        self.closed = False
        self.saved = None
        self.error = None

    def _new_chunk(self):
        np = self.np
        chunk = {'t': np.full(CHUNK, np.nan), 'state_seq': np.full(CHUNK, -1, np.int64),
                 'state_time_us': np.full(CHUNK, -1, np.int64)}
        chunk.update({key: np.full((CHUNK, 20), np.nan) for key in CONTRACT})
        chunk.update({key: np.full((CHUNK, 20), np.nan, np.float32) for key in EXTRAS})
        self.chunks.append(chunk)
        self.rows = 0
        return chunk

    def append(self, t, *, q_ref, q_target, state, i_ff, kp, kd, dq_ref, q_goal, current_limit_a=None):
        """One published command. ``state`` is the driver snapshot used for that tick."""
        with self.lock:
            if self.closed:
                return False
            if self.first_t is None:
                self.first_t = t
            if t-self.first_t > self.max_seconds:
                self.truncated = True
                return False
            chunk = self.chunks[-1] if self.chunks and self.rows < CHUNK else self._new_chunk()
            r = self.rows
            chunk['t'][r] = t
            for key, values in (('q_ref', q_ref), ('q_target', q_target), ('i_ff', i_ff), ('kp', kp),
                                ('kd', kd), ('dq_ref', dq_ref), ('q_goal', q_goal),
                                ('current_limit_a', current_limit_a)):
                if values is not None:
                    chunk[key][r] = values
            diagnostics = state.get('diagnostics') or {}
            for key, values in (('q_meas', state.get('positions')), ('dq_meas', state.get('velocity_rad_s')),
                                ('current', state.get('effort_a')), ('current_diag', diagnostics.get('current')),
                                ('temperature_c', diagnostics.get('mcu_temp_c_fb'))):
                if values is not None and len(values) == 20:
                    chunk[key][r] = [math.nan if v is None else v for v in values]
            header = (state.get('source_headers') or {}).get('state') or {}
            if 'sequence' in header:
                chunk['state_seq'][r] = header['sequence']
                chunk['state_time_us'][r] = header.get('timestamp_us', -1)
            self.rows += 1
            self.count += 1
            return True

    def close(self):
        with self.lock:
            self.closed = True

    def finish(self):
        """Stop accepting rows; returns the concatenated arrays."""
        np = self.np
        with self.lock:
            self.closed = True
            chunks, rows = list(self.chunks), self.rows
        if not chunks:
            arrays = {'t': np.zeros(0), 'state_seq': np.zeros(0, np.int64), 'state_time_us': np.zeros(0, np.int64)}
            arrays.update({key: np.zeros((0, 20)) for key in CONTRACT})
            arrays.update({key: np.zeros((0, 20), np.float32) for key in EXTRAS})
            return arrays
        lengths = [CHUNK]*(len(chunks)-1) + [rows]
        arrays = {key: np.concatenate([chunk[key][:n] for chunk, n in zip(chunks, lengths)])
                  for key in chunks[0]}
        for key in ('q_meas', 'dq_meas', 'current', 'current_diag', 'temperature_c'):
            if self.excluded:
                arrays[key][:, self.excluded] = np.nan
        return arrays

    def save(self):
        try:
            arrays = self.finish()
            self.directory.mkdir(parents=True, exist_ok=True)
            path = self.directory/f'hand_{self.side}.npz'
            temporary = self.directory/f'.hand_{self.side}.{uuid.uuid4().hex}.npz'
            with open(temporary, 'wb') as stream:
                self.np.savez(stream, **arrays)
            os.replace(temporary, path)
            t = arrays['t']
            self.saved = {'path': str(path), 'samples': int(t.shape[0]),
                          'duration_s': float(t[-1]-t[0]) if t.shape[0] > 1 else 0.,
                          'observed_hz': (float((t.shape[0]-1)/(t[-1]-t[0])) if t.shape[0] > 1 and t[-1] > t[0]
                                          else None),
                          'truncated': self.truncated}
        except Exception as exc:  # noqa: BLE001 -- reported through record status.
            self.error = f'{type(exc).__name__}: {exc}'
        return self.saved

    def summary(self):
        return {'directory': str(self.directory), 'side': self.side, 'samples': self.count,
                'started_at': self.started_at, 'truncated': self.truncated, 'closed': self.closed,
                'saved': self.saved, 'error': self.error, 'backend': self.backend,
                'nominal_rate_hz': self.nominal_rate_hz, **self.info}


def write_meta(directory, *, sides, nominal_rate_hz, controller=None, skill=None, variant=None,
               params=None, success=None, started_at=None, backend='real', notes='', **extra):
    """Write meta.json with the shared keys first; extra keys are appended."""
    directory = safe_directory(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if backend not in BACKENDS:
        raise ValueError(f'backend must be one of {BACKENDS}')
    if success is not None and type(success) is not bool:
        raise ValueError('success must be true, false or null')
    value = {'sides': list(sides), 'joint_names': list(JOINT_NAMES), 'nominal_rate_hz': nominal_rate_hz,
             'controller': controller, 'skill': skill, 'variant': variant, 'params': params or {},
             'success': success, 'started_at': started_at or iso_now(), 'backend': backend,
             'notes': notes or '', 'trace_format': FORMAT, **extra}
    temporary = directory/f'.meta.{uuid.uuid4().hex}.json'
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False, default=str) + '\n')
    os.replace(temporary, directory/'meta.json')
    return value


def compact(values):
    if isinstance(values, list) and values and len(set(values)) == 1:
        return values[0]
    return values


def hand_context(status):
    """Per-side meta details from a hand owner status snapshot."""
    status = status or {}
    stream = status.get('stream') or {}
    return {key: status.get(key) for key in ('serial', 'handedness', 'firmware', 'profile_revision',
                                              'disabled_joints', 'motion_mode')} | {
        'smooth_profile': status.get('smooth_profile'), 'stream_settings': stream.get('settings'),
        'current_limit_a': compact((status.get('control_parameters') or {}).get('current_limit_a')),
        'command_timing': {k: (status.get('command_timing') or {}).get(k)
                           for k in ('requested_hz', 'runtime_hz', 'observed_hz', 'max_interval_s', 'skipped_slots')}}


class TrialRecorder:
    """Host-side bookkeeping for one trial over one or more hand owners.

    ``hands`` maps side -> an object with record_start(directory, **info), record_stop()
    and status() (WujiClient, or the battery's IPC link). Each owner writes its own
    hand_<side>.npz; this writes meta.json once every side has saved.
    """
    def __init__(self, root):
        self.root = Path(root)
        self.active = None

    def start(self, hands, *, controller=None, skill=None, variant=None, params=None, notes='',
              max_seconds=None, label=None, directory=None):
        if self.active is not None:
            raise RuntimeError(f"trace trial {self.active['trial']} is still recording; stop it first")
        if directory is not None:
            directory = safe_directory(directory)
            trial = directory.name
        else:
            trial = trial_id(label or controller or skill or 'trial')
            directory = safe_directory(self.root/trial)
        started, info = {}, {k: v for k, v in (('controller', controller), ('skill', skill),
                                               ('variant', variant), ('trial', trial)) if v is not None}
        if max_seconds is not None:
            info['max_seconds'] = max_seconds
        try:
            for side, hand in hands.items():
                started[side] = hand.record_start(directory, **info)
        except BaseException:
            for side in started:
                try:
                    hands[side].record_stop()
                except Exception:
                    pass
            raise
        context = {side: hand_context(hand.status()) for side, hand in hands.items()}
        caps = {side: value.get('current_limit_a') for side, value in context.items()}
        cap = (next(iter(caps.values())) if len({json.dumps(v) for v in caps.values()}) == 1 else caps)
        self.active = {'trial': trial, 'directory': directory, 'hands': dict(hands), 'started_at': iso_now(),
                       'meta': {'controller': controller, 'skill': skill, 'variant': variant,
                                'params': params or {}, 'notes': notes or ''},
                       'current_limit_a': cap, 'context': context}
        return {'trial': trial, 'directory': str(directory), 'sides': list(hands), 'recording': started}

    def stop(self, *, success=None, notes=None, **extra):
        if self.active is None:
            raise RuntimeError('no trace trial is recording')
        active, self.active = self.active, None
        saved, errors = {}, {}
        for side, hand in active['hands'].items():
            try:
                saved[side] = hand.record_stop()
            except Exception as exc:  # noqa: BLE001 -- the other side must still be saved.
                errors[side] = str(exc)
        sides = [side for side in active['hands'] if side in saved]
        rates = [value.get('nominal_rate_hz') for value in saved.values() if value.get('nominal_rate_hz')]
        backends = {value.get('backend', 'real') for value in saved.values()} or {'real'}
        meta = active['meta']
        if notes:
            meta['notes'] = (meta['notes'] + '\n' + notes).strip()
        value = write_meta(active['directory'], sides=sides, nominal_rate_hz=max(rates) if rates else None,
                           controller=meta['controller'], skill=meta['skill'], variant=meta['variant'],
                           params=meta['params'], success=success, started_at=active['started_at'],
                           backend='real' if 'real' in backends else sorted(backends)[0], notes=meta['notes'],
                           current_limit_a=active['current_limit_a'],
                           hands={side: {**active['context'].get(side, {}), 'trace': saved.get(side),
                                         'error': errors.get(side)} for side in active['hands']},
                           **extra)
        return {'trial': active['trial'], 'directory': str(active['directory']), 'sides': sides,
                'errors': errors, 'meta': value}

    def status(self):
        if self.active is None:
            return None
        return {'trial': self.active['trial'], 'directory': str(self.active['directory']),
                'sides': list(self.active['hands']), 'started_at': self.active['started_at']}


def load_trial(directory):
    """{'meta': dict, 'hands': {side: {array: ndarray}}, 'arms': {...}} for analysis and tests."""
    import numpy as np
    directory = Path(directory)
    result = {'meta': json.loads((directory/'meta.json').read_text()), 'hands': {}, 'arms': {}}
    for path in sorted(directory.glob('hand_*.npz')):
        with np.load(path) as data:
            result['hands'][path.stem.removeprefix('hand_')] = {key: data[key] for key in data.files}
    for path in sorted(directory.glob('arm_*.npz')):
        with np.load(path) as data:
            result['arms'][path.stem.removeprefix('arm_')] = {key: data[key] for key in data.files}
    return result
