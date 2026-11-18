"""Per-trial trace recorders.

* ``ServiceRecorder``: the robot service writes the trace. It is used when the ctx
  lists a start/stop tool pair (``trace_start``/``trace_stop`` or one of
  ``CANDIDATES``); the trial directory is passed when the start tool accepts one, and
  files written elsewhere are copied in.
* ``PollingRecorder``: fallback from status telemetry (``wuji_status``-shaped dicts) at a
  fixed poll rate. Coarser than a service trace: timing metrics then describe polling,
  not commands, and fast slips are under-resolved. meta.trace_source records "poll".
* ``AudioRecorder``: optional 48 kHz microphone capture with ``arecord`` (ALSA).

A recorder's ``start(trial_dir, info)`` begins capture; ``stop()`` returns
``{'source': str, 'meta': partial meta dict, 'error': str|None}``.
"""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time

import numpy as np

from .trace import N_HAND, SIDES, save_hand

CANDIDATES = (('trace_start', 'trace_stop'), ('wuji_trace_start', 'wuji_trace_stop'),
              ('dex_trace_start', 'dex_trace_stop'), ('record_start', 'record_stop'),
              ('wuji_record_start', 'wuji_record_stop'), ('trace_record_start', 'trace_record_stop'))
DIRECTORY_ARGS = ('directory', 'trace_dir', 'out_dir', 'path', 'output_dir')
LABEL_ARGS = ('label', 'trial_id', 'name')
TRACE_FILES = ('hand_left.npz', 'hand_right.npz', 'arm_left.npz', 'arm_right.npz', 'audio.wav')


def list_tools(ctx):
    try:
        return {t['name']: t for t in ctx.request('tools/list', {}).get('tools', [])}
    except Exception:
        return {}


class ServiceRecorder:
    def __init__(self, ctx, start_tool, stop_tool, tools=None):
        self.ctx, self.start_tool, self.stop_tool = ctx, start_tool, stop_tool
        tools = tools or list_tools(ctx)
        self.start_args = set(tools.get(start_tool, {}).get('inputSchema', {}).get('properties', {}))
        self.stop_args = set(tools.get(stop_tool, {}).get('inputSchema', {}).get('properties', {}))
        self.trial_dir = None
        self.started = False

    @classmethod
    def detect(cls, ctx, pair=None):
        """A recorder for the first available start/stop pair, or None."""
        tools = list_tools(ctx)
        for start, stop in ([tuple(pair)] if pair else CANDIDATES):
            if start in tools and stop in tools:
                return cls(ctx, start, stop, tools)
        return None

    def _arguments(self, accepted, info):
        args = {}
        for key in DIRECTORY_ARGS:
            if key in accepted:
                args[key] = str(self.trial_dir)
                break
        for key in LABEL_ARGS:
            if key in accepted:
                args[key] = str(info.get('trial_id', ''))
                break
        if 'sides' in accepted and info.get('sides'):
            args['sides'] = list(info['sides'])
        return args

    def start(self, trial_dir, info):
        self.trial_dir = Path(trial_dir)
        self.started_value = self.ctx.value(self.start_tool, **self._arguments(self.start_args, info))
        self.started = True

    def stop(self):
        if not self.started:
            return {'source': f'service:{self.start_tool}', 'meta': {}, 'error': 'not started'}
        self.started = False
        try:
            value = self.ctx.value(self.stop_tool, **self._arguments(self.stop_args, {}))
        except Exception as exc:
            return {'source': f'service:{self.start_tool}', 'meta': {}, 'error': f'{type(exc).__name__}: {exc}'}
        reported = None
        for source in (value, getattr(self, 'started_value', None)):
            if isinstance(source, dict):
                reported = next((source[k] for k in DIRECTORY_ARGS if isinstance(source.get(k), str)), None)
                if reported:
                    break
        meta = {}
        if reported and Path(reported).resolve() != self.trial_dir.resolve():
            source_dir = Path(reported)
            for name in [*TRACE_FILES, *(p.name for p in source_dir.glob('video_*.mp4'))]:
                if (source_dir / name).is_file():
                    shutil.copy2(source_dir / name, self.trial_dir / name)
            if (source_dir / 'meta.json').is_file():
                meta = json.loads((source_dir / 'meta.json').read_text())
        elif (self.trial_dir / 'meta.json').is_file():
            meta = json.loads((self.trial_dir / 'meta.json').read_text())
        meta.setdefault('trace_source', f'service:{self.start_tool}')
        return {'source': f'service:{self.start_tool}', 'meta': meta, 'error': None, 'service_result': value}


def _vector(value, default=np.nan):
    """20-vector from a status field (list with None, scalar, or missing)."""
    if value is None:
        return np.full(N_HAND, default)
    if isinstance(value, (int, float)):
        return np.full(N_HAND, float(value))
    out = np.array([default if v is None else float(v) for v in value], dtype=float)
    return out if out.shape == (N_HAND,) else np.full(N_HAND, default)


def status_row(status):
    """(q_ref, q_target, q_meas, dq_meas, current, i_ff, kp, kd, temperature) from a hand status."""
    motion = status.get('motion') or {}
    q_ref = _vector(status.get('target_positions'))
    q_target = _vector(motion.get('requested_positions')) if motion.get('requested_positions') else q_ref
    parameters = status.get('control_parameters') or {}
    diagnostics = status.get('diagnostics') or {}
    temperature = diagnostics.get('mcu_temp_c_fb') if isinstance(diagnostics, dict) else None
    return (q_ref, q_target, _vector(status.get('positions')), _vector(status.get('velocity_rad_s')),
            _vector(status.get('effort_a')), _vector(status.get('command_effort_a'), 0.0),
            _vector(parameters.get('kp'), 0.0), _vector(parameters.get('kd'), 0.0), _vector(temperature))


class PollingRecorder:
    def __init__(self, status_fn, sides, rate_hz=100.0, clock=time.monotonic):
        self.status_fn, self.sides, self.rate_hz, self.clock = status_fn, list(sides), float(rate_hz), clock
        self.rows = {side: [] for side in self.sides}
        self.errors = []
        self._stop = threading.Event()
        self._thread = None

    def _loop(self):
        period = 1.0 / self.rate_hz
        deadline = self.clock()
        while not self._stop.is_set():
            for side in self.sides:
                try:
                    status = self.status_fn(side)
                    self.rows[side].append((self.clock(), *status_row(status)))
                except Exception as exc:
                    if len(self.errors) < 20:
                        self.errors.append(f'{side}: {type(exc).__name__}: {exc}')
            deadline += period
            self._stop.wait(max(0.0, deadline - self.clock()))

    def start(self, trial_dir, info):
        self.trial_dir = Path(trial_dir)
        self.rows = {side: [] for side in self.sides}
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name='dex-eval-poll', daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        written = []
        for side, rows in self.rows.items():
            if len(rows) < 2:
                continue
            t, q_ref, q_target, q_meas, dq, current, i_ff, kp, kd, temperature = (np.asarray(x) for x in zip(*rows))
            extra = {'temperature_c': temperature} if np.isfinite(temperature).any() else {}
            save_hand(self.trial_dir / f'hand_{side}.npz', t=t.astype(float), q_ref=q_ref, q_target=q_target,
                      q_meas=q_meas, dq_meas=dq, current=current, i_ff=i_ff, kp=kp, kd=kd, **extra)
            written.append(side)
        return {'source': 'poll', 'error': '; '.join(self.errors) or None,
                'meta': {'sides': written, 'nominal_rate_hz': self.rate_hz, 'trace_source': 'poll'}}


def hand_service_status(session_root, bimanual=True):
    """status_fn reading a session's private hand-service sockets (read-only 'status' op)."""
    from services.wuji.client import request, socket_path

    def status(side):
        return request(socket_path(session_root, side if bimanual else None), 'status', timeout=0.5)
    return status


def backend_status(backend):
    """status_fn calling a backend's wuji_status directly (bypasses the frontend's dispatch lock,
    so polling continues while a skill's blocking move runs). The backend must be thread-safe."""
    def status(side):
        value, _ = backend.call('wuji_status', {'hand': side})
        return value
    return status


class AudioRecorder:
    """48 kHz mono capture with arecord into <trial>/audio.wav; start time on the monotonic clock."""

    def __init__(self, device, rate=48000, channels=1):
        self.device, self.rate, self.channels = device, int(rate), int(channels)
        self.process = None

    def start(self, trial_dir, info):
        if shutil.which('arecord') is None:
            raise RuntimeError('arecord (alsa-utils) is required for --audio-device')
        self.path = Path(trial_dir) / 'audio.wav'
        self.process = subprocess.Popen(['arecord', '-q', '-D', self.device, '-f', 'S16_LE', '-r', str(self.rate),
                                         '-c', str(self.channels), str(self.path)],
                                        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.start_t = time.monotonic()

    def stop(self):
        if self.process is None:
            return {'source': 'arecord', 'meta': {}, 'error': 'not started'}
        self.process.send_signal(signal.SIGINT)
        try:
            _, err = self.process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            _, err = self.process.communicate()
        error = err.decode(errors='replace').strip() if self.process.returncode not in (0, 1, -2) else None
        meta = {'audio': {'start_t': self.start_t, 'device': self.device, 'rate': self.rate,
                          'source': 'arecord', 'note': 'start_t is the monotonic time at process start; '
                                                       'refine with offset_s from a sync clap'}}
        return {'source': 'arecord', 'meta': meta, 'error': error}


__all__ = ['ServiceRecorder', 'PollingRecorder', 'AudioRecorder', 'hand_service_status', 'backend_status',
           'status_row', 'CANDIDATES', 'SIDES']
