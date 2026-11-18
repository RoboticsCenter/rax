"""Hardware-free bimanual xArm 6 + Wuji Hand 2 rig for the evaluation runner and its tests.

It implements a small, schema-compatible subset of the bimanual MCP tools
(wuji_status / wuji_move_joints / wuji_home / wuji_hold with a ``hand`` selector,
xarm_status / xarm_move_joints / xarm_hold) plus ``trace_start`` / ``trace_stop``, which
write the shared trace format the way a robot service would. Time is simulated: a
move advances a virtual monotonic clock and returns at once, so nothing sleeps.

Plant per hand joint: a damped second-order tracker of the commanded reference,
followed by breakaway friction (the joint sticks until the tracker leads it by
``break_rad``, then slips over ``slip_s``) and Gaussian sensor noise. Slow references
therefore produce stick-slip, as observed on the real hands. Backend options:

    sides            ["left", "right"]
    nominal_rate_hz  200
    disabled_joints  {"left": ["thumb_S3", "middle_S1"]}
    omega, zeta      40 rad/s, 0.9
    break_rad        0.008   (0 disables breakaway; about the 0.013 rad slips seen on the real hands)
    slip_s           0.008
    noise_rad        1e-4
    current_limit_a  0.8
    max_velocity_rad_s 0.6   (default move duration)
    trace_tools      true    (expose trace_start/trace_stop)
    seed             0
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import threading

import numpy as np

from .synthetic import min_jerk
from .trace import JOINT_NAMES, N_ARM, N_HAND, default_meta, save_arm, save_hand

SETTLE_S = 0.3


def _schema(properties=None, required=()):
    return {'type': 'object', 'properties': properties or {}, 'required': list(required),
            'additionalProperties': False}


class _HandPlant:
    def __init__(self, fs, options, disabled, rng):
        self.fs = fs
        self.omega = float(options.get('omega', 40.0))
        self.zeta = float(options.get('zeta', 0.9))
        self.break_rad = float(options.get('break_rad', 0.008))
        self.n_slip = max(1, int(round(float(options.get('slip_s', 0.008)) * fs)))
        self.noise = float(options.get('noise_rad', 1e-4))
        self.cap = float(options.get('current_limit_a', 0.8))
        self.kp, self.kd = float(options.get('kp', 8.0)), float(options.get('kd', 0.15))
        self.rng = rng
        self.disabled = np.zeros(N_HAND, dtype=bool)
        for name in disabled:
            self.disabled[JOINT_NAMES.index(name)] = True
        self.ref = np.zeros(N_HAND)
        self.target = np.zeros(N_HAND)
        self.x = np.zeros(N_HAND)        # second-order tracker state
        self.v = np.zeros(N_HAND)
        self.q = np.zeros(N_HAND)        # output after breakaway friction
        self.dq = np.zeros(N_HAND)
        self.remaining = np.zeros(N_HAND, dtype=int)
        self.slip_to = np.zeros(N_HAND)

    def step(self, reference):
        self.ref = np.asarray(reference, dtype=float)
        dt = 1.0 / self.fs / 4
        for _ in range(4):
            a = self.omega ** 2 * (self.ref - self.x) - 2 * self.zeta * self.omega * self.v
            self.v = self.v + a * dt
            self.x = self.x + self.v * dt
        previous = self.q.copy()
        if self.break_rad > 0:
            sliding = self.remaining > 0
            self.q[sliding] += (self.slip_to[sliding] - self.q[sliding]) / self.remaining[sliding]
            self.remaining[sliding] -= 1
            start = (self.remaining == 0) & ~sliding & (np.abs(self.x - self.q) > self.break_rad)
            self.slip_to[start] = self.x[start]
            self.remaining[start] = self.n_slip
        else:
            self.q = self.x.copy()
        self.dq = (self.q - previous) * self.fs

    def row(self):
        q_meas = self.q + self.rng.normal(0.0, self.noise, N_HAND)
        current = np.clip(2 * self.kp * (self.ref - self.q) - 2 * self.kd * self.dq, -self.cap, self.cap)
        measured = [q_meas, self.dq.copy(), current]
        for array in measured:
            array[self.disabled] = np.nan
        return (self.ref.copy(), self.target.copy(), *measured)


class MockRigBackend:
    mutating = frozenset({'wuji_move_joints', 'wuji_home', 'xarm_move_joints'})

    def __init__(self, config):
        self.config = config
        options = dict(getattr(config, 'backend_options', {}) or {})
        self.options = options
        self.sides = list(options.get('sides', ['left', 'right']))
        self.fs = float(options.get('nominal_rate_hz', 200.0))
        self.vmax = float(options.get('max_velocity_rad_s', 0.6))
        rng = np.random.default_rng(int(options.get('seed', 0)))
        disabled = options.get('disabled_joints', {'left': ['thumb_S3', 'middle_S1']})
        self.hands = {side: _HandPlant(self.fs, options, disabled.get(side, []), rng) for side in self.sides}
        self.arms = {side: {'ref': np.zeros(N_ARM), 'q': np.zeros(N_ARM), 'v': np.zeros(N_ARM)}
                     for side in self.sides}
        self.clock = 1000.0
        self.lock = threading.RLock()
        self.recording = None
        self.calls = []
        self.closed = False

    # ------------------------------------------------------------------ MCP surface
    def tools(self):
        hand = {'type': 'string', 'enum': self.sides}
        positions = {'type': 'array', 'items': {'type': 'number'}, 'minItems': 20, 'maxItems': 20}
        tools = [
            {'name': 'wuji_status', 'description': 'Mock hand status.', 'inputSchema': _schema({'hand': hand}, ['hand'])},
            {'name': 'wuji_move_joints', 'description': 'Mock min-jerk move of 20 joints (rad).',
             'inputSchema': _schema({'hand': hand, 'positions': positions,
                                     'duration_s': {'type': 'number', 'minimum': 0.05, 'maximum': 60}},
                                    ['hand', 'positions'])},
            {'name': 'wuji_home', 'description': 'Mock move to zeros.', 'inputSchema': _schema({'hand': hand}, ['hand'])},
            {'name': 'wuji_hold', 'description': 'Mock hold.', 'inputSchema': _schema({'hand': hand}, ['hand'])},
            {'name': 'xarm_status', 'description': 'Mock rig status.', 'inputSchema': _schema()},
            {'name': 'xarm_move_joints', 'description': 'Mock min-jerk arm joint move (rad).',
             'inputSchema': _schema({'arm': hand, 'joints': {'type': 'array', 'items': {'type': 'number'},
                                                             'minItems': 6, 'maxItems': 6},
                                     'duration_s': {'type': 'number', 'minimum': 0.05, 'maximum': 60}},
                                    ['arm', 'joints'])},
            {'name': 'xarm_hold', 'description': 'Mock stop of every device.', 'inputSchema': _schema()},
        ]
        if self.options.get('trace_tools', True):
            tools += [
                {'name': 'trace_start', 'description': 'Start recording the shared trace format.',
                 'inputSchema': _schema({'directory': {'type': 'string'}, 'label': {'type': 'string'}})},
                {'name': 'trace_stop', 'description': 'Stop recording and write the trace directory.',
                 'inputSchema': _schema()},
            ]
        return tools

    def validate(self, name, arguments):
        if name == 'wuji_move_joints':
            positions = arguments.get('positions')
            if not isinstance(positions, list) or len(positions) != 20:
                raise ValueError('positions must contain exactly 20 numbers')
            if any(abs(float(p)) > 2.1 for p in positions):
                raise ValueError('positions outside the mock joint range')
        if name in ('wuji_move_joints', 'wuji_home', 'wuji_hold', 'wuji_status') and arguments.get('hand') not in self.hands:
            raise ValueError(f'hand must be one of {self.sides}')

    def call(self, name, arguments):
        with self.lock:
            self.calls.append(name)
            return getattr(self, '_' + name)(**arguments), []

    def status(self):
        with self.lock:
            return {'robot': 'mock_rig', 'clock': self.clock, 'recording': self.recording is not None}

    def close(self):
        self.closed = True

    # ------------------------------------------------------------------ simulation
    def _tick(self, hand_refs=None, arm_refs=None):
        """Advance one period; hand_refs/arm_refs: {side: reference} (others hold)."""
        self.clock += 1.0 / self.fs
        for side, plant in self.hands.items():
            plant.step((hand_refs or {}).get(side, plant.ref))
        for side, arm in self.arms.items():
            ref = (arm_refs or {}).get(side, arm['ref'])
            arm['ref'] = np.asarray(ref, dtype=float)
            # First-order lag (time constant 30 ms) for the arm.
            arm['q'] = arm['q'] + (arm['ref'] - arm['q']) * min(1.0, 1.0 / (0.03 * self.fs))
        if self.recording is not None:
            for side, plant in self.hands.items():
                self.recording['hands'][side].append((self.clock, *plant.row()))
            for side, arm in self.arms.items():
                self.recording['arms'][side].append((self.clock, arm['ref'].copy(), arm['q'].copy()))

    def _duration(self, delta, duration_s):
        if duration_s:
            return float(duration_s)
        return max(0.3, 1.875 * float(np.max(np.abs(delta))) / self.vmax)

    def _move_hand(self, side, goal, duration_s=None):
        plant = self.hands[side]
        goal = np.asarray(goal, dtype=float)
        start = plant.ref.copy()
        plant.target = goal.copy()
        duration = self._duration(goal - start, duration_s)
        path = min_jerk(start, goal, max(2, int(round(duration * self.fs))) + 1)[1:]
        for row in path:
            self._tick(hand_refs={side: row})
        for _ in range(int(round(SETTLE_S * self.fs))):
            self._tick()
        active = ~plant.disabled
        error = float(np.max(np.abs(plant.q[active] - goal[active]))) if active.any() else 0.0
        return {'hand': side, 'outcome': 'reached', 'reached': error <= 0.04, 'duration_s': duration,
                'max_error_rad': error}

    def _wuji_status(self, hand):
        plant = self.hands[hand]
        q = plant.q.copy()
        positions = [None if plant.disabled[j] else float(q[j]) for j in range(N_HAND)]
        return {'hand': hand, 'fresh': True, 'enabled': True, 'positions': positions,
                'target_positions': plant.ref.tolist(),
                'velocity_rad_s': [None if plant.disabled[j] else float(plant.dq[j]) for j in range(N_HAND)],
                'effort_a': [0.0] * N_HAND, 'command_effort_a': [0.0] * N_HAND,
                'control_parameters': {'kp': [plant.kp] * N_HAND, 'kd': [plant.kd] * N_HAND,
                                       'current_limit_a': [plant.cap] * N_HAND},
                'joint_names': list(JOINT_NAMES),
                'disabled_joints': [JOINT_NAMES[j] for j in np.flatnonzero(plant.disabled)],
                'motion': None, 'clock': self.clock}

    def _wuji_move_joints(self, hand, positions, duration_s=None):
        return self._move_hand(hand, positions, duration_s)

    def _wuji_home(self, hand):
        return self._move_hand(hand, np.zeros(N_HAND))

    def _wuji_hold(self, hand):
        for _ in range(int(round(0.05 * self.fs))):
            self._tick()
        return {'hand': hand, 'outcome': 'held'}

    def _xarm_status(self):
        return {'arms': {s: {'joints': a['q'].tolist()} for s, a in self.arms.items()},
                'hands': {s: self._wuji_status(s) for s in self.hands}, 'motion': {'armed': True}}

    def _xarm_move_joints(self, arm, joints, duration_s=None):
        state = self.arms[arm]
        goal = np.asarray(joints, dtype=float)
        duration = self._duration(goal - state['ref'], duration_s)
        for row in min_jerk(state['ref'], goal, max(2, int(round(duration * self.fs))) + 1)[1:]:
            self._tick(arm_refs={arm: row})
        for _ in range(int(round(SETTLE_S * self.fs))):
            self._tick()
        return {'arm': arm, 'outcome': 'reached', 'joints': state['q'].tolist()}

    def _xarm_hold(self):
        return {'outcome': 'held', 'control_certain': True}

    def _trace_start(self, directory=None, label=None):
        if self.recording is not None:
            raise RuntimeError('a trace is already recording')
        root = Path(directory) if directory else Path(self.config.root) / 'traces' / (label or f'{self.clock:.3f}')
        self.recording = {'directory': root, 'hands': {s: [] for s in self.hands}, 'arms': {s: [] for s in self.arms},
                          'started_at': datetime.now(timezone.utc).isoformat()}
        for _ in range(int(round(0.2 * self.fs))):  # Lead-in hold.
            self._tick()
        return {'recording': True, 'directory': str(root)}

    def _trace_stop(self):
        if self.recording is None:
            raise RuntimeError('no trace is recording')
        for _ in range(int(round(0.2 * self.fs))):  # Lead-out hold.
            self._tick()
        recording, self.recording = self.recording, None
        root = recording['directory']
        root.mkdir(parents=True, exist_ok=True)
        for side, rows in recording['hands'].items():
            t, q_ref, q_target, q_meas, dq_meas, current = (np.asarray(x) for x in zip(*rows))
            plant = self.hands[side]
            T = t.shape[0]
            save_hand(root / f'hand_{side}.npz', t=t, q_ref=q_ref, q_target=q_target, q_meas=q_meas,
                      dq_meas=dq_meas, current=current, i_ff=np.zeros((T, N_HAND)),
                      kp=np.full((T, N_HAND), plant.kp), kd=np.full((T, N_HAND), plant.kd))
        for side, rows in recording['arms'].items():
            t, q_ref, q_meas = (np.asarray(x) for x in zip(*rows))
            if np.ptp(q_ref, axis=0).max() > 0:  # Only arms that moved.
                save_arm(root / f'arm_{side}.npz', t=t, q_ref=q_ref, q_meas=q_meas)
        meta = default_meta(sides=sorted(self.hands), nominal_rate_hz=self.fs, backend='mock',
                            controller=self.options.get('controller', 'mock-second-order-breakaway'),
                            started_at=recording['started_at'],
                            current_limit_a=float(self.options.get('current_limit_a', 0.8)),
                            trace_source='service:mock_rig')
        (root / 'meta.json').write_text(json.dumps(meta, indent=2) + '\n')
        return {'recording': False, 'directory': str(root),
                'samples': {s: len(r) for s, r in recording['hands'].items()}}
