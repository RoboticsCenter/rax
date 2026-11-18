"""The Wuji Hand 2's joint drivers, as the simulator runs them (firmware 2.13.1 behaviour).

Per joint, every 1 ms physics step (MuJoCo evaluates the law inside mj_step through
the hand's `general` actuators, see model.py):

    current = clamp(g * (kp (q_ref - q) + kd (dq_ref - dq) + i_ff + i_fw), -cap, cap)
    torque  = kt * current

- g: the effective gain factor (RAX measured ~1.98 x the documented law);
- kp [A/rad], kd [A s/rad]: `mit_params`; cap [A]: `effort_limit`;
- (q_ref, dq_ref, i_ff): the last `JointCommand` (position, velocity, effort in A),
  applied `command_latency_s` after it was sent;
- i_fw: firmware feed-forward compensation (`set_constant`), zero unless the session
  says the firmware's compensation is effective (spec.HAND_DEFAULTS);
- kt [N m/A]: ASSUMED per joint (spec.PEAK_CURRENT_A).

Joint states: 1 Ready (disabled, no current), 2 Enabled, 3 Stopped (after an emergency
stop). A hardware-faulted joint (spec RIG_FAULTED_JOINTS) never enables and reports
its code. A stall warning (0x0007) is raised while a joint pushes at its cap without
moving; it does not stop output. Joint MCU temperature follows a first-order I^2 model.
"""
from __future__ import annotations

from collections import deque
import math

import numpy as np

from services.wuji.config import JOINT_NAMES

from .spec import ERRORS

READY, ENABLED, STOPPED = 1, 2, 3
POWER_ON_KP, POWER_ON_KD, POWER_ON_LIMIT_A = 5.0, 0.1, 1.5  # firmware 2.12+ reset defaults; vendor examples' 1.5 A


class FirmwareError(RuntimeError):
    pass


def mask_indices(joints):
    """The SDK's mask: None = all joints; else a flat-20 list, nonzero = act on that joint."""
    if joints is None:
        return list(range(20))
    joints = list(joints)
    if len(joints) != 20:
        raise FirmwareError('a joint mask has 20 entries')
    return [i for i, v in enumerate(joints) if v]


class HandFirmware:
    def __init__(self, side, serial, handedness, options, kt, faulted=None, *, clock=None):
        self.side, self.serial, self.handedness = side, serial, handedness
        self.opts = options
        self.kt = np.asarray(kt, dtype=float)
        self.gain = float(options['gain_factor'])
        self.latency = float(options['command_latency_s'])
        self.faulted = {JOINT_NAMES.index(name) if isinstance(name, str) else int(name): int(code)
                        for name, code in (faulted or {}).items()}
        for code in self.faulted.values():
            if code not in ERRORS:
                raise ValueError(f'unsupported simulated fault code {code:#x}')
        self.firmware_version = options['firmware_version']
        self.max_current = float(options['max_current_a'])
        self.coulomb_a = float(options['coulomb_friction_a'])
        self.power_on()

    # ---- power and commands -------------------------------------------------------------------------------------------

    def power_on(self):
        self.kp = np.full(20, POWER_ON_KP)
        self.kd = np.full(20, POWER_ON_KD)
        self.limit = np.full(20, POWER_ON_LIMIT_A)
        self.state = np.full(20, READY, dtype=int)
        self.error = np.zeros(20, dtype=int)
        for index, code in self.faulted.items():
            self.error[index] = code
        self.ff_mode = np.zeros(20, dtype=int)
        self.position_table = np.zeros(20, dtype=bool)
        self.q_ref = None  # until the first command: hold the measured position on enable
        self.dq_ref = np.zeros(20)
        self.i_ff = np.zeros(20)
        self.pending = deque()
        self.current = np.zeros(20)
        self.temperature = np.full(20, float(self.opts['ambient_c']))
        self.stall_time = np.zeros(20)
        self.commands_received = 0
        self.last_command_time = None
        self.dirty = True

    def command(self, t, q, dq, i_ff):
        q, dq, i_ff = (np.asarray(v, dtype=float) for v in (q, dq, i_ff))
        if q.shape != (20,) or dq.shape != (20,) or i_ff.shape != (20,) or not (
                np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(i_ff).all()):
            raise FirmwareError('a Wuji Hand 2 command carries 20 finite joints')
        self.pending.append((t + self.latency, q, dq, i_ff))
        self.commands_received += 1
        self.last_command_time = t

    def _apply_pending(self, t):
        latest = None
        while self.pending and self.pending[0][0] <= t + 1e-12:
            latest = self.pending.popleft()
        if latest is not None:
            _, self.q_ref, self.dq_ref, self.i_ff = latest

    # ---- parameters (SDK resources; readback as written) -----------------------------------------------------------

    def set_effort_limit(self, value):
        values = np.full(20, float(value)) if np.isscalar(value) else np.asarray(value, dtype=float)
        if values.shape != (20,) or not np.isfinite(values).all() or (values < 0).any():
            raise FirmwareError('effort_limit takes one value or 20 nonnegative amperes')
        over = values > self.max_current + 1e-9
        # A joint over its ceiling keeps its previous limit and an error is reported.
        self.limit = np.where(over, self.limit, values)
        self.dirty = True
        if over.any():
            raise FirmwareError(f'effort limit above the {self.max_current:g} A ceiling on joints '
                                f'{[JOINT_NAMES[i] for i in np.flatnonzero(over)]}')

    def set_mit_params(self, pairs):
        pairs = list(pairs)
        if len(pairs) == 2 and all(np.isscalar(v) for v in pairs):
            pairs = [pairs] * 20
        if len(pairs) != 20:
            raise FirmwareError('mit_params takes one (kp, kd) or 20')
        kp = np.array([float(p[0]) for p in pairs])
        kd = np.array([float(p[1]) for p in pairs])
        if not (np.isfinite(kp).all() and np.isfinite(kd).all()) or (kp < 0).any() or (kd < 0).any():
            raise FirmwareError('kp and kd must be finite and nonnegative')
        self.kp, self.kd = kp, kd
        self.dirty = True

    def mit_params(self):
        return [[float(a), float(b)] for a, b in zip(self.kp, self.kd)]

    def set_constant(self, mode, joints=None):
        mode = int(mode)
        if not 0 <= mode <= 3:
            raise FirmwareError('feed-forward mode is 0..3')
        for i in mask_indices(joints):
            self.ff_mode[i] = mode
        self.dirty = True

    def set_position_table(self, on, joints=None):
        for i in mask_indices(joints):
            self.position_table[i] = bool(on)

    # ---- joint state machine --------------------------------------------------------------------------------------------

    def enable(self, joints=None):
        for i in mask_indices(joints):
            if i in self.faulted or ERRORS.get(int(self.error[i]), {}).get('severity') in ('ImmediateStop', 'Fatal'):
                continue  # a faulted joint stays disabled
            self.state[i] = ENABLED
        self.dirty = True

    def disable(self, joints=None):
        for i in mask_indices(joints):
            self.state[i] = READY
        self.dirty = True

    def emergency_stop(self):
        self.state[:] = np.where(self.state == ENABLED, STOPPED, self.state)
        self.pending.clear()
        self.dirty = True

    def clear_fault(self, joints=None):
        for i in mask_indices(joints):
            code = int(self.error[i])
            if code and ERRORS.get(code, {}).get('clear_policy') != 'NonClearable':
                self.error[i] = 0
            if i in self.faulted:  # a persistent hardware fault is reported again at once
                self.error[i] = self.faulted[i]
            self.stall_time[i] = 0.0

    @property
    def enabled(self):
        return self.state == ENABLED

    # ---- the control law ------------------------------------------------------------------------------------------------

    def feedforward(self):
        """Firmware compensation current (A), when the session declares it effective."""
        if not self.opts['firmware_feedforward_effective']:
            return np.zeros(20)
        out = np.zeros(20)
        constant = (self.ff_mode == 1) | (self.ff_mode == 3)
        table = (self.ff_mode == 2) | (self.ff_mode == 3)
        out += np.where(constant, self.coulomb_a * np.sign(self.dq_ref), 0.0)
        out += np.where(table, self.coulomb_a * np.tanh(self.dq_ref / 0.02), 0.0)
        return out

    def write_actuators(self, model, data, actuators, q_measured):
        """Load the law's parameters into the hand's actuators (only after a change) and
        its combined command u = kp q_ref + kd dq_ref + i_ff (+ firmware compensation).
        Before any command has arrived a joint holds where it is (ASSUMED)."""
        on = self.enabled.astype(float)
        q_ref = self.q_ref if self.q_ref is not None else np.asarray(q_measured, dtype=float)
        if self.dirty:
            g = self.gain * self.kt * on
            model.actuator_gainprm[actuators, 0] = g
            model.actuator_biasprm[actuators, 1] = -g * self.kp
            model.actuator_biasprm[actuators, 2] = -g * self.kd
            cap = self.kt * self.limit * on
            model.actuator_forcerange[actuators, 0] = -cap
            model.actuator_forcerange[actuators, 1] = cap
            self.dirty = False
        data.ctrl[actuators] = (self.kp * q_ref + self.kd * self.dq_ref + self.i_ff + self.feedforward()) * on

    def control(self, t, model, data, actuators, q_measured):
        self._apply_pending(t)
        self.write_actuators(model, data, actuators, q_measured)

    def after_step(self, data, actuators, dq, dt):
        """Read the motor currents the law produced; stall and heat bookkeeping."""
        self.current = np.where(self.enabled, data.actuator_force[actuators] / self.kt, 0.0)
        if self.opts['stall']:
            pushing = self.enabled & (np.abs(self.current) >= 0.98 * self.limit) & (self.limit > 0) & (np.abs(dq) < 0.05)
            self.stall_time = np.where(pushing, self.stall_time + dt, 0.0)
            stalled = self.stall_time >= float(self.opts['stall_time_s'])
            for i in np.flatnonzero(stalled):
                if self.error[i] == 0:
                    self.error[i] = 7
            for i in np.flatnonzero(~stalled):
                if self.error[i] == 7:
                    self.error[i] = 0  # AutoClear
        if self.opts['thermal']:
            tau = float(self.opts['thermal_tau_s'])
            ambient = float(self.opts['ambient_c'])
            rise = float(self.opts['thermal_rise_c_per_a2']) * self.current ** 2
            self.temperature += (ambient + rise - self.temperature) * (dt / tau)

    # ---- reports ---------------------------------------------------------------------------------------------------------

    def diagnostics(self):
        limited = self.enabled & (np.abs(self.current) >= 0.98 * self.limit) & (self.limit > 0)
        return {'states': self.state.copy(), 'error_code_current': self.error.copy(),
                'current': self.current.copy(), 'vbus_v_fb': np.full(20, float(self.opts['vbus_v'])),
                'mcu_temp_c_fb': self.temperature.copy(), 'current_limit_active': limited}

    def info(self):
        return {'serial_number': self.serial, 'firmware_version': self.firmware_version,
                'handedness': self.handedness.capitalize()}

    def snapshot(self):
        """Privileged firmware internals for traces and tests (never exposed to agents)."""
        return {'q_ref': None if self.q_ref is None else self.q_ref.copy(), 'dq_ref': self.dq_ref.copy(),
                'i_ff': self.i_ff.copy(), 'kp': self.kp.copy(), 'kd': self.kd.copy(), 'limit': self.limit.copy(),
                'state': self.state.copy(), 'current': self.current.copy(), 'gain_factor': self.gain,
                'kt': self.kt.copy(), 'ff_mode': self.ff_mode.copy()}


def stribeck(dq, static_a, coulomb_a, v_s):
    """Friction magnitude (A) at joint speed dq: static at rest, Coulomb when sliding."""
    return coulomb_a + (static_a - coulomb_a) * np.exp(-(np.asarray(dq) / max(v_s, 1e-9)) ** 2)


def describe_error(code):
    detail = ERRORS.get(int(code))
    if detail is None:
        return None
    return {'code': int(code), **detail, 'is_test': False, 'desc': f"simulated {detail['name']}",
            'cause': 'simulation', 'resolution': 'reset the simulated device'}


def nid_of(index):
    """Firmware stream NID of flat joint index (bus * 5 + node, nodes 1-4)."""
    return (int(index) // 4) * 5 + int(index) % 4 + 1


def index_of_nid(nid):
    bus, node = divmod(int(nid), 5)
    if not 1 <= node <= 4 or not 0 <= bus <= 4:
        return None
    return bus * 4 + node - 1


assert all(index_of_nid(nid_of(i)) == i for i in range(20))
assert math.isfinite(POWER_ON_KP)
