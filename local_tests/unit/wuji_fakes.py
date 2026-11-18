"""Hardware-free hand feedback source. Targets take effect on the next report."""
import time
from copy import deepcopy

from services.wuji.config import validate_config


def hand_config(**overrides):
    return validate_config({'type': 'wuji_hand2', 'serial': 'FAKE-HAND-ONLY', 'handedness': 'right',
                            'profile_revision': 'test-v1', 'lower_rad': [-1.]*20, 'upper_rad': [1.]*20,
                            'current_limit_a': .5, 'kp': 1.5, 'kd': .1,
                            'postures': {'test_pose': [.1]*20}, **overrides})


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class FakeHand:
    backend_kind = 'mock'

    def __init__(self, cfg, clock=time.monotonic, continuous=False):
        self.cfg, self.clock, self.continuous = cfg, clock, continuous
        self.connected = False
        self.q, self.goal = [0.]*20, [0.]*20
        self.velocity = [0.]*20
        self.command_velocity = [0.]*20
        self.command_effort = [0.]*20
        self.effort_writes = []
        self.control_parameters = {k: list(cfg[k]) for k in ('kp', 'kd', 'current_limit_a')}
        self.states, self.names = [1]*20, ['Ready']*20
        self.writes = []
        self.follow = True
        self.faults = []
        self.received = {'state': clock(), 'diagnostics': clock()}
        self.on_send = None
        self.stop_error = None
        self.profile = None

    def connect(self):
        self.connected = True
        self.refresh()

    def refresh(self, state=True, diagnostics=True):
        if state:
            if self.follow:
                self.q = list(self.goal)
            self.received['state'] = self.clock()
        if diagnostics:
            self.received['diagnostics'] = self.clock()

    def snapshot(self):
        if self.continuous:
            self.refresh()
        ages = {k: self.clock()-v for k, v in self.received.items()}
        fresh = self.connected and max(ages.values()) <= self.cfg['max_feedback_age_s']
        return {'connected': self.connected, 'fresh': fresh, 'enabled': fresh and self.states == [2]*20,
                'positions': list(self.q), 'velocity_rad_s': list(self.velocity), 'effort_a': [0.]*20,
                'serial': self.cfg['serial'], 'handedness': self.cfg['handedness'],
                'diagnostics': {'states': list(self.states), 'state_names': list(self.names)},
                'faults': deepcopy(self.faults), 'age_s': ages, 'received_monotonic': dict(self.received)}

    def apply_profile(self):
        self.writes.append(('profile',))
        self.profile = self.cfg['profile_revision']
        self.control_parameters = {k: list(self.cfg[k]) for k in ('kp', 'kd', 'current_limit_a')}

    def set_current_limits(self, values):
        self.writes.append(('current_limits', list(values)))
        self.control_parameters['current_limit_a'] = list(values)

    def set_impedance(self, kp, kd):
        self.writes.append(('impedance', list(kp), list(kd)))
        self.control_parameters.update(kp=list(kp), kd=list(kd))

    def set_control_parameters(self, values):
        self.writes.append(('control_parameters', deepcopy(values)))
        self.control_parameters = {k: list(values[k]) for k in ('kp', 'kd', 'current_limit_a')}

    def enable(self):
        self.writes.append(('enable',))
        self.states, self.names = [2]*20, ['Enabled']*20

    def send(self, q, velocity=None, effort_a=None):
        self.writes.append(('send', list(q)))
        self.goal = list(q)
        self.command_velocity = [0.]*20 if velocity is None else list(velocity)
        self.command_effort = [0.]*20 if effort_a is None else list(effort_a)
        self.effort_writes.append((self.clock(), list(self.command_effort)))
        if self.on_send:
            self.on_send()

    def stop(self, behavior='emergency'):
        self.writes.append((behavior,))
        if self.stop_error:
            raise self.stop_error
        self.states, self.names = [1]*20, ['Ready']*20
        self.command_effort = [0.]*20

    def close(self):
        self.connected = False


class FakeHandClient:
    def __init__(self, config=None):
        self.cfg = config or hand_config()
        self.q = self.target = [0.]*20
        self.moves, self.stops, self.closed = [], 0, False
        self.fail_move = self.fail_stop = False
        self.command_effort = [0.]*20
        self.control_parameters = {k: list(self.cfg[k]) for k in ('kp', 'kd', 'current_limit_a')}
        self.control_calls = []

    def status(self):
        return {'fresh': True, 'enabled': True, 'mode': 'holding', 'positions': list(self.q),
                'target_positions': list(self.target), 'profile_revision': self.cfg['profile_revision'],
                'command_effort_a': list(self.command_effort), 'velocity_rad_s': [0.]*20,
                'control_parameters': deepcopy(self.control_parameters)}

    def require_ready(self):
        return self.status()

    def move(self, positions, duration_s=None, completion='position'):
        self.moves.append(list(positions))
        if self.fail_move:
            raise RuntimeError('injected hand transport loss')
        self.q = self.target = list(positions)
        return {'outcome': 'reached', 'reached': True, 'positions': self.q, 'max_joint_error_rad': 0.}

    def hold(self):
        return {'outcome': 'held', 'target_positions': self.target, 'confirmed': False}

    def set_effort(self, effort_a, duration_s=None):
        self.control_calls.append(('set_effort', list(effort_a)))
        self.command_effort = list(effort_a)
        return {'outcome': 'effort_set', 'command_effort_a': list(effort_a), 'contact_force_verified': False}

    def set_impedance(self, **parameters):
        self.control_calls.append(('set_impedance', deepcopy(parameters)))
        self.control_parameters.update(deepcopy(parameters))
        return {'outcome': 'impedance_set', 'control_parameters': deepcopy(self.control_parameters)}

    def stop(self):
        self.stops += 1
        if self.fail_stop:
            raise RuntimeError('hand stop failed')
        return {'requested': 'emergency', 'sdk_returned': True, 'confirmed': True}

    # -- smooth layer: records calls; applies targets immediately (no dynamics).
    def stream_check(self, op, args):
        from services.wuji.smooth import parse_stream, parse_trajectory
        parse = parse_stream if op == 'stream' else parse_trajectory
        command = parse(dict(args), self.cfg, list(self.target))
        return {'valid': True, 'target_positions': command['target']}

    def stream(self, **args):
        self.control_calls.append(('stream', deepcopy(args)))
        from services.wuji.smooth import parse_stream
        if self.fail_move:
            raise RuntimeError('injected hand transport loss')
        self.q = self.target = parse_stream(dict(args), self.cfg, list(self.target))['target']
        return {'accepted': True, 'sequence': len(self.control_calls), 'target_positions': list(self.target)}

    def trajectory(self, *, waypoints, wait=True, timeout_s=None):
        self.control_calls.append(('trajectory', deepcopy(waypoints), wait))
        from services.wuji.smooth import parse_trajectory
        self.q = self.target = parse_trajectory({'waypoints': waypoints}, self.cfg, list(self.target))['target']
        return {'accepted': True, 'waited': wait, 'settled': wait, 'outcome': 'settled' if wait else None,
                'target_positions': list(self.target)}

    def stream_config(self, **settings):
        self.control_calls.append(('stream_config', deepcopy(settings)))
        return {'settings': settings, 'applies': 'next_tick'}

    def set_profile(self, **args):
        self.control_calls.append(('set_profile', deepcopy(args)))
        return {'outcome': 'profile_set', 'profile': {'name': args.get('name')}, 'readback_verified': True}

    def wait(self, timeout_s=None, after=None):
        return {'outcome': 'settled', 'settled': True, 'positions': list(self.q),
                'target_positions': list(self.target), 'max_joint_error_rad': 0.}

    def stream_status(self):
        return {'mode': 'holding', 'stream': {'accepted': 0, 'processed': 0}}

    def record_start(self, directory, **info):
        self.control_calls.append(('record_start', str(directory), deepcopy(info)))
        self.recording = str(directory)
        return {'directory': str(directory), 'side': self.cfg['handedness'], 'samples': 0}

    def record_stop(self):
        import numpy as np
        from pathlib import Path
        directory = Path(self.recording)
        directory.mkdir(parents=True, exist_ok=True)
        arrays = {'t': np.arange(3, dtype=float)}
        for key in ('q_ref', 'q_target', 'q_meas', 'dq_meas', 'current', 'i_ff', 'kp', 'kd'):
            arrays[key] = np.zeros((3, 20))
        np.savez(directory/f"hand_{self.cfg['handedness']}.npz", **arrays)
        self.control_calls.append(('record_stop',))
        return {'saved': {'samples': 3}, 'nominal_rate_hz': self.cfg['command_hz'], 'backend': 'mock'}

    def record_status(self):
        return None

    def close(self):
        self.closed = True
