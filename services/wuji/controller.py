"""Single-writer hand controller and private operator/backend IPC service.

The socket thread never calls the SDK. Stop increments an epoch immediately, so a
late native call cannot be followed by another target from the cancelled action.
"""
import argparse
import json
import math
import os
import signal
import socketserver
import threading
import time
from collections import OrderedDict, deque
from copy import deepcopy
from pathlib import Path

from services.xarm import arming

from .config import JOINT_NAMES, completion_mode, effort, impedance, number, target, validate_config, validate_tool
from .guards import EffortVelocityGuard
from .motion import FeedbackPath, SettlingWindow, smooth_minimum_duration
from .scurve import SCurvePath, next_deadline, require_ruckig
from .sdk import WujiDriver


class Cancelled(RuntimeError):
    pass


class Trajectory:
    def __init__(self, start, goal, cfg, duration=None):
        self.start, self.goal = target(start, cfg), target(goal, cfg)
        self.delta = [b-a for a, b in zip(self.start, self.goal)]
        distance = max(map(abs, self.delta))
        if distance > cfg['max_step_rad']:
            raise ValueError(f'hand move exceeds max_step_rad={cfg["max_step_rad"]}')
        minimum = max(.1, 1.875*distance/cfg['max_velocity_rad_s'],
                      math.sqrt(5.773503*distance/cfg['max_acceleration_rad_s2']))
        if cfg['motion_mode'] == 'feedback':
            minimum = max(minimum, smooth_minimum_duration(distance, cfg))
        if duration is not None:
            duration = number(duration, 'duration_s', .1, cfg['max_duration_s'])
            if duration < minimum-1e-9:
                raise ValueError(f'duration_s must be at least {minimum:.3f} for the configured reference limits')
        self.duration = minimum if duration is None else duration
        if self.duration > cfg['max_duration_s']:
            raise ValueError('trajectory exceeds max_duration_s')

    def sample(self, seconds):
        u = min(1., max(0., seconds/self.duration))
        blend = u**3 * (10 + u*(-15 + 6*u))
        return [a + blend*d for a, d in zip(self.start, self.delta)]


class EffortRamp:
    def __init__(self, start, goal, cfg, duration=None):
        self.start, self.goal = list(start), list(goal)
        self.delta = [b-a for a, b in zip(start, goal)]
        minimum = max(.1, 1.875*max(map(abs, self.delta))/cfg['max_effort_slew_a_s'])
        self.duration = minimum if duration is None else number(duration, 'duration_s', .1, cfg['max_duration_s'])
        if self.duration < minimum-1e-9 or self.duration > cfg['max_duration_s']:
            raise ValueError(f'effort ramp requires {minimum:.3f}..{cfg["max_duration_s"]:g} seconds')

    sample = Trajectory.sample


class Controller:
    """Deterministically tickable execution engine; SDK calls occur only in tick()."""
    def __init__(self, config, driver, armed, *, clock=time.monotonic, parent_alive=lambda: True):
        self.cfg, self.driver, self.armed, self.clock, self.parent_alive = config, driver, armed, clock, parent_alive
        self.lock = threading.RLock()
        self.epoch = 0
        self.pending = deque()
        self.stop_request = None
        self.results = OrderedDict()
        self.active = self.stopping = None
        self.target = None
        self.command_velocity = [0.]*20
        self.command_acceleration = [0.]*20
        self.command_effort = [0.]*20
        self.effort_velocity_guard = EffortVelocityGuard(config)
        self.control_parameters = {k: list(config[k]) for k in ('kp', 'kd', 'current_limit_a')}
        self.first_command_at = self.last_command_at = None
        self.command_count = self.skipped_slots = 0
        self.max_command_interval = 0.
        self.mode = 'disconnected'
        self.fault = None
        self.owns_control = False
        self.parent_seen = self.previous_tick = clock()
        self.closed = False
        self.cached = {'mode': self.mode, 'fresh': False, 'connected': False}
        self.updated = clock()

    def _finish(self, job, outcome, **extra):
        if job is not None and job['id'] is not None:
            with self.lock:
                self.results[job['id']] = {'done': True, 'ok': outcome in ('reached', 'prepared', 'held', 'stopped', 'connected',
                                                                         'effort_set', 'impedance_set', 'held_short_of_target'),
                                           'outcome': outcome, 'command_id': job['id'], **extra}

    def submit(self, ident, op, args=None):
        if op not in ('connect', 'prepare', 'move', 'hold', 'emergency', 'disable', 'set_effort', 'set_impedance'):
            raise ValueError('unknown hand operation')
        if not isinstance(ident, str) or not 1 <= len(ident) <= 80 or not isinstance(args or {}, dict):
            raise ValueError('invalid command id or arguments')
        with self.lock:
            if ident in self.results:
                return {'command_id': ident}  # Idempotent retries never replay motion.
            if len(self.results) >= 8192:
                raise RuntimeError('hand command ledger is full; start a new session')
            if op not in ('hold', 'emergency', 'disable') and (self.pending or self.active or self.stopping or self.stop_request):
                raise RuntimeError('hand operation already active')
            job = {'id': ident, 'op': op, 'args': deepcopy(args or {}), 'epoch': self.epoch}
            self.results[ident] = {'done': False, 'command_id': ident}
            if op in ('hold', 'emergency', 'disable'):
                # A hold must not replace an emergency/disable request.
                if op == 'hold' and (self.stopping or (self.stop_request and self.stop_request['op'] != 'hold')):
                    self._finish(job, 'rejected', reason='emergency/disable stop is in progress')
                    return {'command_id': ident}
                self.epoch += 1
                job['epoch'] = self.epoch
                for old in [*self.pending, self.active, self.stop_request, self.stopping]:
                    self._finish(old, 'stopped', ok=False, reason='cancelled by stop', confirmed=False)
                self.pending.clear()
                self.stop_request = job
            else:
                self.pending.append(job)
            return {'command_id': ident}

    def heartbeat(self):
        with self.lock:
            self.parent_seen = self.clock()
        return self.status()

    def result(self, ident):
        with self.lock:
            if ident not in self.results:
                raise ValueError('unknown hand command id')
            return deepcopy(self.results[ident])

    def status(self):
        with self.lock:
            value = deepcopy(self.cached)
            age = max(0., self.clock()-self.updated)
        value['controller_age_s'] = age
        for key, previous in value.get('age_s', {}).items():
            if previous is not None:
                value['age_s'][key] = previous + age
        if age > self.cfg['max_scheduler_gap_s'] or any(
                v is None or v > self.cfg['max_feedback_age_s'] for v in value.get('age_s', {}).values()):
            value['fresh'] = value['enabled'] = False
        return value

    def _publish(self):
        sample = self.driver.snapshot()
        path = (self.active.get('scurve') or self.active.get('feedback')) if self.active else None
        value = {**sample, 'mode': self.mode, 'fault': self.fault, 'target_positions': deepcopy(self.target),
                 'command_effort_a': list(self.command_effort),
                 'command_velocity_rad_s': list(self.command_velocity), 'motion_mode': self.cfg['motion_mode'],
                 'command_acceleration_rad_s2': (list(self.command_acceleration)
                                                 if self.cfg['motion_mode'] == 'scurve' else None),
                 'motion': path.status() if path else None,
                 'command_timing': {'requested_hz': self.cfg['command_hz'], 'commands': self.command_count,
                                    'observed_hz': ((self.command_count-1)/(self.last_command_at-self.first_command_at)
                                                    if self.command_count > 1 and self.last_command_at > self.first_command_at else None),
                                    'max_interval_s': self.max_command_interval, 'skipped_slots': self.skipped_slots},
                 'joint_names': list(JOINT_NAMES), 'profile_revision': self.cfg['profile_revision'],
                 'postures': sorted(self.cfg['postures']), 'owns_control': self.owns_control,
                 'action': self.active['id'] if self.active else None,
                 'limits': {k: deepcopy(self.cfg[k]) for k in ('lower_rad', 'upper_rad', 'max_velocity_rad_s',
                            'max_acceleration_rad_s2', 'max_jerk_rad_s3', 'max_step_rad', 'max_tracking_error_rad',
                            'tolerance_rad', 'settle_s', 'settle_timeout_s', 'settle_position_range_rad')},
                 'control_parameters': deepcopy(self.control_parameters),
                 'profile_control_parameters': {k: deepcopy(self.cfg[k]) for k in ('kp', 'kd', 'current_limit_a')},
                 'effort_control': {'units': 'A', 'calibrated_contact_force': False,
                                    'max_slew_a_s': self.cfg['max_effort_slew_a_s'],
                                    'velocity_guard': self.effort_velocity_guard.status(),
                                    'persists_until_changed_or_stopped': True},
                 'diagnostics_policy': {k: self.cfg[k] for k in ('allow_encoder_warning', 'max_temperature_c')},
                 'stop': deepcopy(self.stopping)}
        with self.lock:
            self.cached, self.updated = value, self.clock()

    def _admit(self, epoch):
        if epoch != self.epoch:
            raise Cancelled('command cancelled by stop')
        if not self.armed():
            raise PermissionError('operator has not armed motion')
        if not self.parent_alive() or self.clock()-self.parent_seen > self.cfg['parent_timeout_s']:
            raise RuntimeError('backend heartbeat expired')
        state = self.driver.snapshot()
        if not state['fresh'] or state['faults']:
            raise RuntimeError('fresh fault-free positions and diagnostics required')
        return state

    def _write(self, fn, epoch):
        self._admit(epoch)
        self.owns_control = True
        started, streaming = self.clock(), self.mode in ('holding', 'executing', 'enabling')
        fn()
        # A stop can arrive while a native SDK call blocks.
        if epoch != self.epoch:
            raise Cancelled('stop arrived during SDK call')
        if streaming and self.clock()-started > self.cfg['max_scheduler_gap_s']:
            raise RuntimeError('native hand write exceeded the scheduling deadline')

    def _send(self, epoch, *, seed=False):
        def publish():
            # Cover planning, feedback processing and scheduling, not just time
            # spent inside the native send call itself.
            if (not seed and self.mode in ('holding', 'executing', 'enabling') and self.last_command_at is not None
                    and self.clock()-self.last_command_at > self.cfg['max_scheduler_gap_s']):
                raise RuntimeError('hand command gap exceeded the scheduling deadline')
            if self.cfg['motion_mode'] in ('feedback', 'scurve'):
                self.driver.send(self.target, self.command_velocity, self.command_effort)
            else:
                self.driver.send(self.target, None, self.command_effort)
            if seed:
                return  # The one disabled seed precedes enable RPCs, not the periodic stream.
            sent = self.clock()
            if self.first_command_at is None:
                self.first_command_at = sent
            if self.last_command_at is not None:
                self.max_command_interval = max(self.max_command_interval, sent-self.last_command_at)
            self.last_command_at = sent
            self.command_count += 1
        self._write(publish, epoch)

    def _fail(self, reason):
        self.command_effort = [0.]*20
        self.effort_velocity_guard.reset()
        self.fault = str(reason)
        self._finish(self.active, 'fault', reason=self.fault, control_certain=False)
        self.active = None
        self.mode = 'faulted'
        with self.lock:
            if self.stop_request is None:
                self.epoch += 1
                for job in self.pending:
                    self._finish(job, 'rejected', reason=self.fault)
                self.pending.clear()
                if self.owns_control:
                    self.stop_request = {'id': None, 'op': 'emergency', 'args': {}, 'epoch': self.epoch}

    def _start_stop(self, job):
        self.active = self.stopping = None
        state = self.driver.snapshot()
        healthy_hold = (job['op'] == 'hold' and self.fault is None and self.armed() and state['fresh'] and state['enabled']
                        and not state['faults'] and self.target is not None
                        and max(abs(a-b) for a, b in zip(self.target, state['positions']))
                        <= self.cfg['max_tracking_error_rad'])
        if healthy_hold:
            moving = any(abs(x) > 1e-10 for x in [*self.command_velocity, *self.command_acceleration])
            if self.cfg['motion_mode'] == 'scurve' and moving:
                try:
                    path = SCurvePath(self.target, self.target, self.cfg, velocity=self.command_velocity,
                                      acceleration=self.command_acceleration, brake=True)
                except Exception as exc:  # noqa: BLE001 -- native planning failure must still request emergency stop.
                    job = {**job, 'hold_fallback_reason': str(exc)}
                else:
                    self.active = {**job, 'trajectory': path, 'scurve': path, 'braking': True,
                                   'started': self.clock()}
                    self.mode = 'executing'
                    return
            else:
                self.command_velocity, self.command_acceleration = [0.]*20, [0.]*20
                self._send(job['epoch'])
                self.mode = 'holding'
                self._finish(job, 'held', requested='hold', sdk_returned=True, confirmed=False,
                             target_positions=list(self.target), note='stationary command; physical settling not confirmed')
                return
        self.command_velocity, self.command_acceleration = [0.]*20, [0.]*20
        behavior = 'disable' if job['op'] == 'disable' else 'emergency'
        self.command_effort = [0.]*20
        self.effort_velocity_guard.reset()
        self.mode = 'stopping'
        self.target = None
        since = self.clock()
        try:
            self.driver.stop(behavior)
        except Exception as exc:
            self.mode = 'uncertain'
            self.fault = f'stop delivery failed: {exc}'
            self._finish(job, 'uncertain', requested=behavior, sdk_returned=False, confirmed=False, reason=self.fault)
            return
        if job['epoch'] != self.epoch:
            return  # A newer stop will be applied before any other write.
        self.stopping = {**job, 'behavior': behavior, 'since': since, 'deadline': self.clock()+self.cfg['stop_timeout_s']}

    def _check_stop(self, state):
        job = self.stopping
        if job is None:
            return
        received = state.get('received_monotonic', {})
        names = state.get('diagnostics', {}).get('state_names', [])
        confirmed = (state['fresh'] and len(names) == 20
                     and all(n.lower() in ('ready', 'stopped', 'disabled') for n in names)
                     and all(received.get(k, -1) > job['since'] for k in ('state', 'diagnostics'))
                     and max(map(abs, state['velocity_rad_s'])) <= self.cfg['stopped_velocity_rad_s'])
        if confirmed or self.clock() >= job['deadline']:
            self.mode = ('faulted' if self.fault else 'disabled') if confirmed else 'uncertain'
            self._finish(job, 'stopped' if confirmed else 'uncertain', requested=job['behavior'],
                         sdk_returned=True, confirmed=bool(confirmed), control_certain=bool(confirmed),
                         **({'hold_fallback_reason': job['hold_fallback_reason']} if 'hold_fallback_reason' in job else {}))
            self.stopping = None
            self.owns_control = not confirmed

    def _begin(self, job):
        if job['epoch'] != self.epoch:
            raise Cancelled('cancelled before dispatch')
        if job['op'] == 'connect':
            if self.owns_control:
                raise RuntimeError('stop and confirm disabled before reconnecting')
            self.driver.close()
            self.driver.connect()
            self.mode, self.fault = 'disabled', None
            self._finish(job, 'connected')
            return
        state = self._admit(job['epoch'])
        if job['op'] == 'prepare':
            names = state['diagnostics'].get('state_names', [])
            if len(names) != 20 or not all(n.lower() in ('ready', 'stopped', 'disabled') for n in names):
                raise ValueError('all joints must be disabled/Ready before operator preparation')
            if self.cfg['motion_mode'] == 'scurve':
                require_ruckig()  # Refuse missing/incompatible planner before enabling.
                if max(map(abs, state['velocity_rad_s'])) > self.cfg['stopped_velocity_rad_s']:
                    raise ValueError('S-curve initialization requires a measured stationary hand')
            self.target = target(state['positions'], self.cfg)
            self.command_velocity = [0.]*20
            self.command_acceleration = [0.]*20
            self.command_effort = [0.]*20
            self.effort_velocity_guard.reset()
            self.first_command_at = self.last_command_at = None
            self.command_count = self.skipped_slots = 0
            self.max_command_interval = 0.
            self.active, self.fault = job, None
            self._write(self.driver.apply_profile, job['epoch'])
            self.control_parameters = {k: list(self.cfg[k]) for k in ('kp', 'kd', 'current_limit_a')}
            if self.cfg['motion_mode'] == 'scurve':
                # Parameter RPCs may take time while the hand is disabled. Seed
                # from a fresh measurement immediately before enabling.
                state = self._admit(job['epoch'])
                if max(map(abs, state['velocity_rad_s'])) > self.cfg['stopped_velocity_rad_s']:
                    raise ValueError('hand moved during preparation')
                self.target = target(state['positions'], self.cfg)
            self._send(job['epoch'], seed=True)
            self._write(self.driver.enable, job['epoch'])
            self.mode = 'enabling'
            job['started'] = self.clock()
            return
        if self.mode != 'holding' or not state['enabled'] or self.target is None:
            raise RuntimeError('hand requires explicit operator preparation before motion')
        if max(abs(a-b) for a, b in zip(self.target, state['positions'])) > self.cfg['max_tracking_error_rad']:
            raise RuntimeError('unresolved tracking error before hand motion')
        if job['op'] == 'set_effort':
            args = validate_tool('wuji_set_effort', job['args'], self.cfg)
            goal = effort(args['effort_a'], self.control_parameters['current_limit_a'])
            ramp = EffortRamp(self.command_effort, goal, self.cfg, args['duration_s'])
            self.active = {**job, 'effort_ramp': ramp, 'started': self.clock()}
            self.mode = 'executing'
            return
        if job['op'] == 'set_impedance':
            values = impedance(job['args'], self.cfg)
            if any(abs(v) > 1e-10 for v in self.command_effort):
                raise ValueError('ramp feed-forward effort to zero before changing impedance/current limits')
            if max(map(abs, state['velocity_rad_s'])) > self.cfg['stopped_velocity_rad_s']:
                raise ValueError('impedance changes require a measured stationary hand')
            desired = {**self.control_parameters, **values}
            # Mark active before the first write: partial failures must stop,
            # never look like a pure pre-dispatch argument rejection.
            self.active = job
            if 'current_limit_a' in values:
                self._write(lambda: self.driver.set_current_limits(desired['current_limit_a']), job['epoch'])
            if 'kp' in values or 'kd' in values:
                self._write(lambda: self.driver.set_impedance(desired['kp'], desired['kd']), job['epoch'])
            self.control_parameters = deepcopy(desired)
            job['completion'] = ('impedance_set', {'control_parameters': deepcopy(desired),
                                                  'target_positions': list(self.target), 'readback_verified': True})
            return
        completion = completion_mode(job['args'].get('completion', 'position'))
        if self.cfg['motion_mode'] == 'scurve':
            trajectory = SCurvePath(self.target, job['args']['positions'], self.cfg, job['args'].get('duration_s'),
                                    velocity=self.command_velocity, acceleration=self.command_acceleration)
        else:
            trajectory = Trajectory(self.target, job['args']['positions'], self.cfg, job['args'].get('duration_s'))
        self.active = {**job, 'trajectory': trajectory, 'started': self.clock(), 'settled_since': None}
        if completion == 'contact':
            self.active['contact_settler'] = SettlingWindow(trajectory.goal, self.cfg['max_tracking_error_rad'],
                                                          self.cfg['settle_s'], self.cfg['settle_position_range_rad'])
        if self.cfg['motion_mode'] == 'feedback':
            self.active['feedback'] = FeedbackPath(trajectory, self.cfg, self.active['started'])
        elif self.cfg['motion_mode'] == 'scurve':
            self.active['scurve'] = trajectory
        if self.cfg['motion_mode'] in ('feedback', 'scurve'):
            self.active['settler'] = SettlingWindow(trajectory.goal, self.cfg['tolerance_rad'],
                                                  self.cfg['settle_s'], self.cfg['settle_position_range_rad'])
        self.mode = 'executing'

    def _contact_settled(self, job, state, reference_complete):
        window = job.get('contact_settler')
        if window is None:
            return False
        if not reference_complete or max(map(abs, state['velocity_rad_s'])) > self.cfg['stopped_velocity_rad_s']:
            window.rows.clear()
            return False
        return window.update(state['received_monotonic']['state'], state['positions'])

    def _contact_result(self, goal, state):
        errors = [a-b for a, b in zip(goal, state['positions'])]
        return {'completion': 'contact', 'stable': True, 'holding': True, 'contact_verified': False,
                'joint_errors_rad': errors,
                'shortfall_joints': [name for name, error in zip(JOINT_NAMES, errors)
                                     if abs(error) > self.cfg['tolerance_rad']],
                'reason': 'Stable shortfall accepted for deliberate contact; target remains held. '
                          'Contact and object retention require separate observation.'}

    def _advance_scurve(self, job, state, elapsed):
        path = job['scurve']
        self.target, self.command_velocity, self.command_acceleration = path.advance()
        if job.get('braking'):
            if path.complete:
                job['completion'] = ('held', {'requested': 'hold', 'sdk_returned': True, 'confirmed': False,
                                             'target_positions': list(self.target), 'braking_duration_s': path.duration,
                                             'note': 'reference brake completed; physical settling not confirmed'})
            elif elapsed > self.cfg['stop_timeout_s']:
                raise RuntimeError('S-curve hold braking exceeded its stop deadline')
            return
        if max(map(abs, state['velocity_rad_s'])) > self.cfg['stopped_velocity_rad_s']:
            job['settler'].rows.clear()
            settled = False
        else:
            settled = path.complete and job['settler'].update(state['received_monotonic']['state'], state['positions'])
        contact = not settled and self._contact_settled(job, state, path.complete)
        expired = elapsed > path.duration+self.cfg['settle_timeout_s']
        if expired and not path.complete:
            raise RuntimeError('S-curve reference missed its wall-time deadline; emergency stop required')
        if settled or contact or expired:
            error = max(abs(a-b) for a, b in zip(path.goal, state['positions']))
            result = {'reached': bool(settled), 'positions': state['positions'],
                      'target_positions': list(path.goal), 'command_positions': list(self.target),
                      'max_joint_error_rad': error, 'seconds': elapsed, 'motion_mode': 'scurve',
                      'profile_revision': self.cfg['profile_revision'],
                      'accepted_tolerance_rad': self.cfg['tolerance_rad'], 'trajectory_complete': path.complete,
                      **path.status()}
            if contact:
                result.update(self._contact_result(path.goal, state))
            elif not settled:
                result['reason'] = 'S-curve finished but measured joints did not settle within tolerance'
            job['completion'] = ('reached' if settled else 'held_short_of_target' if contact else 'target_not_reached', result)

    def _advance_feedback(self, job, state, elapsed):
        path = job['feedback']
        self.target, self.command_velocity = path.advance(self.clock(), state['positions'])
        error = max(abs(a-b) for a, b in zip(path.path.goal, state['positions']))
        settled = path.complete and job['settler'].update(state['received_monotonic']['state'], state['positions'])
        contact = not settled and self._contact_settled(job, state, path.complete)
        if settled or contact or elapsed > path.path.duration+self.cfg['settle_timeout_s']:
            result = {'reached': bool(settled), 'positions': state['positions'],
                      'target_positions': list(path.path.goal), 'command_positions': list(self.target),
                      'max_joint_error_rad': error, 'seconds': elapsed,
                      'profile_revision': self.cfg['profile_revision'], 'motion_mode': 'feedback',
                      'accepted_tolerance_rad': self.cfg['tolerance_rad'],
                      'trajectory_complete': path.complete, **path.status()}
            if contact:
                result.update(self._contact_result(path.path.goal, state))
            elif not settled:
                result['reason'] = 'feedback motion did not reach stable tolerance within its bounded time budget'
            job['completion'] = ('reached' if settled else 'held_short_of_target' if contact else 'target_not_reached', result)
            self.command_velocity = [0.]*20

    def tick(self, *, publish=True):
        now = self.clock()
        previous = (self.mode, id(self.active), id(self.stopping), self.fault, self.epoch)
        try:
            with self.lock:
                stop = self.stop_request
                self.stop_request = None
                tick_epoch = self.epoch
            if stop is not None:
                self._start_stop(stop)
            state = self.driver.snapshot()
            if self.mode in ('holding', 'executing', 'enabling'):
                if now-self.previous_tick > self.cfg['max_scheduler_gap_s']:
                    raise RuntimeError('hand scheduler delayed')
                self._admit(self.epoch)
                if self.mode != 'enabling' and not state['enabled']:
                    raise RuntimeError('one or more hand joints left Enabled state')
            self._check_stop(state)
            with self.lock:
                job = self.pending.popleft() if self.pending and not self.active and not self.stopping else None
            if job is not None:
                try:
                    self._begin(job)
                except Exception as exc:
                    self._finish(job, 'rejected', reason=str(exc))
                    if self.active is not None or isinstance(exc, Cancelled):
                        raise
            job = self.active
            if job and self.mode == 'enabling':
                state = self._admit(job['epoch'])
                if state['enabled'] and all(state['received_monotonic'].get(k, -1) > job['started']
                                            for k in ('state', 'diagnostics')):
                    self.mode = 'holding'
                    self._finish(job, 'prepared', profile_revision=self.cfg['profile_revision'])
                    self.active = None
                elif self.clock()-job['started'] > self.cfg['stop_timeout_s']:
                    raise RuntimeError('enable not confirmed by all twenty joints')
            elif job and self.mode == 'executing':
                elapsed = self.clock()-job['started']
                if 'effort_ramp' in job:
                    ramp = job['effort_ramp']
                    self.command_effort = ramp.sample(elapsed)
                    if elapsed >= ramp.duration:
                        self.command_effort = list(ramp.goal)
                        job['completion'] = ('effort_set', {'command_effort_a': list(ramp.goal),
                                                           'target_positions': list(self.target),
                                                           'ramp_seconds': ramp.duration, 'units': 'A',
                                                           'contact_force_verified': False})
                elif 'scurve' in job:
                    self._advance_scurve(job, state, elapsed)
                elif 'feedback' in job:
                    self._advance_feedback(job, state, elapsed)
                else:
                    self.target = job['trajectory'].sample(elapsed)
                if ('effort_ramp' not in job and 'feedback' not in job and 'scurve' not in job
                        and elapsed >= job['trajectory'].duration):
                    error = max(abs(a-b) for a, b in zip(self.target, state['positions']))
                    job['settled_since'] = (job['settled_since'] or self.clock()) if error <= self.cfg['tolerance_rad'] else None
                    if job['settled_since'] is not None and self.clock()-job['settled_since'] >= self.cfg['settle_s']:
                        job['completion'] = ('reached', {'reached': True, 'positions': state['positions'],
                            'target_positions': list(self.target), 'max_joint_error_rad': error,
                            'seconds': elapsed, 'profile_revision': self.cfg['profile_revision']})
                    elif self._contact_settled(job, state, True):
                        job['completion'] = ('held_short_of_target', {
                            'reached': False, 'positions': state['positions'], 'target_positions': list(self.target),
                            'command_positions': list(self.target), 'max_joint_error_rad': error,
                            'seconds': elapsed, 'profile_revision': self.cfg['profile_revision'],
                            'trajectory_complete': True, 'motion_mode': 'timed',
                            'accepted_tolerance_rad': self.cfg['tolerance_rad'],
                            **self._contact_result(self.target, state)})
                    elif elapsed > job['trajectory'].duration+self.cfg['settle_timeout_s']:
                        job['completion'] = ('target_not_reached', {'reached': False, 'max_joint_error_rad': error,
                            'positions': state['positions'], 'target_positions': list(self.target),
                            'reason': 'target did not settle; bounded target remains held, inspect before retry'})
            if self.mode in ('holding', 'executing', 'enabling'):
                state = self._admit(self.epoch)
                if max(abs(a-b) for a, b in zip(self.target, state['positions'])) > self.cfg['max_tracking_error_rad']:
                    raise RuntimeError('hand tracking error exceeded the configured bound')
                if any(self.command_effort):
                    target(state['positions'], self.cfg)  # Measured joint limits also bound current-driven motion.
                    self.effort_velocity_guard.check(state, self.clock())
                else:
                    self.effort_velocity_guard.reset()
                self._send(tick_epoch)
                with self.lock:
                    # Do not report success until the final write returned, and
                    # never replace a priority cancellation with a late success.
                    if self.active and 'completion' in self.active:
                        if self.active['epoch'] != self.epoch:
                            raise Cancelled('stop arrived before command completion')
                        outcome, result = self.active['completion']
                        self._finish(self.active, outcome, **result)
                        self.active, self.mode = None, 'holding'
        except Exception as exc:
            self._fail(exc)
        finally:
            self.previous_tick = self.clock()
            current = (self.mode, id(self.active), id(self.stopping), self.fault, self.epoch)
            if publish or previous != current or self.clock()-self.updated >= .01:
                self._publish()

    def run(self, stop):
        try:
            try:
                self.driver.connect()
                self.mode = 'disabled'
            except Exception as exc:
                self.fault = f'connection failed: {exc}'
            self._publish()
            due = self.clock()
            period = 1/self.cfg['command_hz']
            while not stop.is_set():
                self.tick(publish=False)
                due, skipped = next_deadline(due, self.clock(), period)
                self.skipped_slots += skipped
                while not stop.is_set():
                    remaining = due-self.clock()
                    if remaining <= 0:
                        break
                    if remaining > .0002:
                        stop.wait(remaining-.0002)
        finally:
            try:
                if self.owns_control:
                    self.driver.stop('emergency')
            finally:
                try:
                    self.driver.close()
                finally:
                    self.closed = True


def serve(config, root, socket_path, *, driver=None, parent_pid=None):
    root, socket_path = Path(root), Path(socket_path)
    stop = threading.Event()
    engine = Controller(config, driver or WujiDriver(config), lambda: arming.read(root)['armed'],
                        parent_alive=lambda: parent_pid is None or os.getppid() == parent_pid)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(.5)
            try:
                line = self.rfile.readline(65537)
                if len(line) > 65536 or not line.endswith(b'\n'):
                    raise ValueError('invalid IPC frame')
                request = json.loads(line)
                op = request['op']
                if op == 'status':
                    value = engine.status()
                elif op == 'heartbeat':
                    value = engine.heartbeat()
                elif op == 'result':
                    value = engine.result(request['id'])
                else:
                    value = engine.submit(request['id'], op, request.get('args', {}))
                response = {'ok': True, 'value': value}
            except Exception as exc:
                response = {'ok': False, 'error': str(exc)}
            self.wfile.write(json.dumps(response, allow_nan=False).encode()+b'\n')

    class Server(socketserver.UnixStreamServer):
        timeout = .05

    socket_path.unlink(missing_ok=True)
    worker = threading.Thread(target=engine.run, args=(stop,), name='wuji-writer', daemon=True)
    state_path = root/'operator'/'wuji-state.json'
    state_path.parent.mkdir(parents=True, exist_ok=True)
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, lambda *_: stop.set())
        signal.signal(signal.SIGINT, lambda *_: stop.set())
    try:
        with Server(str(socket_path), Handler) as server:
            socket_path.chmod(0o600)
            worker.start()
            last_record = 0.
            while not stop.is_set():
                server.handle_request()
                if time.monotonic()-last_record >= .1:
                    snapshot = engine.status()
                    snapshot['recorded_unix'] = time.time()
                    temporary = state_path.with_suffix('.tmp')
                    temporary.write_text(json.dumps(snapshot, allow_nan=False))
                    temporary.chmod(0o600)
                    os.replace(temporary, state_path)
                    last_record = time.monotonic()
                if engine.closed:
                    break
                if parent_pid is not None and os.getppid() != parent_pid:
                    stop.set()
    finally:
        stop.set()
        worker.join(timeout=config['stop_timeout_s']+2)
        socket_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    args = parser.parse_args()
    from services.controller.config import Configuration
    session = Configuration.load(args.config)
    config = validate_config(session.backend_options['end_effector'])
    serve(config, session.root, args.socket, parent_pid=args.parent_pid)


if __name__ == '__main__':
    main()
