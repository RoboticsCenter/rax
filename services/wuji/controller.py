"""Single-writer hand controller and private operator/backend IPC service.

The socket thread never calls the SDK. Stop increments an epoch immediately, so a
late native call cannot be followed by another target from the cancelled action.

While holding, the writer also runs the smooth streaming layer (services.wuji.smooth):
non-blocking targets and timed trajectories are validated on the socket thread, queued
with the current epoch, and applied by the writer, which interpolates and jerk-limits
them at the command rate. Blocking moves, hold, stop, disarm and every existing guard
keep their behavior; an idle stream sends exactly the previous hold command.
"""
import argparse
import json
import math
import os
import signal
import socketserver
import sys
import threading
import time
from collections import OrderedDict, deque
from copy import deepcopy
from pathlib import Path

from services.xarm import arming

from .config import JOINT_NAMES, completion_mode, effort, impedance, number, target, validate_config, validate_tool
from .guards import EffortVelocityGuard
from .motion import FeedbackPath, SettlingWindow, smooth_minimum_duration
from .partial import (DISABLED_STATES, active_indices, control_enabled, disabled_indices,
                      effective_parameters, max_error, max_velocity, validate_isolated_targets)
from .scurve import SCurvePath, next_deadline, require_ruckig
from .sdk import WujiDriver
from .smooth import (StreamEngine, parse_stream, parse_trajectory, profile_parameters, profile_summary,
                     resolve_settings, Spline)
from .trace import TraceRecorder

DIRECT_OPS = ('stream', 'trajectory', 'stream_config', 'record', 'stream_status')
# GIL hand-off interval for the hand process. The default 5 ms lets an IPC, snapshot
# or SDK callback thread delay the writer by several 1 kHz periods.
SWITCH_INTERVAL_S = .0005


class Cancelled(RuntimeError):
    pass


class Disarmed(PermissionError):
    """An operator disarm requests a stop, not a controller fault."""


class Trajectory:
    def __init__(self, start, goal, cfg, duration=None):
        self.start, self.goal = target(start, cfg), target(goal, cfg)
        validate_isolated_targets(self.start, self.goal, cfg)
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
        self.duration = minimum if duration is None else max(minimum, duration)
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
        self.duration = minimum if duration is None else max(minimum, number(duration, 'duration_s', .1, cfg['max_duration_s']))
        if self.duration > cfg['max_duration_s']:
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
        self.control_parameters = effective_parameters(config)
        self.first_command_at = self.last_command_at = None
        self.command_count = self.skipped_slots = 0
        self.max_command_interval = 0.
        self.timing_delays = {}
        self.mode = 'disconnected'
        self.fault = None
        self.owns_control = False
        self.parent_seen = self.previous_tick = clock()
        self.closed = False
        self.cached = {'mode': self.mode, 'fresh': False, 'connected': False}
        self.updated = clock()
        # Smooth streaming layer (writer-owned engine; socket thread uses the inbox).
        self.stream = StreamEngine(config)
        self.stream_inbox = deque()
        self.stream_target = None          # latest accepted full target, None when not streaming-ready
        self.stream_accepted = self.stream_processed = 0
        self.stream_error = None
        self.stream_pending_settings = None
        self.stream_effort = [0.]*20       # friction feed-forward added to command_effort
        self.command_offset = [0.]*20      # hold-deadband position offset added at send
        self.runtime_hz = config['command_hz']
        self.profile_name = 'profile'
        self.recorder = self.last_recording = None
        self._last_sent = None

    def _finish(self, job, outcome, **extra):
        if job is not None and job['id'] is not None:
            if outcome == 'target_not_reached' and extra.get('trajectory_complete') is True:
                # A bounded reference finished and remains held. Report measured
                # shortfall without treating it as an execution/transport failure.
                # Feedback, tracking, native fault and current guards still apply.
                extra.update(ok=True, reached=False, holding=True, contact_verified=False,
                             note='Reference completed; position tolerance was not met. '
                                  'The bounded target remains held; inspect measured shortfall before continuing.')
            if self.cfg['disabled_joints']:
                extra.update(partial_hand=True,
                             completion_scope='all_joints' if outcome in ('stopped', 'uncertain') else 'active_joints',
                             active_joints=[JOINT_NAMES[i] for i in active_indices(self.cfg)],
                             disabled_joints=list(self.cfg['disabled_joints']))
            with self.lock:
                self.results[job['id']] = {'done': True, 'ok': outcome in ('reached', 'prepared', 'held', 'stopped', 'connected',
                                                                         'effort_set', 'impedance_set', 'held_short_of_target',
                                                                         'profile_set'),
                                           'outcome': outcome, 'command_id': job['id'], **extra}

    def submit(self, ident, op, args=None):
        if op not in ('connect', 'prepare', 'move', 'hold', 'emergency', 'disable', 'set_effort', 'set_impedance',
                      'set_profile'):
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
                self.stream_inbox.clear()  # Queued stream targets never outlive a stop/hold.
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

    # ------------------------------------------------------------ smooth streaming (socket thread)
    def direct(self, op, args=None):
        """Non-job requests answered on the socket thread; none of them calls the SDK."""
        args = dict(args or {})
        if op == 'stream_status':
            return self.stream_status()
        if op == 'record':
            return self.record(args)
        if op == 'stream_config':
            return self.stream_configure(args)
        if op in ('stream', 'trajectory'):
            return self.stream_command(op, args)
        raise ValueError('unknown hand operation')

    def _stream_admission(self):
        if (self.mode != 'holding' or self.active is not None or self.pending or self.stopping
                or self.stop_request is not None or self.stream_target is None):
            busy = 'a blocking hand operation is active' if (self.active or self.pending) else f'mode is {self.mode}'
            raise RuntimeError(f'hand streaming requires a prepared, idle, holding hand ({busy})')
        if self.fault:
            raise RuntimeError(f'hand is faulted: {self.fault}')
        if not self.owns_control or not self.armed():
            raise RuntimeError('operator has not armed motion or the hand is not under control')

    def stream_command(self, op, args):
        dry_run = args.pop('dry_run', False) is True
        with self.lock:
            self._stream_admission()
            base = list(self.stream_target)
            if op == 'stream':
                command = parse_stream(args, self.cfg, base)
            else:
                command = parse_trajectory(args, self.cfg, base)
                # Conservative duration check from the latest target at rest.
                times = [0., *command['times']]
                columns = [[base[i], *[row[i] for row in command['rows']]] for i in command['indices']]
                peak_v = peak_a = 0.
                settings = self.stream_pending_settings or self.stream.settings
                for c, i in enumerate(command['indices']):
                    v, a = Spline.peaks(times, [columns[c]], [0.])
                    peak_v = max(peak_v, v/settings['max_velocity_rad_s'][i])
                    peak_a = max(peak_a, a/settings['max_acceleration_rad_s2'][i])
                scale = max(1., peak_v, math.sqrt(peak_a))
                if times[-1]*scale > self.cfg['max_duration_s']:
                    raise ValueError(f'trajectory needs about {times[-1]*scale:.2f} s at the configured limits, '
                                     f'above max_duration_s={self.cfg["max_duration_s"]}')
                command['estimated_time_scale'] = scale
            if dry_run:
                return {'valid': True, 'target_positions': list(command['target'])}
            command['epoch'] = self.epoch
            self.stream_accepted += 1
            command['accepted'] = self.stream_accepted
            self.stream_inbox.append(command)
            self.stream_target = list(command['target'])
            value = {'accepted': True, 'sequence': self.stream_accepted, 'target_positions': list(command['target'])}
            if op == 'trajectory':
                value.update(estimated_duration_s=command['times'][-1]*command['estimated_time_scale'],
                             estimated_time_scale=command['estimated_time_scale'])
            return value

    def stream_configure(self, args):
        reset = args.pop('reset', False)
        if type(reset) is not bool:
            raise ValueError('reset must be true or false')
        with self.lock:
            if self.mode not in ('holding', 'executing') or self.stream_target is None:
                raise RuntimeError('stream settings apply to a prepared hand; preparation restores the defaults')
            current = None if reset else (self.stream_pending_settings or self.stream.settings)
            settings = resolve_settings(args, self.cfg, current)
            # The writer adopts settings at the next rest, never mid-motion.
            self.stream_pending_settings = settings
            at_rest = self.mode == 'holding' and self.stream.converged
        from .smooth import public_settings
        return {'settings': public_settings(settings), 'applies': 'next_tick' if at_rest else 'at_rest'}

    def record(self, args):
        action = args.get('action')
        if action == 'start':
            directory = args.get('directory')
            if not isinstance(directory, str):
                raise ValueError('record start needs an absolute directory')
            max_seconds = number(args.get('max_seconds', self.cfg['max_record_s']), 'max_seconds',
                                 .1, self.cfg['max_record_s'])
            info = {k: args[k] for k in ('controller', 'skill', 'variant', 'trial') if isinstance(args.get(k), str)}
            with self.lock:
                if self.recorder is not None and not self.recorder.closed:
                    raise RuntimeError('a hand recording is already active; stop it first')
                recorder = TraceRecorder(directory, self.cfg['handedness'], max_seconds=max_seconds,
                                         nominal_rate_hz=self.runtime_hz, cfg=self.cfg,
                                         backend=getattr(self.driver, 'backend_kind', 'real'), info=info)
                self.recorder = recorder
            return recorder.summary()
        if action == 'stop':
            with self.lock:
                recorder, self.recorder = self.recorder, None
            if recorder is None:
                raise RuntimeError('no hand recording is active')
            recorder.close()  # Closed before saving: no row can be appended concurrently.
            self.last_recording = recorder
            threading.Thread(target=recorder.save, name='wuji-trace-save', daemon=True).start()
            return {**recorder.summary(), 'saving': True}
        if action == 'status':
            recorder = self.recorder or self.last_recording
            return None if recorder is None else {**recorder.summary(), 'active': recorder is self.recorder}
        raise ValueError('record action must be start, stop or status')

    def stream_status(self):
        with self.lock:
            cached = self.cached
            value = {key: deepcopy(cached.get(key)) for key in (
                'mode', 'fault', 'fresh', 'positions', 'velocity_rad_s', 'target_positions', 'stream',
                'smooth_profile', 'command_effort_a', 'action', 'temperature_max_c')}
            age = max(0., self.clock()-self.updated)
        value['controller_age_s'] = age
        if age > self.cfg['max_scheduler_gap_s']:
            value['fresh'] = False
        return value

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
            value['fresh'] = value['enabled'] = value['active_enabled'] = False
        return value

    def _publish(self):
        sample = self.driver.snapshot()
        path = (self.active.get('scurve') or self.active.get('feedback')) if self.active else None
        value = {**sample, 'mode': self.mode, 'fault': self.fault, 'target_positions': deepcopy(self.target),
                 'partial_hand': bool(self.cfg['disabled_joints']),
                 'disabled_joints': list(self.cfg['disabled_joints']),
                 'active_joints': [JOINT_NAMES[i] for i in active_indices(self.cfg)],
                 'active_enabled': control_enabled(sample, self.cfg),
                 'command_effort_a': list(self.command_effort),
                 'command_velocity_rad_s': list(self.command_velocity), 'motion_mode': self.cfg['motion_mode'],
                 'command_acceleration_rad_s2': (list(self.command_acceleration)
                                                 if self.cfg['motion_mode'] == 'scurve' else None),
                 'motion': path.status() if path else None,
                 'stream': self._stream_public(sample),
                 'smooth_profile': profile_summary(self.control_parameters, self.cfg, self.profile_name),
                 'command_offset_rad': list(self.command_offset) if any(self.command_offset) else None,
                 'friction_effort_a': list(self.stream_effort) if any(self.stream_effort) else None,
                 'recording': None if self.recorder is None else self.recorder.summary(),
                 'command_timing': {'requested_hz': self.cfg['command_hz'], 'runtime_hz': self.runtime_hz,
                                    'commands': self.command_count,
                                    'observed_hz': ((self.command_count-1)/(self.last_command_at-self.first_command_at)
                                                    if self.command_count > 1 and self.last_command_at > self.first_command_at else None),
                                    'max_interval_s': self.max_command_interval, 'skipped_slots': self.skipped_slots,
                                    'warning_threshold_s': min(.05, self.cfg['max_scheduler_gap_s']),
                                    'stop_threshold_s': self.cfg['max_scheduler_gap_s'],
                                    'delays': deepcopy(self.timing_delays)},
                 'temperature_max_c': self._hottest(sample),
                 'joint_names': list(JOINT_NAMES), 'profile_revision': self.cfg['profile_revision'],
                 'postures': sorted(self.cfg['postures']), 'owns_control': self.owns_control,
                 'action': self.active['id'] if self.active else None,
                 'limits': {k: deepcopy(self.cfg[k]) for k in ('lower_rad', 'upper_rad', 'max_velocity_rad_s',
                            'max_acceleration_rad_s2', 'max_jerk_rad_s3', 'max_step_rad', 'max_tracking_error_rad',
                            'tolerance_rad', 'settle_s', 'settle_timeout_s', 'settle_position_range_rad')},
                 'control_parameters': deepcopy(self.control_parameters),
                 'profile_control_parameters': effective_parameters(self.cfg),
                 'effort_control': {'units': 'A', 'calibrated_contact_force': False,
                                    'max_slew_a_s': self.cfg['max_effort_slew_a_s'],
                                    'velocity_guard': self.effort_velocity_guard.status(),
                                    'persists_until_changed_or_stopped': True},
                 'diagnostics_policy': {k: deepcopy(self.cfg[k]) for k in (
                     'allow_encoder_warning', 'allow_init_current_calib_failed', 'max_temperature_c',
                     'stall_warning_grace_s',
                     'disabled_joints', 'isolated_joint_faults')},
                 'stop': deepcopy(self.stopping)}
        with self.lock:
            self.cached, self.updated = value, self.clock()

    def _hottest(self, sample):
        temperatures = (sample.get('diagnostics') or {}).get('mcu_temp_c_fb')
        if not temperatures or len(temperatures) != 20:
            return None
        return max(temperatures[i] for i in active_indices(self.cfg))

    def _stream_public(self, sample):
        value = self.stream.status(self.clock())
        positions = sample.get('positions')
        goal = self.stream.goal
        if self.stream_target is not None:
            value['target_positions'] = list(self.stream_target)
        value.update(accepted=self.stream_accepted, processed=self.stream_processed,
                     ready=self.stream_target is not None,
                     settings_pending=self.stream_pending_settings is not None, last_error=self.stream_error,
                     max_error_rad=(max_error(self.cfg, goal, positions)
                                    if positions is not None and len(positions) == 20 else None))
        return value

    def _admit(self, epoch):
        if epoch != self.epoch:
            raise Cancelled('command cancelled by stop')
        if not self.armed():
            raise Disarmed('operator has not armed motion')
        if not self.parent_alive() or self.clock()-self.parent_seen > self.cfg['parent_timeout_s']:
            raise RuntimeError('backend heartbeat expired')
        state = self.driver.snapshot()
        if state['faults']:
            details = '; '.join(f"{fault.get('joint', 'hand')}: "
                                f"{fault.get('name') or fault.get('reason') or fault.get('code')}"
                                for fault in state['faults'])
            raise RuntimeError(f'hand diagnostic fault: {details}')
        if not state['fresh']:
            raise RuntimeError(f"fresh positions and diagnostics required; ages_s={state.get('age_s')}; "
                               f"stream_errors={state.get('stream_errors', {})}")
        if self.cfg['disabled_joints']:
            states = state.get('diagnostics', {}).get('states', [])
            if len(states) != 20 or any(states[i] not in DISABLED_STATES for i in disabled_indices(self.cfg)):
                raise RuntimeError('excluded joint is not confirmed disabled')
        return state

    def _check_timing(self, source, seconds):
        # Short host scheduling delays slow the tick-driven S-curve; they do
        # not imply a native hand fault. Retain diagnostics, including the
        # attempted interval when the watchdog prevents the next write.
        if seconds > min(.05, self.cfg['max_scheduler_gap_s']):
            previous = self.timing_delays.get(source, {})
            self.timing_delays[source] = {
                'count': previous.get('count', 0)+1,
                'max_s': max(seconds, previous.get('max_s', 0.)),
                'last_s': seconds, 'last_monotonic': self.clock(),
            }
        if seconds > self.cfg['max_scheduler_gap_s']:
            if not self.armed():
                raise Disarmed('operator has not armed motion')
            raise RuntimeError(f'hand {source.replace("_", " ")} {seconds:.3f}s exceeded '
                               f'host watchdog {self.cfg["max_scheduler_gap_s"]:.3f}s')

    def _write(self, fn, epoch):
        self._admit(epoch)
        self.owns_control = True
        started, streaming = self.clock(), self.mode in ('holding', 'executing', 'enabling')
        fn()
        # A stop can arrive while a native SDK call blocks.
        if epoch != self.epoch:
            raise Cancelled('stop arrived during SDK call')
        if streaming:
            self._check_timing('native_write', self.clock()-started)

    def _send(self, epoch, *, seed=False):
        def publish():
            # Cover planning, feedback processing and scheduling, not just time
            # spent inside the native send call itself.
            if not seed and self.mode in ('holding', 'executing', 'enabling') and self.last_command_at is not None:
                self._check_timing('command_gap', self.clock()-self.last_command_at)
            # Hold-deadband offsets and friction feed-forward are zero unless the
            # smooth layer is engaged; then the stream also supplies its velocity.
            positions = (self.target if not any(self.command_offset)
                         else [q+o for q, o in zip(self.target, self.command_offset)])
            currents = (self.command_effort if not any(self.stream_effort)
                        else [b+f for b, f in zip(self.command_effort, self.stream_effort)])
            if self.cfg['motion_mode'] in ('feedback', 'scurve') or any(self.command_velocity):
                self.driver.send(positions, self.command_velocity, currents)
            else:
                self.driver.send(positions, None, currents)
            if seed:
                return  # The one disabled seed precedes enable RPCs, not the periodic stream.
            sent = self.clock()
            self._last_sent = (sent, positions, currents)
            if self.first_command_at is None:
                self.first_command_at = sent
            if self.last_command_at is not None:
                self.max_command_interval = max(self.max_command_interval, sent-self.last_command_at)
            self.last_command_at = sent
            self.command_count += 1
        self._write(publish, epoch)

    def _release_stream(self):
        """Stops and faults end streaming: no offsets, friction or queued targets survive."""
        self.stream_effort, self.command_offset = [0.]*20, [0.]*20
        self.stream_pending_settings = None
        with self.lock:
            self.stream_inbox.clear()
            self.stream_target = None

    def _adopt_reference(self):
        """Called with the lock held when control returns to holding at a new target."""
        self.stream_inbox.clear()
        self.stream_target = list(self.target)
        self.stream_effort, self.command_offset = [0.]*20, [0.]*20
        self.stream.sync(self.target)
        self.stream.watch_settle = True  # wuji_wait also reports settling after blocking moves.

    def _fail(self, reason):
        self.command_effort = [0.]*20
        self._release_stream()
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

    def _stop_for_disarm(self):
        self.command_effort = [0.]*20
        self._release_stream()
        self.effort_velocity_guard.reset()
        with self.lock:
            for job in [*self.pending, self.active]:
                self._finish(job, 'stopped', ok=False, reason='operator disarmed motion', confirmed=False)
            self.pending.clear()
            self.active = None
            if self.stop_request is not None:
                # Disarm cannot retain an enabled hold. Preserve the stop's ID
                # so its caller still receives physical stop confirmation.
                self.stop_request['op'] = 'emergency'
            elif self.owns_control and self.stopping is None:
                self.epoch += 1
                self.stop_request = {'id': None, 'op': 'emergency', 'args': {}, 'epoch': self.epoch}
            self.mode = ('stopping' if self.stop_request or self.stopping else
                         'faulted' if self.fault else 'disabled')

    def _start_stop(self, job):
        previous_mode, previous_active = self.mode, self.active
        self.active = self.stopping = None
        state = self.driver.snapshot()
        healthy_hold = (job['op'] == 'hold' and self.fault is None and self.armed() and control_enabled(state, self.cfg)
                        and not state['faults'] and self.target is not None
                        and max_error(self.cfg, self.target, state['positions'])
                        <= self.cfg['max_tracking_error_rad'])
        if healthy_hold:
            streaming = ((previous_mode == 'holding' and previous_active is None
                          and (self.stream.moving or not self.stream.goal_static))
                         or bool(previous_active and previous_active.get('stream_brake') and self.stream.braking))
            if streaming:
                # Brake the stream reference to rest with its own jerk/acceleration limits.
                with self.lock:
                    self.stream_inbox.clear()
                if not self.stream.braking:
                    self.stream.brake()
                self.active = {**job, 'stream_brake': True, 'started': self.clock()}
                self.mode = 'executing'
                return
            moving = any(abs(x) > 1e-10 for x in [*self.command_velocity, *self.command_acceleration])
            if self.cfg['motion_mode'] == 'scurve' and moving:
                try:
                    path = SCurvePath(self.target, self.target, self._runtime_cfg(), velocity=self.command_velocity,
                                      acceleration=self.command_acceleration, brake=True)
                except Exception as exc:  # noqa: BLE001 -- native planning failure must still request emergency stop.
                    job = {**job, 'hold_fallback_reason': str(exc)}
                else:
                    self.active = {**job, 'trajectory': path, 'scurve': path, 'braking': True,
                                   'started': self.clock()}
                    self.mode = 'executing'
                    return
            else:
                self.active = job
                self.command_velocity, self.command_acceleration = [0.]*20, [0.]*20
                self._send(job['epoch'])
                with self.lock:
                    self.mode = 'holding'
                    self._adopt_reference()
                self._finish(job, 'held', requested='hold', sdk_returned=True, confirmed=False,
                             target_positions=list(self.target), note='stationary command; physical settling not confirmed')
                self.active = None
                return
        self.command_velocity, self.command_acceleration = [0.]*20, [0.]*20
        behavior = 'disable' if job['op'] == 'disable' else 'emergency'
        self.command_effort = [0.]*20
        self._release_stream()
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

    def _runtime_cfg(self):
        """S-curve paths advance one period per tick: use the active writer rate."""
        if self.runtime_hz == self.cfg['command_hz']:
            return self.cfg
        return {**self.cfg, 'command_hz': self.runtime_hz}

    def _set_control_parameters(self, desired):
        setter = getattr(self.driver, 'set_control_parameters', None)
        if setter is not None:
            setter(desired)
            return
        if self.cfg['disabled_joints']:
            raise ValueError('this hand driver cannot write partial-hand control parameters')
        self.driver.set_current_limits(desired['current_limit_a'])
        self.driver.set_impedance(desired['kp'], desired['kd'])

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
            self.stopping = None
            self.owns_control = not confirmed
            # IPC may read status as soon as the result becomes available. Publish
            # the confirmed stop before exposing completion, not at the next tick.
            self._publish()
            self._finish(job, 'stopped' if confirmed else 'uncertain', requested=job['behavior'],
                         sdk_returned=True, confirmed=bool(confirmed), control_certain=bool(confirmed),
                         **({'hold_fallback_reason': job['hold_fallback_reason']} if 'hold_fallback_reason' in job else {}))

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
        if self.cfg['disabled_joints'] and job['op'] in ('set_effort', 'set_impedance'):
            raise ValueError('partial-hand mode supports explicit joint moves and hold only')
        if self.cfg['disabled_joints'] and job['op'] == 'move' and job['args'].get('completion', 'position') != 'position':
            raise ValueError('partial-hand mode requires position completion on active joints')
        if job['op'] == 'prepare':
            names = state['diagnostics'].get('state_names', [])
            if len(names) != 20 or not all(n.lower() in ('ready', 'stopped', 'disabled') for n in names):
                raise ValueError('all joints must be disabled/Ready before operator preparation')
            if self.cfg['motion_mode'] == 'scurve':
                require_ruckig()  # Refuse missing/incompatible planner before enabling.
                if max_velocity(self.cfg, state['velocity_rad_s']) > self.cfg['stopped_velocity_rad_s']:
                    raise ValueError('S-curve initialization requires a measured stationary hand')
            self.target = target(state['positions'], self.cfg)
            self.command_velocity = [0.]*20
            self.command_acceleration = [0.]*20
            self.command_effort = [0.]*20
            self._release_stream()
            self.stream.reset(self.target)  # Preparation restores default stream settings.
            self.runtime_hz, self.profile_name = self.cfg['command_hz'], 'profile'
            self.effort_velocity_guard.reset()
            self.first_command_at = self.last_command_at = None
            self.command_count = self.skipped_slots = 0
            self.max_command_interval = 0.
            self.active, self.fault = job, None
            self._write(self.driver.apply_profile, job['epoch'])
            self.control_parameters = effective_parameters(self.cfg)
            if self.cfg['motion_mode'] == 'scurve':
                # Parameter RPCs may take time while the hand is disabled. Seed
                # from a fresh measurement immediately before enabling.
                state = self._admit(job['epoch'])
                if max_velocity(self.cfg, state['velocity_rad_s']) > self.cfg['stopped_velocity_rad_s']:
                    raise ValueError('hand moved during preparation')
                self.target = target(state['positions'], self.cfg)
            self._send(job['epoch'], seed=True)
            if self.cfg['disabled_joints']:
                self._write(lambda: self.driver.enable(check=lambda: self._admit(job['epoch'])), job['epoch'])
            else:
                self._write(self.driver.enable, job['epoch'])
            self.mode = 'enabling'
            job['started'] = self.clock()
            return
        if self.mode != 'holding' or not control_enabled(state, self.cfg) or self.target is None:
            raise RuntimeError('hand requires explicit operator preparation before motion')
        if max_error(self.cfg, self.target, state['positions']) > self.cfg['max_tracking_error_rad']:
            raise RuntimeError('unresolved tracking error before hand motion')
        if not self.stream.converged or self.stream_inbox:
            raise ValueError('the hand stream reference is still moving; call wuji_wait or wuji_hold '
                             'before a blocking move, effort, impedance or profile change')
        if job['op'] == 'set_profile':
            desired, info = profile_parameters(job['args'], self.cfg, self.control_parameters)
            if max_velocity(self.cfg, state['velocity_rad_s']) > self.cfg['stopped_velocity_rad_s']:
                raise ValueError('profile changes require a measured stationary hand')
            for name, bias, cap in zip(JOINT_NAMES, self.command_effort, desired['current_limit_a']):
                if abs(bias) > cap:
                    raise ValueError(f'{name} feed-forward bias {bias:g} A exceeds the requested cap {cap:g} A')
            self.active = job  # Partial writes must stop, never look like a pure rejection.
            self.stream_effort, self.command_offset = [0.]*20, [0.]*20
            self._write(lambda: self._set_control_parameters(desired), job['epoch'])
            self.control_parameters = deepcopy(desired)
            self.profile_name = info['name']
            settings = self.stream_pending_settings or self.stream.settings
            settings = {**settings, 'hold_deadband_rad': info['hold_deadband_rad']}
            if self.stream_pending_settings is not None:
                self.stream_pending_settings = settings
            else:
                self.stream.settings = settings
            job['completion'] = ('profile_set', {
                'profile': profile_summary(desired, self.cfg, info['name']),
                'control_parameters': deepcopy(desired), 'hold_deadband_rad': info['hold_deadband_rad'],
                'target_positions': list(self.target), 'readback_verified': True})
            return
        if job['op'] in ('move', 'set_effort', 'set_impedance'):
            # Blocking operations send the plain reference, without stream offsets.
            self.stream_effort, self.command_offset = [0.]*20, [0.]*20
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
            if max_velocity(self.cfg, state['velocity_rad_s']) > self.cfg['stopped_velocity_rad_s']:
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
            trajectory = SCurvePath(self.target, job['args']['positions'], self._runtime_cfg(), job['args'].get('duration_s'),
                                    velocity=self.command_velocity, acceleration=self.command_acceleration)
        else:
            trajectory = Trajectory(self.target, job['args']['positions'], self.cfg, job['args'].get('duration_s'))
        self.active = {**job, 'trajectory': trajectory, 'started': self.clock(), 'settled_since': None}
        if completion == 'contact':
            self.active['contact_settler'] = SettlingWindow(trajectory.goal, self.cfg['max_tracking_error_rad'],
                                                          self.cfg['settle_s'], self.cfg['settle_position_range_rad'],
                                                          indices=active_indices(self.cfg))
        if self.cfg['motion_mode'] == 'feedback':
            self.active['feedback'] = FeedbackPath(trajectory, self.cfg, self.active['started'])
        elif self.cfg['motion_mode'] == 'scurve':
            self.active['scurve'] = trajectory
        if self.cfg['motion_mode'] in ('feedback', 'scurve'):
            self.active['settler'] = SettlingWindow(trajectory.goal, self.cfg['tolerance_rad'],
                                                  self.cfg['settle_s'], self.cfg['settle_position_range_rad'],
                                                  indices=active_indices(self.cfg))
        self.mode = 'executing'

    def _contact_settled(self, job, state, reference_complete):
        window = job.get('contact_settler')
        if window is None:
            return False
        if not reference_complete or max_velocity(self.cfg, state['velocity_rad_s']) > self.cfg['stopped_velocity_rad_s']:
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
        if max_velocity(self.cfg, state['velocity_rad_s']) > self.cfg['stopped_velocity_rad_s']:
            job['settler'].rows.clear()
            settled = False
        else:
            settled = path.complete and job['settler'].update(state['received_monotonic']['state'], state['positions'])
        contact = not settled and self._contact_settled(job, state, path.complete)
        if not path.complete and elapsed > path.reference_deadline_s:
            raise RuntimeError('S-curve reference missed its wall-time deadline; emergency stop required')
        if path.complete:
            job.setdefault('reference_completed_at', elapsed)
        expired = path.complete and elapsed-job['reference_completed_at'] > self.cfg['settle_timeout_s']
        if settled or contact or expired:
            error = max_error(self.cfg, path.goal, state['positions'])
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
        error = max_error(self.cfg, path.path.goal, state['positions'])
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

    # ------------------------------------------------------------ smooth streaming (writer thread)
    def _drain_stream(self, now):
        with self.lock:
            commands = list(self.stream_inbox)
            self.stream_inbox.clear()
        for command in commands:
            self.stream_processed = max(self.stream_processed, command.get('accepted', 0))
            if command['epoch'] != self.epoch:
                continue  # Accepted before a stop/hold: never applied.
            try:
                self.stream.apply(command, now)
                self.stream_error = None
            except ValueError as exc:
                self.stream_error = f'{command["kind"]} rejected: {exc}'
                with self.lock:
                    self.stream_target = list(self.stream.latest_target)
        pending = self.stream_pending_settings
        if pending is not None and self.stream.converged:
            self.stream.apply({'kind': 'settings', 'settings': pending}, now)
            self.stream_pending_settings = None
            self.runtime_hz = pending['command_hz']

    def _advance_stream(self, state):
        now = self.clock()
        if not self.stream.same_reference(self.target):
            # Safety net: something other than the stream moved the reference.
            with self.lock:
                self._adopt_reference()
        if self.stream_inbox or self.stream_pending_settings is not None:
            self._drain_stream(now)
        if not self.stream.engaged:
            if any(self.command_offset) or any(self.stream_effort):
                self.command_offset, self.stream_effort = [0.]*20, [0.]*20
            return
        q, v, a, offset, friction = self.stream.step(now, state, self.command_effort,
                                                     self.control_parameters['current_limit_a'])
        self.target, self.command_velocity, self.command_acceleration = q, v, a
        self.command_offset, self.stream_effort = offset, friction

    def _advance_stream_brake(self, job, state, elapsed):
        q, v, a, offset, friction = self.stream.step(self.clock(), state, self.command_effort,
                                                     self.control_parameters['current_limit_a'])
        self.target, self.command_velocity, self.command_acceleration = q, v, a
        self.command_offset, self.stream_effort = offset, friction
        if not self.stream.braking:
            self.command_velocity, self.command_acceleration = [0.]*20, [0.]*20
            job['completion'] = ('held', {'requested': 'hold', 'sdk_returned': True, 'confirmed': False,
                                         'target_positions': list(self.target), 'braking_duration_s': elapsed,
                                         'note': 'stream reference braked to rest; physical settling not confirmed'})
        elif elapsed > self.cfg['stop_timeout_s']:
            raise RuntimeError('stream hold braking exceeded its stop deadline')

    def _record(self, state):
        recorder, sent = self.recorder, self._last_sent
        if recorder is None or sent is None:
            return
        job = self.active
        path = job.get('trajectory') if job else None
        if path is not None and hasattr(path, 'goal'):
            requested = path.goal
        elif self.stream_target is not None:
            requested = self.stream_target
        else:
            requested = self.target
        streaming = self.mode == 'holding' and job is None or bool(job and job.get('stream_brake'))
        recorder.append(sent[0], q_ref=sent[1], q_target=requested, state=state, i_ff=sent[2],
                        kp=self.control_parameters['kp'], kd=self.control_parameters['kd'],
                        dq_ref=self.command_velocity, q_goal=self.stream.goal if streaming else self.target,
                        current_limit_a=self.control_parameters['current_limit_a'])

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
                self._admit(self.epoch)
                self._check_timing('scheduler_gap', now-self.previous_tick)
                if self.mode != 'enabling' and not control_enabled(state, self.cfg):
                    raise RuntimeError('one or more active hand joints left Enabled state')
            self._check_stop(state)
            with self.lock:
                job = self.pending.popleft() if self.pending and not self.active and not self.stopping else None
            if job is not None:
                try:
                    self._begin(job)
                except Exception as exc:
                    self._finish(job, 'rejected', reason=str(exc))
                    if self.active is not None or isinstance(exc, (Cancelled, Disarmed)):
                        raise
            job = self.active
            if job and self.mode == 'enabling':
                state = self._admit(job['epoch'])
                if control_enabled(state, self.cfg) and all(state['received_monotonic'].get(k, -1) > job['started']
                                            for k in ('state', 'diagnostics')):
                    with self.lock:
                        self.mode = 'holding'
                        self._adopt_reference()
                    self._finish(job, 'prepared', profile_revision=self.cfg['profile_revision'])
                    self.active = None
                elif self.clock()-job['started'] > self.cfg['stop_timeout_s']:
                    raise RuntimeError('enable not confirmed by all active joints with excluded joints disabled')
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
                elif 'stream_brake' in job:
                    self._advance_stream_brake(job, state, elapsed)
                elif 'scurve' in job:
                    self._advance_scurve(job, state, elapsed)
                elif 'feedback' in job:
                    self._advance_feedback(job, state, elapsed)
                elif 'trajectory' in job:
                    self.target = job['trajectory'].sample(elapsed)
                if ('effort_ramp' not in job and 'feedback' not in job and 'scurve' not in job
                        and 'stream_brake' not in job and elapsed >= job['trajectory'].duration):
                    error = max_error(self.cfg, self.target, state['positions'])
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
                            'trajectory_complete': True,
                            'reason': 'target did not settle; bounded target remains held, inspect before retry'})
            if self.mode == 'holding' and self.active is None:
                self._advance_stream(state)
            if self.mode in ('holding', 'executing', 'enabling'):
                state = self._admit(self.epoch)
                if max_error(self.cfg, self.target, state['positions']) > self.cfg['max_tracking_error_rad']:
                    raise RuntimeError('hand tracking error exceeded the configured bound')
                if any(self.command_effort):
                    target(state['positions'], self.cfg)  # Measured joint limits also bound current-driven motion.
                    self.effort_velocity_guard.check(state, self.clock())
                else:
                    self.effort_velocity_guard.reset()
                self._send(tick_epoch)
                if self.recorder is not None:
                    self._record(state)
                with self.lock:
                    # Do not report success until the final write returned, and
                    # never replace a priority cancellation with a late success.
                    if self.active and 'completion' in self.active:
                        if self.active['epoch'] != self.epoch:
                            raise Cancelled('stop arrived before command completion')
                        job, (outcome, result) = self.active, self.active['completion']
                        self.active, self.mode = None, 'holding'
                        self._adopt_reference()
                        # Status (profile, targets) must be current once the result is visible.
                        self._publish()
                        self._finish(job, outcome, **result)
        except Cancelled:
            # submit() has already cancelled the job and queued a priority
            # stop. Let the next tick deliver it without latching a fault.
            pass
        except Disarmed:
            self._stop_for_disarm()
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
            while not stop.is_set():
                self.tick(publish=False)
                period = 1/self.runtime_hz  # Operator rate, or a lower stream_config rate.
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


def serve(config, root, socket_path, *, driver=None, parent_pid=None, side=None):
    root, socket_path = Path(root), Path(socket_path)
    stop = threading.Event()
    armed = arming.CachedReader(root)
    engine = Controller(config, driver or WujiDriver(config), lambda: armed()['armed'],
                        parent_alive=lambda: parent_pid is None or os.getppid() == parent_pid)
    sys.setswitchinterval(SWITCH_INTERVAL_S)
    try:
        import numpy  # noqa: F401 -- loaded before the loop so a first trace start cannot stall it.
    except ImportError:
        pass

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.request.settimeout(.5)
            try:
                # Timed trajectories (up to 256 x 20 waypoints) exceed the old 64 KiB frame.
                line = self.rfile.readline(1048577)
                if len(line) > 1048576 or not line.endswith(b'\n'):
                    raise ValueError('invalid IPC frame')
                request = json.loads(line)
                op = request['op']
                if op == 'status':
                    value = engine.status()
                elif op == 'heartbeat':
                    value = engine.heartbeat()
                elif op == 'result':
                    value = engine.result(request['id'])
                elif op in DIRECT_OPS:
                    value = engine.direct(op, request.get('args', {}))
                else:
                    value = engine.submit(request['id'], op, request.get('args', {}))
                response = {'ok': True, 'value': value}
            except Exception as exc:
                response = {'ok': False, 'error': str(exc),
                            'kind': 'invalid' if isinstance(exc, ValueError) else 'state'}
            try:
                self.wfile.write(json.dumps(response, allow_nan=False).encode()+b'\n')
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass  # A departed reader must not block other clients or the writer.

    class Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
        timeout = .05
        daemon_threads = True
        block_on_close = False

    socket_path.unlink(missing_ok=True)
    worker = threading.Thread(target=engine.run, args=(stop,), name='wuji-writer', daemon=True)
    from .client import state_path as snapshot_path
    state_path = snapshot_path(root, side)
    state_path.parent.mkdir(parents=True, exist_ok=True)
    def record_snapshots():
        # Dashboard persistence is not a control transport. Disk stalls must not
        # prevent accepting parent heartbeats or priority stop requests.
        while not stop.is_set():
            try:
                snapshot = engine.status()
                snapshot['recorded_unix'] = time.time()
                temporary = state_path.with_suffix('.tmp')
                temporary.write_text(json.dumps(snapshot, allow_nan=False))
                temporary.chmod(0o600)
                os.replace(temporary, state_path)
            except OSError:
                # Consumers age old snapshots out; live IPC and device watchdogs
                # remain authoritative. Retry persistence without blocking them.
                pass
            stop.wait(.1)
    recorder = threading.Thread(target=record_snapshots, name='wuji-snapshots', daemon=True)
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, lambda *_: stop.set())
        signal.signal(signal.SIGINT, lambda *_: stop.set())
    try:
        with Server(str(socket_path), Handler) as server:
            socket_path.chmod(0o600)
            worker.start()
            recorder.start()
            while not stop.is_set():
                server.handle_request()
                if engine.closed:
                    break
                if parent_pid is not None and os.getppid() != parent_pid:
                    stop.set()
    finally:
        stop.set()
        worker.join(timeout=config['stop_timeout_s']+2)
        if recorder.ident is not None:
            recorder.join(timeout=1)
        socket_path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--socket', type=Path, required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--side', choices=('left', 'right'))
    args = parser.parse_args()
    from services.controller.config import Configuration
    session = Configuration.load(args.config)
    config = validate_config(session.backend_options['end_effector'])
    serve(config, session.root, args.socket, parent_pid=args.parent_pid, side=args.side)


if __name__ == '__main__':
    main()
