"""Independent bounded locomotion worker; MCP never owns a servo loop."""
from copy import deepcopy
import math
import threading
import time


class MotionError(RuntimeError):
    control_certain = False


def healthy(state, cfg):
    if state.get('feedback_error'):
        raise MotionError(state['feedback_error'])
    age = state.get('feedback_age_s')
    if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= cfg['max_feedback_age_s']:
        raise MotionError('G1 feedback is stale')
    for key, length in (('joints', 29), ('joint_velocity', 29), ('rpy', 3), ('motor_faults', 29)):
        values = state.get(key, [])
        if len(values) != length or any(type(x) not in (float, int) or not math.isfinite(x) for x in values):
            raise MotionError(f'Invalid G1 {key} feedback')
    if any(state['motor_faults']):
        raise MotionError('G1 motor fault reported')
    if max(abs(x) for x in state['rpy'][:2]) > cfg['max_tilt_rad']:
        raise MotionError('G1 tilt exceeds the commissioned envelope')
    allowed = cfg['allowed_fsm_ids'] if cfg['transport'] == 'unitree' else [500]
    if state.get('fsm_id') not in allowed:
        raise MotionError('G1 is outside its commissioned locomotion mode')


class LocomotionWorker:
    def __init__(self, transport, cfg, permit):
        self.transport, self.cfg, self.permit = transport, cfg, permit
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.closed = False
        self.job = None
        self.stop_requested = False
        self.stop_done = threading.Event()
        self.stop_error = None
        self.fault = None
        self.cached = {}
        self.updated = 0.
        self.thread = threading.Thread(target=self._run, name='g1-locomotion', daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            value = deepcopy(self.cached)
            if 'feedback_age_s' in value:
                value['feedback_age_s'] += time.monotonic() - self.updated
            value.update(active_motion=self.job is not None, fault=self.fault,
                         worker_alive=self.thread.is_alive())
            return value

    def require_ready(self):
        value = self.snapshot()
        if value['fault'] or not value['worker_alive']:
            raise MotionError(value['fault'] or 'G1 worker stopped')
        healthy(value, self.cfg)
        self.permit()

    def move(self, velocity, duration):
        self.require_ready()
        job = {'velocity': velocity, 'duration': duration, 'done': threading.Event(),
               'started': None, 'result': None, 'error': None}
        with self.lock:
            if self.closed or self.stop_requested or self.job is not None:
                raise MotionError('G1 worker is busy or closed')
            self.job = job
        self.wake.set()
        if not job['done'].wait(duration + 2.):
            self.stop()
            raise MotionError('G1 command timed out; stop requested')
        if job['error']:
            raise MotionError(job['error'])
        return job['result']

    def stop(self):
        with self.lock:
            if self.closed:
                raise MotionError('G1 worker is closed')
            self.stop_done.clear()
            self.stop_requested = True
        self.wake.set()
        if not self.stop_done.wait(1.):
            raise MotionError('G1 stop acknowledgement timed out')
        if self.stop_error:
            raise MotionError(self.stop_error)
        return {'requested': True, 'acknowledged': True, 'stationary_verified': False}

    def _finish(self, job, error=None):
        if job is None:
            return
        with self.lock:
            job['error'] = error
            job['result'] = {'outcome': 'command_window_completed', 'command_accepted': True,
                             'target_reached': None, 'stationary_verified': False,
                             'state': deepcopy(self.cached)}
            if self.job is job:
                self.job = None
            job['done'].set()

    def _zero(self):
        self.transport.velocity(0., 0., 0., self.cfg['command_lease_s'])

    def _run(self):
        while True:
            with self.lock:
                closed, stop, job = self.closed, self.stop_requested, self.job
            if closed:
                return
            if stop:
                error = None
                try:
                    self._zero()
                except Exception as exc:
                    error = str(exc)
                with self.lock:
                    self.stop_error = error
                    self.stop_requested = False
                    if error:
                        self.fault = error
                    self._finish(job, error or 'Motion interrupted by stop')
                    self.stop_done.set()
            try:
                state = self.transport.state()
                with self.lock:
                    self.cached, self.updated = state, time.monotonic()
                with self.lock:
                    job = self.job
                if job is not None:
                    self.permit()
                    healthy(state, self.cfg)
                    now = time.monotonic()
                    if job['started'] is None:
                        job['started'] = now
                    remaining = job['duration'] - (now - job['started'])
                    if remaining <= 0:
                        self._zero()
                        self._finish(job)
                    else:
                        # Native expiry also bounds motion if this process dies.
                        self.transport.velocity(*job['velocity'], min(remaining, self.cfg['command_lease_s']))
            except Exception as exc:
                with self.lock:
                    job = self.job
                    if not self.cached:
                        self.cached = {'feedback_error': str(exc)}
                    else:
                        self.cached['feedback_error'] = str(exc)
                if job is not None:
                    error = str(exc)
                    try:
                        self._zero()
                    except Exception as stop_exc:
                        error += f'; stop failed: {stop_exc}'
                    with self.lock:
                        self.fault = error
                    self._finish(job, error)
            self.wake.wait(.05)
            self.wake.clear()

    def close(self):
        if self.closed:
            return
        try:
            # Only send a shutdown stop if this worker has an active command.
            if self.snapshot()['active_motion']:
                self.stop()
        finally:
            with self.lock:
                self.closed = True
            self.wake.set()
            self.thread.join(1.)
            if self.thread.is_alive():
                raise MotionError('G1 worker did not exit; command lease will expire')
            self.transport.close()
