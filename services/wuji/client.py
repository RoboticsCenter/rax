"""Backend/operator client for the private hand service; no vendor SDK imports."""
import hashlib
import json
import os
import socket
import subprocess
import sys
import threading
import time
import uuid
from copy import deepcopy
from pathlib import Path

from services.json_io import write_private_json
from services.wuji import arming

from .config import validate_config
from .partial import control_enabled

PROJECT = Path(__file__).resolve().parents[2]


def side_suffix(side):
    if side not in (None, 'left', 'right'):
        raise ValueError('hand side must be left or right')
    return '' if side is None else f'-{side}'


def socket_path(root, side=None):
    return Path(f'/tmp/rax-wuji-{os.getuid()}')/(hashlib.sha256(str(Path(root).resolve()).encode()).hexdigest()[:16]
                                               + side_suffix(side) + '.sock')


def state_path(root, side=None):
    return Path(root)/'operator'/f'wuji{side_suffix(side)}-state.json'


def request(path, op, *, timeout=.5, **payload):
    with socket.socket(socket.AF_UNIX) as connection:
        connection.settimeout(timeout)
        connection.connect(str(path))
        connection.sendall(json.dumps({'op': op, **payload}, allow_nan=False).encode()+b'\n')
        with connection.makefile('rb') as stream:
            line = stream.readline(262145)
    if not line or len(line) > 262144 or not line.endswith(b'\n'):
        raise ConnectionError('hand service returned an invalid response')
    response = json.loads(line)
    if not response['ok']:
        raise RuntimeError(response['error'])
    return response['value']


class HandError(RuntimeError):
    def __init__(self, result, state=None):
        super().__init__(result.get('reason') or result.get('outcome') or 'hand execution failed')
        self.result = result
        self.control_certain = False
        self.robot_state = {'hand': state, 'hand_result': result}


def operation(root, op, args=None, *, timeout=10., side=None):
    """Operator access to the existing owner; does not create a second SDK client."""
    path = socket_path(root, side) if side is not None else socket_path(root)
    ident = uuid.uuid4().hex
    request(path, op, id=ident, args=args or {})
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        try:
            result = request(path, 'result', id=ident,
                             timeout=min(.5, max(.01, deadline-time.monotonic())))
        except TimeoutError:
            # A native SDK call can delay the service's Python IPC thread. Query
            # the same command's result again; never resend the motor operation.
            continue
        if result['done']:
            if not result['ok']:
                raise HandError(result)
            return result
        time.sleep(.02)
    try:
        request(path, 'emergency', id=uuid.uuid4().hex)
    except Exception:
        pass
    raise HandError({'outcome': 'uncertain', 'reason': f'{op} timed out; emergency stop requested'})


def recorded_status(root, max_age_s=.5, *, side=None):
    """Dashboard snapshot with explicit age; never opens a hand connection."""
    try:
        value = json.loads(state_path(root, side).read_text())
        age = max(0., time.time()-value['recorded_unix'])
        value['record_age_s'] = age
        if age > max_age_s:
            value['fresh'] = value['enabled'] = value['active_enabled'] = False
            value['fault'] = value.get('fault') or 'hand service snapshot is stale'
        return value
    except (OSError, ValueError, KeyError) as exc:
        return {'fresh': False, 'connected': False, 'enabled': False, 'mode': 'unavailable', 'fault': str(exc)}


class WujiClient:
    def __init__(self, session, on_fault=None, *, side=None):
        self.cfg = validate_config(session.backend_options['end_effector'])
        self.side = side
        self.root, self.path = session.root, socket_path(session.root, side)
        self.on_fault = on_fault
        self.stop_event = threading.Event()
        self.cached = {'connected': False, 'fresh': False, 'enabled': False, 'mode': 'starting'}
        self.received = time.monotonic()
        self.process = self.thread = None
        from .sdk import private_directory
        private_directory()
        config_path = self.root/'operator'/f'wuji{side_suffix(side)}-configuration.json'
        config_path.parent.mkdir(parents=True, exist_ok=True)
        write_private_json(config_path, session.to_dict())
        if self.path.exists():
            raise RuntimeError('hand service socket already exists; stop its owner before restarting this session')
        with open(self.root/f'wuji{side_suffix(side)}-controller.log', 'ab') as log:
            self.process = subprocess.Popen(
                [sys.executable, '-m', 'services.wuji.controller', '--config', str(config_path),
                 '--socket', str(self.path), '--parent-pid', str(os.getpid()),
                 *(['--side', side] if side else [])], cwd=PROJECT,
                env={**os.environ, 'PYTHONPATH': str(PROJECT), 'WUJI_TELEMETRY_DISABLE': '1'},
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+5
        while not self.path.exists():
            if self.process.poll() is not None or time.monotonic() > deadline:
                self.close()
                raise RuntimeError('hand service did not start; see wuji-controller.log')
            time.sleep(.02)
        self.thread = threading.Thread(target=self._monitor, name='wuji-supervisor', daemon=True)
        self.thread.start()

    def _monitor(self):
        previously_owned = notified = False
        while not self.stop_event.is_set():
            try:
                value = request(self.path, 'heartbeat')
                previously_owned = previously_owned or value.get('owns_control', False)
                expected_stop = (not arming.read(self.root)['armed'] and value.get('fresh')
                                 and not value.get('fault') and not value.get('faults')
                                 and value.get('mode') in ('disabled', 'stopping'))
                bad = previously_owned and not expected_stop and (value.get('mode') in ('faulted', 'uncertain', 'disconnected', 'disabled', 'stopping')
                                             or not value.get('fresh'))
                if expected_stop and value.get('mode') == 'disabled' and not value.get('owns_control'):
                    previously_owned = False
                self.cached, self.received = value, time.monotonic()
            except Exception as exc:
                bad = previously_owned
                self.cached = {**self.cached, 'fresh': False, 'enabled': False, 'active_enabled': False,
                               'mode': 'uncertain',
                               'fault': f'hand service heartbeat IPC failed ({type(exc).__name__}): {exc}'}
            if bad and not notified:
                notified = True
                if not self.cached.get('fresh'):
                    try:
                        request(self.path, 'emergency', id=uuid.uuid4().hex)
                    except Exception:
                        pass
                if self.on_fault:
                    try:
                        self.on_fault()
                    except Exception as exc:
                        self.cached['arm_stop_error'] = str(exc)
            elif not bad and self.cached.get('mode') == 'holding':
                notified = False
            self.stop_event.wait(.05)

    def status(self):
        value = deepcopy(self.cached)
        age = max(0., time.monotonic()-self.received)
        value['service_snapshot_age_s'] = age
        for key, v in value.get('age_s', {}).items():
            if v is not None:
                value['age_s'][key] = v + age
        if age > self.cfg['max_feedback_age_s'] or any(
                v is None or v > self.cfg['max_feedback_age_s'] for v in value.get('age_s', {}).values()):
            value['fresh'] = value['enabled'] = value['active_enabled'] = False
        return value

    def require_ready(self, *, allow_executing=False):
        # The supervisor polls at 20 Hz. Immediately after a completed move its
        # cache can still say "executing", or miss a more recent stop/fault.
        # Admission needs the owner's current snapshot; this read never enables
        # the hand and the owner independently validates every submitted move.
        state = request(self.path, 'status')
        modes = ('holding', 'executing') if allow_executing else ('holding',)
        if (not control_enabled(state, self.cfg) or state.get('fault') or state.get('faults')
                or state.get('mode') not in modes
                or (allow_executing and not state.get('owns_control'))):
            required = 'holding or executing its combined move' if allow_executing else 'holding'
            raise RuntimeError(f'Wuji hand is not prepared and {required} with fresh fault-free feedback; ask the operator')
        return state

    def move(self, positions, duration_s=None, completion='position'):
        duration = self.cfg['max_duration_s']
        if self.cfg['motion_mode'] == 'scurve':
            duration = max(duration*self.cfg['reference_timeout_scale'], duration+self.cfg['settle_timeout_s'])
        result = self._operation('move', {'positions': positions, 'duration_s': duration_s,
                                              'completion': completion},
                           timeout=duration+self.cfg['settle_timeout_s']+2)
        return result

    def hold(self):
        return self._operation('hold', timeout=self.cfg['stop_timeout_s']+2)

    def set_effort(self, effort_a, duration_s=None):
        args = {'effort_a': effort_a}
        if duration_s is not None:
            args['duration_s'] = duration_s
        return self._operation('set_effort', args, timeout=self.cfg['max_duration_s']+2)

    def set_impedance(self, **parameters):
        return self._operation('set_impedance', parameters)

    def stop(self):
        return self._operation('emergency', timeout=self.cfg['stop_timeout_s']+2)

    def _operation(self, op, args=None, **kwargs):
        if getattr(self, 'side', None) is not None:
            kwargs['side'] = self.side
        return operation(self.root, op, args, **kwargs)

    def close(self):
        if self.process is not None:
            try:
                if self.process.poll() is None and self.cached.get('owns_control'):
                    self.stop()
            except Exception:
                pass
            self.stop_event.set()
            if self.thread and self.thread is not threading.current_thread():
                self.thread.join(timeout=1)
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=self.cfg['stop_timeout_s']+3)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            self.process = None

