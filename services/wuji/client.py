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

from .config import validate_config

PROJECT = Path(__file__).resolve().parents[2]


def socket_path(root):
    return Path(f'/tmp/rax-wuji-{os.getuid()}')/(hashlib.sha256(str(Path(root).resolve()).encode()).hexdigest()[:16]+'.sock')


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


def operation(root, op, args=None, *, timeout=10.):
    """Operator access to the existing owner; does not create a second SDK client."""
    path = socket_path(root)
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


def recorded_status(root, max_age_s=.5):
    """Dashboard snapshot with explicit age; never opens a hand connection."""
    try:
        value = json.loads((Path(root)/'operator'/'wuji-state.json').read_text())
        age = max(0., time.time()-value['recorded_unix'])
        value['record_age_s'] = age
        if age > max_age_s:
            value['fresh'] = value['enabled'] = False
            value['fault'] = value.get('fault') or 'hand service snapshot is stale'
        return value
    except (OSError, ValueError, KeyError) as exc:
        return {'fresh': False, 'connected': False, 'enabled': False, 'mode': 'unavailable', 'fault': str(exc)}


class WujiClient:
    def __init__(self, session, on_fault=None):
        self.cfg = validate_config(session.backend_options['end_effector'])
        self.root, self.path = session.root, socket_path(session.root)
        self.on_fault = on_fault
        self.stop_event = threading.Event()
        self.cached = {'connected': False, 'fresh': False, 'enabled': False, 'mode': 'starting'}
        self.received = time.monotonic()
        self.process = self.thread = None
        from .sdk import private_directory
        private_directory()
        config_path = self.root/'operator'/'wuji-configuration.json'
        config_path.parent.mkdir(parents=True, exist_ok=True)
        write_private_json(config_path, session.to_dict())
        if self.path.exists():
            raise RuntimeError('hand service socket already exists; stop its owner before restarting this session')
        with open(self.root/'wuji-controller.log', 'ab') as log:
            self.process = subprocess.Popen(
                [sys.executable, '-m', 'services.wuji.controller', '--config', str(config_path),
                 '--socket', str(self.path), '--parent-pid', str(os.getpid())], cwd=PROJECT,
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
                bad = previously_owned and (value.get('mode') in ('faulted', 'uncertain', 'disconnected', 'disabled', 'stopping')
                                             or not value.get('fresh'))
                self.cached, self.received = value, time.monotonic()
            except Exception as exc:
                bad = previously_owned
                self.cached = {**self.cached, 'fresh': False, 'enabled': False, 'mode': 'uncertain', 'fault': str(exc)}
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
            value['fresh'] = value['enabled'] = False
        return value

    def require_ready(self):
        # The supervisor polls at 20 Hz. Immediately after a completed move its
        # cache can still say "executing", or miss a more recent stop/fault.
        # Admission needs the owner's current snapshot; this read never enables
        # the hand and the owner independently validates every submitted move.
        state = request(self.path, 'status')
        if not state.get('fresh') or not state.get('enabled') or state.get('fault') or state.get('mode') != 'holding':
            raise RuntimeError('Wuji hand is not prepared and holding with fresh fault-free feedback; ask the operator')
        return state

    def move(self, positions, duration_s=None):
        result = operation(self.root, 'move', {'positions': positions, 'duration_s': duration_s},
                           timeout=self.cfg['max_duration_s']+self.cfg['settle_timeout_s']+2)
        return result

    def hold(self):
        return operation(self.root, 'hold', timeout=self.cfg['stop_timeout_s']+2)

    def set_effort(self, effort_a, duration_s=None):
        args = {'effort_a': effort_a}
        if duration_s is not None:
            args['duration_s'] = duration_s
        return operation(self.root, 'set_effort', args, timeout=self.cfg['max_duration_s']+2)

    def set_impedance(self, **parameters):
        return operation(self.root, 'set_impedance', parameters)

    def stop(self):
        return operation(self.root, 'emergency', timeout=self.cfg['stop_timeout_s']+2)

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

