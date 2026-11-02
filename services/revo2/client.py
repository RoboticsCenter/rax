"""Bounded RPC client; a hand native crash cannot take down the arm watchdog."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time


class RevoFault(RuntimeError):
    operator_required = True


class RevoHand:
    def __init__(self, session, on_fault=None):
        from .config import validate_config
        settings = validate_config(session.backend_options['end_effector'])
        self.settings = settings
        self.root = session.root
        self.on_fault = on_fault
        self.lock = threading.Lock()
        self.sequence = 0
        self.closed = False
        self.failed = False
        self.process = None
        self.buffer = bytearray()
        directory = Path(session.root)/'operator'
        directory.mkdir(parents=True, exist_ok=True)
        self.log = (directory / f'revo-{settings["handedness"]}-worker.log').open('ab')
        try:
            self.process = subprocess.Popen([sys.executable, '-m', 'services.revo2.worker'],
                                            cwd=Path(__file__).resolve().parents[2],
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.log, bufsize=0)
            self.process.stdin.write((json.dumps({'settings': settings, 'root': str(session.root)}, allow_nan=False) + '\n').encode())
            value = self._read(settings['hand_start_timeout_s'])
            if value.get('ready') is not True:
                raise RuntimeError(value.get('error', 'Hand worker did not start'))
            if value['hand']['serial'] != settings['serial']:
                raise RuntimeError('Hand worker identity mismatch')
        except BaseException:
            self.close()
            raise

    def _read(self, timeout):
        deadline = time.monotonic() + timeout
        while b'\n' not in self.buffer and len(self.buffer) <= 65536:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], remaining)[0]:
                raise TimeoutError('BrainCo worker timed out; command outcome may be uncertain')
            chunk = os.read(self.process.stdout.fileno(), 65536)
            if not chunk:
                raise RuntimeError('BrainCo worker exited; inspect its operator-only log')
            self.buffer.extend(chunk)
        line, separator, remainder = self.buffer.partition(b'\n')
        if not separator or len(line) > 65536:
            raise RuntimeError('Oversized hand worker response')
        self.buffer = bytearray(remainder)
        return json.loads(line)

    def request(self, operation, **arguments):
        with self.lock:
            return self._request_locked(operation, **arguments)

    def _request_locked(self, operation, **arguments):
        if self.closed or self.failed or self.process.poll() is not None:
            raise RuntimeError('BrainCo worker is unavailable')
        self.sequence += 1
        try:
            self.process.stdin.write((json.dumps({'id': self.sequence, 'op': operation, **arguments},
                                                 allow_nan=False) + '\n').encode())
            value = self._read(self.settings['hand_rpc_timeout_s'])
            if value.get('id') != self.sequence:
                raise RuntimeError('Hand RPC response identity mismatch; execution uncertain')
        except Exception:
            self.failed = True
            self.process.kill()
            raise
        if 'error' in value:
            raise RuntimeError(value['error'])
        return value['result']

    def status(self):
        # During a finite worker command, use its independently refreshed telemetry
        # instead of blocking the arm watchdog behind the command's RPC lock.
        if not self.lock.acquire(blocking=False):
            value = json.loads((self.root/'operator'/f'revo-{self.settings["handedness"]}-state.json').read_text())
            value['fresh'] = 0 <= time.time() - value['time'] <= .5
        else:
            try:
                value = self._request_locked('revo_status')
            finally:
                self.lock.release()
        if value.get('serial') != self.settings['serial']:
            raise RuntimeError('Hand status identity mismatch')
        return value

    def require_ready(self):
        try:
            value = self.status()
        except Exception as exc:
            raise RevoFault(f'Revo feedback unavailable: {exc}') from exc
        if not value.get('fresh') or value.get('fault'):
            raise RevoFault('Revo feedback is stale or a control fault requires operator recovery')
        return value

    def execute(self, name, arguments):
        return self.request(name, arguments=arguments)

    def hold(self):
        return self.request('revo_hold')

    stop = hold

    def close(self):
        if self.closed:
            return
        try:
            if self.process is not None and self.process.poll() is None:
                try:
                    self.request('close')
                    self.process.wait(timeout=2.)
                except Exception:
                    self.process.kill()
                    self.process.wait(timeout=2.)
        finally:
            self.closed = True
            if self.process is not None:
                for pipe in (self.process.stdin, self.process.stdout):
                    if pipe is not None:
                        pipe.close()
            self.log.close()
