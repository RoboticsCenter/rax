"""Bounded RPC client; a hand native crash cannot take down the arm watchdog."""
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time


class RevoHand:
    def __init__(self, settings, session):
        self.settings = settings
        self.lock = threading.Lock()
        self.sequence = 0
        self.closed = False
        self.failed = False
        self.process = None
        self.buffer = bytearray()
        Path(session).mkdir(parents=True, exist_ok=True)
        self.log = (Path(session) / 'revo-worker.log').open('ab')
        try:
            self.process = subprocess.Popen([sys.executable, '-m', 'services.piper.hand_worker'],
                                            cwd=Path(__file__).resolve().parents[2],
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=self.log, bufsize=0)
            self.process.stdin.write((json.dumps(settings, allow_nan=False) + '\n').encode())
            value = self._read(settings['hand_start_timeout_s'])
            if value.get('ready') is not True:
                raise RuntimeError(value.get('error', 'Hand worker did not start'))
            if value['hand']['serial'] != settings['hand_serial']:
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
            if self.closed or self.failed or self.process.poll() is not None:
                raise RuntimeError('BrainCo worker is unavailable')
            self.sequence += 1
            self.process.stdin.write((json.dumps({'id': self.sequence, 'op': operation, **arguments},
                                                 allow_nan=False) + '\n').encode())
            try:
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

    def state(self):
        value = self.request('state')
        if value.get('serial') != self.settings['hand_serial']:
            raise RuntimeError('Hand status identity mismatch')
        return value

    def move(self, positions):
        return self.request('move', positions=positions)

    def hold(self):
        return self.request('hold')

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
