"""Exclusive RAX pair lease and receive-only Piper feedback freshness monitor."""
import fcntl
import os
from pathlib import Path
import socket
import stat
import struct
import threading
import time

import numpy as np


class PairLease:
    def __init__(self, identity):
        if not identity.isalnum():
            raise ValueError('Invalid robot USB serial')
        root = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))
        path = root / f'rax-piper-{os.getuid()}-{identity}.lock'
        self.fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
        try:
            info = os.fstat(self.fd)
            if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid():
                raise RuntimeError('Unsafe robot ownership lock')
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class PassiveFeedback:
    """Independent receive-only CAN freshness check; never sends CAN frames."""

    required = (0x2A1, 0x2A5, 0x2A6, 0x2A7, *range(0x261, 0x267))

    def __init__(self, interface):
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.settimeout(.05)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.frames = {}
        self.error = None
        try:
            self.socket.bind((interface,))
        except BaseException:
            self.socket.close()
            raise
        self.thread = threading.Thread(target=self._receive, name='piper-feedback', daemon=True)
        self.thread.start()

    def _receive(self):
        while not self.stop.is_set():
            try:
                raw = self.socket.recv(16)
                can_id, size, payload = struct.unpack('=IB3x8s', raw)
                if can_id in self.required and size == 8:
                    with self.lock:
                        self.frames[can_id] = (time.monotonic(), payload)
            except socket.timeout:
                continue
            except Exception as exc:
                self.error = str(exc)
                return

    def check(self, sdk_degrees):
        deadline = time.monotonic() + .3
        while True:
            with self.lock:
                latest = dict(self.frames)
            now = time.monotonic()
            if self.error:
                raise RuntimeError('Passive CAN receiver failed: ' + self.error)
            if all(key in latest and now - latest[key][0] <= .25 for key in self.required):
                measured = []
                for key in (0x2A5, 0x2A6, 0x2A7):
                    measured.extend(item / 1000 for item in struct.unpack('>ii', latest[key][1]))
                agrees = bool(np.max(np.abs(np.asarray(measured) - sdk_degrees)) <= .25)
                if not agrees:
                    raise RuntimeError('Arm SDK disagrees with independent CAN feedback')
                status = latest[0x2A1][1]
                motors = [latest[key][1][5] for key in range(0x261, 0x267)]
                return {'joints_deg': measured, 'sdk_agrees': agrees,
                        'arm_status': status[1], 'error': int.from_bytes(status[6:8], 'big'),
                        'enabled': [bool(value & 64) for value in motors],
                        'motor_faults': [value & 191 for value in motors]}
            if now >= deadline:
                raise RuntimeError('Missing or stale independent Piper CAN feedback')
            time.sleep(.005)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=.2)
        self.socket.close()
