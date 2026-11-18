"""Shared-memory blocks for the hands' 1 kHz streams between the sim server and the
processes that run the SDK mimic (one writer per block, any number of readers).

A block is a file in a private tmpfs directory, mapped with numpy: a header of int64
[sequence, timestamp_ns, count, reserved] and named float64 arrays. Writers use a
sequence lock (odd while writing), so readers never see a torn frame.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import tempfile
import time

import numpy as np

HEADER = 4
STATE_FIELDS = {'q': 20, 'dq': 20, 'effort': 20}
DIAG_FIELDS = {'states': 20, 'error': 20, 'current': 20, 'vbus': 20, 'temperature': 20, 'current_limited': 20}
COMMAND_FIELDS = {'q': 20, 'dq': 20, 'effort': 20}


def private_directory(token):
    base = Path('/dev/shm') if Path('/dev/shm').is_dir() else Path(tempfile.gettempdir())
    path = base/f'rax-sim-{os.getuid()}'/token
    path.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(path.parent, 0o700)
    return path


def token_for(root):
    return hashlib.sha256(str(Path(root).resolve()).encode()).hexdigest()[:16]


class Block:
    def __init__(self, path, fields, *, create=False):
        self.path, self.fields = Path(path), dict(fields)
        size = 8 * (HEADER + sum(self.fields.values()))
        if create:
            fd = os.open(self.path, os.O_RDWR | os.O_CREAT | os.O_TRUNC, 0o600)
            try:
                os.ftruncate(fd, size)
            finally:
                os.close(fd)
        elif self.path.stat().st_size != size:
            raise ValueError(f'{self.path} has the wrong layout')
        self.memory = np.memmap(self.path, dtype=np.float64, mode='r+', shape=(HEADER + sum(self.fields.values()),))
        self.header = self.memory[:HEADER].view(np.int64)
        self.arrays, offset = {}, HEADER
        for name, size in self.fields.items():
            self.arrays[name] = self.memory[offset:offset + size]
            offset += size

    def write(self, t_ns, values):
        header = self.header
        header[0] += 1                   # odd: a frame is being written
        for name, value in values.items():
            self.arrays[name][:] = value
        header[1] = int(t_ns)
        header[2] += 1
        header[0] += 1                   # even: complete

    def count(self):
        return int(self.header[2])

    def read(self, timeout_s=0.05):
        """(count, timestamp_ns, {name: array copy}) of the last complete frame.

        A writer can be descheduled mid-frame (its interpreter switches threads), so a
        reader waits for the frame to complete, up to timeout_s, yielding as it waits."""
        header = self.header
        deadline = None
        attempt = 0
        while True:
            before = int(header[0])
            if not before & 1:
                values = {name: array.copy() for name, array in self.arrays.items()}
                t_ns, count = int(header[1]), int(header[2])
                if int(header[0]) == before:
                    return count, t_ns, values
            attempt += 1
            if attempt > 20:
                now = time.monotonic()
                deadline = deadline or now + timeout_s
                if now > deadline:
                    raise TimeoutError(f'{self.path}: no consistent frame within {timeout_s} s')
                time.sleep(0.00005)

    def close(self):
        memory, self.memory = self.memory, None
        if memory is not None:
            del memory

    def unlink(self):
        self.close()
        self.path.unlink(missing_ok=True)


def hand_paths(directory, serial):
    directory = Path(directory)
    return {'state': directory/f'{serial}.state', 'diag': directory/f'{serial}.diag',
            'command': directory/f'{serial}.command'}
