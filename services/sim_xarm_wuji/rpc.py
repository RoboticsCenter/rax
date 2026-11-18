"""The sim server's wire: one JSON object per line over a private Unix socket.

Request {'op': 'domain.name', 'args': {...}}; reply {'ok': true, 'value': ...} or
{'ok': false, 'error': '...'}. Clients keep one connection and serialize calls on it.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import socket
import threading

import numpy as np


def socket_path(root):
    directory = Path(f'/tmp/rax-sim-{os.getuid()}')
    directory.mkdir(mode=0o700, exist_ok=True)
    os.chmod(directory, 0o700)
    return directory/(hashlib.sha256(str(Path(root).resolve()).encode()).hexdigest()[:16] + '.sock')


def encode(value):
    def default(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, (np.bool_,)):
            return bool(obj)
        if isinstance(obj, Path):
            return str(obj)
        raise TypeError(f'not JSON serializable: {type(obj).__name__}')
    return (json.dumps(value, default=default) + '\n').encode()


class SimError(RuntimeError):
    pass


class Client:
    def __init__(self, path, *, timeout=30.0):
        self.path, self.timeout = str(path), timeout
        self.lock = threading.Lock()
        self.sock = self.reader = None

    def _connect(self):
        sock = socket.socket(socket.AF_UNIX)
        sock.settimeout(self.timeout)
        sock.connect(self.path)
        self.sock, self.reader = sock, sock.makefile('rb')

    def call(self, op, **args):
        message = encode({'op': op, 'args': args})
        with self.lock:
            try:
                if self.sock is None:
                    self._connect()
                self.sock.sendall(message)
            except OSError:
                # A stale connection fails on send: the server never saw the request.
                self.close_socket()
                self._connect()
                self.sock.sendall(message)
            try:
                line = self.reader.readline(64 * 1024 * 1024)
            except OSError:
                self.close_socket()
                raise
            if not line:
                self.close_socket()
                raise ConnectionError('the sim server closed the connection')
        reply = json.loads(line)
        if not reply.get('ok'):
            raise SimError(reply.get('error', f'{op} failed'))
        return reply.get('value')

    def close_socket(self):
        for obj in (self.reader, self.sock):
            try:
                if obj is not None:
                    obj.close()
            except OSError:
                pass
        self.sock = self.reader = None

    close = close_socket
