"""Operator motion arming for xArm sessions.

This intentionally matches services.franka.arming: the arming contract is a robot-neutral
operator-owned file under the session root.
"""
import json
import os
from pathlib import Path
import time

from services.json_io import write_private_json


def arm_file(session_root):
    return Path(session_root)/'operator'/'armed.json'


def read(session_root, now=None):
    """{'armed': bool, ...}; expired or unreadable arming counts as disarmed."""
    path = arm_file(session_root)
    try:
        value = json.loads(path.read_text())
    except (OSError, ValueError):
        return {'armed': False, 'reason': 'not armed by the operator'}
    now = time.time() if now is None else now
    until = value.get('until')
    if value.get('armed') is not True:
        return {'armed': False, 'reason': value.get('reason') or 'disarmed by the operator'}
    if until is not None and now >= until:
        return {'armed': False, 'reason': 'operator arming expired'}
    return {'armed': True, 'until': until, 'by': value.get('by'), 'since': value.get('since')}


class CachedReader:
    """read() for a high-rate control loop: re-parse only when the file changes.

    Arming writes replace the file atomically (new inode), so one stat() per call detects
    every arm/disarm. Expiry is evaluated against the clock on every call, and the file is
    re-read at least every ``refresh_s`` as a backstop.
    """
    def __init__(self, session_root, refresh_s=.1):
        self.path, self.refresh_s = arm_file(session_root), refresh_s
        self.cache = None  # (key, value, loaded): replaced as one object, safe across threads.

    def __call__(self, now=None):
        try:
            stat = os.stat(self.path)
            key = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
        except OSError:
            self.cache = None
            return {'armed': False, 'reason': 'not armed by the operator'}
        monotonic, cache = time.monotonic(), self.cache
        if cache is None or cache[0] != key or monotonic-cache[2] > self.refresh_s:
            try:
                value = json.loads(self.path.read_text())
                if not isinstance(value, dict):
                    raise ValueError('arming file must hold an object')
            except (OSError, ValueError):
                self.cache = None
                return {'armed': False, 'reason': 'not armed by the operator'}
            cache = self.cache = (key, value, monotonic)
        value = cache[1]
        now = time.time() if now is None else now
        until = value.get('until')
        if value.get('armed') is not True:
            return {'armed': False, 'reason': value.get('reason') or 'disarmed by the operator'}
        if until is not None and now >= until:
            return {'armed': False, 'reason': 'operator arming expired'}
        return {'armed': True, 'until': until, 'by': value.get('by'), 'since': value.get('since')}


def write(session_root, armed, *, minutes=None, by=None, reason=None):
    path = arm_file(session_root)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    now = time.time()
    value = {'armed': bool(armed), 'since': now, 'by': by or os.environ.get('USER'),
             'until': now + 60 * minutes if armed and minutes else None,
             'reason': reason}
    # Operator and fault-monitor processes may revoke arming concurrently.
    write_private_json(path, value)
    return value
