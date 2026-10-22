"""Operator motion arming for xArm sessions.

This intentionally matches services.franka.arming: the arming contract is a robot-neutral
operator-owned file under the session root.
"""
import json
import os
from pathlib import Path
import time


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


def write(session_root, armed, *, minutes=None, by=None, reason=None):
    path = arm_file(session_root)
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    now = time.time()
    value = {'armed': bool(armed), 'since': now, 'by': by or os.environ.get('USER'),
             'until': now + 60 * minutes if armed and minutes else None,
             'reason': reason}
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value))
    tmp.chmod(0o600)
    os.replace(tmp, path)
    return value
