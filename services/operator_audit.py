"""Durable operator wording and action history, separate from robot control."""
from datetime import datetime, timezone
import fcntl
from functools import wraps
import hashlib
import inspect
import json
import os
from pathlib import Path
import sys
import time
import uuid


def record(root, event, *, source, text=None, at_ns=None, **details):
    """Append without trimming text. Logging failure must never prevent a stop."""
    if root is None:
        return None
    now = time.time_ns() if at_ns is None else at_ns
    value = {'id': uuid.uuid4().hex, 'event': event, 'source': source,
             'time_ns': now, 'time': now/1e9,
             'utc': datetime.fromtimestamp(now/1e9, timezone.utc).isoformat(),
             'recorded_at_ns': time.time_ns(), **details}
    if text is not None:
        if not isinstance(text, str):
            raise TypeError('operator wording must be text')
        value.update(text=text, text_sha256=hashlib.sha256(text.encode('utf-8')).hexdigest())
    path = Path(root)/'operator'/'events.jsonl'
    try:
        path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        raw = (json.dumps(value, ensure_ascii=False, default=str)+'\n').encode('utf-8')
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        with os.fdopen(fd, 'ab') as out:
            fcntl.flock(out, fcntl.LOCK_EX)
            out.write(raw)
            out.flush()
            os.fsync(out.fileno())
        return value
    except OSError as exc:
        print(f'Operator audit could not be saved to {path}: {exc}', file=sys.stderr, flush=True)
        return None


def action(name, source):
    """Record operator lifecycle outcomes after control actions have completed."""
    def decorate(function):
        signature = inspect.signature(function)
        @wraps(function)
        def run(*args, **kwargs):
            bound = signature.bind(*args, **kwargs).arguments
            root = bound.get('root')
            started = time.time_ns()
            outcome, error = 'completed', None
            result = None
            try:
                result = function(*args, **kwargs)
                return result
            except BaseException as exc:
                outcome, error = 'failed', f'{type(exc).__name__}: {exc}'
                raise
            finally:
                record(root, name, source=source, at_ns=started, outcome=outcome,
                       completed_at_ns=time.time_ns(), error=error,
                       result=result,
                       parameters={k: v for k, v in bound.items() if k in ('minutes', 'confirmed', 'side')})
        return run
    return decorate
