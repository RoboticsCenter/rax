"""Agent -> operator requests (help, scene reset) answered from the dashboard or `xarm reply`.

Files live in the session root, outside the agent's writable workspace:

    <session>/operator/requests/<id>.json         written by the robot service
    <session>/operator/requests/<id>.reply.json   written by the operator tools

The `request_operator` tool blocks until the reply file appears or the wait times out;
the robot does nothing meanwhile (the arm holds its last pose).
"""
import json
import os
from pathlib import Path
import re
import shutil
import time
import uuid

KINDS = ('help', 'reset')
REQUEST_ID = re.compile(r'[0-9a-f]{12}')


def directory(session_root):
    return Path(session_root)/'operator'/'requests'


def _write(path, value):
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}')
    temporary.write_text(json.dumps(value, indent=1))
    temporary.chmod(0o600)
    os.replace(temporary, path)


def _read(path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def snapshot_learning(session_root, label='manual', *, request=None, reply=None):
    """Archive session-local agent learning outside the agent's writable workspace.

    This captures the evolving memory/code/output plus the copied skill instructions. Large
    runtime streams, frame captures, motion videos, tmp files and pyc caches are intentionally
    omitted; those already live elsewhere in the session artifacts.
    """
    root = Path(session_root)
    workspace = root/'agent-workspace'
    if not workspace.is_dir():
        return None
    safe = ''.join(c if c.isalnum() or c in '-_.' else '_' for c in str(label))[:80] or 'snapshot'
    folder = root/'learning-snapshots'
    folder.mkdir(mode=0o700, exist_ok=True)
    stamp = time.strftime('%Y%m%dT%H%M%S', time.localtime())
    dest = folder/f'{stamp}_{safe}'
    suffix = 1
    while dest.exists():
        suffix += 1
        dest = folder/f'{stamp}_{safe}_{suffix}'
    dest.mkdir(mode=0o700)

    ignore = shutil.ignore_patterns('__pycache__', '*.pyc', 'runtime', 'tmp')
    copied = []
    for name in ('memory', 'code', 'output'):
        source = workspace/name
        if source.exists():
            shutil.copytree(source, dest/name, ignore=ignore)
            copied.append(name)
    skills = workspace/'.agents'/'skills'
    if skills.exists():
        shutil.copytree(skills, dest/'skills', ignore=ignore)
        copied.append('skills')
    for name in ('TASK.md', 'AGENTS.md', 'MCP_SESSION.md', 'MCP_CONTRACT.json', 'START_PROMPT.md'):
        source = workspace/name
        if source.is_file():
            shutil.copy2(source, dest/name)
            copied.append(name)
    manifest = {
        'time': time.time(),
        'session': root.name,
        'source_workspace': str(workspace),
        'label': label,
        'copied': copied,
        'request': request,
        'reply': reply,
        'note': 'Session-local learning snapshot: memory, code, output, copied skills, and task/contract files. '
                'Runtime frames, streams, videos, tmp files and pyc caches are intentionally omitted.',
    }
    (dest/'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return dest


def validate_request(kind, message):
    if kind not in KINDS:
        raise ValueError(f'kind must be one of {KINDS}')
    if not isinstance(message, str) or not message.strip() or len(message) > 4000:
        raise ValueError('message: 1..4000 characters')


def open_request(session_root, kind, message, *, source=None):
    validate_request(kind, message)
    request = {'id': uuid.uuid4().hex[:12], 'kind': kind, 'message': message.strip(), 'created': time.time(),
               'status': 'pending'}
    if source is not None:
        request['source'] = source
    _write(directory(session_root)/f"{request['id']}.json", request)
    return request


def fault_request(session_root, source, message):
    """Notify without blocking the failing tool or opening duplicate fault alerts."""
    for request in pending(session_root):
        if request.get('source') == source:
            return request
    return open_request(session_root, 'help', message, source=source)


def reply(session_root, request_id, text='', *, done=True, by=None):
    """Answer a pending request (operator side)."""
    if not REQUEST_ID.fullmatch(str(request_id)):
        raise ValueError('invalid request id')
    path = directory(session_root)/f'{request_id}.json'
    request = _read(path)
    if request is None:
        raise ValueError(f'unknown request {request_id}')
    if request.get('status') != 'pending':
        raise ValueError(f"request {request_id} is {request.get('status')}, not pending")
    answer = {'reply': str(text)[:4000], 'done': bool(done), 'by': by or os.environ.get('USER'), 'time': time.time()}
    _write(directory(session_root)/f'{request_id}.reply.json', answer)
    try:
        snapshot = snapshot_learning(session_root, f"{request.get('kind', 'request')}_{request_id}",
                                     request=request, reply=answer['reply'])
        if snapshot is not None:
            answer['learning_snapshot'] = str(snapshot)
    except Exception as exc:
        answer['learning_snapshot_error'] = f'{type(exc).__name__}: {exc}'
    return answer


def pending(session_root):
    folder = directory(session_root)
    requests = [_read(p) for p in sorted(folder.glob('*.json')) if not p.name.endswith('.reply.json')] \
        if folder.is_dir() else []
    return [r for r in requests if r and r.get('status') == 'pending'
            and not (folder/f"{r['id']}.reply.json").exists()]


def wait(session_root, request, timeout_s, *, poll_s=1.0, clock=time.monotonic, sleep=time.sleep):
    """Block until the operator answers; returns the tool result."""
    folder = directory(session_root)
    started = clock()
    while clock() - started < timeout_s:
        answer = _read(folder/f"{request['id']}.reply.json")
        if answer is not None:
            _write(folder/f"{request['id']}.json", {**request, 'status': 'answered', 'answered': answer['time']})
            return {'status': 'answered', 'reply': answer['reply'], 'done': answer['done'],
                    'waited_s': round(clock() - started, 1)}
        sleep(poll_s)
    _write(folder/f"{request['id']}.json", {**request, 'status': 'expired'})
    return {'status': 'timeout', 'reply': None, 'waited_s': round(clock() - started, 1),
            'note': 'No operator answer in time. Continue on your own or ask again later.'}
