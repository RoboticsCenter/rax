#!/usr/bin/env python3
"""Private log backup, with optional verified archival cleanup. Never controls robots."""
import argparse
import fcntl
import hashlib
import json
import logging
import os
from pathlib import Path
import re
import shutil
import sqlite3
import time

BINARY = {'.jpg', '.jpeg', '.png', '.webp', '.mp4', '.webm', '.npy', '.npz', '.pdf'}
TEXT = {'.json', '.jsonl', '.log', '.md', '.txt', '.py', '.yaml', '.yml', '.csv', '.toml', '.sh', '.html', '.js', '.css', '.svg', '.xml', '.urdf'}
ROOTS = ['operator/requests', 'operator/events.jsonl', 'agent-workspace/memory',
         'agent-workspace/code', 'run-logs', 'motion-archive', 'learning-snapshots',
         'agent-workspace/output', 'progress-report']
BLOCKED = re.compile(r'credential|secret|password|(^|[._-])token([._-]|$)|\.env($|\.)', re.I)
KEYS = re.compile(rb'(?i)(["\'](?:api[_-]?key|access[_-]?token|refresh[_-]?token|authorization|password|client_secret)["\']\s*[:=]\s*["\'])[^"\'\r\n]+')
TOKEN = re.compile(rb'\b(?:sk-[A-Za-z0-9_-]{16,}|hf_[A-Za-z0-9]{16,})\b')
BEARER = re.compile(rb'(?i)\bBearer\s+[A-Za-z0-9._~+/-]{12,}=*')


def allowed(path):
    if any(p.startswith('.') or p in ('__pycache__', 'tmp', 'node_modules') or BLOCKED.search(p) for p in path.parts):
        return False
    return path.suffix.lower() in BINARY | TEXT or '.jsonl.previous-' in path.name


def redact(data, secrets):
    for secret in secrets:
        data = data.replace(secret, b'[REDACTED]')
    data = TOKEN.sub(b'[REDACTED]', data)
    data = BEARER.sub(b'Bearer [REDACTED]', data)
    return KEYS.sub(lambda m: m[1] + b'[REDACTED]', data)


def secrets_from(root, token_path):
    values = {token_path.read_bytes().strip()}
    def walk(value, key=''):
        if isinstance(value, dict):
            for k, v in value.items(): walk(v, k)
        elif isinstance(value, list):
            for v in value: walk(v, key)
        elif isinstance(value, str) and re.search('token|secret|password|api.?key', key, re.I):
            values.add(value.encode())
    credentials = root/'operator/credentials'
    if credentials.exists():
        for p in credentials.iterdir():
            if p.is_file() and not p.is_symlink():
                if p.suffix == '.token': values.add(p.read_bytes().strip())
                elif p.suffix == '.json': walk(json.loads(p.read_text()))
    return sorted((v for v in values if len(v) >= 12), key=len, reverse=True)


def candidates(root):
    # Important small session logs go first, then recordings and snapshot history.
    sources = [p for p in sorted(root.iterdir()) if p.is_file() and
               (p.suffix == '.log' or p.name in ('mcp.jsonl', 'model-usage.jsonl', 'model-calls.json', 'agent-lifecycle.json'))]
    sources += [root/p for p in ROOTS]
    for source in sources:
        if source.is_symlink(): continue
        if source.is_file():
            if allowed(source.relative_to(root)): yield source
            continue
        for base, dirs, files in os.walk(source, followlinks=False):
            dirs[:] = sorted(d for d in dirs if not (Path(base)/d).is_symlink() and
                             not d.startswith('.') and d not in ('__pycache__', 'tmp', 'node_modules') and not BLOCKED.search(d))
            for name in sorted(files):
                p = Path(base)/name
                if not p.is_symlink() and allowed(p.relative_to(root)): yield p


def signature(stat):
    return f'{stat.st_dev}:{stat.st_ino}:{stat.st_size}:{stat.st_mtime_ns}:{stat.st_ctime_ns}'


def stage(source, dest, secrets):
    dest.parent.mkdir(parents=True, exist_ok=True)
    if source.suffix.lower() in BINARY:
        shutil.copyfile(source, dest)
    else:
        with source.open('rb') as src, dest.open('wb') as out:
            for line in src:
                out.write(redact(line, secrets))
    size = dest.stat().st_size
    sha = hashlib.sha256()
    git = hashlib.sha1(f'blob {size}\0'.encode())
    with dest.open('rb') as src:
        for chunk in iter(lambda: src.read(1024*1024), b''):
            sha.update(chunk); git.update(chunk)
    return size, sha.hexdigest(), git.hexdigest()


def verify_remote(api, repo, revision, batch):
    found = {}
    for i in range(0, len(batch), 100):
        for item in api.get_paths_info(repo, [x['remote'] for x in batch[i:i+100]],
                                       repo_type='dataset', revision=revision):
            found[item.path] = item
    for row in batch:
        item = found.get(row['remote'])
        if item is None or item.size != row['size']:
            raise RuntimeError(f'Remote size/missing-file verification failed: {row["remote"]}')
        lfs = getattr(item, 'lfs', None)
        actual = (lfs.get('sha256') if isinstance(lfs, dict) else getattr(lfs, 'sha256', None)) if lfs else item.blob_id
        if actual != row['sha256' if lfs else 'git_sha1']:
            raise RuntimeError(f'Remote hash verification failed: {row["remote"]}')


def history_candidates(root, now=None):
    """Never include live workspace, current transcripts, configs, or credentials."""
    now = time.time() if now is None else now
    snapshots = root/'learning-snapshots'
    newest = sorted(p.name for p in snapshots.iterdir() if p.is_dir() and not p.is_symlink())[-2:] if snapshots.exists() else []
    for p in candidates(root):
        rel = p.relative_to(root)
        if len(rel.parts) < 3 or p.name == 'manifest.json': continue
        if rel.parts[0] == 'learning-snapshots':
            if rel.parts[1] in newest: continue
            manifest = root/rel.parts[0]/rel.parts[1]/'manifest.json'
        elif rel.parts[0] == 'motion-archive':
            manifest = root/rel.parts[0]/rel.parts[1]/'manifest.json'
        elif rel.parts[:2] == ('run-logs', 'claude-transcripts') and '.jsonl.previous-' in p.name:
            manifest = None
        else: continue
        try:
            if now-p.stat().st_mtime < 3600: continue
            if manifest is not None:
                if manifest.is_symlink() or now-manifest.stat().st_mtime < 3600: continue
                meta = json.loads(manifest.read_text())
                if rel.parts[0] == 'motion-archive' and (not meta.get('end_ms') or meta.get('status') == 'recording'): continue
            yield p
        except (OSError, ValueError):
            continue


def prune_history(args, api, db):
    """Recheck remote HEAD and exact local identity before unlinking archival files."""
    info = api.repo_info(args.repo, repo_type='dataset')
    if not info.private: raise RuntimeError('Cleanup requires the private backup repository')
    revision = info.sha
    batch = []
    removed_files = removed_bytes = 0
    def remove_batch():
        nonlocal removed_files, removed_bytes
        if not batch: return
        verify_remote(api, args.repo, revision, batch)
        # A durable journal records intent before each removal, including its remote hash.
        with (args.state/'cleanup.jsonl').open('a') as audit:
            for row in batch:
                p = args.session/row['local']
                try:
                    if p.is_symlink() or p.resolve() != p.absolute() or signature(p.stat()) != row['signature']: continue
                    record = {k: row[k] for k in ('local', 'signature', 'remote', 'size', 'sha256')}
                    record.update(revision=revision, time=time.time(), action='unlink_verified_history')
                    audit.write(json.dumps(record)+'\n');audit.flush();os.fsync(audit.fileno())
                    size = p.stat().st_size
                    p.unlink()
                    removed_files += 1;removed_bytes += size
                except FileNotFoundError: continue
        batch.clear()
    for p in history_candidates(args.session):
        rel = p.relative_to(args.session).as_posix()
        row = db.execute('SELECT signature,size,sha256,git_sha1 FROM uploaded WHERE path=?', (rel,)).fetchone()
        if row is None or not row[3]: continue  # Legacy entries must remain until fully hash-indexed.
        try:
            if signature(p.stat()) != row[0]: continue
        except FileNotFoundError: continue
        batch.append(dict(local=rel, remote=f'sessions/{args.session.name}/{rel}',
                          signature=row[0], size=row[1], sha256=row[2], git_sha1=row[3]))
        if len(batch) >= 100: remove_batch()
    remove_batch()
    status = {'removed_files': removed_files, 'removed_bytes': removed_bytes, 'time': time.time(),
              'verified_remote_revision': revision, 'preserved': 'live session, newest two snapshots, archive manifests'}
    (args.state/'cleanup-status.json').write_text(json.dumps(status, indent=2))
    logging.info('Archival cleanup removed %d verified files / %.2f GiB; live session preserved', removed_files, removed_bytes/2**30)


def sync(args):
    from huggingface_hub import HfApi, CommitOperationAdd
    token = args.token_file.read_text().strip()
    api = HfApi(token=token)
    if not api.repo_info(args.repo, repo_type='dataset').private:
        raise RuntimeError('Refusing to upload session logs to a public dataset')
    secrets = secrets_from(args.session, args.token_file)
    db = sqlite3.connect(args.state/'uploaded.sqlite3')
    db.execute('CREATE TABLE IF NOT EXISTS uploaded (path TEXT PRIMARY KEY, signature TEXT, size INTEGER, sha256 TEXT, revision TEXT, verified REAL)')
    if 'git_sha1' not in [r[1] for r in db.execute('PRAGMA table_info(uploaded)')]:
        db.execute('ALTER TABLE uploaded ADD COLUMN git_sha1 TEXT');db.commit()
    identity = {'repo': args.repo, 'session': str(args.session)}
    identity_path = args.state/'identity.json'
    if identity_path.exists() and json.loads(identity_path.read_text()) != identity:
        raise RuntimeError('State directory belongs to another session or repository')
    if not identity_path.exists():
        temporary = identity_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(identity))
        temporary.replace(identity_path)
    staging_root = args.staging_root or args.state
    staging_root.mkdir(parents=True, exist_ok=True, mode=0o700)
    staging = staging_root/'staging'
    # This directory contains only disposable export copies.
    if staging.exists(): shutil.rmtree(staging)
    staging.mkdir()
    batch, total, commits = [], 0, 0
    def commit():
        nonlocal batch, total, commits
        if not batch: return
        logging.info('Uploading %d files (%.1f MiB)', len(batch), total/2**20)
        info = api.create_commit(args.repo, repo_type='dataset', num_threads=2,
            commit_message=f'Back up {args.session.name}: {len(batch)} files',
            operations=[CommitOperationAdd(path_in_repo=r['remote'], path_or_fileobj=r['staged']) for r in batch])
        verify_remote(api, args.repo, info.oid, batch)
        with db:
            for row in batch:
                db.execute('INSERT OR REPLACE INTO uploaded (path,signature,size,sha256,revision,verified,git_sha1) VALUES (?,?,?,?,?,?,?)',
                           (row['local'], row['signature'], row['size'], row['sha256'], info.oid, time.time(), row['git_sha1']))
        count, size = db.execute('SELECT COUNT(*), COALESCE(SUM(size),0) FROM uploaded').fetchone()
        status = {'verified_files': count, 'verified_bytes': size, 'last_commit': info.oid, 'updated_at': time.time(), 'repo': args.repo}
        temp = args.state/'status.tmp'; temp.write_text(json.dumps(status, indent=2));temp.replace(args.state/'status.json')
        logging.info('Verified commit %s; total %d files / %.2f GiB', info.oid, count, size/2**30)
        shutil.rmtree(staging); staging.mkdir(); batch, total = [], 0; commits += 1
    try:
        if args.prune_during_upload:
            prune_history(args, api, db)
        fully_scanned, changed = False, False
        for p in candidates(args.session):
            try:
                st = p.stat(); sig = signature(st); rel = p.relative_to(args.session).as_posix()
                if time.time()-st.st_mtime < args.settle_seconds: continue
                previous = db.execute('SELECT signature,git_sha1 FROM uploaded WHERE path=?', (rel,)).fetchone()
                if previous and previous[0] == sig and previous[1]: continue
                if batch and (total+st.st_size > args.batch_mib*2**20 or len(batch) >= 100):
                    commit()
                    if args.max_batches and commits >= args.max_batches: break
                if shutil.disk_usage(staging_root).free < st.st_size + 1024**3:
                    raise RuntimeError('Less than 1 GiB would remain after staging; upload paused')
                if shutil.disk_usage(args.state).free < 64*1024**2:
                    raise RuntimeError('Less than 64 MiB remains for the verification ledger; upload paused')
                dest = staging/rel
                size, sha, git = stage(p, dest, secrets)
                if signature(p.stat()) != sig:
                    changed = True; dest.unlink(); continue  # Wait for a stable completed version.
                batch.append({'local': rel, 'remote': f'sessions/{args.session.name}/{rel}', 'signature': sig,
                              'staged': str(dest), 'size': size, 'sha256': sha, 'git_sha1': git})
                total += size
            except FileNotFoundError:
                continue  # Rotating files can disappear while scanning.
        else:
            fully_scanned = True
        commit()
        if fully_scanned and not changed:
            (args.state/'initial-backlog-complete.json').write_text(json.dumps({
                'time': time.time(), 'scope': 'all eligible settled files in the configured session export'}))
        if args.prune_during_upload or (fully_scanned and not changed and args.prune_verified_history):
            prune_history(args, api, db)
    finally:
        db.close()
        if staging.exists(): shutil.rmtree(staging)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--repo', required=True)
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--state', type=Path, required=True)
    parser.add_argument('--staging-root', type=Path,
                        help='temporary export location, e.g. a dedicated tmpfs directory')
    parser.add_argument('--settle-seconds', type=int, default=120)
    parser.add_argument('--batch-mib', type=int, default=128)
    parser.add_argument('--max-batches', type=int, default=0)
    parser.add_argument('--prune-verified-history', action='store_true',
                        help='after a full upload pass, remove verified old archives; preserve live session')
    parser.add_argument('--prune-during-upload', action='store_true',
                        help='remove verified old archives before and after each upload pass, even during backlog')
    args = parser.parse_args()
    os.umask(0o077)
    args.session = args.session.resolve(); args.state = args.state.resolve()
    args.state.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')
    logging.getLogger('httpx').setLevel(logging.WARNING)
    logging.getLogger('httpx2').setLevel(logging.WARNING)
    with (args.state/'lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        sync(args)


if __name__ == '__main__':
    main()
