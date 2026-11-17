import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('hf_log_sync', Path(__file__).parents[2]/'scripts/sync_hf_logs.py')
sync = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sync)


def setup_archive(tmp_path):
    root = tmp_path/'session';root.mkdir()
    state = tmp_path/'state';state.mkdir()
    paths = ['learning-snapshots/001/output/old.txt', 'learning-snapshots/002/output/new.txt',
             'learning-snapshots/003/output/newest.txt', 'motion-archive/old/video.mp4',
             'run-logs/claude-transcripts/run.jsonl.previous-1', 'run-logs/claude-transcripts/run.jsonl',
             'agent-workspace/code/main.py', 'operator/credentials/auth.token', 'configuration.json',
             'claude-home/projects/current.jsonl']
    for name in paths:
        p = root/name;p.parent.mkdir(parents=True, exist_ok=True);p.write_text('retained content')
        os.utime(p, (1, 1))
    for name in ['learning-snapshots/001','learning-snapshots/002','learning-snapshots/003','motion-archive/old']:
        p=root/name/'manifest.json';p.write_text(json.dumps({'status':'completed','end_ms':1}));os.utime(p,(1,1))
    args = SimpleNamespace(session=root, state=state, repo='owner/private')
    db=sqlite3.connect(':memory:')
    db.execute('CREATE TABLE uploaded (path TEXT, signature TEXT, size INTEGER, sha256 TEXT, git_sha1 TEXT)')
    items={}
    for name in paths:
        p=root/name;data=p.read_bytes();sha=hashlib.sha256(data).hexdigest();git=hashlib.sha1(f'blob {len(data)}\0'.encode()+data).hexdigest()
        db.execute('INSERT INTO uploaded VALUES (?,?,?,?,?)',(name,sync.signature(p.stat()),len(data),sha,git))
        remote=f'sessions/session/{name}';items[remote]=SimpleNamespace(path=remote,size=len(data),lfs=None,blob_id=git)
    api=SimpleNamespace(repo_info=lambda *a,**kw:SimpleNamespace(private=True,sha='verified-commit'),
                        get_paths_info=lambda repo,paths,**kw:[items[p] for p in paths])
    return args, db, api, items


def test_cleanup_preserves_resumable_session_and_latest_snapshots(tmp_path):
    args,db,api,_=setup_archive(tmp_path)
    sync.prune_history(args,api,db)
    assert not (args.session/'learning-snapshots/001/output/old.txt').exists()
    assert not (args.session/'motion-archive/old/video.mp4').exists()
    assert not (args.session/'run-logs/claude-transcripts/run.jsonl.previous-1').exists()
    for name in ['learning-snapshots/001/manifest.json','learning-snapshots/002/output/new.txt',
                 'learning-snapshots/003/output/newest.txt','run-logs/claude-transcripts/run.jsonl',
                 'agent-workspace/code/main.py','operator/credentials/auth.token','configuration.json',
                 'claude-home/projects/current.jsonl']:
        assert (args.session/name).exists(),name


def test_remote_hash_mismatch_prevents_cleanup(tmp_path):
    args,db,api,items=setup_archive(tmp_path)
    for item in items.values():item.blob_id='wrong'
    with pytest.raises(RuntimeError,match='hash verification'):
        sync.prune_history(args,api,db)
    assert (args.session/'learning-snapshots/001/output/old.txt').exists()
    assert (args.session/'motion-archive/old/video.mp4').exists()


def test_changed_and_unverified_files_remain(tmp_path):
    args,db,api,_=setup_archive(tmp_path)
    p=args.session/'learning-snapshots/001/output/old.txt';p.write_text('changed');os.utime(p,(1,1))
    db.execute("DELETE FROM uploaded WHERE path='motion-archive/old/video.mp4'")
    sync.prune_history(args,api,db)
    assert p.exists()
    assert (args.session/'motion-archive/old/video.mp4').exists()


def test_sensitive_paths_and_symlinks_excluded(tmp_path):
    args,_,_,_=setup_archive(tmp_path)
    p=args.session/'learning-snapshots/001/output/link.txt';p.symlink_to(args.session/'configuration.json')
    assert p not in list(sync.history_candidates(args.session))
    for name in ['hf.token','.hf.token.swp','operator/credentials/auth.json','.env']:
        assert not sync.allowed(Path(name))
    secret=b'hf_'+b'a'*30
    out=sync.redact(b'{"api_key":"secret-value","token":"'+secret+b'"}',[])
    assert b'secret-value' not in out and secret not in out
