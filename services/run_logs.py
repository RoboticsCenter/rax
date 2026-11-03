"""Retain raw Claude transcripts alongside the existing MCP/video artifacts."""
import json
from pathlib import Path
import shutil
import threading
import time

from services.json_io import write_private_json
from services.operator_audit import record
from services.context_archive import ContextRecorder


class TranscriptRecorder:
    def __init__(self, root):
        self.root = Path(root)
        self.source = self.root/'claude-home'/'projects'
        self.destination = self.root/'run-logs'/'claude-transcripts'
        self.seen = {}
        self.errors = []
        self.context = ContextRecorder(root)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, name='rax-transcript-retention', daemon=True)

    def start(self):
        self.thread.start()

    def sync(self):
        for source in self.source.rglob('*.jsonl'):
            stat = source.stat()
            relative = source.relative_to(self.source)
            dest = self.destination/relative
            previous = self.seen.get(str(relative))
            identity = (stat.st_dev, stat.st_ino)
            offset = previous[1] if previous and previous[0] == identity else 0
            if stat.st_size < offset:
                offset = 0
            if offset == 0 and dest.exists() and dest.stat().st_size:
                dest.rename(dest.with_name(dest.name+f'.previous-{time.time_ns()}'))
            dest.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            with source.open('rb') as incoming, dest.open('ab') as outgoing:
                incoming.seek(offset)
                shutil.copyfileobj(incoming, outgoing, 1024*1024)
                offset = incoming.tell()
            dest.chmod(0o600)
            self.seen[str(relative)] = (identity, offset)
        self.context.sync()
        write_private_json(self.root/'run-logs'/'retention.json', {
            'updated_at': time.time(), 'files': len(self.seen),
            'bytes': sum(v[1] for v in self.seen.values()), 'errors': self.errors,
            'source': str(self.source), 'destination': str(self.destination)})

    def _run(self):
        while not self.stop.wait(2):
            try:
                self.sync()
            except Exception as exc:
                message = f'{type(exc).__name__}: {exc}'
                if not self.errors or self.errors[-1] != message:
                    self.errors.append(message)
                    record(self.root, 'transcript_retention_error', source='launcher', error=message)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            record(self.root, 'transcript_retention_error', source='launcher', error='retention worker did not stop')
            return
        try:
            self.sync()
        except Exception as exc:
            record(self.root, 'transcript_retention_error', source='launcher', error=str(exc))
