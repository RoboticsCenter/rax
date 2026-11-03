"""Keep operator-supplied context bytes independently of mutable workspace files."""
import hashlib
import json
import mimetypes
import os
from pathlib import Path
import re
import shutil
import tempfile

from services.operator_audit import record

MEDIA = r'(?:mp4|mov|m4v|webm|avi|mkv|gif|png|jpe?g|webp|pdf|wav|mp3|m4a)'
MEDIA_REFERENCE = re.compile(
    rf'"([^"\n]+\.{MEDIA})"|\'([^\'\n]+\.{MEDIA})\'|`([^`\n]+\.{MEDIA})`|'
    rf'(?<![\w:])(@?(?:~?/|\./)?[^\s<>\[\]"\'`]+\.{MEDIA})(?=$|[\s),;:.])', re.I)


def archive_file(root, source, *, source_event_id=None, origin='operator_import'):
    root, source = Path(root), Path(source).expanduser().resolve(strict=True)
    if not source.is_file():
        raise ValueError(f'context is not a regular file: {source}')
    folder = root/'run-logs'/'attachments'
    folder.mkdir(parents=True, exist_ok=True, mode=0o700)
    digest = hashlib.sha256()
    fd, temporary = tempfile.mkstemp(prefix='.copy-', dir=folder)
    temporary = Path(temporary)
    try:
        with source.open('rb') as incoming, os.fdopen(fd, 'wb') as outgoing:
            before = os.fstat(incoming.fileno())
            for chunk in iter(lambda: incoming.read(1024*1024), b''):
                digest.update(chunk)
                outgoing.write(chunk)
            outgoing.flush()
            os.fsync(outgoing.fileno())
            after = os.fstat(incoming.fileno())
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError(f'context changed while copying; retry: {source}')
        sha = digest.hexdigest()
        # Store bytes once per digest. Keep original filename in the journal.
        destination = folder/sha/source.name
        destination.parent.mkdir(exist_ok=True, mode=0o700)
        os.replace(temporary, destination)
        event = record(root, 'attachment_archived', source=origin, actor='operator_context',
                       source_event_id=source_event_id, original_path=str(source),
                       original_name=source.name, archive_path=str(destination.relative_to(root)),
                       sha256=sha, size_bytes=after.st_size, source_mtime_ns=after.st_mtime_ns,
                       media_type=mimetypes.guess_type(source.name)[0],
                       evidence='file preserved; does not prove model inspection')
        if event is None:
            raise OSError('attachment copied but audit journal could not be saved')
        return event
    finally:
        temporary.unlink(missing_ok=True)


def publish_context(root):
    """Publish staged imports only after normal workspace deployment has succeeded."""
    root = Path(root)
    workspace = root/'agent-workspace'
    if not (workspace/'MCP_CONTRACT.json').is_file():
        return
    journal = root/'operator'/'events.jsonl'
    if not journal.exists():
        return
    supplied = {}
    for line in journal.read_text().split('\n'):
        try:
            event = json.loads(line)
        except ValueError:
            continue
        if event.get('event') != 'context_staged':
            continue
        supplied[event['workspace_path']] = event['sha256']
        destination = workspace/event['workspace_path']
        if not destination.resolve().is_relative_to(workspace.resolve()):
            raise ValueError('context destination must stay inside the agent workspace')
        if destination.exists():
            continue
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/event['archive_path'], destination)
    if supplied:
        # Attach a separate file inventory to Claude Code's workspace context.
        # The operator's task text and the inherited model/API route stay unchanged.
        index = workspace/'context'/'INDEX.md'
        claude = workspace/'CLAUDE.md'
        for path in (index, claude):
            if not path.resolve().is_relative_to(workspace.resolve()):
                raise ValueError('context instructions must stay inside the agent workspace')
        index.write_text('# Supplied context files\n\n' + '\n'.join(
            f'- File: `{path}`\n  SHA-256: `{digest}`' for path, digest in supplied.items()) + '\n')
        directive = '@context/INDEX.md'
        if claude.is_file() and directive not in claude.read_text().splitlines():
            with claude.open('a') as stream:
                stream.write('\n'+directive+'\n')
        record(root, 'context_published', source='context_import',
               context_files=[{'workspace_path': path, 'sha256': digest} for path, digest in supplied.items()],
               evidence='file inventory included in workspace context; does not prove video inspection')


def import_context(root, source):
    root = Path(root).resolve()
    if not (root/'operator-input.json').is_file() and not (root/'configuration.json').is_file():
        raise ValueError('choose an existing configured RAX session')
    attachment = archive_file(root, source)
    relative = Path('context')/attachment['sha256']/attachment['original_name']
    staged = record(root, 'context_staged', source='operator_import', actor='operator',
           attachment_id=attachment['id'], archive_path=attachment['archive_path'],
           sha256=attachment['sha256'], workspace_path=str(relative),
           evidence='available to agent after deployment; not yet a submitted instruction')
    if staged is None:
        raise OSError('context copied but staging event could not be saved')
    publish_context(root)
    return str(relative)


class ContextRecorder:
    """Best-effort archival of local media paths in submitted prompts, off the hook path."""
    def __init__(self, root):
        self.root = Path(root)
        self.offset = 0

    def sync(self):
        path = self.root/'operator'/'events.jsonl'
        if not path.exists():
            return
        with path.open('rb') as stream:
            stream.seek(self.offset)
            # Freeze the read boundary: our own archival events must wait for next pass.
            data = stream.read()
        complete = data.rfind(b'\n') + 1
        self.offset += complete
        events = [json.loads(line) for line in data[:complete].splitlines()]
        completed = {e.get('source_event_id') for e in events
                     if e.get('event') == 'context_scan_complete'}
        for event in events:
            if event.get('event') != 'prompt_submitted' or event['id'] in completed:
                continue
            cwd = Path(event.get('hook_payload', {}).get('cwd') or self.root/'agent-workspace')
            paths = set()
            for match in MEDIA_REFERENCE.finditer(event['text']):
                value = next(g for g in match.groups() if g).removeprefix('@')
                if '://' in value:
                    continue  # A URL is retained as text; do not silently download it.
                source = Path(value).expanduser()
                paths.add(source if source.is_absolute() else cwd/source)
            for source in paths:
                try:
                    archive_file(self.root, source, source_event_id=event['id'], origin='prompt_media_reference')
                except (OSError, ValueError) as exc:
                    record(self.root, 'attachment_unavailable', source='prompt_media_reference',
                           source_event_id=event['id'], original_path=str(source), error=str(exc))
            record(self.root, 'context_scan_complete', source='retention_worker',
                   source_event_id=event['id'], media_references=len(paths))
