"""Share immutable archive copies; never hard-link the mutable workspace."""
import hashlib
import os
from pathlib import Path
import shutil
import stat
import uuid


def identity(path):
    s = path.stat()
    return (s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns)


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.digest()


def metadata(path):
    s = path.stat()
    return (s.st_size, stat.S_IMODE(s.st_mode), s.st_mtime_ns)


class SnapshotCopier:
    def __init__(self, destination, previous=None):
        self.destination = Path(destination)
        self.previous = Path(previous) if previous else None

    def __call__(self, source, destination):
        source, destination = Path(source), Path(destination)
        old = self.previous / destination.relative_to(self.destination) if self.previous else None
        # Files are matched by bytes as well as metadata: equal mtime/size is
        # insufficient. The workspace is only read, never linked to an archive.
        if old and old.is_file() and not old.is_symlink() and not source.is_symlink():
            try:
                before, old_before = identity(source), identity(old)
                if (metadata(source) == metadata(old) and digest(source) == digest(old)
                        and identity(source) == before and identity(old) == old_before):
                    os.link(old, destination)
                    return str(destination)
            except OSError:
                pass  # Missing/pruned prior archive or unsupported links: copy.
        return shutil.copy2(source, destination)


def deduplicate_snapshots(root):
    """Atomically share equal files in completed archives, preserving metadata.

    No workspace files, manifests, symlinks or incomplete snapshots are changed.
    Removing any archive later is safe: remaining hard links retain the bytes.
    """
    root = Path(root)
    canonical, hashed = {}, {}
    count = saved = 0
    for folder in sorted(root.iterdir()):
        if folder.is_symlink() or not (folder / 'manifest.json').is_file():
            continue
        for base, dirs, files in os.walk(folder, followlinks=False):
            dirs[:] = sorted(d for d in dirs if not (Path(base) / d).is_symlink())
            for name in sorted(files):
                p = Path(base) / name
                s = p.lstat()
                if name == 'manifest.json' or not stat.S_ISREG(s.st_mode) or not s.st_size:
                    continue
                before = identity(p)
                inode = (s.st_dev, s.st_ino)
                content = hashed.get(before)
                if content is None:
                    content = digest(p)
                    if identity(p) != before:
                        continue
                    hashed[before] = content
                key = (metadata(p), content)
                other = canonical.get(key)
                if other is None:
                    canonical[key] = p
                    continue
                if identity(other)[:2] == inode:
                    continue
                other_before = identity(other)
                # Revalidate canonical content if its identity changed since hashing.
                if hashed.get(other_before) != content:
                    if digest(other) != content or identity(other) != other_before:
                        canonical[key] = p
                        continue
                temporary = p.with_name(f'.dedup-{uuid.uuid4().hex}')
                try:
                    os.link(other, temporary)
                    if identity(p) != before:
                        continue
                    os.replace(temporary, p)
                    count += 1
                    if s.st_nlink == 1:
                        saved += s.st_blocks * 512
                    hashed[identity(other)] = content
                finally:
                    temporary.unlink(missing_ok=True)
    return {'files_shared': count, 'bytes_freed': saved}
