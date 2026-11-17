import os
import shutil

from services.controller.operator import snapshot_learning
from services.controller.snapshot_storage import deduplicate_snapshots
from services.controller.snapshot_storage import SnapshotCopier


def test_snapshots_share_archives_but_not_workspace(tmp_path):
    source = tmp_path/'agent-workspace/output/result.txt'
    source.parent.mkdir(parents=True)
    source.write_text('first')
    first = snapshot_learning(tmp_path)
    second = snapshot_learning(tmp_path)
    a, b = first/'output/result.txt', second/'output/result.txt'
    assert a.stat().st_ino == b.stat().st_ino != source.stat().st_ino
    stamp = source.stat().st_mtime_ns
    source.write_text('other')  # Same length and mtime must not fool comparison.
    os.utime(source, ns=(stamp, stamp))
    third = snapshot_learning(tmp_path)
    assert (third/'output/result.txt').read_text() == 'other'
    assert a.read_text() == b.read_text() == 'first'
    shutil.rmtree(first)
    assert b.read_text() == 'first'


def test_existing_archives_preserve_content_and_skip_incomplete(tmp_path):
    for name in ('001', '002', '003'):
        folder = tmp_path/name
        folder.mkdir()
        (folder/'data').write_text('saved bytes')
        os.utime(folder/'data', ns=(1000000000, 1000000000))
        if name != '003':
            (folder/'manifest.json').write_text('{}')
    (tmp_path/'002/link').symlink_to(tmp_path/'001/data')
    result = deduplicate_snapshots(tmp_path)
    assert result['files_shared'] == 1
    a, b, c = [tmp_path/name/'data' for name in ('001', '002', '003')]
    assert a.stat().st_ino == b.stat().st_ino != c.stat().st_ino
    assert all(p.read_text() == 'saved bytes' for p in (a, b, c))
    assert (tmp_path/'002/link').is_symlink()
    assert deduplicate_snapshots(tmp_path)['files_shared'] == 0


def test_different_content_or_metadata_is_not_shared(tmp_path):
    for name, content, mode in [('001', 'one', 0o600), ('002', 'two', 0o600),
                                ('003', 'one', 0o644)]:
        folder = tmp_path/name
        folder.mkdir()
        (folder/'manifest.json').write_text('{}')
        (folder/'data').write_text(content)
        (folder/'data').chmod(mode)
        os.utime(folder/'data', ns=(1000000000, 1000000000))
    assert deduplicate_snapshots(tmp_path)['files_shared'] == 0


def test_pruned_previous_file_falls_back_to_independent_copy(tmp_path):
    source = tmp_path/'live'
    source.write_text('keep')
    previous, destination = tmp_path/'previous', tmp_path/'next'
    previous.mkdir()
    destination.mkdir()
    SnapshotCopier(destination, previous)(source, destination/'file')
    assert (destination/'file').read_text() == 'keep'
    assert (destination/'file').stat().st_ino != source.stat().st_ino
