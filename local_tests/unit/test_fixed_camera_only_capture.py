import io
import json
from pathlib import Path
import runpy
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[2]/'scripts/xarm-bimanual-fixed-camera-calibrate'


def test_fixed_capture_uses_frozen_geometry_and_does_not_invent_arm_measurements(tmp_path, monkeypatch):
    module = runpy.run_path(str(SCRIPT))
    sent = []
    class Connection:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def settimeout(self, value): pass
        def connect(self, path): pass
        def sendall(self, data):
            sent.append(json.loads(data))
            Path(sent[-1]['out_dir']).mkdir(parents=True)
        def makefile(self, mode):
            return io.BytesIO(json.dumps({'ok': True, 'cameras': {'side_a': {'rgb': 'side_a.jpg'}}}).encode())
    monkeypatch.setattr(module['socket'], 'socket', lambda *args: Connection())
    cal = SimpleNamespace(digest='frozen-hash', manifest={'camera_specs': [{'name': 'side_a', 'mount': 'fixed'}]},
                          poses=lambda spec, states: {'left': np.eye(4)})
    folder = module['observe_fixed'](tmp_path, '/unused-camera-socket', ['side_a'], cal)
    meta = json.loads((folder/'meta.json').read_text())
    assert sent[0]['op'] == 'grab' and sent[0]['depth'] is False
    assert meta['fixed_camera_only'] and meta['calibration']['sha256'] == 'frozen-hash'
    assert meta['cameras']['side_a']['T_left_base_cam'] == np.eye(4).tolist()
    assert 'arms' not in meta and 'arms_before' not in meta


def test_camera_only_mode_refuses_wrist_views_without_arm_telemetry(tmp_path):
    module = runpy.run_path(str(SCRIPT))
    cal = SimpleNamespace(digest='hash', manifest={'camera_specs': [{'name': 'wrist_left', 'mount': 'tcp'}]})
    with pytest.raises(ValueError, match='fixed cameras exclusively'):
        module['observe_fixed'](tmp_path, '/no-socket', ['wrist_left'], cal)
    (tmp_path/'meta.json').write_text(json.dumps({
        'calibration': {'sha256': 'hash'}, 'fixed_camera_only': True,
        'cameras': {'wrist_left': {}}}))
    with pytest.raises(ValueError, match='cannot contain wrist cameras'):
        module['detect'](tmp_path, cal, None, None, {}, None)
