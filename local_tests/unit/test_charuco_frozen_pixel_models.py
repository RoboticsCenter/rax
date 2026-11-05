"""A refit may reuse corrected pixels only with their exact frozen model."""
import runpy
from pathlib import Path

import numpy as np
import pytest


workflow = runpy.run_path(str(Path(__file__).resolve().parents[2] / 'scripts/xarm-bimanual-charuco-calibrate'))


class Driver:
    def state(self):
        return {'tcp_position': np.zeros(3), 'tcp_quaternion': np.array([0., 0., 0., 1.])}


MODEL = {'serial': 'test', 'width': 1280, 'height': 720, 'fps': 6,
         'K': np.eye(3).tolist(), 'distortion_coeffs': [0.] * 5,
         'distortion_model': 'none', 'intrinsics_source': 'charuco_rectified'}


class Camera:
    def __init__(self, model):
        self.model = model

    def request(self, request):
        Path(request['out_dir']).mkdir(parents=True)
        return {'cameras': {'wrist_left': {**self.model, 'rgb': 'rgb.jpg'}}}


def capture(tmp_path, model, frozen):
    spec = {'name': 'wrist_left', 'mount': 'tcp', 'arm': 'left',
            'serial': 'test', 'width': 1280, 'height': 720}
    if frozen:
        spec['frozen_pixel_model'] = MODEL
    return workflow['capture'](Camera(model), tmp_path, [spec],
                               {'left': Driver(), 'right': Driver()})


def test_explicit_frozen_corrected_model_is_preserved(tmp_path):
    _, meta = capture(tmp_path, MODEL, True)
    assert meta['cameras']['wrist_left']['intrinsics_source'] == 'charuco_rectified'


@pytest.mark.parametrize('change', [{'K': (np.eye(3)*2).tolist()},
                                    {'distortion_coeffs': [.1]*5}, {'serial': 'other'}])
def test_frozen_model_mismatch_aborts(tmp_path, change):
    with pytest.raises(ValueError, match='frozen pixel model changed'):
        capture(tmp_path, {**MODEL, **change}, True)


def test_corrected_pixels_still_rejected_by_default(tmp_path):
    with pytest.raises(ValueError, match='raw factory RGB'):
        capture(tmp_path, MODEL, False)
