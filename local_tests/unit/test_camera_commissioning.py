"""Calibration contracts and geometry without hardware access."""
import json
import runpy
from pathlib import Path
from types import SimpleNamespace

import cv2 as cv
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from services.xarm import charuco
from services.xarm.calibration import (
    ColorRectification,
    DepthRegistration,
    depth_to_color_transform,
    validate_intrinsics,
)
from services.xarm.charuco import detect_board, distortion, make_board

PROJECT = Path(__file__).resolve().parents[2]


def calibration_script():
    return runpy.run_path(str(PROJECT/'scripts/xarm-charuco-calibrate'))


def test_cropped_legacy_board_preserves_marker_coordinates():
    settings = {'dictionary': 'DICT_4X4_100', 'squares_x': 10, 'squares_y': 13,
                'layout_squares_y': 14, 'square_m': .02, 'marker_m': .015, 'legacy_pattern': True}
    board, detector = make_board(settings)
    original = board.generateImage((600, 840), marginSize=0)
    cropped = cv.copyMakeBorder(original[:780], 25, 25, 25, 25, cv.BORDER_CONSTANT, value=255)
    corners, ids, _, _ = detect_board(cropped, board, detector, settings)
    assert corners is not None and len(ids) >= 90
    object_points = np.asarray(board.getChessboardCorners())[ids.ravel()]
    assert np.max(object_points[:, 1]) < settings['squares_y']*settings['square_m']
    # A regenerated odd-row board changes checker parity and cannot match these IDs.
    wrong, wrong_detector = make_board({**settings, 'layout_squares_y': 13})
    _, wrong_ids, _, _ = detect_board(cropped, wrong, wrong_detector, settings)
    assert wrong_ids is None or len(wrong_ids) < 10


def test_factory_reference_rejects_wrong_device_profile_or_values(tmp_path):
    profile = {'width': 640, 'height': 480, 'fps': 6, 'K': [[600., 0., 320.], [0., 600., 240.], [0., 0., 1.]],
               'distortion_model': 'inverse_brown_conrady', 'distortion_coeffs': [0.]*5}
    reference = tmp_path/'intrinsics.json'
    reference.write_text(json.dumps({'serial': 'abc', 'color_profiles': {'640x480@6': profile}}))
    spec = {'name': 'wrist', 'serial': 'abc', 'intrinsics': str(reference)}
    validate_intrinsics(spec, profile)
    with pytest.raises(ValueError, match='serial'):
        validate_intrinsics({**spec, 'serial': 'other'}, profile)
    with pytest.raises(ValueError, match='no recorded'):
        validate_intrinsics(spec, {**profile, 'width': 1280})
    with pytest.raises(ValueError, match='live K'):
        validate_intrinsics(spec, {**profile, 'K': np.eye(3).tolist()})
    with pytest.raises(ValueError, match='distortion model'):
        validate_intrinsics(spec, {**profile, 'distortion_model': 'brown_conrady'})


def test_nonzero_inverse_distortion_is_not_silently_treated_as_brown():
    assert np.all(distortion({'distortion_model': 'inverse_brown_conrady', 'distortion_coeffs': [0.]*5}) == 0)
    with pytest.raises(ValueError, match='unsupported'):
        distortion({'distortion_model': 'inverse_brown_conrady', 'distortion_coeffs': [.1, 0, 0, 0, 0]})


def test_rectification_preserves_camera_rays():
    K = np.array([[40., 0., 32.], [0., 42., 24.], [0., 0., 1.]])
    D = np.array([.2, -.05, .01, -.01, 0.])
    live = {'K': K.tolist(), 'width': 64, 'height': 48, 'fps': 6}
    fitted = {'K': K.tolist(), 'distortion_coeffs': D.tolist(), 'distortion_model': 'brown_conrady'}
    model = ColorRectification({'calibrated_color_profiles': {'64x48@6': fitted}}, live)
    y, x = np.indices((48, 64), dtype=np.float32)
    source = np.stack([x, y, x*0+1], axis=-1)
    color = model.apply(source)
    # Independent forward projection of desired pinhole rays locates the source
    # pixel each output must represent; intrinsic metadata must describe those rays.
    pixels = np.array([[10., 10.], [40., 30.], [24., 17.]])
    rays = np.c_[(pixels[:, 0]-32)/40, (pixels[:, 1]-24)/42, np.ones(3)]
    projected, _ = cv.projectPoints(rays, np.zeros(3), np.zeros(3), K, D)
    for pixel, expected in zip(pixels.astype(int), projected.reshape(-1, 2)):
        u, v = pixel
        np.testing.assert_allclose(color[v, u, :2], expected, atol=.04)
    assert model.K == live['K'] and model.source == 'charuco_rectified'


def test_rectification_requires_a_matching_valid_profile():
    live = {'K': np.eye(3).tolist(), 'width': 64, 'height': 48, 'fps': 6}
    with pytest.raises(ValueError, match='no calibrated'):
        ColorRectification({'calibrated_color_profiles': {}}, live)
    bad = {'K': np.eye(3).tolist(), 'distortion_model': 'inverse_brown_conrady', 'distortion_coeffs': [0.]*5}
    with pytest.raises(ValueError, match='invalid calibrated'):
        ColorRectification({'calibrated_color_profiles': {'64x48@6': bad}}, live)
    plain = ColorRectification(None, live)
    rgb = np.ones((2, 2, 3))
    assert plain.apply(rgb) is rgb


def test_depth_registration_uses_color_z_and_nearest_surface():
    K = np.eye(3)
    identity = DepthRegistration(K, (2, 3), [0.]*5, K, np.eye(3), [0, 0, 0], (2, 3))
    depth = np.array([[1., 0., 2.], [3., 4., 5.]], np.float32)
    np.testing.assert_array_equal(identity.apply(depth), depth)
    # Different source rays overlap after projection. Nearest color-camera Z
    # wins; do not blend them or return the depth camera's original Z.
    small_focal = np.diag([.1, 1., 1.])
    moved = DepthRegistration(K, (1, 2), [0.]*5, small_focal, np.eye(3), [.1, 0, 1], (1, 2))
    np.testing.assert_array_equal(moved.apply([[1., 2.]]), [[2., 2.]])
    np.testing.assert_array_equal(moved.apply([[0., 0.]]), [[0., 0.]])
    with pytest.raises(ValueError, match='native depth distortion'):
        DepthRegistration(K, (1, 2), [.1, 0, 0, 0, 0], K, np.eye(3), [0, 0, 0], (1, 2))


def test_stereo_transform_is_guarded_by_native_and_factory_geometry():
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('y', .02).as_matrix()
    matrix[:3, 3] = [.012, -.001, .01]
    reference = {
        'calibrated_color_profiles': {'640x480@6': {}},
        'depth_to_color': {'rotation_column_major': np.eye(3).ravel().tolist(),
                           'translation_m': [.015, 0, 0]},
        'calibrated_depth_to_color': {
            'T_color_depth': matrix.tolist(),
            'native_depth_profile': {'K': np.eye(3).tolist(), 'width': 3, 'height': 2,
                                     'distortion_coeffs': [0.]*5}}}
    args = [np.eye(3), (2, 3), [0.]*5, np.eye(3), [.015, 0, 0]]
    R, t, source = depth_to_color_transform(reference, *args)
    np.testing.assert_allclose(R, matrix[:3, :3])
    np.testing.assert_allclose(t, matrix[:3, 3])
    assert source == 'charuco_stereo'
    with pytest.raises(ValueError, match='native depth geometry'):
        depth_to_color_transform(reference, args[0], (3, 2), *args[2:])
    with pytest.raises(ValueError, match='native depth geometry'):
        depth_to_color_transform(reference, *args[:-1], [.016, 0, 0])
    reference['calibrated_depth_to_color']['T_color_depth'][0][0] = 2.
    with pytest.raises(ValueError, match='must be rigid'):
        depth_to_color_transform(reference, *args)


def test_native_capture_uses_the_matching_rgb_timestamp(tmp_path):
    import threading

    from services.xarm.camera_server import Cameras
    cameras = object.__new__(Cameras)
    cameras.frames_root, cameras.fps = tmp_path, 6
    image = np.zeros((4, 4, 3), np.uint8)
    worker = SimpleNamespace(
        newest=lambda depth, after: (123, image, None), K=np.eye(3).tolist(),
        distortion_model='none', distortion_coeffs=[0.]*5,
        rectification=SimpleNamespace(source='realsense_factory'), spec={'serial': 'abc'},
        depth_scale=.001, depth_enabled=False, depth_registration_source=None,
        calibration_native=True, lock=threading.Condition(), native_metadata={},
        native_frames={123: (np.full((4, 4), 11, np.uint8), np.full((4, 4), 500, np.uint16)),
                       124: (np.full((4, 4), 22, np.uint8), np.full((4, 4), 800, np.uint16))})
    cameras.workers = {'wrist': worker}
    entry = cameras.grab(['wrist'], False, tmp_path/'capture')['wrist']
    native = entry['native_calibration']
    assert native['timestamp_ns'] == entry['timestamp_ns'] == 123
    assert np.all(cv.imread(native['infrared'], 0) == 11)
    assert np.all(cv.imread(native['depth'], cv.IMREAD_UNCHANGED) == 500)


def test_hand_eye_fit_and_held_out_geometry():
    script = calibration_script()
    make, invert = charuco.T_from_params, charuco.invert
    expected = make([.2, -.1, 1.4], [.04, -.03, .06])
    board = make([.1, -.2, .3], [.4, .1, .05])
    samples = []
    for i, goal in enumerate(script['pose_plan'](np.array([.3, 0, .3]), [0., 0., 0., 1.])):
        tcp = charuco.mat_from_pose(goal['position'], goal['quaternion'])
        samples.append({'index': i, 'held_out': goal['held_out'], 'T_base_tcp': tcp,
                        'T_cam_board': invert(expected)@invert(tcp)@board, 'detection': {}})
    fitted, fitted_board = charuco.solve_hand_eye([s for s in samples if not s['held_out']])
    np.testing.assert_allclose(fitted, expected, atol=1e-6)
    assert charuco.quality_ok(charuco.fit_report(fitted, fitted_board, samples))
    samples[-1]['T_cam_board'][:3, 3] += [.02, 0, 0]
    assert not charuco.quality_ok(charuco.fit_report(fitted, fitted_board, samples))


@pytest.mark.parametrize('visible', [True, False])
def test_preflight_never_constructs_motion_backend_arms_or_holds(tmp_path, monkeypatch, visible):
    script = calibration_script()
    scope = script['main'].__globals__
    calls = []
    class Arm:
        arm = SimpleNamespace(mode=0, tcp_offset=[0, 0, 172, 0, 0, 0], tcp_load=[.82, [0, 0, 48]])
        def __init__(self, options):
            pass
        def state(self):
            return {'tcp_position': np.array([.3, 0, .3]), 'tcp_quaternion': np.array([0., 0., 0., 1.]),
                    'state': 5, 'error_code': 0}
        def check_tcp_limit(self, p, q):
            calls.append('check_target')
        def close(self):
            calls.append('disconnect')
    class Cameras:
        def __init__(self, *args):
            pass
        def start(self):
            calls.append('camera_start')
        def close(self):
            calls.append('camera_close')
    def forbidden(*args, **kwargs):
        pytest.fail('preflight attempted a motion backend or arming')
    opts = tmp_path/'options.json'
    opts.write_text(json.dumps({'cameras': [{'name': 'wrist', 'mount': 'tcp'}, {'name': 'side', 'mount': 'fixed'}]}))
    root = tmp_path/'run'
    monkeypatch.setattr('sys.argv', ['calibrate', '--preflight', '--root', str(root), '--backend-options', str(opts)])
    monkeypatch.setitem(scope, 'XArmDriver', Arm)
    monkeypatch.setitem(scope, 'XArmBackend', forbidden)
    monkeypatch.setitem(scope, 'CameraService', Cameras)
    monkeypatch.setitem(scope, 'capture', lambda *args: (tmp_path, {}))
    monkeypatch.setitem(scope, 'charuco_pose', lambda *args, **kwargs:
                        (np.eye(4) if visible else None, {'charuco_corners': 30 if visible else 0,
                                                         'reprojection_mean_px': .2}))
    monkeypatch.setattr(scope['arming'], 'write', forbidden)
    monkeypatch.setattr(scope['time'], 'sleep', lambda _: None)
    if visible:
        script['main']()
        assert not (root/'result.json').exists()
    else:
        with pytest.raises(RuntimeError, match='visibility'):
            script['main']()
        assert json.loads((root/'result.json').read_text())['ok'] is False
    assert calls.count('check_target') == 18
    assert calls[-2:] == ['camera_close', 'disconnect']
