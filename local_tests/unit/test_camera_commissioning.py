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
    calibrated_depth_scale,
    depth_to_color_transform,
    validate_intrinsics,
)
from services.xarm.charuco import detect_board, detect_small_board, distortion, make_board

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


def test_small_infrared_board_returns_original_pixel_coordinates():
    settings = {'dictionary': 'DICT_4X4_100', 'squares_x': 10, 'squares_y': 13,
                'layout_squares_y': 14, 'square_m': .02, 'marker_m': .015, 'legacy_pattern': True}
    board, detector = make_board(settings)
    rendered = board.generateImage((800, 1120), marginSize=0)
    small = cv.resize(rendered, (160, 224), interpolation=cv.INTER_AREA)
    frame = np.full((480, 640), 255, np.uint8)
    frame[80:304, 200:360] = small
    original = frame.copy()
    corners, ids, markers, marker_ids = detect_small_board(frame, board, detector, settings)
    assert ids is not None and len(ids) >= 80
    # Each square spans 16 native pixels; intensity boundaries lie between pixels.
    expected = np.asarray(board.getChessboardCorners())[ids.ravel(), :2] / .02 * 16
    expected += np.array([199.5, 79.5])
    np.testing.assert_allclose(corners.reshape(-1, 2), expected, atol=.35)
    assert len(markers) == len(marker_ids)
    assert all(np.all(marker[..., 0] < 360) and np.all(marker[..., 1] < 304) for marker in markers)
    np.testing.assert_array_equal(frame, original)


def test_nonzero_inverse_distortion_is_not_silently_treated_as_brown():
    assert np.all(distortion({'distortion_model': 'inverse_brown_conrady', 'distortion_coeffs': [0.]*5}) == 0)
    with pytest.raises(ValueError, match='unsupported'):
        distortion({'distortion_model': 'inverse_brown_conrady', 'distortion_coeffs': [.1, 0, 0, 0, 0]})


def test_distant_oblique_board_preserves_projected_corner_geometry():
    settings = {'dictionary': 'DICT_4X4_100', 'squares_x': 10, 'squares_y': 13,
                'layout_squares_y': 14, 'square_m': .02, 'marker_m': .015, 'legacy_pattern': True}
    board, detector = make_board(settings)
    rendered = board.generateImage((800, 1120), marginSize=0)[:1040]
    source = np.float32([[0, 0], [799, 0], [799, 1039], [0, 1039]])
    destination = np.float32([[281, 250], [400, 245], [488, 356], [337, 373]])
    transform = cv.getPerspectiveTransform(source, destination)
    frame = cv.warpPerspective(rendered, transform, (640, 480),
                               flags=cv.INTER_AREA, borderValue=255)
    frame = cv.imdecode(cv.imencode('.jpg', frame, [cv.IMWRITE_JPEG_QUALITY, 85])[1], 0)
    previous_window = detector.getDetectorParameters().adaptiveThreshWinSizeMax
    corners, ids, _, _ = detect_small_board(frame, board, detector, settings)
    assert ids is not None and len(ids) >= 12
    original_pixels = board.getChessboardCorners()[ids.ravel(), :2]/.02*80 - .5
    expected = cv.perspectiveTransform(original_pixels.reshape(-1, 1, 2), transform)
    assert np.max(np.linalg.norm(corners-expected, axis=2)) < .8
    assert detector.getDetectorParameters().adaptiveThreshWinSizeMax == previous_window


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


def test_metric_depth_scale_is_guarded_and_applied_before_stereo_translation():
    reference = {'calibrated_depth_to_color': {
        'depth_scale_factor': 1.005, 'factory_depth_units_m': .001}}
    assert calibrated_depth_scale({}, .001) == 1.
    factor = calibrated_depth_scale(reference, .001)
    registration = DepthRegistration(np.eye(3), (1, 1), [0.]*5, np.eye(3), np.eye(3),
                                     [0, 0, .01], (1, 1), depth_scale_factor=factor)
    np.testing.assert_allclose(registration.apply([[.6]]), [[.613]], atol=1e-7)
    with pytest.raises(ValueError, match='factory depth units'):
        calibrated_depth_scale(reference, .0001)
    for invalid in [float('nan'), .5, 1.03]:
        reference['calibrated_depth_to_color']['depth_scale_factor'] = invalid
        with pytest.raises(ValueError, match='depth scale'):
            calibrated_depth_scale(reference, .001)
        with pytest.raises(ValueError, match='depth scale'):
            DepthRegistration(np.eye(3), (1, 1), [0.]*5, np.eye(3), np.eye(3),
                              [0, 0, 0], (1, 1), depth_scale_factor=invalid)


def test_metric_depth_fit_excludes_reserved_views(tmp_path):
    script = runpy.run_path(str(PROJECT/'scripts/xarm-charuco-stereo-fit'))
    board = np.eye(4)
    board[2, 3] = 1.
    result = {'T_cam_tcp': np.eye(4).tolist(), 'T_base_board': board.tolist()}
    pixels = np.array([[x, y] for y in [8, 16, 24] for x in [8, 16, 24, 32]], np.float32)
    samples = []
    for index in range(12):
        held = index >= 9
        path = tmp_path/f'{index}.png'
        cv.imwrite(str(path), np.full((40, 40), 400 if held else 995, np.uint16))
        samples.append({'index': index, 'held_out': held,
                        'object': np.zeros((12, 1, 3), np.float32), 'ir': pixels.reshape(-1, 1, 2),
                        'native': {'depth': str(path), 'depth_scale_m': .001},
                        'meta': {'tcp_position': [0, 0, 0], 'tcp_quaternion': [0, 0, 0, 1]}})
    fitted = script['fit_metric_depth_scale'](samples, np.eye(4), result)
    assert fitted['factor'] == pytest.approx(1/.995)
    assert fitted['training_samples'] == 108
    assert [v['index'] for v in fitted['training_views']] == list(range(9))
    with pytest.raises(ValueError, match='nine training views'):
        script['fit_metric_depth_scale'](samples[:8] + samples[9:], np.eye(4), result)


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


def test_calibration_transitions_respect_fifty_mm_limit_and_preserve_capture_poses():
    script = calibration_script()
    state = {'tcp_position': np.array([.23058, -.00155, .43342]),
             'tcp_quaternion': Rotation.from_euler('x', 179.17, degrees=True).as_quat()}
    plan = script['pose_plan'](state['tcp_position'], state['tcp_quaternion'])
    options = {'max_translation_m': .05, 'workspace_min': [.2, -.35, .02],
               'workspace_max': [.8, .35, .9]}
    steps = script['motion_plan'](plan, state, options)
    previous = state['tcp_position']
    checked = []
    for step in steps:
        assert np.linalg.norm(np.asarray(step['position'])-previous) <= .04+1e-12
        assert np.isclose(np.linalg.norm(step['quaternion']), 1.)
        previous = np.asarray(step['position'])
    endpoints = [s for s in steps if s['final_step'] and s['capture_index'] != 'return']
    assert len(endpoints) == 18
    for actual, expected in zip(endpoints, plan):
        assert actual['capture_index'] == expected['index']
        np.testing.assert_allclose(actual['position'], expected['position'])
        np.testing.assert_allclose(Rotation.from_quat(actual['quaternion']).as_matrix(),
                                   Rotation.from_quat(expected['quaternion']).as_matrix(), atol=1e-12)
    np.testing.assert_allclose(steps[-1]['position'], state['tcp_position'])
    np.testing.assert_allclose(Rotation.from_quat(steps[-1]['quaternion']).as_matrix(),
                               Rotation.from_quat(state['tcp_quaternion']).as_matrix(), atol=1e-12)
    arm = SimpleNamespace(check_tcp_limit=lambda p, q: checked.append(p.tolist()))
    script['validate_plan'](plan, state, arm, options)
    assert checked == [s['position'] for s in steps]


@pytest.mark.parametrize('visible', [True, False])
@pytest.mark.parametrize('installed_intrinsics', [True, False])
def test_preflight_never_constructs_motion_backend_arms_or_holds(tmp_path, monkeypatch, visible,
                                                               installed_intrinsics):
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
        def __init__(self, options, *args):
            assert all(c['calibration_raw_color'] is not installed_intrinsics for c in options['cameras'])
        def start(self):
            calls.append('camera_start')
        def close(self):
            calls.append('camera_close')
    def forbidden(*args, **kwargs):
        pytest.fail('preflight attempted a motion backend or arming')
    opts = tmp_path/'options.json'
    opts.write_text(json.dumps({'cameras': [{'name': 'wrist', 'mount': 'tcp'}, {'name': 'side', 'mount': 'fixed'}]}))
    root = tmp_path/'run'
    argv = ['calibrate', '--preflight', '--root', str(root), '--backend-options', str(opts)]
    if installed_intrinsics:
        argv.append('--use-installed-intrinsics')
    monkeypatch.setattr('sys.argv', argv)
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
    assert calls.count('check_target') == 19  # Eighteen capture poses plus the return pose.
    assert calls[-2:] == ['camera_close', 'disconnect']
