"""Offline ChArUco geometry shared by camera calibration tools."""
import json
import shutil
import time
from pathlib import Path

import cv2 as cv
import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def board_settings(args):
    if getattr(args, 'board', None):
        return json.loads(Path(args.board).read_text())
    return {key: getattr(args, key) for key in
            ('dictionary', 'squares_x', 'squares_y', 'square_m', 'marker_m', 'legacy_pattern')}


def make_board(settings):
    columns, rows = settings['squares_x'], settings['squares_y']
    layout_rows = settings.get('layout_squares_y', rows)
    if not (type(columns) is int and type(rows) is int and type(layout_rows) is int
            and columns >= 3 and rows >= 3 and layout_rows >= rows):
        raise ValueError('board needs at least 3x3 physical squares within its original layout')
    if not 0 < settings['marker_m'] < settings['square_m']:
        raise ValueError('marker size must be positive and smaller than square size')
    board = cv.aruco.CharucoBoard((columns, layout_rows), settings['square_m'], settings['marker_m'],
                                 cv.aruco.getPredefinedDictionary(getattr(cv.aruco, settings['dictionary'])))
    board.setLegacyPattern(bool(settings['legacy_pattern']))
    return board, cv.aruco.CharucoDetector(board)


def detect_board(gray, board, detector, settings):
    corners, ids, markers, marker_ids = detector.detectBoard(gray)
    if ids is not None:
        # The retained top rows keep their original ID-to-coordinate mapping. Do
        # not use a synthetic corner bordering the removed bottom row.
        coords = np.asarray(board.getChessboardCorners())[np.asarray(ids).ravel()]
        keep = coords[:, 1] < settings['squares_y']*settings['square_m'] - 1e-6
        corners = np.asarray(corners, np.float32).reshape(-1, 1, 2)[keep]
        ids = np.asarray(ids, np.int32).reshape(-1, 1)[keep]
        if len(ids) == 0:
            corners = ids = None
    return corners, ids, markers, marker_ids


def distortion(entry):
    values = np.asarray(entry.get('distortion_coeffs', [0.]*5), dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError('nonfinite distortion coefficients')
    if np.any(values) and entry.get('distortion_model') != 'brown_conrady':
        raise ValueError(f"unsupported nonzero distortion model: {entry.get('distortion_model')}")
    return values


def detect_small_board(gray, board, detector, settings):
    """Detect small markers, returning coordinates in the original image.

    Cubic enlargement helps ArUco decode markers only a few pixels across.
    Undo OpenCV resize's pixel-center mapping before fitting any geometry.
    """
    def at_scale(scale, selected_detector):
        enlarged = cv.resize(gray, None, fx=scale, fy=scale, interpolation=cv.INTER_CUBIC)
        corners, ids, markers, marker_ids = detect_board(enlarged, board, selected_detector, settings)
        if corners is not None:
            corners = (corners + .5) / scale - .5
        markers = tuple((marker + .5) / scale - .5 for marker in markers)
        return corners, ids, markers, marker_ids

    def count(result):
        return 0 if result[1] is None else len(result[1])

    best = at_scale(3, detector)
    if count(best) < 12:
        # Distant, oblique boards can have fewer than ten pixels per marker.
        # Try larger decoding images with threshold windows covering the larger
        # markers. Choose solely by decoded corners, never by geometric residual.
        # Dictionary correction, corner support and validation gates are unchanged.
        params = detector.getDetectorParameters()
        params.adaptiveThreshWinSizeMax = max(53, params.adaptiveThreshWinSizeMax)
        larger_detector = cv.aruco.CharucoDetector(
            board, detector.getCharucoParameters(), params, detector.getRefineParameters())
        for scale in (4, 5):
            candidate = at_scale(scale, larger_detector)
            if count(candidate) > count(best):
                best = candidate
    return best


def invert(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def mat_from_pose(position, quaternion):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(np.asarray(quaternion, dtype=np.float64)).as_matrix()
    T[:3, 3] = np.asarray(position, dtype=np.float64)
    return T


def T_from_params(rotvec, trans):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = trans
    return T


def params_from_T(T):
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def charuco_pose(frame_dir, camera, board, detector, *, overlay_dir=None, settings=None):
    meta = json.loads((frame_dir / "meta.json").read_text())
    entry = meta["cameras"][camera]
    K = np.asarray(entry["K"], dtype=np.float64)
    dist = distortion(entry)
    image = cv.imread(str(frame_dir / entry["rgb"]))
    if image is None:
        return None, {"error": f"could not read {frame_dir / entry['rgb']}"}
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    charuco_corners, charuco_ids, marker_corners, marker_ids = (
        detect_board(gray, board, detector, settings) if settings else detector.detectBoard(gray))
    if settings and (charuco_ids is None or len(charuco_ids) < 12):
        enlarged = detect_small_board(gray, board, detector, settings)
        if enlarged[1] is not None and (charuco_ids is None or len(enlarged[1]) > len(charuco_ids)):
            # The existing detector maps corners back to native pixels; retain
            # the same pose fit and reprojection gates on small distant boards.
            charuco_corners, charuco_ids, marker_corners, marker_ids = enlarged
    markers = 0 if marker_ids is None else len(marker_ids)
    corners = 0 if charuco_ids is None else len(charuco_ids)

    if overlay_dir is not None:
        overlay = image.copy()
        if marker_ids is not None and len(marker_ids):
            cv.aruco.drawDetectedMarkers(overlay, marker_corners, marker_ids)
        if charuco_ids is not None and len(charuco_ids):
            cc = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 1, 2)
            ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1, 1)
            if len(cc) == len(ids):
                cv.aruco.drawDetectedCornersCharuco(overlay, cc, ids, (0, 0, 255))
        cv.imwrite(str(overlay_dir / f"{frame_dir.name}_{camera}_charuco.jpg"), overlay)

    if charuco_ids is None or len(charuco_ids) < 6 or board.checkCharucoCornersCollinear(charuco_ids):
        return None, {"markers": markers, "charuco_corners": corners}

    obj, img = board.matchImagePoints(charuco_corners, charuco_ids)
    obj = np.asarray(obj, dtype=np.float64).reshape(-1, 3)
    img = np.asarray(img, dtype=np.float64).reshape(-1, 2)
    ok, rvec, tvec = cv.solvePnP(obj, img, K, dist, flags=cv.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, {"markers": markers, "charuco_corners": corners, "solvepnp": False}

    projected, _ = cv.projectPoints(obj, rvec, tvec, K, dist)
    err = np.linalg.norm(projected.reshape(-1, 2) - img, axis=1)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(rvec).reshape(3)).as_matrix()
    T[:3, 3] = np.asarray(tvec).reshape(3)
    return T, {
        "markers": markers,
        "charuco_corners": corners,
        "reprojection_mean_px": float(err.mean()),
        "reprojection_max_px": float(err.max()),
    }


def residual(params, samples):
    T_tcp_cam = T_from_params(params[:3], params[3:6])
    T_base_board = T_from_params(params[6:9], params[9:12])
    out = []
    for sample in samples:
        pred = sample["T_base_tcp"] @ T_tcp_cam @ sample["T_cam_board"]
        delta = invert(T_base_board) @ pred
        out.extend((delta[:3, 3] / 0.01).tolist())
        out.extend((Rotation.from_matrix(delta[:3, :3]).as_rotvec() / 0.02).tolist())
    return np.asarray(out)


def solve_hand_eye(samples):
    best = None
    first = samples[0]
    guesses = [
        [0, 0, 0], [np.pi, 0, 0], [0, np.pi, 0], [0, 0, np.pi],
        [np.pi / 2, 0, 0], [-np.pi / 2, 0, 0],
        [0, np.pi / 2, 0], [0, -np.pi / 2, 0],
    ]
    for euler in guesses:
        T_tcp_cam0 = T_from_params(Rotation.from_euler("xyz", euler).as_rotvec(), np.array([0.0, 0.0, 0.05]))
        T_base_board0 = first["T_base_tcp"] @ T_tcp_cam0 @ first["T_cam_board"]
        guess = np.r_[params_from_T(T_tcp_cam0), params_from_T(T_base_board0)]
        result = least_squares(residual, guess, args=(samples,), loss="soft_l1", f_scale=1.0, max_nfev=50000)
        rms = float(np.sqrt(np.mean(residual(result.x, samples) ** 2)))
        if best is None or rms < best[0]:
            best = (rms, result)

    rms, result = best
    T_tcp_cam = T_from_params(result.x[:3], result.x[3:6])
    T_base_board = T_from_params(result.x[6:9], result.x[9:12])
    return T_tcp_cam, T_base_board


def write_matrix(path, matrix, source):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        backup = path.with_suffix(path.suffix + f".bak-{time.strftime('%Y%m%dT%H%M%S')}")
        shutil.copy2(path, backup)
    path.write_text(yaml.safe_dump({"matrix": np.asarray(matrix, dtype=float).tolist(), "source": source}, sort_keys=False))


def fit_report(T_tcp_cam, T_base_board, samples):
    result = []
    for sample in samples:
        error = invert(T_base_board) @ sample['T_base_tcp'] @ T_tcp_cam @ sample['T_cam_board']
        result.append({'index': sample['index'], 'held_out': sample['held_out'],
                       'translation_error_m': float(np.linalg.norm(error[:3, 3])),
                       'rotation_error_deg': float(np.degrees(Rotation.from_matrix(error[:3, :3]).magnitude()))})
    return result


def quality_ok(samples):
    return bool(samples) and all(s['translation_error_m'] <= .005 and s['rotation_error_deg'] <= 1.0
                                 for s in samples)


def fit_fixed_camera(observations, T_base_board):
    """Fit a stationary camera from repeated views of the same fixed board."""
    if len(observations) < 6:
        return {'ok': False, 'reason': 'fewer than 6 fixed-camera observations'}
    mean = np.eye(4)
    mean[:3, :3] = Rotation.from_matrix(np.array([p[:3, :3] for p in observations])).mean().as_matrix()
    mean[:3, 3] = np.mean([p[:3, 3] for p in observations], axis=0)
    errors = []
    for pose in observations:
        delta = invert(mean) @ pose
        errors.append({'translation_error_m': float(np.linalg.norm(delta[:3, 3])),
                       'rotation_error_deg': float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude()))})
    return {'ok': quality_ok(errors), 'T_cam_base': (mean @ invert(T_base_board)).tolist(),
            'T_cam_board': mean.tolist(), 'stationary_board_residuals': errors}
