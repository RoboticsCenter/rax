"""Camera extrinsics in the xArm base frame.

The cameras are the backend option ``cameras`` (see examples/xarm.json): one entry per
RealSense with its name, serial, mount ('fixed' or 'tcp'), optional depth_min_m/depth_max_m,
and its extrinsics YAML. A relative extrinsics path resolves under $XARM_CALIBRATION_DIR.
"""
from dataclasses import dataclass
import json
import os
from pathlib import Path

import numpy as np
import yaml

CALIBRATION_ENVIRONMENT = 'XARM_CALIBRATION_DIR'


def calibration_dir():
    value = os.environ.get(CALIBRATION_ENVIRONMENT)
    return Path(value).expanduser() if value else None


def camera_specs(options):
    cameras = options.get('cameras')
    if not cameras:
        raise ValueError("xarm backend option 'cameras' is required: list each RealSense camera's name, serial, "
                         "mount and extrinsics file (see examples/xarm.json)")
    return cameras


def extrinsics_path(spec):
    path = Path(spec['extrinsics']).expanduser()
    if path.is_absolute():
        return path
    base = calibration_dir()
    if base is None:
        raise ValueError(f"camera {spec['name']}: the relative extrinsics path {path} needs "
                         f"{CALIBRATION_ENVIRONMENT} (or give an absolute path)")
    return base/path


def validate_intrinsics(spec, live):
    """Check recorded factory intrinsics against the active camera profile.

    The factory block is a configuration guard. A separate fitted color block,
    when present, drives explicit image rectification and depth registration.
    Older configurations without a reference continue using live SDK values.
    """
    if not spec.get('intrinsics'):
        return
    path = Path(spec['intrinsics']).expanduser()
    if not path.is_absolute():
        base = calibration_dir()
        if base is None:
            raise ValueError('relative intrinsics path needs XARM_CALIBRATION_DIR')
        path = base/path
    reference = json.loads(path.read_text())
    if str(reference['serial']) != str(spec['serial']):
        raise ValueError(f"camera {spec['name']}: intrinsics serial mismatch")
    key = f"{live['width']}x{live['height']}@{live['fps']}"
    expected = reference['color_profiles'].get(key)
    if expected is None:
        raise ValueError(f"camera {spec['name']}: no recorded intrinsics for {key}")
    for name in ('K', 'distortion_coeffs'):
        actual, recorded = np.asarray(live[name], float), np.asarray(expected[name], float)
        if actual.shape != recorded.shape or not np.allclose(actual, recorded, rtol=0, atol=1e-5):
            raise ValueError(f"camera {spec['name']}: live {name} differs from recorded factory intrinsics")
    if live['distortion_model'] != expected['distortion_model']:
        raise ValueError(f"camera {spec['name']}: distortion model mismatch")
    return reference


class ColorRectification:
    """Rectify raw color to a fitted pinhole model."""
    def __init__(self, reference, live):
        import cv2 as cv
        key = f"{live['width']}x{live['height']}@{live['fps']}"
        profiles = (reference or {}).get('calibrated_color_profiles')
        self.maps, self.K, self.source = None, live['K'], 'realsense_factory'
        if profiles is None:
            return
        if key not in profiles:
            raise ValueError(f'no calibrated color profile for {key}')
        profile = profiles[key]
        K, D = np.asarray(profile['K'], float), np.asarray(profile['distortion_coeffs'], float)
        if (profile.get('distortion_model') != 'brown_conrady' or K.shape != (3, 3)
                or D.shape != (5,) or not np.all(np.isfinite(K)) or not np.all(np.isfinite(D))
                or K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1])):
            raise ValueError('invalid calibrated color model')
        self.K, self.source = K.tolist(), 'charuco_rectified'
        self.maps = cv.initUndistortRectifyMap(K, D, np.eye(3), K,
                                             (live['width'], live['height']), cv.CV_32FC1)

    def apply(self, color):
        if self.maps is None:
            return color
        import cv2 as cv
        return cv.remap(color, *self.maps, cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)


class DepthRegistration:
    """Project native depth into the fitted, rectified color camera.

    SDK color alignment uses factory color intrinsics, so remapping that aligned
    depth with a new color model would retain incorrect pixel correspondences.
    Register native points instead, using the selected depth-to-color geometry and a
    nearest-surface Z buffer. Invalid input and out-of-view points stay invalid.
    """
    def __init__(self, depth_K, depth_shape, depth_coeffs, color_K, rotation, translation, color_shape,
                 *, depth_scale_factor=1.0):
        if np.any(np.asarray(depth_coeffs)):
            raise ValueError('calibrated color registration currently requires zero native depth distortion')
        if not np.isfinite(depth_scale_factor) or not .98 <= depth_scale_factor <= 1.02:
            raise ValueError('calibrated depth scale factor must be within two percent of factory units')
        self.depth_scale_factor = float(depth_scale_factor)
        y, x = np.indices(depth_shape, dtype=np.float32)
        K = np.asarray(depth_K, np.float32)
        rays = np.stack([(x-K[0, 2])/K[0, 0], (y-K[1, 2])/K[1, 1], np.ones_like(x)], axis=-1)
        self.rays = rays.reshape(-1, 3)@np.asarray(rotation, np.float32).T
        self.translation = np.asarray(translation, np.float32)
        self.K, self.shape = np.asarray(color_K, np.float32), color_shape

    def apply(self, depth):
        values = np.asarray(depth, np.float32).ravel() * self.depth_scale_factor
        valid = np.isfinite(values) & (values > 0)
        points = self.rays[valid]*values[valid, None]+self.translation
        points = points[points[:, 2] > 0]
        height, width = self.shape
        output = np.full(height*width, np.inf, np.float32)
        projected = points@self.K.T
        uv = projected[:, :2]/projected[:, 2, None]
        # Splat onto the adjacent pixel centers without interpolating depths
        # across foreground/background boundaries. Exact-center samples retain
        # their single pixel footprint (floor == ceil).
        low, high = np.floor(uv).astype(int), np.ceil(uv).astype(int)
        for u in (low[:, 0], high[:, 0]):
            for v in (low[:, 1], high[:, 1]):
                inside = (u >= 0) & (v >= 0) & (u < width) & (v < height)
                np.minimum.at(output, v[inside]*width+u[inside], points[inside, 2])
        output[~np.isfinite(output)] = 0
        return output.reshape(height, width)


def calibrated_depth_scale(reference, factory_units_m):
    """Use an optional board-fitted metric scale only with its recorded units."""
    fitted = (reference or {}).get('calibrated_depth_to_color') or {}
    if 'depth_scale_factor' not in fitted:
        return 1.0
    factor = float(fitted['depth_scale_factor'])
    expected = float(fitted['factory_depth_units_m'])
    if (not np.isfinite(factor) or not .98 <= factor <= 1.02 or not np.isfinite(expected)
            or expected <= 0 or not np.isclose(factory_units_m, expected, rtol=0, atol=1e-10)):
        raise ValueError('calibrated depth scale factor or factory depth units mismatch')
    return factor


def depth_to_color_transform(reference, native_K, native_shape, native_coeffs, rotation, translation):
    """Select a qualified stereo transform, guarded by its factory reference."""
    fitted = (reference or {}).get('calibrated_depth_to_color')
    if fitted is None:
        return rotation, translation, 'realsense_factory'
    if not reference.get('calibrated_color_profiles'):
        raise ValueError('calibrated stereo transform requires its fitted color model')
    expected = fitted['native_depth_profile']
    factory = reference['depth_to_color']
    checks = ((native_K, expected['K']), (native_coeffs, expected['distortion_coeffs']),
              (rotation, np.asarray(factory['rotation_column_major']).reshape(3, 3, order='F')),
              (translation, factory['translation_m']))
    if (list(native_shape) != [expected['height'], expected['width']]
            or any(np.asarray(a).shape != np.asarray(b).shape
                   or not np.allclose(a, b, rtol=0, atol=1e-5) for a, b in checks)):
        raise ValueError('live native depth geometry differs from stereo calibration reference')
    matrix = np.asarray(fitted['T_color_depth'], float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)):
        raise ValueError('calibrated depth-to-color transform must be rigid')
    return matrix[:3, :3], matrix[:3, 3], 'charuco_stereo'


def load_matrix(path):
    with open(path) as handle:
        matrix = np.asarray(yaml.safe_load(handle)['matrix'], dtype=np.float64)
    if matrix.shape != (4, 4) or not np.allclose(matrix[3], [0, 0, 0, 1]):
        raise ValueError(f'{path}: expected a 4x4 homogeneous matrix')
    return matrix


@dataclass(frozen=True)
class CameraModel:
    name: str
    serial: str
    mount: str            # 'fixed' (T_cam_base) or 'tcp' (T_cam_tcp, eye in hand)
    matrix: np.ndarray

    @classmethod
    def from_spec(cls, spec):
        if spec['mount'] not in ('fixed', 'tcp'):
            raise ValueError(f"camera {spec['name']}: mount must be 'fixed' or 'tcp'")
        return cls(str(spec['name']), str(spec['serial']), spec['mount'], load_matrix(extrinsics_path(spec)))

    def base_from_camera(self, T_base_tcp=None):
        """T_base_cam: the camera's pose in the base frame at the given TCP pose."""
        if self.mount == 'fixed':
            return np.linalg.inv(self.matrix)
        if T_base_tcp is None:
            raise ValueError(f'{self.name} is eye-in-hand; its pose needs the TCP pose')
        return np.asarray(T_base_tcp) @ np.linalg.inv(self.matrix)


def backproject(u, v, depth_m, K):
    """Pixel + metric depth -> camera-frame point (pinhole, aligned color image)."""
    return np.array([(u - K[0][2]) / K[0][0] * depth_m, (v - K[1][2]) / K[1][1] * depth_m, depth_m])


def project(point_base, T_base_cam, K):
    p = np.linalg.inv(T_base_cam) @ np.append(np.asarray(point_base, dtype=np.float64), 1.0)
    if p[2] <= 0:
        return None
    uv = np.asarray(K) @ p[:3]
    return (uv[:2] / uv[2]).tolist()
