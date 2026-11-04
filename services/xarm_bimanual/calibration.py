"""Load verified RGB geometry while allowing additional uncalibrated fixed views."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from services.json_io import write_private_json
from services.xarm.calibration import CameraModel
from services.xarm.charuco import mat_from_pose

PROJECT = Path(__file__).resolve().parents[2]


def rigid(value, label):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1])
            or not np.allclose(matrix[:3, :3].T@matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)):
        raise ValueError(f'{label}: expected a finite rigid transform')
    return matrix


class BimanualCalibration:
    def __init__(self, options):
        path = Path(options['calibration_manifest']).expanduser()
        self.path = path.resolve() if path.is_absolute() else (PROJECT/path).resolve()
        raw = self.path.read_bytes()
        self.manifest = data = json.loads(raw)
        self.digest = hashlib.sha256(raw).hexdigest()
        self.unverified = frozenset(options.get('unverified_camera_extrinsics', []))
        if not data.get('ok') or not data.get('fresh_verification', {}).get('ok'):
            raise ValueError('bimanual calibration needs a passing fit and fresh physical verification')
        specs = {c['name']: c for c in options['cameras']}
        self.camera_names = tuple(specs)
        recorded = {c['name']: c for c in data['camera_specs']}
        if not set(recorded) <= set(specs) or set(recorded) != set(data['camera_models']):
            raise ValueError('configured cameras differ from the verified calibration')
        additional = set(specs) - set(recorded)
        if (not additional <= self.unverified or not self.unverified <= set(specs)
                or any(specs[name]['mount'] != 'fixed' for name in additional)):
            raise ValueError('additional cameras must be fixed views explicitly marked extrinsics unverified')
        self.models = {}
        self.left_from_right = rigid(data['T_left_base_right_base'], 'inter-base transform')
        self.right_from_left = np.linalg.inv(self.left_from_right)
        for side in ('left', 'right'):
            if options['arms'][side]['xarm_ip'] != data['arm_identity'][side]['ip']:
                raise ValueError(f'{side}: arm IP differs from calibration')
        for name, spec in specs.items():
            if spec.get('intrinsics') or spec.get('extrinsics') or spec.get('calibration_raw_color'):
                raise ValueError('the verified bundle supplies camera models; omit per-camera calibration overrides')
            if name in additional:
                continue
            if any(spec.get(k) != recorded[name].get(k) for k in ('mount', 'arm')):
                raise ValueError(f'{name}: camera mount or owning arm changed')
            if spec.get('serial') != recorded[name].get('serial'):
                if spec['mount'] == 'fixed' and name in self.unverified:
                    # A replacement fixed camera can supply images, but neither
                    # its predecessor's optical model nor its poses are reusable.
                    continue
                raise ValueError(f'{name}: camera serial changed; fixed replacements must be explicitly unverified')
            color = data['camera_models'][name]
            if (spec.get('width', 640) != color['width'] or spec.get('height', 480) != color['height']
                    or options.get('stream_fps', 6) != color['fps']):
                raise ValueError(f'{name}: use the verified {color["width"]}x{color["height"]}@{color["fps"]} profile')
            if color['serial'] != spec['serial']:
                raise ValueError(f'{name}: color model serial mismatch')
            if spec['mount'] == 'tcp':
                value = data['wrists'][spec['arm']]['T_cam_tcp']
            else:
                value = data['fixed'][name]['T_cam_left_base']
            self.models[name] = CameraModel(name, spec['serial'], spec['mount'], rigid(value, name))

    def camera_specs(self, options, directory):
        """Adapt the bundle to the existing xArm rectification implementation."""
        specs = deepcopy(options['cameras'])
        for spec in specs:
            if spec['name'] not in self.models:
                continue  # New/replacement views use their live factory RGB model.
            model = self.manifest['camera_models'].get(spec['name'])
            if model is None:
                continue  # Additional uncalibrated views retain their live factory RGB model.
            factory = model.get('factory_profile', model)
            key = f"{model['width']}x{model['height']}@{model['fps']}"
            value = {'serial': model['serial'], 'color_profiles': {key: factory},
                     'source': {'manifest': str(self.path), 'sha256': self.digest}}
            if 'factory_profile' in model:
                value['calibrated_color_profiles'] = {key: model}
            path = Path(directory)/f"{spec['name']}_intrinsics.json"
            write_private_json(path, value)
            spec['intrinsics'] = str(path)
        return specs

    def validate_arms(self, arms):
        for side, assembly in arms.items():
            code, serial = assembly.arm.arm.get_robot_sn()
            if code != 0 or serial != self.manifest['arm_identity'][side]['serial']:
                raise ValueError(f'{side}: live arm serial differs from calibration')
        # The SDK's TCP-offset cache begins at zero until its first report.
        deadline = time.monotonic()+3
        while True:
            try:
                self.validate_offsets(arms)
                return
            except ValueError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.1)

    def validate_offsets(self, arms):
        for side, assembly in arms.items():
            actual = np.asarray(assembly.arm.arm.tcp_offset, float)
            expected = self.manifest['arm_identity'][side]['tcp_offset_mm_rad']
            if actual.shape != (6,) or not np.allclose(actual, expected, rtol=0, atol=1e-5):
                raise ValueError(f'{side}: TCP offset differs from the verified camera calibration')

    def poses(self, spec, states):
        if spec['name'] in self.unverified:
            return None
        camera = self.models[spec['name']]
        if camera.mount == 'fixed':
            left = camera.base_from_camera()
        else:
            side = spec['arm']
            state = states[side]
            own = camera.base_from_camera(mat_from_pose(state['tcp_position'], state['tcp_quaternion']))
            left = own if side == 'left' else self.left_from_right@own
        return {'left': left, 'right': self.right_from_left@left}

    @property
    def status(self):
        return 'rgb_extrinsics_partial' if self.unverified else 'rgb_extrinsics_verified'

    def camera_status(self, name):
        return ('extrinsics_unverified' if name in self.unverified or name not in self.models
                else 'rgb_extrinsics_verified')

    def summary(self):
        planes = {}
        for side in ('left', 'right'):
            board = np.asarray(self.manifest['wrists'][side]['T_base_board'])
            normal = board[:3, 2]
            planes[side] = {'normal': normal.tolist(), 'offset_m': float(-normal@board[:3, 3]),
                            'meaning': 'original calibration-board plane; confirm scene support before using it'}
        return {'status': self.status, 'manifest': str(self.path), 'sha256': self.digest,
                'unverified_camera_extrinsics': sorted(self.unverified),
                'camera_status': {name: self.camera_status(name) for name in self.camera_names},
                'reference_base_frame': 'left_base', 'T_left_base_right_base': self.left_from_right.tolist(),
                'reference_board_planes': planes, 'depth_metric_validation': 'pending',
                'verification': {name: value for name, value in self.manifest['fresh_verification']['cameras'].items()
                                 if name not in self.unverified}}
