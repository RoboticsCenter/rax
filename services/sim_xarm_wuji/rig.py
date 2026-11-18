"""The rig's geometry as RAX's verified calibration records it.

The simulator places both arm bases, both TCPs and all four cameras exactly where the
calibration manifest says they are, so the transforms RAX reports in `xarm_observe`
(services/xarm_bimanual/calibration.py) agree with what the simulated cameras draw.

Conventions are RAX's own (services/xarm/calibration.py CameraModel):
- fixed camera: `T_cam_left_base` maps left-base points into the camera's OpenCV
  optical frame, so the camera's pose is its inverse;
- wrist camera: `T_cam_tcp`; the camera's pose in the base is T_base_tcp inv(T_cam_tcp).
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from .spec import SIDES, TCP_OFFSET

PROJECT = Path(__file__).resolve().parents[2]
D435_COLOR = {'K': [[910.0, 0.0, 640.0], [0.0, 910.0, 360.0], [0.0, 0.0, 1.0]], 'width': 1280, 'height': 720}


def resolve(path):
    path = Path(path).expanduser()
    return path.resolve() if path.is_absolute() else (PROJECT/path).resolve()


class Rig:
    def __init__(self, options):
        """options: the bimanual backend options (arms, cameras, calibration_manifest)."""
        self.options = deepcopy(options)
        self.cameras = {c['name']: c for c in options.get('cameras', [])}
        manifest = options.get('calibration_manifest')
        self.manifest = json.loads(resolve(manifest).read_text()) if manifest else None
        self.manifest_path = str(resolve(manifest)) if manifest else None
        data = self.manifest or {}
        identity = data.get('arm_identity', {})
        self.left_from_right = np.asarray(data.get('T_left_base_right_base', _default_inter_base()), dtype=float)
        self.tcp_offset = {side: list(identity.get(side, {}).get('tcp_offset_mm_rad', TCP_OFFSET))
                           for side in SIDES}
        self.tcp_load = {side: identity.get(side, {}).get('tcp_load') for side in SIDES}
        self.arm_serial = {side: identity.get(side, {}).get('serial', f'SIM-XARM6-{side.upper()}')
                           for side in SIDES}

    def base_pose(self, side):
        return np.eye(4) if side == 'left' else self.left_from_right.copy()

    def _color(self, name):
        model = (self.manifest or {}).get('camera_models', {}).get(name)
        if model is None:
            return deepcopy(D435_COLOR)
        return {'K': model['K'], 'width': model['width'], 'height': model['height']}

    def wrist_camera(self, side):
        for name, spec in self.cameras.items():
            if spec.get('mount') == 'tcp' and spec.get('arm') == side:
                wrists = (self.manifest or {}).get('wrists', {})
                if side not in wrists:
                    return None
                return {'name': name, 'serial': spec['serial'],
                        'T_cam_tcp': np.asarray(wrists[side]['T_cam_tcp'], dtype=float), **self._color(name)}
        return None

    def fixed_cameras(self):
        out = {}
        fixed = (self.manifest or {}).get('fixed', {})
        for name, spec in self.cameras.items():
            if spec.get('mount') == 'fixed' and name in fixed:
                out[name] = {'name': name, 'serial': spec['serial'],
                             'T_cam_left_base': np.asarray(fixed[name]['T_cam_left_base'], dtype=float),
                             **self._color(name)}
        return out

    def model_camera(self, name):
        """The compiled model's camera name for a configured camera name."""
        spec = self.cameras[name]
        return f"{spec['arm']}_wrist_camera" if spec.get('mount') == 'tcp' else name

    def summary(self):
        return {'manifest': self.manifest_path,
                'manifest_sha256': hashlib.sha256(resolve(self.manifest_path).read_bytes()).hexdigest()
                if self.manifest_path else None,
                'T_left_base_right_base': self.left_from_right.tolist(), 'tcp_offset_mm_rad': self.tcp_offset,
                'arm_serial': self.arm_serial, 'cameras': sorted(self.cameras)}


def _default_inter_base():
    """Without a manifest: the right base 0.628 m to the right of the left (the rig's)."""
    T = np.eye(4)
    T[1, 3] = -0.628
    return T
