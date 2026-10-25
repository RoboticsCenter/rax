"""xArm backend logic against a fake SDK: nothing here touches hardware."""
import json
from pathlib import Path

import cv2 as cv
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from services.controller.config import Configuration
from services.xarm import arming
from services.xarm.backend import HOME_Q, XArmBackend, matrix
from services.xarm.calibration import project
from services.xarm.sdk import MotionStopped, TargetRejected
from services.xarm.sdk import XArmDriver

START_Q = np.array([0.0, -0.72, -1.18, 0.0, 1.90, 0.0])


class FakeArm:
    def __init__(self, q=START_Q, on_step=None):
        self.q = np.asarray(q, dtype=float)
        self.pos = np.array([0.42, 0.0, 0.30], dtype=float)
        self.quat = Rotation.from_euler('xyz', [np.pi, 0.0, 0.0]).as_quat()
        self.commands = 0
        self.holds = 0
        self.on_step = on_step
        self.reject = False
        self.gripper_width = 0.085

    def state(self):
        if self.on_step and self.commands:
            self.on_step(self)
        return {'tcp_position': self.pos.copy(), 'tcp_quaternion': self.quat.copy(), 'joints': self.q.copy(),
                'state': 0, 'mode': 0, 'error_code': 0, 'warn_code': 0}

    def check_tcp_limit(self, position, quaternion):
        return 'limit' if position[0] > 0.8 else None

    def check_joint_limit(self, joints):
        return None

    def forward_kinematics(self, joints):
        joints = np.asarray(joints)
        p = np.array([0.42 + 0.04 * joints[0], 0.02 * joints[3], 0.32 + 0.25 * (joints[1] + 0.72)])
        return p, self.quat.copy()

    def move_pose(self, position, quaternion, **kwargs):
        if self.reject:
            raise TargetRejected('C40')
        self.commands += 1
        self.pos = np.asarray(position, dtype=float)
        self.quat = np.asarray(quaternion, dtype=float)

    def move_joints(self, joints, **kwargs):
        self.commands += 1
        self.q = np.asarray(joints, dtype=float)
        self.pos, _ = self.forward_kinematics(self.q)

    def hold(self):
        self.holds += 1

    def gripper_state(self):
        return {'width': self.gripper_width, 'max_width': 0.085, 'raw_position': self.gripper_width / 0.085 * 850,
                'is_grasped': False, 'is_moving': False, 'stale': False}

    def gripper_move(self, width, **kwargs):
        self.gripper_width = float(width)


class FakeCameras:
    def close(self):
        pass


@pytest.fixture
def make_backend(tmp_path, monkeypatch):
    monkeypatch.setenv('RAX_ARTIFACT_ROOT', str(tmp_path))
    extrinsic = tmp_path/'cam.yaml'
    yaml.safe_dump({'matrix': np.eye(4).tolist()}, extrinsic.open('w'))

    def build(arm=None, hand=None, **options):
        root = tmp_path/'sessions'/'s1'
        root.mkdir(parents=True, exist_ok=True)
        (root/'operator-input.json').write_text('{}')
        cameras = [{'name': 'left', 'serial': '1', 'mount': 'fixed', 'extrinsics': str(extrinsic)}]
        config = Configuration(root=root, task='t', robot='xarm', backend_options={'cameras': cameras, **options})
        arm = arm or FakeArm()
        return XArmBackend(config, arm=arm, cameras=FakeCameras(), hand=hand), arm, root
    return build


def test_cameras_come_from_the_operator_configuration(make_backend, tmp_path, monkeypatch):
    with pytest.raises(ValueError, match="option 'cameras' is required"):
        make_backend(cameras=None)
    example = json.loads((Path(__file__).resolve().parents[2]/'examples/xarm.json').read_text())
    # The site example now contains real serials and absolute calibration paths.
    # Exercise the supported relative-path contract with temporary calibrations.
    for spec in example['cameras']:
        spec['extrinsics'] = Path(spec['extrinsics']).name
    monkeypatch.delenv('XARM_CALIBRATION_DIR', raising=False)
    with pytest.raises(ValueError, match='needs XARM_CALIBRATION_DIR'):
        make_backend(cameras=example['cameras'])
    calibration = tmp_path/'calibration'
    calibration.mkdir()
    shift = np.eye(4)
    shift[:3, 3] = [0.1, 0.2, 0.3]
    for spec in example['cameras']:
        yaml.safe_dump({'matrix': shift.tolist()}, (calibration/spec['extrinsics']).open('w'))
    monkeypatch.setenv('XARM_CALIBRATION_DIR', str(calibration))
    options = {k: v for k, v in example.items() if k != 'cameras'}
    backend, _, _ = make_backend(cameras=example['cameras'], **options)
    assert {name: (model.serial, model.mount) for name, model in backend.models.items()} == {
        spec['name']: (spec['serial'], spec['mount']) for spec in example['cameras']}
    assert np.allclose(backend.models['left'].base_from_camera()[:3, 3], [-0.1, -0.2, -0.3])


def test_launcher_options_are_accepted_and_unknown_ones_refused(make_backend):
    backend, _, _ = make_backend(agent_python='/opt/agent-env/bin/python', protected_paths=['/opt/robot'])
    assert backend.options['agent_python'] == '/opt/agent-env/bin/python'
    with pytest.raises(ValueError, match="unknown xarm backend options: \\['camera_serials'\\]"):
        make_backend(camera_serials=[1])


def test_motion_refused_until_armed(make_backend):
    backend, arm, root = make_backend()
    with pytest.raises(PermissionError, match='not armed'):
        backend.validate('xarm_move_pose', {'position': [0, 0, 0.01], 'relative': True})
    assert arm.commands == 0
    arming.write(root, True)
    backend.validate('xarm_move_pose', {'position': [0, 0, 0.01], 'relative': True})


def test_pose_move_and_rejections(make_backend):
    backend, arm, root = make_backend()
    arming.write(root, True)
    value, images = backend.call('xarm_move_pose', {'position': [0.45, 0.02, 0.32]})
    assert value['reached'] and value['position_error_mm'] == 0 and images == []
    with pytest.raises(ValueError, match='outside the workspace box'):
        backend.validate('xarm_move_pose', {'position': [0.45, 0.02, 0.02]})
    with pytest.raises(ValueError, match='per call'):
        backend.validate('xarm_move_pose', {'position': [0.80, 0.50, 0.65]})
    arm.reject = True
    with pytest.raises(TargetRejected, match='refused'):
        backend.call('xarm_move_pose', {'position': [0.46, 0.02, 0.32]})
    assert arm.holds >= 1


def test_disarm_stops_a_running_move(make_backend):
    holder = {}

    def disarm_midway(arm):
        if arm.commands == 1:
            arming.write(holder['root'], False)
    backend, arm, root = make_backend(arm=FakeArm(on_step=disarm_midway))
    holder['root'] = root
    arming.write(root, True)
    with pytest.raises(MotionStopped, match='disarmed'):
        backend.call('xarm_move_pose', {'position': [0.44, 0.0, 0.30]})
    assert arm.holds >= 1


def test_joint_path_checked_by_controller_fk(make_backend):
    backend, arm, root = make_backend()
    arming.write(root, True)
    backend.validate('xarm_move_joints', {'preset': 'home'})
    low = START_Q.copy()
    low[1] = -1.9
    with pytest.raises(ValueError, match='workspace box|minimum'):
        backend.validate('xarm_move_joints', {'positions': low.tolist()})
    with pytest.raises(ValueError, match='per call'):
        backend.validate('xarm_move_joints', {'positions': (START_Q + [1.3, 0, 0, 0, 0, 0]).tolist()})
    value, _ = backend.call('xarm_move_joints', {'preset': 'home'})
    assert np.allclose(arm.q, HOME_Q) and value['max_joint_error_rad'] == 0


def test_gripper_requires_arming_and_moves(make_backend):
    backend, arm, root = make_backend()
    with pytest.raises(PermissionError):
        backend.call('xarm_gripper', {'action': 'open'})
    arming.write(root, True)
    value, _ = backend.call('xarm_gripper', {'action': 'grasp', 'width': 0.02, 'force': 30})
    assert value['width'] == 0.02 and value['raw_position'] == pytest.approx(200)


def test_g2_gripper_uses_integer_sdk_arguments():
    class FakeG2:
        def __init__(self):
            self.kwargs = None

        def set_gripper_g2_position(self, raw, **kwargs):
            self.raw = raw
            self.kwargs = kwargs
            assert isinstance(raw, int)
            assert isinstance(kwargs['speed'], int)
            assert isinstance(kwargs['force'], int)
            return 0

    driver = XArmDriver.__new__(XArmDriver)
    driver.arm = FakeG2()
    driver.options = {'gripper_type': 'g2', 'gripper_max_width_m': 0.085,
                      'gripper_closed_position': 0, 'gripper_open_position': 850,
                      'gripper_speed': 5000, 'gripper_force': 60}
    driver.gripper_move(0.04, speed=1000.0, force=42.0)
    assert driver.arm.raw == 40
    assert driver.arm.kwargs['speed'] == 225
    assert driver.arm.kwargs['force'] == 42
    assert driver._raw_to_width(83) == pytest.approx(0.083)


def test_pixel_to_position_inverts_projection(make_backend):
    backend, arm, root = make_backend()
    frame = backend.frames_root/'abc123'
    frame.mkdir(parents=True)
    K = [[700.0, 0, 640], [0, 700.0, 360], [0, 0, 1]]
    T_base_cam = np.eye(4)
    T_base_cam[:3, 3] = [0.5, 0.0, 1.0]
    T_base_cam[:3, :3] = Rotation.from_euler('x', 180, degrees=True).as_matrix()
    point = np.array([0.55, 0.05, 0.12])
    u, v = project(point, T_base_cam, K)
    depth = np.zeros((720, 1280), np.uint16)
    depth[:] = round((1.0 - 0.12) * 1000)
    cv.imwrite(str(frame/'left_depth.png'), depth)
    (frame/'meta.json').write_text(json.dumps({'cameras': {'left': {
        'depth': 'left_depth.png', 'K': K, 'width': 1280, 'height': 720, 'T_base_cam': T_base_cam.tolist()}}}))
    value, _ = backend.call('xarm_pixel_to_position', {'frame_id': 'abc123', 'camera': 'left',
                                                       'pixels': [[u, v], [5000, 1]]})
    assert np.allclose(value['points'][0]['position'], point, atol=1e-3)
    assert value['points'][1]['error'] == 'outside the image'
    with pytest.raises(ValueError):
        backend.validate('xarm_pixel_to_position', {'frame_id': '../x', 'camera': 'left', 'pixels': [[1, 1]]})
