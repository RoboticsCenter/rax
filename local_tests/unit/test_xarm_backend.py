"""xArm backend logic against a fake SDK: nothing here touches hardware."""
import json
from pathlib import Path
from types import SimpleNamespace

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


@pytest.mark.parametrize('requested,expected', [(None, .05), (.015, .015), (.05, .05)])
def test_cartesian_speed_is_per_move_and_reaches_sdk(make_backend, requested, expected):
    class RecordingArm(FakeArm):
        def move_pose(self, position, quaternion, **kwargs):
            self.speed = kwargs['speed_mps']
            super().move_pose(position, quaternion, **kwargs)

    backend, arm, root = make_backend(arm=RecordingArm(), cartesian_speed_mps=.05,
                                       max_translation_m=.35, settle_s=0)
    arming.write(root, True)
    arguments = {'position': [.2, 0, 0], 'relative': True}
    if requested is not None:
        arguments['speed_mps'] = requested
    backend.validate('xarm_move_pose', arguments)
    result, _ = backend.call('xarm_move_pose', arguments)
    assert result['reached'] and result['speed_mps'] == expected
    assert arm.speed == expected and arm.commands == 1
    assert backend.options['cartesian_speed_mps'] == .05
    # A slower contact move does not lower the speed of subsequent clear moves.
    result, _ = backend.call('xarm_move_pose', {'position': [0, .01, 0], 'relative': True})
    assert result['speed_mps'] == .05 and arm.speed == .05


@pytest.mark.parametrize('speed', [0, -.01, .0005, .051, float('nan'), float('inf'), True])
def test_cartesian_speed_rejected_before_dispatch(make_backend, speed):
    backend, arm, root = make_backend(cartesian_speed_mps=.05)
    arming.write(root, True)
    arguments = {'position': [.01, 0, 0], 'relative': True, 'speed_mps': speed}
    with pytest.raises(ValueError, match='speed_mps'):
        backend.validate('xarm_move_pose', arguments)
    with pytest.raises(ValueError, match='speed_mps'):
        backend.call('xarm_move_pose', arguments)
    assert arm.commands == 0


@pytest.mark.parametrize('requested,duration,succeeds', [
    (None, 240., True), (600., 540., True), (5., 10., False), (None, 310., False),
])
def test_slow_rotation_uses_selected_deadline(make_backend, monkeypatch, requested, duration, succeeds):
    clock = [100.]
    def advance(seconds):
        clock[0] += seconds
    monkeypatch.setattr('services.xarm.backend.time', SimpleNamespace(
        monotonic=lambda: clock[0], sleep=advance, time=lambda: clock[0]))

    class SlowRotationArm(FakeArm):
        def move_pose(self, position, quaternion, **kwargs):
            self.commands += 1
            self.started = clock[0]
            self.target = np.asarray(quaternion)
            self.sdk_timeout = kwargs['timeout_s']

        def state(self):
            if self.commands and clock[0]-self.started >= duration:
                self.quat = self.target.copy()
            return super().state()

    backend, arm, root = make_backend(arm=SlowRotationArm(), stream_enabled=False, settle_s=0)
    arming.write(root, True)
    arguments = {'position': arm.pos.tolist(),
                 'quaternion': Rotation.from_euler('xyz', [np.pi, 0, np.pi/2]).as_quat().tolist(),
                 'speed_mps': .015}
    if requested is not None:
        arguments['timeout_s'] = requested
    backend.validate('xarm_move_pose', arguments)
    expected_timeout = 300. if requested is None else requested
    if succeeds:
        result, _ = backend.call('xarm_move_pose', arguments)
        assert result['reached'] and result['timeout_s'] == expected_timeout
        assert result['seconds'] >= duration and arm.holds == 0
    else:
        with pytest.raises(TimeoutError, match=f'pose not reached in {expected_timeout:.1f}s'):
            backend.call('xarm_move_pose', arguments)
        assert arm.holds >= 1
        assert expected_timeout < clock[0]-arm.started < expected_timeout+.1
    assert arm.sdk_timeout == expected_timeout and arm.commands == 1
    assert backend.options['move_timeout_s'] == 300.


@pytest.mark.parametrize('timeout', [0, -1, 601, float('nan'), float('inf'), True, '90'])
def test_cartesian_timeout_rejected_before_dispatch(make_backend, timeout):
    backend, arm, root = make_backend(stream_enabled=False)
    arming.write(root, True)
    arguments = {'position': arm.pos.tolist(), 'timeout_s': timeout}
    with pytest.raises(ValueError, match='timeout_s'):
        backend.validate('xarm_move_pose', arguments)
    with pytest.raises(ValueError, match='timeout_s'):
        backend.call('xarm_move_pose', arguments)
    assert arm.commands == 0


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
    backend.validate('xarm_move_joints', {'positions': (START_Q + [1.3, 0, 0, 0, 0, 0]).tolist()})
    value, _ = backend.call('xarm_move_joints', {'preset': 'home'})
    assert np.allclose(arm.q, HOME_Q) and value['max_joint_error_rad'] == 0


@pytest.mark.parametrize('distance,lag', [(2.25, 0.), (2.38, .019)])
def test_long_home_is_one_call_with_bounded_sdk_commands(make_backend, distance, lag):
    class LaggingArm(FakeArm):
        def __init__(self):
            super().__init__(q=np.asarray(HOME_Q) + [distance, 0, 0, 0, 0, 0])
            self.steps, self.speeds = [], []

        def move_joints(self, joints, **kwargs):
            self.steps.append(float(np.max(np.abs(joints-self.q))))
            self.speeds.append(kwargs['speed_radps'])
            super().move_joints(joints, **kwargs)
            self.q = self.q.copy()
            self.q[0] += lag

    backend, arm, root = make_backend(arm=LaggingArm())
    arming.write(root, True)
    backend.validate('xarm_move_joints', {'preset': 'home'})
    result, _ = backend.call('xarm_move_joints', {'preset': 'home'})
    assert result['segments'] == arm.commands and arm.commands >= 2
    assert max(arm.steps) <= backend.options['max_joint_step_rad']
    assert arm.speeds == [backend.options['joint_speed_radps']]*arm.commands
    assert result['max_joint_error_rad'] <= backend.options['joint_tolerance_rad']


def test_long_move_checks_later_path_before_any_dispatch(make_backend):
    class ObstructedArm(FakeArm):
        def forward_kinematics(self, joints):
            p, q = super().forward_kinematics(joints)
            if 1.7 < joints[0] < 2.0:
                p[2] = .01
            return p, q

    backend, arm, root = make_backend(arm=ObstructedArm())
    arming.write(root, True)
    with pytest.raises(ValueError, match='workspace box|minimum'):
        backend.call('xarm_move_joints', {'positions': (START_Q + [2.25, 0, 0, 0, 0, 0]).tolist()})
    assert arm.commands == 0


def test_disarm_prevents_remaining_joint_segments(make_backend):
    backend, arm, root = make_backend()
    arm.on_step = lambda _: arming.write(root, False)
    arming.write(root, True)
    with pytest.raises(MotionStopped, match='disarmed'):
        backend.call('xarm_move_joints', {'positions': (START_Q + [2.25, 0, 0, 0, 0, 0]).tolist()})
    assert arm.commands == 1 and arm.holds


def test_changed_feedback_cannot_make_next_sdk_segment_exceed_limit(make_backend):
    arm = FakeArm(q=np.asarray(HOME_Q) + [2.25, 0, 0, 0, 0, 0])
    reads = [0]

    def drift_after_completion(arm):
        reads[0] += 1
        if reads[0] == 2:  # First segment completed; next segment has not dispatched.
            arm.q = arm.q + [.3, 0, 0, 0, 0, 0]

    arm.on_step = drift_after_completion
    backend, arm, root = make_backend(arm=arm)
    arming.write(root, True)
    with pytest.raises(ValueError, match='per SDK command'):
        backend.call('xarm_move_joints', {'preset': 'home'})
    assert arm.commands == 1 and arm.holds


def test_joint_segment_rejection_never_dispatches_later_segments(make_backend):
    class RejectingArm(FakeArm):
        def move_joints(self, joints, **kwargs):
            if self.commands == 1:
                self.commands += 1
                raise TargetRejected('injected second segment refusal')
            super().move_joints(joints, **kwargs)

    backend, arm, root = make_backend(arm=RejectingArm())
    arming.write(root, True)
    with pytest.raises(TargetRejected, match='second segment'):
        backend.call('xarm_move_joints', {'positions': (START_Q + [4., 0, 0, 0, 0, 0]).tolist()})
    assert arm.commands == 2 and arm.holds


def test_segmenting_does_not_restart_the_move_timeout(make_backend, monkeypatch):
    clock = [100.]

    class SlowArm(FakeArm):
        def move_joints(self, joints, **kwargs):
            super().move_joints(joints, **kwargs)
            clock[0] += 31.

    backend, arm, root = make_backend(arm=SlowArm(), move_timeout_s=30.)
    monkeypatch.setattr('services.xarm.backend.time.monotonic', lambda: clock[0])
    arming.write(root, True)
    with pytest.raises(TimeoutError, match='timeout before the next segment'):
        backend.call('xarm_move_joints', {'positions': (START_Q + [2.25, 0, 0, 0, 0, 0]).tolist()})
    assert arm.commands == 1 and arm.holds


def test_joint_segments_share_the_permissive_default_deadline(make_backend, monkeypatch):
    clock = [100.]
    remaining = []

    class SlowArm(FakeArm):
        def move_joints(self, joints, **kwargs):
            remaining.append(kwargs['timeout_s'])
            super().move_joints(joints, **kwargs)
            clock[0] += 100.

    backend, arm, root = make_backend(arm=SlowArm())
    monkeypatch.setattr('services.xarm.backend.time.monotonic', lambda: clock[0])
    arming.write(root, True)
    result, _ = backend.call('xarm_move_joints', {'positions': (START_Q + [2.25, 0, 0, 0, 0, 0]).tolist()})
    assert result['seconds'] == 200. and result['segments'] == 2
    assert remaining == [300., 200.] and not arm.holds


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


def test_no_geometry_cannot_publish_placeholder_base_coordinates(make_backend):
    backend, _, _ = make_backend()
    backend.models = {}
    with pytest.raises(ValueError, match='geometry is unavailable'):
        backend.call('xarm_observe', {})


@pytest.mark.parametrize('tool, arguments', [
    ('xarm_status', {'unexpected': 1}),
    ('xarm_hold', {'unexpected': 1}),
    ('xarm_observe', {'cameras': ['unknown']}),
    ('xarm_observe', {'project': [[.3, .2]]}),
    ('xarm_stream', {'seconds_ago': [-1]}),
    ('xarm_stream', {'seconds_ago': [601]}),
    ('xarm_stream', {'seconds_ago': [0], 'sheet': {'seconds': 1}}),
    ('xarm_stream', {'sheet': {'seconds': 1, 'count': True}}),
    ('xarm_move_pose', {'position': [.4, False, .3]}),
    ('xarm_move_pose', {'position': [.4, 0, .3], 'timeout_s': -1}),
    ('xarm_move_pose', {'position': [.4, 0, .3], 'quaternion': [0, 0, 1]}),
    ('xarm_move_joints', {'positions': [0] * 5}),
    ('xarm_move_joints', {'preset': 'missing'}),
    ('xarm_pixel_to_position', {'frame_id': 'abc', 'camera': 'left', 'pixels': [[2]]}),
    ('xarm_pixel_to_position', {'frame_id': 'abc', 'camera': 'left', 'pixels': [[2, 3]], 'radius': -1}),
    ('xarm_pixel_to_position', {'frame_id': 'abc', 'camera': 'left', 'pixels': [[float('nan'), 3]]}),
])
def test_declared_tool_contract_rejects_before_dispatch(make_backend, tool, arguments):
    backend, arm, root = make_backend()
    arming.write(root, True)
    with pytest.raises(ValueError):
        backend.validate(tool, arguments)
    assert arm.commands == arm.holds == 0


def test_sdk_mode_property_is_reported_without_get_mode():
    class ReportOnlySDK:
        mode = 0

        def get_position(self, **kwargs):
            return 0, [300., 0., 300., 0., 0., 0.]

        def get_servo_angle(self, **kwargs):
            return 0, [0.] * 6

        def get_state(self):
            return 0, 4

    driver = XArmDriver.__new__(XArmDriver)
    driver.arm = ReportOnlySDK()
    state = driver.state()
    assert state['mode'] == 0 and state['state'] == 4
