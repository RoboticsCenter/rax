import json
from pathlib import Path
import threading
import time

import cv2 as cv
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from services.controller.config import Configuration
from services.franka import arming
from services.piper.backend import PiperBackend
from services.piper.config import DEFAULTS, options
from services.piper.kinematics import LIMITS, fk, ik, pose_error
from services.piper.transport import PairLease, PassiveFeedback
from services.workspace import Setup, build, recipe_for


class FakeArm:
    def __init__(self):
        self.joints = np.deg2rad([0, 60, -60, 0, 0, 0])
        self.commands = []
        self.stops = 0
        self.on_segment = None
        self.disabled = False
        self.fail_hold = False

    def state(self):
        return {'joints': self.joints.tolist(), 'serial': DEFAULTS['arm_usb_serial'],
                'side': 'right', 'interface': 'can1', 'driver_state': 'disabled' if self.disabled else 'enabled',
                'streaming': not self.disabled, 'arm_status': 0, 'error': 0,
                'enabled': [not self.disabled] * 6, 'busy': False}

    def segment(self, goal, seconds):
        self.commands.append((np.asarray(goal).copy(), seconds))
        self.joints = np.asarray(goal).copy()
        if self.on_segment:
            self.on_segment()

    def hold(self):
        self.stops += 1
        if self.fail_hold:
            raise RuntimeError('stop acknowledgement missing')

    def close(self):
        pass


class FakeHand:
    def __init__(self):
        self.positions = [380, 340, 50, 50, 50, 50]
        self.commands = []
        self.stops = 0

    def state(self):
        return {'positions': self.positions.copy(), 'speeds': [0] * 6,
                'motor_states': [0] * 6, 'serial': DEFAULTS['hand_serial']}

    def move(self, positions):
        self.commands.append(positions)
        self.positions = list(positions)

    def hold(self):
        self.stops += 1

    def close(self):
        pass


class FakeCamera:
    def grab(self):
        ok, image = cv.imencode('.jpg', np.zeros((24, 32, 3), np.uint8))
        assert ok
        return time.time(), image.tobytes()

    def close(self):
        pass


def backend(tmp_path, **settings):
    config = Configuration(root=tmp_path / 'session', task='test', robot='piper',
                           observation_profile='rgb-only', backend_options=settings)
    return PiperBackend(config, arm=FakeArm(), hand=FakeHand(), camera=FakeCamera())


def armed_backend(tmp_path, **settings):
    result = backend(tmp_path, motion_commissioned=True, **settings)
    arming.write(result.config.root, True, minutes=5)
    return result


def test_default_port_is_read_only_and_status_does_not_enable(tmp_path):
    robot = backend(tmp_path)
    robot.arm.disabled = True
    value, _ = robot.call('piper_status', {})
    assert not value['motion_ready'] and value['arm_driver_state'] == 'disabled'
    with pytest.raises(ValueError, match='not commissioned'):
        robot.call('piper_move_joints', {'positions': robot.arm.joints.tolist()})
    assert not robot.arm.commands and not robot.hand.commands
    robot.close()
    robot.close()
    assert robot.arm.stops == robot.hand.stops == 0


def test_status_does_not_claim_motion_ready_outside_the_joint_envelope(tmp_path):
    robot = armed_backend(tmp_path)
    robot.arm.joints[1] = -.01
    result = robot.status()
    assert not result['motion_ready']
    assert 'joint limits' in result['not_ready_reason']


def test_failed_assembly_stop_is_control_uncertain_in_the_actual_frontend(tmp_path):
    from services.controller.frontend import Frontend
    from services.controller.session import Session
    robot = backend(tmp_path)
    robot.arm.fail_hold = True
    frontend = Frontend(Session(robot.config), robot)
    result = frontend.call('piper_hold', {})
    assert result['isError'] and frontend.control_uncertain
    failure = json.loads(result['content'][0]['text'])['error']
    assert failure['control_uncertain']


def test_arming_and_hardware_readiness_are_independent_gates(tmp_path):
    robot = backend(tmp_path, motion_commissioned=True)
    with pytest.raises(ValueError, match='not armed'):
        robot.call('revo_move', {'positions': robot.hand.positions})
    arming.write(robot.config.root, True)
    robot.arm.disabled = True
    with pytest.raises(RuntimeError, match='already be healthy'):
        robot.call('revo_move', {'positions': robot.hand.positions})
    assert not robot.hand.commands


@pytest.mark.parametrize('arguments', [{}, {'positions': [0] * 7}, {'positions': [False] * 6},
                                        {'positions': [float('nan')] * 6},
                                        {'positions': [0] * 6, 'speed': 100}])
def test_invalid_arguments_are_rejected_before_motion(tmp_path, arguments):
    robot = armed_backend(tmp_path)
    with pytest.raises(ValueError):
        robot.call('piper_move_joints', arguments)
    assert not robot.arm.commands


def test_small_joint_move_uses_short_segments_and_speed_limits(tmp_path):
    robot = armed_backend(tmp_path)
    start = robot.arm.joints.copy()
    goal = start + [.01, 0, 0, 0, 0, 0]
    result, images = robot.call('piper_move_joints', {'positions': goal.tolist()})
    assert result['ok'] and not images
    assert len(robot.arm.commands) > 1
    previous = start
    for target, seconds in robot.arm.commands:
        assert seconds <= robot.options['segment_s']
        assert np.max(np.abs(target - previous)) / seconds <= robot.options['joint_speed_radps']
        previous = target
    assert np.allclose(robot.arm.joints, goal)


def test_disarm_during_motion_stops_both_devices(tmp_path):
    robot = armed_backend(tmp_path)
    robot.arm.on_segment = lambda: arming.write(robot.config.root, False)
    goal = robot.arm.joints + [.01, 0, 0, 0, 0, 0]
    with pytest.raises(ValueError, match='disarmed'):
        robot.call('piper_move_joints', {'positions': goal.tolist()})
    assert len(robot.arm.commands) == 1
    assert robot.arm.stops == robot.hand.stops == 1


def test_unacknowledged_stop_is_never_reported_safe(tmp_path):
    robot = armed_backend(tmp_path)
    robot.arm.fail_hold = True
    with pytest.raises(RuntimeError, match='stop unconfirmed'):
        robot.call('piper_hold', {})
    assert robot.hand.stops == 1


def test_joint_limits_step_and_clearance_block_all_dispatch(tmp_path):
    robot = armed_backend(tmp_path)
    for goal in ([0] * 6, (robot.arm.joints + [.3, 0, 0, 0, 0, 0]).tolist()):
        with pytest.raises(ValueError):
            robot.call('piper_move_joints', {'positions': goal})
    robot.box_max[2] = .2
    with pytest.raises(ValueError, match='workspace'):
        robot.call('piper_move_joints', {'positions': robot.arm.joints.tolist()})
    assert not robot.arm.commands


def test_joint_path_checks_intermediate_configurations(tmp_path, monkeypatch):
    robot = armed_backend(tmp_path)
    checked = []
    actual = robot._check_configuration
    def guarded(joints):
        checked.append(joints.copy())
        if .004 < joints[0] < .008:
            raise ValueError('intermediate collision')
        actual(joints)
    monkeypatch.setattr(robot, '_check_configuration', guarded)
    with pytest.raises(ValueError, match='intermediate collision'):
        robot.call('piper_move_joints', {'positions': (robot.arm.joints + [.01, 0, 0, 0, 0, 0]).tolist()})
    assert len(checked) > 2 and not robot.arm.commands


def test_cartesian_requires_measured_tool_calibration(tmp_path):
    robot = armed_backend(tmp_path)
    transform = fk(robot.arm.joints)
    with pytest.raises(ValueError, match='calibrated hand TCP'):
        robot.call('piper_move_pose', {'position': transform[:3, 3].tolist(),
                                      'quaternion': Rotation.from_matrix(transform[:3, :3]).as_quat().tolist()})


def test_revo_targets_are_bounded_normalized_motors_not_gripper_width(tmp_path):
    robot = armed_backend(tmp_path)
    with pytest.raises(ValueError):
        robot.call('revo_move', {'positions': [0] * 6})
    with pytest.raises(ValueError, match='step too large'):
        robot.call('revo_move', {'positions': [800, 340, 50, 50, 50, 50]})
    with pytest.raises(ValueError, match='integer'):
        robot.call('revo_move', {'positions': [380., 340, 50, 50, 50, 50]})
    result, _ = robot.call('revo_move', {'positions': [390, 340, 60, 60, 60, 60]})
    assert result['ok'] and result['grasp_verified'] is False
    assert len(robot.hand.commands) == 1


def test_rgb_observation_saves_evidence_without_fake_calibration(tmp_path):
    robot = backend(tmp_path)
    result, images = robot.call('piper_observe', {})
    path = robot.config.root / 'agent-workspace' / result['directory']
    saved = json.loads((path / 'meta.json').read_text())
    assert (path / 'overview.jpg').is_file()
    assert saved['cameras']['overview']['depth'] is None
    assert 'K' not in saved['cameras']['overview']
    assert not saved['camera_geometry'] and len(images) == 1
    assert not robot.arm.commands and not robot.hand.commands


def test_stream_missing_and_invalid_ages(tmp_path):
    robot = backend(tmp_path)
    result, images = robot.call('piper_stream', {})
    assert not images and result['frames'][0]['available'] is False
    for age in (-1, 601, float('inf')):
        with pytest.raises(ValueError):
            robot.call('piper_stream', {'seconds_ago': [age]})


def test_ik_uses_full_rotation_and_joint_bounds():
    seed = np.deg2rad([0, 60, -60, 0, 0, 0])
    transform = fk(seed)
    result = ik(transform, seed)
    assert np.linalg.norm(pose_error(fk(result), transform)) < .001
    half_turn = transform.copy()
    half_turn[:3, :3] = Rotation.from_rotvec([np.pi, 0, 0]).as_matrix() @ transform[:3, :3]
    assert np.linalg.norm(pose_error(transform, half_turn)[3:]) == pytest.approx(np.pi)
    assert np.all(result >= LIMITS[:, 0]) and np.all(result <= LIMITS[:, 1])


@pytest.mark.parametrize('settings', [{'motion_commissioned': 'yes'}, {'joint_speed_radps': float('nan')},
                                     {'segment_s': 1}, {'hand_max_step': 1000}, {'unknown': 1},
                                     {'tool_quaternion_xyzw': [0, 0, 0, 0]},
                                     {'hand_target_min': [0] * 6}, {'cameras': []}])
def test_bad_operator_settings_fail_closed(settings):
    with pytest.raises(ValueError):
        options(settings)


def test_pair_lease_is_exclusive_and_reusable(monkeypatch, tmp_path):
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    first = PairLease('ABC123')
    with pytest.raises(BlockingIOError):
        PairLease('ABC123')
    first.close()
    second = PairLease('ABC123')
    second.close()


def test_independent_can_freshness_and_sdk_agreement():
    feedback = PassiveFeedback.__new__(PassiveFeedback)
    feedback.lock = threading.Lock()
    feedback.error = None
    now = time.monotonic()
    feedback.frames = {key: (now, bytes(8)) for key in feedback.required}
    feedback.check(np.zeros(6))
    with pytest.raises(RuntimeError, match='disagrees'):
        feedback.check(np.ones(6) * 2)
    feedback.frames[0x2A1] = (now - 10, bytes(8))
    with pytest.raises(RuntimeError, match='stale'):
        feedback.check(np.zeros(6))


def test_piper_workspace_advertises_only_actual_tools(tmp_path):
    workspace = tmp_path / 'workspace'
    build(recipe_for(), Setup('piper', 'test', observation_profile='rgb-only'), workspace)
    tools = json.loads((workspace / 'MCP_CONTRACT.json').read_text())['tools']
    robot = backend(tmp_path)
    assert set(tools) - {'status', 'request_operator', 'gemini_generate'} == {item['name'] for item in robot.tools()}
    for path in workspace.rglob('*.md'):
        text = path.read_text()
        assert '{{' not in text and '{%' not in text
        assert 'franka_' not in text and 'xarm_' not in text
    interface = (workspace / '.agents/skills/robot-interface/SKILL.md').read_text()
    assert 'six integer normalized' in interface
    assert 'No commissioned wrist camera' in (workspace / '.agents/skills/wrist-camera-inspection/SKILL.md').read_text()
    manual = (workspace / '.agents/skills/manual-robot-control/SKILL.md').read_text()
    assert 'with an empty gripper' not in manual
    assert 'No wrist camera is available' in manual
    assert 'no wrist camera is available' in (workspace / 'AGENTS.md').read_text()
    stream = (workspace / '.agents/skills/live-stream/SKILL.md').read_text()
    assert 'api.stream frames' not in stream
    assert 'api.stream sheet overview' in stream
    assert 'api.stream robot --seconds 30' in stream
    autonomous = (workspace / '.agents/skills/autonomous-control/SKILL.md').read_text()
    assert '"ok": True' not in autonomous
    assert 'Controller not implemented' in autonomous
    gemini = (workspace / '.agents/skills/gemini/SKILL.md').read_text()
    assert '### Single-view pointing' in gemini
    assert 'confirm with measured motion' not in gemini


def test_piper_workspace_rejects_unavailable_depth():
    with pytest.raises(ValueError, match='rgb-only'):
        Setup('piper', observation_profile='rgbd')
    assert Setup('piper', observation_profile='rgb-only').context()['robot']['pixel'] is None


def test_piper_sandbox_protects_both_owners_and_operator_inputs(tmp_path):
    from harness.claude_cli.launcher import protected_paths
    robot = backend(tmp_path)
    paths = protected_paths(robot.config)
    assert Path('/tmp/piper_right_arm.sock') in paths
    assert Path('/tmp/piper_left_arm.sock') in paths
    assert robot.config.root / 'operator' in paths
    assert robot.config.root / 'operator-input.json' in paths
    assert Path('/dev/bus/usb') in paths
