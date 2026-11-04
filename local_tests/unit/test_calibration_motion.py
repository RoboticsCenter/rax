import json
import runpy
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from services.xarm.calibration_motion import validate_sweep_path


class ReadOnlyArm:
    baseline = np.array([0., -.5, -1.5, 0., 1., 0.])
    origin = np.array([.4, 0., .5])

    def __init__(self, reject_middle=False, singular=False):
        self.arm = self
        self.reject_middle, self.singular = reject_middle, singular

    def get_inverse_kinematics(self, pose, **kwargs):
        position = np.asarray(pose[:3]) / 1000.
        if self.reject_middle and .409 < position[0] < .411:
            return 10, []
        q = self.baseline + np.r_[(position - self.origin) / .3, pose[3:]]
        return 0, q.tolist()

    def check_joint_limit(self, joints):
        return None

    def forward_kinematics(self, joints):
        delta = joints - self.baseline
        if self.singular:
            delta[5] = 0.
        return self.origin + .3 * delta[:3], Rotation.from_euler('xyz', delta[3:]).as_quat()


def inputs():
    start = {'tcp_position': ReadOnlyArm.origin.copy(), 'tcp_quaternion': [0., 0., 0., 1.],
             'joints': ReadOnlyArm.baseline.copy()}
    plan = [{'index': 0, 'position': [.42, 0., .5], 'quaternion': [0., 0., 0., 1.]}]
    return start, plan


def test_screen_checks_intermediate_poses_and_return_without_motion_interface():
    start, plan = inputs()
    report = validate_sweep_path(plan, start, ReadOnlyArm(), {})
    assert report['ok']
    assert [segment['index'] for segment in report['segments']] == [0, 'return']
    assert len(report['segments'][0]['samples']) >= 5
    np.testing.assert_allclose(report['segments'][-1]['samples'][-1]['joints'], start['joints'])
    with pytest.raises(ValueError, match='inverse kinematics failed'):
        validate_sweep_path(plan, start, ReadOnlyArm(reject_middle=True), {})


def test_screen_refuses_singular_path_and_workspace_boundary():
    start, plan = inputs()
    with pytest.raises(ValueError, match='singularity margin'):
        validate_sweep_path(plan, start, ReadOnlyArm(singular=True), {})
    plan[0]['position'][0] = .85
    with pytest.raises(ValueError, match='workspace guard'):
        validate_sweep_path(plan, start, ReadOnlyArm(), {})


def test_screen_refuses_joint_branch_jump():
    start, plan = inputs()
    start['joints'][3] = .3
    with pytest.raises(ValueError, match='joint-solution discontinuity'):
        validate_sweep_path(plan, start, ReadOnlyArm(), {})


def test_downward_sweep_retains_rotations_and_reserved_views():
    project = Path(__file__).resolve().parents[2]
    script = runpy.run_path(str(project/'scripts/xarm-bimanual-charuco-calibrate'))
    workflow = script['original_workflow']()
    start, _ = inputs()
    up = script['calibration_plan'](workflow, start, {}, 'up')
    down = script['calibration_plan'](workflow, start, {}, 'down')
    assert len(down) == 18
    assert sum(p['held_out'] for p in down) == 3
    for a, b in zip(up, down):
        assert a['index'] == b['index'] and a['held_out'] == b['held_out']
        np.testing.assert_allclose(a['quaternion'], b['quaternion'])
        np.testing.assert_allclose(a['position'][:2], b['position'][:2])
        assert b['position'][2] <= start['tcp_position'][2]
        assert a['position'][2] + b['position'][2] == pytest.approx(2 * start['tcp_position'][2])


@pytest.mark.parametrize('side,lo,hi', [('left', -15, 0), ('right', 0, 15)])
def test_inward_wrist_sweep_stays_within_original_rotation_envelope(side, lo, hi):
    project = Path(__file__).resolve().parents[2]
    script = runpy.run_path(str(project/'scripts/xarm-bimanual-charuco-calibrate'))
    start, _ = inputs()
    plan = script['calibration_plan'](script['original_workflow'](), start, {}, 'down', side)
    assert len(plan) == 18 and sum(p['held_out'] for p in plan) == 3
    np.testing.assert_allclose(plan[0]['position'], start['tcp_position'])
    np.testing.assert_allclose(plan[0]['quaternion'], start['tcp_quaternion'])
    for pose in plan:
        angles = Rotation.from_quat(pose['quaternion']).as_euler('xyz', degrees=True)
        assert lo - 1e-8 <= angles[0] <= hi + 1e-8
        assert abs(angles[1]) <= 15 + 1e-8 and abs(angles[2]) <= 20 + 1e-8
        assert pose['position'][2] <= start['tcp_position'][2]


def test_coverage_planner_inherits_downward_inward_envelope(tmp_path):
    project = Path(__file__).resolve().parents[2]
    script = runpy.run_path(str(project/'scripts/xarm-bimanual-charuco-calibrate'))
    start, _ = inputs()
    stored_start = {k: np.asarray(v).tolist() for k, v in start.items()}
    settings = {'squares_x': 10, 'squares_y': 13, 'square_m': .02}
    preflight = {'start': {'left': stored_start}, 'board': settings,
                 'initial_frame': str(tmp_path), 'sweep_z_direction': 'down',
                 'frame_wrists_inward': True, 'coverage_margin_px': 20}
    (tmp_path/'preflight.json').write_text(json.dumps(preflight))
    board_pose = np.eye(4)
    board_pose[:3, 3] = [.3, -.13, 1.0]
    fit = {'wrists': {'left': {'T_cam_tcp': np.eye(4).tolist(),
                              'T_base_board': board_pose.tolist()}}}
    (tmp_path/'result.json').write_text(json.dumps(fit))
    camera = {'width': 1280, 'height': 720, 'K': [[600, 0, 640], [0, 600, 360], [0, 0, 1]],
              'distortion_coeffs': [0.] * 5}
    (tmp_path/'meta.json').write_text(json.dumps({'cameras': {'wrist_left': camera}}))
    plan = script['coverage_plan']('left', start, {}, tmp_path, script['original_workflow']())
    assert len(plan) == 18 and sum(p['held_out'] for p in plan) == 3
    assert plan[0]['home_anchor'] and not plan[0]['held_out']
    np.testing.assert_allclose(plan[0]['position'], start['tcp_position'])
    np.testing.assert_allclose(plan[0]['quaternion'], start['tcp_quaternion'])
    for pose in plan[1:]:
        assert -.05 <= pose['translation_delta_m'][2] <= -.015
        assert -15 <= pose['rotation_delta_deg'][0] <= 0
        assert pose['predicted_board_margin_px'] >= 20


def test_wrist_only_scope_requires_both_wrists_and_excludes_fixed_views():
    project = Path(__file__).resolve().parents[2]
    script = runpy.run_path(str(project/'scripts/xarm-bimanual-charuco-calibrate'))
    cameras = [{'name': 'front', 'mount': 'fixed'},
               {'name': 'wrist_left', 'mount': 'tcp', 'arm': 'left'},
               {'name': 'wrist_right', 'mount': 'tcp', 'arm': 'right'}]
    assert script['calibration_camera_specs'](cameras) == cameras
    assert script['calibration_camera_specs'](cameras, True) == cameras[1:]
    with pytest.raises(ValueError, match='one wrist camera per arm'):
        script['calibration_camera_specs'](cameras[:-1], True)
