"""Read-only, sampled kinematic checks for calibration Cartesian sweeps."""
import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .backend import DEFAULTS, LOWER, UPPER


def jacobian_singular_values(driver, joints):
    """Normalize translation by 0.3 m so position and rotation have similar scale."""
    columns = []
    epsilon = .003
    for axis in range(6):
        delta = np.zeros(6)
        delta[axis] = epsilon
        plus, qplus = driver.forward_kinematics(joints + delta)
        minus, qminus = driver.forward_kinematics(joints - delta)
        rotation = (Rotation.from_quat(qplus) * Rotation.from_quat(qminus).inv()).as_rotvec()
        columns.append(np.r_[(plus - minus) / .3, rotation] / (2 * epsilon))
    return np.linalg.svd(np.stack(columns, axis=1), compute_uv=False)


def validate_sweep_path(plan, start, driver, options):
    """Screen the complete path, including return, without sending motion commands.

    This is a sampled kinematic screen, not collision checking. Runtime controller
    protections remain authoritative. Never change the plan after checking it.
    """
    opts = {**DEFAULTS, **options}
    lower = np.asarray(opts['workspace_min']) + opts['workspace_guard_margin_m']
    upper = np.asarray(opts['workspace_max']) - opts['workspace_guard_margin_m']
    previous_position = np.asarray(start['tcp_position'], float)
    previous_rotation = Rotation.from_quat(start['tcp_quaternion'])
    previous_joints = np.asarray(start['joints'], float)
    goals = [*plan, {'index': 'return', 'position': previous_position.tolist(),
                    'quaternion': previous_rotation.as_quat().tolist()}]
    report = {'ok': False, 'translation_step_m': .005, 'rotation_step_deg': 2.,
              'jacobian_translation_scale_m': .3, 'min_singular_value_limit': .05,
              'condition_number_limit': 50., 'max_joint_step_limit_rad': .2, 'segments': []}
    for goal in goals:
        position = np.asarray(goal['position'], float)
        rotation = Rotation.from_quat(goal['quaternion'])
        distance = np.linalg.norm(position - previous_position)
        angle = (previous_rotation.inv() * rotation).magnitude()
        count = max(1, math.ceil(distance / .005), math.ceil(angle / np.radians(2)))
        interpolation = Slerp([0., 1.], Rotation.from_quat(
            [previous_rotation.as_quat(), rotation.as_quat()]))
        segment = {'index': goal['index'], 'samples': []}
        for fraction in np.linspace(0., 1., count + 1):
            point = previous_position + fraction * (position - previous_position)
            attitude = interpolation(fraction)
            label = f"pose {goal['index']} at {fraction:.0%}"
            if np.any(point < lower) or np.any(point > upper) or point[2] < opts['link_min_z']:
                raise ValueError(f'{label}: outside workspace guard')
            pose = [*(point * 1000.).tolist(), *attitude.as_euler('xyz').tolist()]
            code, values = driver.arm.get_inverse_kinematics(
                pose, input_is_radian=True, return_is_radian=True)
            if code != 0:
                raise ValueError(f'{label}: inverse kinematics failed ({code})')
            joints = np.asarray(values[:6], float)
            if joints.shape != (6,) or not np.isfinite(joints).all():
                raise ValueError(f'{label}: invalid inverse kinematics result')
            margin = opts['joint_limit_margin_rad']
            if np.any(joints < LOWER + margin) or np.any(joints > UPPER - margin):
                raise ValueError(f'{label}: outside joint-limit margin')
            jump = float(np.max(np.abs(joints - previous_joints)))
            if jump > .2:
                raise ValueError(f'{label}: joint-solution discontinuity ({jump:.3f} rad)')
            problem = driver.check_joint_limit(joints)
            if problem:
                raise ValueError(f'{label}: {problem}')
            singular = jacobian_singular_values(driver, joints)
            minimum = float(singular[-1])
            condition = float(singular[0] / minimum) if minimum > 0 else float('inf')
            if not np.isfinite(singular).all() or minimum < .05 or condition > 50:
                raise ValueError(f'{label}: insufficient singularity margin '
                                 f'(sigma={minimum:.4f}, condition={condition:.2f})')
            segment['samples'].append({'fraction': float(fraction), 'joints': joints.tolist(),
                                      'min_singular_value': minimum, 'condition_number': condition,
                                      'max_joint_step_rad': jump})
            previous_joints = joints
        report['segments'].append(segment)
        previous_position, previous_rotation = position, rotation
    report['ok'] = True
    return report
