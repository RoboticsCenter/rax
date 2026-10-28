"""Piper offset-variant modified DH model; metres and radians, flange frame by default."""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


LIMITS = np.deg2rad([[-150, 150], [0, 179.91], [-170, 0],
                    [-99.98, 99.98], [-69.9, 69.9], [-120, 120]])
ALPHA = np.array([0, -np.pi / 2, 0, np.pi / 2, -np.pi / 2, np.pi / 2])
LINK_LENGTHS = (0, 0, .28503, -.02198, 0, 0)
LINK_OFFSETS = (.123, 0, 0, .25075, 0, .091)
OFFSET = np.deg2rad([0, -172.22, -102.78, 0, 0, 0])


def vector(value, size, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f'{name} needs {size} finite values')
    return array


def frames(joints, tool_transform=None):
    joints = vector(joints, 6, 'joints')
    transform = np.eye(4)
    result = [transform.copy()]
    for alpha, length, angle, distance in zip(ALPHA, LINK_LENGTHS, joints + OFFSET, LINK_OFFSETS):
        cosine, sine = np.cos(angle), np.sin(angle)
        ca, sa = np.cos(alpha), np.sin(alpha)
        transform = transform @ np.array([
            [cosine, -sine, 0, length],
            [sine * ca, cosine * ca, -sa, -sa * distance],
            [sine * sa, cosine * sa, ca, ca * distance],
            [0, 0, 0, 1],
        ])
        result.append(transform.copy())
    if tool_transform is not None:
        result.append(transform @ tool_transform)
    return result


def fk(joints, tool_transform=None):
    return frames(joints, tool_transform)[-1]


def pose_error(actual, goal):
    return np.concatenate((actual[:3, 3] - goal[:3, 3],
                           Rotation.from_matrix(actual[:3, :3] @ goal[:3, :3].T).as_rotvec()))


def ik(goal, seed, tool_transform=None):
    lower, upper = LIMITS.T
    result = least_squares(
        lambda joints: pose_error(fk(joints, tool_transform), goal),
        np.clip(vector(seed, 6, 'seed'), lower + 1e-8, upper - 1e-8),
        bounds=(lower, upper), max_nfev=400, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    error = pose_error(fk(result.x, tool_transform), goal)
    if np.linalg.norm(error[:3]) > .0005 or np.linalg.norm(error[3:]) > .002:
        raise ValueError('No accurate bounded IK solution from the current pose')
    return result.x
