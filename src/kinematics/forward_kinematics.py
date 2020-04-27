from math import sqrt, atan2, cos
from typing import List

import numpy as np
from numpy.core.multiarray import ndarray
from numpy.linalg import multi_dot

from src.kinematics.joints import BaseJoint


def forward_kinematics(config: List[BaseJoint], joint_coordinates: List[float]) -> ndarray:
    """
    Calculate the forward kinematics for a given configuration of joints and joint coordinates.
    :param config: Tuple of joints
    :param joint_coordinates: Tuple of joint coordinate values (either mm or radian)
    :return: 4x4 Transformation matrix (vectors for TCP coordinate system and vector for TCP position)
    :raises: ValueError if the lengths of config and joint_coordinates are unequal

    T = | xx yx zx x |
        | xy yy zy y |
        | xz yz zz z |
        |  0  0  0 1 |
    """
    if len(config) == len(joint_coordinates):
        for joint, coordinate_value in zip(config, joint_coordinates):
            # Pass in the joint coordinates to get the complete matrices
            joint.mul(joint_value=coordinate_value)
        if len(config) > 1:
            # Do an optimized calculation of the product of all matrices
            return multi_dot([joint.matrix for joint in config])
        # Create a copy
        return np.array(config[0].matrix)
    raise ValueError('Joint coordinates and joints must be of same length.')


def tform2quat(tform: ndarray) -> List[float]:
    """
    Convert a homogenous matrix (4x4) to a quaternion (w, x, y, z)
    :param tform: Homogenous matrix (4x4)
    :return: Quaternion as list of floats
    """
    w = sqrt(tform[1, 1] + tform[2, 2] + tform[3, 3] + 1) / 2
    x = (tform[3, 2] - tform[2, 3]) / (4 * w)
    y = (tform[1, 3] - tform[3, 1]) / (4 * w)
    z = (tform[2, 1] - tform[1, 2]) / (4 * w)
    return [w, x, y, z]


def tform2euler(tform: ndarray) -> List[float]:
    """
    Calculates euler angles from a homogeneous matrix.
    :param tform: Homogenous matrix (4x4)
    :return: ABC in deg as used by Mitsubishi (ZY'X'', alpha and gamma swapped)
    """
    beta = atan2(-tform[2, 0], sqrt(tform[2, 1] ** 2 + tform[2, 2] ** 2))
    c_b = cos(beta)

    if abs(c_b) > 1e-10:
        gamma = atan2(tform[1, 0] / c_b, tform[0, 0] / c_b)
        alpha = atan2(tform[2, 1] / c_b, tform[2, 2] / c_b)
    else:
        gamma = 0
        alpha = atan2(tform[0, 1], tform[1, 1])
        if beta < 0:
            alpha *= -1
    return [alpha, beta, gamma]
