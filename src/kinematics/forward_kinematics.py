from math import sqrt
from typing import List

import numpy as np
from numpy.core.multiarray import ndarray
from numpy.linalg import multi_dot

from src.kinematics.joints import BaseJoint


def forward_kinematics(config: List[BaseJoint], joint_coordinates: List[float]) -> ndarray:
    """
    Calculate the forward kinematics for a given configuration of joints and joint coordinates.
    :param config: Tuple of joints
    :param joint_coordinates: Tuple of joint coordinate values (either m or radian)
    :return: 4x4 Transformation matrix (vectors for TCP coordinate system and vector for TCP position)

    T = | xx yx zx x |
        | xy yy zy y |
        | xz yz zz z |
        |  0  0  0 1 |
    """
    for joint, coordinate_value in zip(config, joint_coordinates):
        # Pass in the joint coordinates to get the complete matrices
        joint.mul(joint_value=coordinate_value)
    if len(config) > 1:
        # Do an optimized calculation of the product of all matrices
        return multi_dot([joint.matrix for joint in config])
    # Create a copy
    return np.array(config[0].matrix)


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
