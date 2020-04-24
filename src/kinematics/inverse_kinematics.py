from math import atan2
from typing import List

import numpy as np

from src.kinematics.joints import BaseJoint


def ik_spherical_wrist(config: List[BaseJoint], tform: np.ndarray, pose_flags=None) -> List[float]:
    """
    Calculate the forward kinematics for a given configuration of joints and joint coordinates.
    :param config: Tuple of joints, offsets are considered
    :param tform: 4x4 Transformation matrix (vectors for TCP coordinate system and vector for TCP position)

    T = | xx yx zx x |
        | xy yy zy y |
        | xz yz zz z |
        |  0  0  0 1 |

    :param pose_flags: Define the preferred pose, defaults to None
    :return: Tuple of joint coordinate values (radian)
    """
    # Validate the config
    if len(config) != 6:
        raise ValueError('Inverse kinematic of function is only possible for 6DOF + spherical wrist.')

    # Extract the relevant data from the matrix
    tcp_pos = tform[0:3, 3]

    # TODO Get the correct wrist center position
    wrist_center_pos = tcp_pos

    # Theta 1 (Hip) represents the angle when the wrist center is given in polar coordinates
    theta1 = atan2(wrist_center_pos[1], wrist_center_pos[0])

    # TODO Theta 2 (Shoulder)
    theta2 = theta1

    # TODO Theta 3
    theta3 = theta2

    # TODO Solve the wrist angles for the TCP orientation

    # TODO Apply the joint offset to the solution(s)

    theta = [theta1, theta2, theta3] * 2
    return [angle - joint.zero_offset for angle, joint in zip(theta, config)]
