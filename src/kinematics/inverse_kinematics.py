from math import atan2, pi, acos
from typing import List

import numpy as np

from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joints import BaseJoint


def ik_spherical_wrist(config: List[BaseJoint], tform: np.ndarray, pose_flags=None) -> List[float]:
    """
    Calculate the forward kinematics for a given configuration of joints and joint coordinates.
    :param config: Tuple of joints, offsets are considered
    :param tform: 4x4 Transformation matrix for flange frame

    T = | xx yx zx x |
        | xy yy zy y |
        | xz yz zz z |
        |  0  0  0 1 |

    Transformation from tool frame to flange frame needs to be done separately.

    :param pose_flags: Define the preferred pose, defaults to None
    :return: Tuple of joint coordinate values (radian)
    """
    # Validate the config
    if len(config) != 6:
        raise ValueError('Inverse kinematic of function is only possible for 6DOF + spherical wrist.')

    # Extract the relevant data from the matrix
    tcp_pos = tform[0:3, 3]
    xdir = tform[0:3, 0]
    ydir = tform[0:3, 1]
    zdir = tform[0:3, 2]

    """
    Calculate theta 1
    """
    # Get the wrist center in base frame from the flange position in base frame
    wrist_center_pos_f1 = tcp_pos - zdir * config[5].d

    # Theta 1 (Hip) represents the angle when the wrist center is given in polar coordinates
    theta1 = atan2(wrist_center_pos_f1[1], wrist_center_pos_f1[0])

    """
    Calculate theta 3
    """
    # Get origin of joint 2 in base frame
    tjoint21 = forward_kinematics([config[0]], [theta1])
    origin_joint2_f1 = tjoint21[0:3, 3]

    # Get the vector from origin of joint 2 to wrist center
    wrist_center_pos_f2 = wrist_center_pos_f1 - origin_joint2_f1

    # TODO Link lengths
    l2 = config[1].a
    l4 = config[3].d

    # Get both solutions for the auxiliary angle phi using cosine law
    vector_len_p14 = np.linalg.norm(wrist_center_pos_f2)
    aux_phi_1 = acos((l2 ** 2 + l4 ** 2 - vector_len_p14) / (2 * l2 * l4))
    aux_phi_2 = 2 * pi - aux_phi_1

    # Calculate solutions for theta3 based on phi
    theta3_1 = 1.5 * pi - aux_phi_1
    theta3_2 = 1.5 * pi - aux_phi_2

    # TODO Select appropriate solution or collect all
    theta3 = theta3_1

    """
    Calculate theta 2
    """
    # Transform vector to wrist center given in frame 2 to frame 1
    tjoint12 = tjoint21.transpose()
    wrist_center_pos_f2_f1 = np.dot(tjoint12[0:3, 0:3], wrist_center_pos_f2)

    # Calculate auxiliary angles beta1 and beta2
    beta1 = atan2(- wrist_center_pos_f2_f1[1], wrist_center_pos_f2_f1[0])
    beta2_1 = acos((l2 ** 2 + vector_len_p14 - l4 ** 2) / (2 * l2 * vector_len_p14))
    beta2_2 = 2 * pi - beta2_1

    # Calculate solutions for theta3 based on beta1 and beta2
    theta2_1 = -(beta1 + beta2_1)
    theta2_2 = -(beta1 + beta2_2)

    # TODO Select appropriate solution or collect all
    theta2 = theta2_1

    # TODO Solve the wrist angles for the TCP orientation
    """
    Calculate theta 4
    """
    theta4 = 0

    """
    Calculate theta 5
    """
    theta5 = 0

    """
    Calculate theta 6
    """
    theta6 = 0

    # Bundle all the angles
    theta = [theta1, theta2, theta3, theta4, theta5, theta6]

    # Apply the joint offset to the solutions
    return [angle - joint.zero_offset for angle, joint in zip(theta, config)]
