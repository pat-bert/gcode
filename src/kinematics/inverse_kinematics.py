from typing import List

import numpy as np

from src.kinematics.joints import BaseJoint


def ik_spherical_wrist(config: List[BaseJoint], tform: np.ndarray) -> List[float]:
    """
    Calculate the forward kinematics for a given configuration of joints and joint coordinates.
    :param config: Tuple of joints
    :param tform: 4x4 Transformation matrix (vectors for TCP coordinate system and vector for TCP position)

    T = | xx yx zx x |
        | xy yy zy y |
        | xz yz zz z |
        |  0  0  0 1 |

    :return: Tuple of joint coordinate values (radian)
    """
    num_joints = len(config)
    tcp_pos = tform[0:3, 3]

    # TODO Solve the arm angles for the position of the wrist center

    # TODO Solve the wrist angles for the TCP orientation

    # TODO Apply the joint offset to the solution(s)

    return [0] * 6
