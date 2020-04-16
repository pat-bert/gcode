from typing import List

from numpy import ones
from numpy.core.multiarray import ndarray
from numpy.linalg import multi_dot

from kinematics.joints import BaseJoint


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
    else:
        # Create a copy
        return multi_dot(config[0].matrix, ones(4))
