from math import sqrt, atan2, cos, atan, hypot, sin
from typing import List

import numpy as np
from numpy.core.multiarray import ndarray
from numpy.linalg import multi_dot, inv

from src.kinematics.joints import BaseJoint, JointType
from src.kinematics.joints import WristSingularity, ElbowSingularity, ShoulderSingularity


def geometric_jacobian(config: List[BaseJoint], joint_coordinates: List[float]) -> ndarray:
    """
    Calculate the jacobian for the current joint coordinates.
    :param config: Tuple of joints
    :param joint_coordinates: Tuple of joint coordinate values (either mm or radian)
    :return: 6xn matrix for n joints
    """
    # Calculate all individual transformation matrices
    tform_i_j_list = [forward_kinematics([joint], [joint_val]) for joint, joint_val in zip(config, joint_coordinates)]

    # Calculate cumulative transformation matrices (transformation from base frame)
    total_tform = np.eye(4)

    # Initialize with base frame
    tform_0_i_list = [np.eye(4)]

    for tform_i_j in tform_i_j_list:
        total_tform = total_tform.dot(tform_i_j)
        tform_0_i_list.append(total_tform)

    # End-effector position
    tform_0_ee = tform_0_i_list[-1]
    p_ee = tform_0_ee[0:3, 3]

    jacobian = np.zeros((6, len(config)))
    for (col, tform_0_i), joint in zip(enumerate(tform_0_i_list[:-1]), config):
        # Select joint axis given in base frame
        current_z_axis_base_frame = tform_0_i[0:3, 2]

        # Distinguish rotational and translational joints, fill current column of jacobian
        if joint.joint_type is JointType.ROTATIONAL:
            joint_origin = tform_0_i[0:3, 3]
            jacobian[0:3, col] = np.cross(current_z_axis_base_frame, p_ee - joint_origin)
            jacobian[3:6, col] = current_z_axis_base_frame
        else:
            jacobian[0:3, col] = current_z_axis_base_frame
    return jacobian


def vec3_cross(a: ndarray, b: ndarray) -> ndarray:
    return np.array(
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]
        ]
    )


def right_generalized_inverse_jacobian(jacobian: ndarray) -> ndarray:
    jacobian_t = jacobian.transpose()
    try:
        return jacobian_t @ inv(jacobian @ jacobian_t)
    except TypeError:
        raise ValueError(jacobian @ jacobian_t)


def forward_kinematics(config: List[BaseJoint], joint_coordinates: List[float], subtract_offset=False) -> ndarray:
    """
    Calculate the forward kinematics for a given configuration of joints and joint coordinates.
    :param config: Tuple of joints
    :param joint_coordinates: Tuple of joint coordinate values (either mm or radian)
    :return: 4x4 Transformation matrix (vectors for TCP coordinate system and vector for TCP position)
    :param subtract_offset:
    :raises: ValueError if the lengths of config and joint_coordinates are unequal

    T = | xx yx zx x |
        | xy yy zy y |
        | xz yz zz z |
        |  0  0  0 1 |
    """
    if len(config) == len(joint_coordinates):
        for joint, coordinate_value in zip(config, joint_coordinates):
            # Pass in the joint coordinates to get the complete matrices
            if subtract_offset:
                joint.mul(joint_value=coordinate_value - joint.zero_offset)
            else:
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


def axang2rotm(axis: List[float], angle: float) -> ndarray:
    k = np.array(
        [
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0]
        ]
    )
    # Rodrigues Equation
    return np.eye(3) + sin(angle) * k + (1 - cos(angle)) * (k @ k)


def tform2euler(tform: ndarray) -> List[float]:
    """
    Calculates euler angles from a homogeneous matrix.
    :param tform: Homogenous matrix (4x4)
    :return: ABC in deg as used by Mitsubishi (ZY'X'', alpha and gamma swapped)
    """
    return rotm2euler(tform[0:3, 0:3])


def rotm2euler(rotm: ndarray) -> List[float]:
    beta = atan2(-rotm[2, 0], hypot(rotm[2, 1], rotm[2, 2]))
    c_b = cos(beta)

    if abs(c_b) > 1e-10:
        gamma = atan2(rotm[1, 0] / c_b, rotm[0, 0] / c_b)
        alpha = atan2(rotm[2, 1] / c_b, rotm[2, 2] / c_b)
    else:
        gamma = 0
        alpha = atan2(rotm[0, 1], rotm[1, 1])
        if beta < 0:
            alpha *= -1
    return [alpha, beta, gamma]


def pose2tform(position: List[float], *, x_angle: float, y_angle: float, z_angle: float, order='ZYX') -> ndarray:
    """
    Convert a pose given as position and euler angles.
    :param position: Position given as list of x, y and z values
    :param x_angle: Value for the angle around the x axis in rad
    :param y_angle: Value for the angle around the y axis in rad
    :param z_angle: Value for the angle around the z axis in rad
    :param order:
    :return: Homogeneous 4x4 matrix
    """
    # Catch invalid orders
    if len(order) != 3:
        raise ValueError(f'Only orders of length 3 are valid (was given {order}).')

    # Calculate unit vectors
    rot_z = np.array(
        [
            [cos(z_angle), -sin(z_angle), 0],
            [sin(z_angle), cos(z_angle), 0],
            [0, 0, 1]
        ]
    )
    rot_y = np.array(
        [
            [cos(y_angle), 0, sin(y_angle)],
            [0, 1, 0],
            [-sin(y_angle), 0, cos(y_angle)]
        ]
    )
    rot_x = np.array(
        [
            [1, 0, 0],
            [0, cos(x_angle), -sin(x_angle)],
            [0, sin(x_angle), cos(x_angle)]
        ]
    )

    # Calculate matrix for order
    to_multiply = []
    for char in order:
        if char == 'X':
            to_multiply.append(rot_x)
        elif char == 'Y':
            to_multiply.append(rot_y)
        elif char == 'Z':
            to_multiply.append(rot_z)
        else:
            raise ValueError(f'Illegal rotation axis: {char}')

    # Get product and transform to tform
    rot = np.linalg.multi_dot(to_multiply)
    return get_tform(rot[0:3, 0], rot[0:3, 1], rot[0:3, 2], position)


def get_tform(xdir: List[float], ydir: List[float], zdir: List[float], pos: List[float]) -> ndarray:
    """
    Compose a homogeneous matrix from the individual components
    :param xdir: Unit vector for x-axis
    :param ydir: Unit vector for y-axis
    :param zdir: Unit vector for z-axis
    :param pos: Position vector
    :return: Homogeneous 4x4 matrix
    """
    tform = np.zeros((4, 4))
    tform[3, 3] = 1
    tform[0:3] = np.array([xdir, ydir, zdir, pos]).transpose()
    return tform


def calculate_pose_flags(config: List[BaseJoint], joint_values: List[float]) -> float:
    """
    Calculate the flags of a roboter pose
    :param config: Tuple of joints
    :param joint_values: Tuple of joint values in manufacturer coordinates
    :return: Float number (1-7)
    """
    # Calculate wrist center position
    tjoint12 = forward_kinematics([config[0]], [joint_values[0]])
    tjoint25 = forward_kinematics(config[1:4], joint_values[1:4])
    tjoint15 = np.dot(tjoint12, tjoint25)
    wrist_center_pos = tjoint15[0:3, 3]

    # Calculate vector normal to plane through J1 axis and parallel to J2 axis
    z_axis_j1 = np.array([0, 0, 1])
    z_axis_j2 = tjoint12[0:3, 2]
    normal_vec = np.cross(z_axis_j2, z_axis_j1)

    # Use dot product to determine side
    if np.dot(normal_vec, wrist_center_pos) > 0:
        right = 1
    elif np.dot(normal_vec, wrist_center_pos) < 0:
        right = 0
    else:
        raise ShoulderSingularity

    # Convert to DH-system
    joint_values_dh = [joint_val + joint.zero_offset for joint_val, joint in zip(joint_values, config)]

    # Theta 3 determines elbow position (above or below plane of joint 2 and joint 3)
    if joint_values_dh[2] > - atan(config[3].d / config[2].a):
        above = 1
    elif joint_values_dh[2] < - atan(config[3].d / config[2].a):
        above = 0
    else:
        raise ElbowSingularity

    # Theta 5 determines flip/nonflip
    if joint_values_dh[4] > 0:
        non_flip = 1
    elif joint_values_dh[4] < 0:
        non_flip = 0
    else:
        raise WristSingularity

    return non_flip + above * 2 + right * 4


def axang2tform(nvec, angle: float):
    # TODO Rodrigues equation
    return np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
    )
