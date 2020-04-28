from math import atan2, pi, acos, sqrt, atan
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

    :param pose_flags: Define the preferred configuration as number (0-7), defaults to None (all solutions are returned)

    0b(l,m,n):
    - l: 1/0 = right/left
    - m: 1/0 = up/down
    - n: 1/0 = non-flip/flip

    :return: Tuple of joint coordinate values (radian)
    :raises: ValueError if config is not 6-DOF.
    :raises: ValueError if pose is outside of specified range
    """
    # Validate the config
    if len(config) != 6:
        raise ValueError('Inverse kinematic of function is only possible for 6DOF + spherical wrist.')

    # TODO Correctly determine link between pose flag and solution chosen
    if pose_flags is None:
        # Default pose flags
        joint2_first = True
        joint3_up = True
        joint5_non_flip = True
    else:
        if 0 <= pose_flags <= 7:
            # Unpack pose flags and convert to boolean
            joint5_non_flip = True if (pose_flags & 1) == 1 else False
            joint3_up = True if (pose_flags & 2) == 2 else False
            joint2_first = True if (pose_flags & 4) == 4 else False
        else:
            raise ValueError('Pose flag must be 0-7.')

    # Extract the relevant data from the matrix
    tcp_pos = tform[0:3, 3]
    xdir = tform[0:3, 0]
    zdir = tform[0:3, 2]

    """
    Calculate theta 1
    """
    # Get the wrist center in base frame from the flange position in base frame
    p04 = tcp_pos - zdir * config[5].d

    # Theta 1 (Hip) represents the angle when the wrist center is given in polar coordinates
    theta1 = atan2(p04[1], p04[0])

    # Get origin of joint 2 in base frame
    tjoint12 = forward_kinematics([config[0]], [theta1], subtract_offset=True)
    p01 = tjoint12[0:3, 3]

    # Get the vector from origin of joint 2 (frame 1) to wrist center
    p14 = p04 - p01

    # Calculate theta 3
    theta3, *_ = _ik_spherical_wrist_joint3(config, joint3_up, p14)

    # Calculate theta 2
    theta2, *_ = _ik_spherical_wrist_joint2(config, joint2_first, tjoint12, p14)

    # Calculate theta 5 (requires theta 1 - 3)
    theta5, *_ = _ik_spherical_wrist_joint5(config, joint5_non_flip, tjoint12, theta2, theta3, zdir)

    # Calculate theta 4 (requires theta 1 - 3)
    theta4, *_ = _ik_spherical_wrist_joint4(config, tjoint12, theta2, theta3, zdir)

    # TODO Calculate theta 6 (requires theta 1 - 5)
    theta6, *_ = _ik_spherical_wrist_joint6(config, tjoint12, theta2, theta3, theta4, theta5, xdir)

    # Bundle all the angles and apply the offset
    theta = [theta1, theta2, theta3, theta4, theta5, theta6]
    print(f'Offsets: {[f"{joint.zero_offset:+.3f}" for joint in config]}')
    return [angle - joint.zero_offset for angle, joint in zip(theta, config)]


def _ik_spherical_wrist_joint2(config, joint2_first, tjoint12, p14) -> List[float]:
    """
    Calculate the second joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param joint2_first: Pose flog
    :param tjoint12: Previously calculated transformation
    :param p14: Vector from frame origin in joint 2 to wrist center
    :return: List of solutions.
    If a configuration is given by non_flip only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Transform vector to wrist center given in frame 2 to frame 1
    tjoint21 = tjoint12.transpose()
    wrist_center_pos_f2_f1 = np.dot(tjoint21[0:3, 0:3], p14)
    vector_len_p14 = np.linalg.norm(p14)

    # Calculate other sides of triangle from link lengths
    l4 = sqrt(config[3].d ** 2 + config[2].a ** 2)
    l2 = config[1].a

    # Calculate auxiliary angles beta1 and beta2
    beta1 = atan2(- wrist_center_pos_f2_f1[1], wrist_center_pos_f2_f1[0])
    beta2_1 = acos((l2 ** 2 + vector_len_p14 ** 2 - l4 ** 2) / (2 * l2 * vector_len_p14))
    beta2_2 = 2 * pi - beta2_1

    # Calculate solutions for theta3 based on beta1 and beta2
    theta2_1 = -(beta1 + beta2_1)
    theta2_2 = -(beta1 + beta2_2)

    # TODO Select the solution based on the pose flag
    if joint2_first is not None:
        if joint2_first:
            print(f'Theta 2: [x] {theta2_1:+.3f} [ ] {theta2_2:+.3f}')
            return [theta2_1]
        print(f'Theta 2: [ ] {theta2_1:+.3f} [x] {theta2_2:+.3f}')
        return [theta2_2]
    return [theta2_1, theta2_2]


def _ik_spherical_wrist_joint3(config, joint3_up, p14) -> List[float]:
    """
    Calculate the third joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param joint3_up: Pose flog
    :param p14: Vector from frame origin in joint 2 to wrist center
    :return: List of solutions.
    If a configuration is given by non_flip only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Calculate other sides of triangle from link lengths
    l4 = sqrt(config[3].d ** 2 + config[2].a ** 2)
    l2 = config[1].a

    # Get both solutions for the auxiliary angle phi using cosine law
    vector_len_p14 = np.linalg.norm(p14)
    aux_phi_1 = acos((l2 ** 2 + l4 ** 2 - vector_len_p14 ** 2) / (2 * l2 * l4))
    aux_phi_2 = 2 * pi - aux_phi_1

    # Calculate solutions for theta3 based on phi
    phi_offset = (pi - atan(config[3].d / config[2].a))
    theta3_1 = phi_offset - aux_phi_1
    theta3_2 = phi_offset - aux_phi_2

    # TODO Select the solution based on the pose flag
    if joint3_up is not None:
        if joint3_up:
            print(f'Theta 3: [x] {theta3_1:+.3f} [ ] {theta3_2:+.3f}')
            return [theta3_1]
        print(f'Theta 3: [ ] {theta3_1:+.3f} [x] {theta3_2:+.3f}')
        return [theta3_2]
    return [theta3_1, theta3_2]


def _ik_spherical_wrist_joint4(config, tjoint12, theta2, theta3, zdir) -> List[float]:
    """
    Calculate the fourth joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param tjoint12: Previously calculated transformation
    :param theta2: Previously calculated shoulder angle
    :param theta3: Previously calculated elbow angle
    :param zdir: Unit vector for z-axis of flange frame given in base frame
    :return: List of solutions.
    If a configuration is given by non_flip only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Get the vector from joint 1 to joint 4 (same plane) based on previous results (neutralize offset)
    tjoint24 = forward_kinematics(config[1:3], [theta2, theta3], subtract_offset=True)
    tjoint14 = np.dot(tjoint12, tjoint24)

    # Get z-direction of joint 4 in base frame
    z3 = tjoint14[0:3, 2]
    y3 = tjoint14[0:3, 1]

    c = np.cross(zdir, z3)

    if np.linalg.norm(c) != 0:
        q4 = acos(np.dot(y3, c))
    else:
        # Singularity (J4 and J6 are collinear)
        q4 = 0

    return [q4]


def _ik_spherical_wrist_joint5(config, non_flip, tjoint12, theta2, theta3, zdir) -> List[float]:
    """
    Calculate the fifth joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param non_flip: Pose flog
    :param tjoint12: Previously calculated transformation
    :param theta2: Previously calculated shoulder angle
    :param theta3: Previously calculated elbow angle
    :param zdir: Unit vector for z-axis of flange frame given in base frame
    :return: List of solutions.
    If a configuration is given by non_flip only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Get the vector from joint 1 to joint 4 (same plane) based on previous results
    tjoint24 = forward_kinematics(config[1:3], [theta2, theta3], subtract_offset=True)
    tjoint14 = np.dot(tjoint12, tjoint24)

    # Get z-direction of joint 4 in base frame
    z3 = tjoint14[0:3, 2]

    # Theta 5 describes angle between z-dir of joint 5 and joint 6
    theta5_1 = acos(np.dot(z3, zdir))
    theta5_2 = 2 * pi - theta5_1

    # Select the solution based on the pose flag
    if non_flip is not None:
        if non_flip:
            print(f'Theta 5: [x] {theta5_1:+.3f} [ ] {theta5_2:+.3f} (non-flip)')
            return [theta5_1]
        print(f'Theta 5: [ ] {theta5_1:+.3f} [x] {theta5_2:+.3f} (flip)')
        return [theta5_2]
    return [theta5_1, theta5_2]


def _ik_spherical_wrist_joint6(config, tjoint12, theta2, theta3, theta4, theta5, xdir) -> List[float]:
    """
    Calculate the sixth joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param tjoint12: Previously calculated transformation
    :param theta2: Previously calculated shoulder angle
    :param theta3: Previously calculated elbow angle
    :param theta4:  Previously calculated angle
    :param theta5:  Previously calculated angle
    :param xdir: Unit vector for x-axis of flange frame given in base frame
    :return: List of solutions.
    If a configuration is given by non_flip only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Get frame of joint 5
    tjoint26 = forward_kinematics(config[1:5], [theta2, theta3, theta4, theta5], subtract_offset=True)
    tjoint16 = np.dot(tjoint12, tjoint26)
    x5 = tjoint16[0:3, 1]

    # Theta 6 describes angle between x-dir of joint 5 and joint 6
    theta6_1 = acos(np.dot(x5, xdir))
    theta6_2 = 2 * pi - theta6_1

    print(f'Theta 6: [x] {theta6_1:+.3f} [ ] {theta6_2:+.3f}')
    return [theta6_1]
