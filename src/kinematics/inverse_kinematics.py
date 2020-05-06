from math import atan2, pi, acos, atan, hypot
from typing import List, Optional, Dict

import numpy as np

from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joints import BaseJoint


class Singularity(ValueError):
    pass


class ShoulderSingularity(Singularity):
    """
    Will be raised if wrist center point is on J1 axis
    """


class WristSingularity(Singularity):
    """
    Will be raised if J4 and J6 align (infinite solutions)
    """


class ElbowSingularity(Singularity):
    """
    Will be raised if the wrist center point is within the plane through J2 and J3
    """


class OutOfReachError(ValueError):
    """
    Will be raised if a pose cannot be reached.
    """


# Apply the offset and wrap to (-pi, pi]
def wrap_to_pi(angle):
    return -((- angle + pi) % (2 * pi) - pi)


WRIST_SINGULARITY_THRESHOLD = 1e-3
ELBOW_SINGULARITY_THRESHOLD = 1e-3
SHOULDER_SINGULARITY_THRESHOLD = 1e-3


def ik_spherical_wrist(config: List[BaseJoint], tform: np.ndarray, pose_flags=None) -> Dict[float, List[float]]:
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

    if pose_flags is None:
        # Default pose flags
        flag_right, flag_elbow_up, flag_non_flip = None, None, None
    elif 0 <= pose_flags <= 7:
        # Unpack pose flags and convert to boolean
        flag_non_flip = bool(pose_flags & 1)  # Only affected by J5
        flag_elbow_up = bool(pose_flags & 2)  # Only affected by J3
        flag_right = bool(pose_flags & 4)  # Affected by J2, J3, J4
    else:
        raise ValueError('Pose flag must be 0-7.')

    # Extract the relevant data from the matrix
    tcp_pos = tform[0:3, 3]
    xdir = tform[0:3, 0]
    zdir = tform[0:3, 2]

    # Get the wrist center in base frame from the flange position in base frame
    p04 = tcp_pos - zdir * config[5].d
    solutions = {}

    # Attempt to calculate theta1, singularity error will propagate
    theta1_solutions = _ik_spherical_wrist_joint1(flag_right, p04)

    # The solutions of theta 1 contain the corresponding right flag
    for idx1, (right, theta1) in enumerate(theta1_solutions.items()):
        # Get the vector from origin of joint 2 (frame 1) to wrist center
        tjoint12 = forward_kinematics([config[0]], [theta1], subtract_offset=True)
        p01 = tjoint12[0:3, 3]
        p14 = p04 - p01

        try:
            # Calculate theta 2 for the current flag_right value
            theta2_solutions = _ik_spherical_wrist_joint2(config, right, flag_elbow_up, tjoint12, p14)

            # The solutions of theta 2 contain the corresponding elbow flag
            for idx2, (elbow_up, theta2) in enumerate(theta2_solutions.items()):
                # No common setup required
                try:
                    # Calculate theta 3
                    theta3 = _ik_spherical_wrist_joint3(config, elbow_up, p14)

                    # Calculate theta 5 (requires theta 1 - 3)
                    theta5_solutions = _ik_spherical_wrist_joint5(config, flag_non_flip, tjoint12, theta2, theta3, zdir)

                    # The solutions of theta 5 contain the corresponding non flip flag
                    for idx5, (non_flip, theta5) in enumerate(theta5_solutions.items()):
                        # No common setup required
                        try:
                            # Calculate theta 4 (requires theta 1 - 3)
                            theta4 = _ik_spherical_wrist_joint4(config, non_flip, tjoint12, theta2, theta3, zdir)

                            # Calculate theta 6 (requires theta 1 - 5)
                            theta6 = _ik_spherical_wrist_joint6(config, tjoint12, theta2, theta3, theta4, theta5, xdir)
                        except ValueError:
                            # Solution failed
                            if len(solutions) == 0 and idx5 == len(theta5_solutions) - 1:
                                # IK failed for all theta 5 solutions so failed for current theta 2 only
                                raise
                        else:
                            # Bundle all the angles and wrap them to (-pi, pi]
                            theta = [theta1, theta2, theta3, theta4, theta5, theta6]
                            theta = [wrap_to_pi(angle - joint.zero_offset) for angle, joint in zip(theta, config)]

                            # Append to solutions using the pose flags as key
                            solutions[4 * right + 2 * elbow_up + non_flip] = theta
                except ValueError:
                    if len(solutions) == 0 and idx2 == len(theta2_solutions) - 1:
                        # IK failed for all theta 2 solutions so failed for current theta 1 only
                        raise
        except ValueError:
            # This is only an issue if this is the last iteration of theta1 and no solution has been found yet
            if len(solutions) == 0 and idx1 == len(theta1_solutions) - 1:
                # IK failed for all theta 1 solutions so failed completely
                raise

    # Return all successfully calculated solutions
    return solutions


def _ik_spherical_wrist_joint1(flag_right: Optional[bool], p04: np.ndarray) -> Dict[bool, float]:
    """
    Calculate the first joint for the spherical wrist robot type
    :param flag_right: Pose flog
    :param p04: Vector from origin in joint 1 to wrist center
    :return:List of solutions.
    If a configuration is given by right only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Singularity occurs if the wrist center point is on axis J1
    if abs(p04[1]) <= SHOULDER_SINGULARITY_THRESHOLD and abs(p04[0]) <= SHOULDER_SINGULARITY_THRESHOLD:
        raise ShoulderSingularity

    # Theta 1 (Hip) represents the angle when the wrist center is given in polar coordinates
    theta1_1 = atan2(p04[1], p04[0])

    # Second solution on opposite side but within [-pi, +pi]
    theta1_2 = (theta1_1 - pi) if theta1_1 >= 0 else (theta1_1 + pi)

    # Theta 1 selection logic
    if flag_right is not None:
        if flag_right:
            # Regular solution
            return {flag_right: theta1_1}
        # Second solution is chosen if flag is left
        # Wrist center behind plane through joint 1 and parallel joint 2
        return {flag_right: theta1_2}
    return {True: theta1_1, False: theta1_2}


def _ik_spherical_wrist_joint2(
        config: List[BaseJoint],
        right: bool,
        elbow_up: Optional[bool],
        tf12: np.ndarray,
        p14_f2: np.ndarray) -> Dict[bool, float]:
    """
    Calculate the second joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param right: Pose flog (always known because tjoint12 is known)
    :param elbow_up: Pose flog
    :param tf12: Previously calculated transformation
    :param p14_f2: Vector from frame origin in joint 2 to wrist center
    :return: Dict of solutions, flag_right is used as key.
    If a configuration is given by flag_right only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Transform vector to wrist center given in frame 2 to frame 1
    tjoint21 = tf12.transpose()
    p14_f1 = np.dot(tjoint21[0:3, 0:3], p14_f2)
    vector_len_p14 = np.linalg.norm(p14_f2)

    # Calculate other sides of triangle from link lengths
    l4 = hypot(config[3].d, config[2].a)
    l2 = config[1].a

    # Calculate auxiliary angles beta1 and beta2
    beta2 = acos_safe((l2 ** 2 + vector_len_p14 ** 2 - l4 ** 2) / (2 * l2 * vector_len_p14))

    # Conditional calculation since only one solution of beta1 is ever used
    if right:
        # Calculate solutions for theta3 based on beta1 and beta2
        beta1_1 = atan2(- p14_f1[1], p14_f1[0])
        theta2_ru = -(beta1_1 + beta2)
        theta2_rd = - (beta1_1 - beta2)
        theta2_all = [theta2_rd, theta2_ru]
    else:
        # Calculate solutions for theta3 based on beta1 and beta2
        beta1_2 = atan2(- p14_f1[1], - p14_f1[0])
        theta2_lu = (beta1_2 - beta2) - pi
        theta2_ld = beta1_2 + beta2 - pi
        theta2_all = [theta2_ld, theta2_lu]

    # Select solutions
    if elbow_up is not None:
        # Only one solution (using bool as index)
        return {elbow_up: theta2_all[elbow_up]}
    # Both solutions
    return {False: theta2_all[0], True: theta2_all[1]}


def _ik_spherical_wrist_joint3(config: List[BaseJoint], elbow_up: bool, p14: np.ndarray) -> float:
    """
    Calculate the third joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param elbow_up: Pose flog
    :param p14: Vector from frame origin in joint 2 to wrist center
    :return: List of solutions.
    If a configuration is given by elbow_up only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Calculate other sides of triangle from link lengths
    l4 = hypot(config[3].d, config[2].a)
    l2 = config[1].a

    # Get both solutions for the auxiliary angle phi using cosine law
    vector_len_p14 = np.linalg.norm(p14)

    phi_arg = (l2 ** 2 + l4 ** 2 - vector_len_p14 ** 2) / (2 * l2 * l4)

    aux_phi_1 = acos_safe(phi_arg)
    aux_phi_2 = 2 * pi - aux_phi_1

    # Calculate solutions for theta3 based on phi
    phi_offset = (pi - atan(config[3].d / config[2].a))

    if elbow_up:
        return phi_offset - aux_phi_1
    else:
        return phi_offset - aux_phi_2


def _ik_spherical_wrist_joint4(
        config: List[BaseJoint],
        non_flip: bool,
        tjoint12: np.ndarray,
        theta2: float,
        theta3: float,
        zdir) -> float:
    """
    Calculate the fourth joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param non_flip: Pose flag
    :param tjoint12: Previously calculated transformation
    :param theta2: Previously calculated shoulder angle
    :param theta3: Previously calculated elbow angle
    :param zdir: Unit vector for z-axis of flange frame given in base frame
    :return: Solution for theta 4
    :raises: WristSingularity if axes of theta 4 and theta 6 are collinear
    """
    # Get the vector from joint 1 to joint 4 (same plane) based on previous results (neutralize offset)
    tjoint24 = forward_kinematics(config[1:3], [theta2, theta3], subtract_offset=True)
    tjoint14 = np.dot(tjoint12, tjoint24)

    # Get z-direction of joint 4 in base frame
    z3 = tjoint14[0:3, 2]
    y3 = tjoint14[0:3, 1]
    x3 = tjoint14[0:3, 0]

    # c needs to be a unit vector
    c = np.cross(zdir, z3)
    c_vector_len = np.linalg.norm(c)
    c /= c_vector_len

    if c_vector_len <= WRIST_SINGULARITY_THRESHOLD:
        # Singularity (J4 and J6 are collinear)
        print('Wrist Singularity detected!')
        raise WristSingularity

    # Regular case (need two angles to determine sign
    delta_q4_abs = acos_safe(np.dot(y3, c))
    chi = acos_safe(np.dot(x3, c))

    # Change sign accordingly
    if chi < pi / 2:
        delta_q4 = - delta_q4_abs
        if not non_flip:
            delta_q4 += pi
    else:
        delta_q4 = delta_q4_abs
        if not non_flip:
            delta_q4 -= pi
    return delta_q4


def _ik_spherical_wrist_joint5(config, non_flip, tjoint12, theta2, theta3, zdir) -> Dict[bool, float]:
    """
    Calculate the fifth joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param non_flip: Pose flog
    :param tjoint12: Previously calculated transformation
    :param theta2: Previously calculated shoulder angle
    :param theta3: Previously calculated elbow angle
    :param zdir: Unit vector for z-axis of flange frame given in base frame
    :return:  Dict of solutions, non_flip is used as key.
    If a configuration is given by non_flip only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Get the vector from joint 1 to joint 4 (same plane) based on previous results
    tjoint24 = forward_kinematics(config[1:3], [theta2, theta3], subtract_offset=True)
    tjoint14 = np.dot(tjoint12, tjoint24)

    # Theta 5 describes angle between z-dir of joint 4 and joint 6
    z3 = tjoint14[0:3, 2]
    theta5_abs = acos_safe(np.dot(z3, zdir))

    # Select the solution based on the pose flag
    if non_flip is not None:
        # Only one solution is chosen
        return {non_flip: (theta5_abs if non_flip else - theta5_abs)}
    # Both solutions are returned
    return {True: theta5_abs, False: - theta5_abs}


def _ik_spherical_wrist_joint6(
        config: List[BaseJoint],
        tjoint12: np.ndarray,
        theta2: float,
        theta3: float,
        theta4: float,
        theta5: float,
        xdir: np.ndarray) -> float:
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
    x5 = tjoint16[0:3, 0]
    y5 = tjoint16[0:3, 1]

    # Theta 6 describes angle between x-dir of joint 5 and joint 6
    theta6_abs = acos_safe(np.dot(x5, xdir))

    # Determine sign of theta 6
    delta = acos_safe(np.dot(y5, xdir))

    # Selection logic for theta 6
    if delta <= pi / 2:
        return theta6_abs
    return -theta6_abs


def acos_safe(arg) -> float:
    """
    Helper function to prevent MathDomainErrors caused by float arithmetic
    :param arg: Argument to be passed to acos
    :return: acos for angle arg rounded to six digits
    :raises: OutOfReachError if acos is not defined for the rounded arg
    """
    # Check whether no solution is available (= OutOfReachError)
    arg_clipped = round(arg, ndigits=6)
    try:
        return acos(arg_clipped)
    except ValueError:
        raise OutOfReachError
