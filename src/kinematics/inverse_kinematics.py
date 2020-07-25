from math import atan2, pi, acos, atan, hypot
from typing import List, Optional, Dict

import numpy as np

from src.kinematics.joints import ShoulderSingularity, WristSingularity, WRIST_SINGULARITY_THRESHOLD, \
    SHOULDER_SINGULARITY_THRESHOLD
from src.kinematics.forward_kinematics import forward_kinematics, vec3_cross
from src.kinematics.joints import BaseJoint

JointSolution = Dict[int, List[float]]


class OutOfReachError(ValueError):
    """
    Will be raised if a pose cannot be reached.
    """


# Apply the offset and wrap to (-pi, pi]
def wrap_to_pi(angle):
    return -((- angle + pi) % (2 * pi) - pi)


def ik_spherical_wrist(config: List[BaseJoint], tform: np.ndarray, pose_flags=None) -> JointSolution:
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
        flag_right, flag_elbow_up, flag_non_flip = bool(pose_flags & 4), bool(pose_flags & 2), bool(pose_flags & 1)
    else:
        raise ValueError('Pose flag must be 0-7.')

    # Extract the relevant data from the matrix
    tcp_pos = tform[0:3, 3]
    xdir = tform[0:3, 0]
    zdir = tform[0:3, 2]

    # Get the wrist center in base frame from the flange position in base frame
    p04 = tcp_pos - zdir * config[5].d - xdir * config[5].a

    # Attempt to calculate j1, singularity error will propagate
    theta1_solutions = _ik_spherical_wrist_joint1(flag_right, p04)

    # Calculate all depending solutions
    return _calc_j1_dependants(config, flag_elbow_up, flag_non_flip, p04, theta1_solutions, xdir, zdir)


def _calc_j1_dependants(config: List[BaseJoint], elbow_up: Optional[bool], non_flip: Optional[bool], p04: np.ndarray,
                        j1_solutions: Dict[bool, float], xdir: np.ndarray, zdir: np.ndarray) \
        -> JointSolution:
    """
    Calculate all dependant angles (2-6) for given j1 solutions
    :param config: Tuple of joints, offsets are considered
    :param elbow_up: Elbow-up flag. If None all possible configurations are calculated.
    :param non_flip: Non-flip flag. If None all possible configurations are calculated.
    :param p04: Wrist center position given in base frame
    :param j1_solutions: Dict of J1 solutions. Boolean values of right flag are keys, angles are values.
    :param xdir: TCP frame x-direction unit vector
    :param zdir: TCP frame z-direction unit vector
    :return: Dictionary of complete IK solutions. Pose flags are key, lists of angles are values.
    """
    out = {}
    # The solutions of theta 1 contain the corresponding flag_sum flag
    for idx1, (right, theta1) in enumerate(j1_solutions.items()):
        # Get the vector from origin of joint 2 (frame 1) to wrist center
        tjoint12 = forward_kinematics([config[0]], [theta1], subtract_offset=True)
        p01 = tjoint12[0:3, 3]
        p14 = p04 - p01

        try:
            # Calculate theta 2 for the current flag_right value
            j2_solutions = _ik_spherical_wrist_joint2(config, right, elbow_up, tjoint12, p14)

            # Calculate the flag summand of the current level
            flag_sum = 4 * right

            # Calculate all angles depending on the j2 solutions
            inner_out = _calc_j2_dependants(config, non_flip, p14, flag_sum, theta1, j2_solutions, tjoint12, xdir, zdir)

            # Collect the solutions
            out.update(inner_out)
        except ValueError:
            # This is only an issue if this is the last iteration of j1 and no solution has been found yet
            if len(out) == 0 and idx1 == len(j1_solutions) - 1:
                # IK failed for all theta 1 solutions so failed completely
                raise
    return out


def _calc_j2_dependants(conf: List[BaseJoint], non_flip: Optional[bool], p14: np.ndarray, flag_sum: int, j1: float,
                        j2_solutions: Dict[bool, float], tf12: np.ndarray, xdir: np.ndarray, zdir: np.ndarray) \
        -> JointSolution:
    """
    Calculate all dependant angles (3-6) for given j2 solutions
    :param conf: Tuple of joints, offsets are considered
    :param non_flip: Non-flip flag. If None all possible configurations are calculated.
    :param p14: Vector from frame origin in joint 2 to wrist center
    :param flag_sum: Sum of previous levels. All solutions from here are have flag = flagsum + k, k=0..3
    :param j1: Value of calculated J1 angle
    :param j2_solutions: Dict of J2 solutions. Boolean values of right flag are keys, angles are values.
    :param tf12: Transformation matrix from joint 1 to joint 2
    :param xdir: TCP frame x-direction unit vector
    :param zdir: TCP frame z-direction unit vector
    :return: Dictionary of complete IK solutions. Pose flags are key, lists of angles are values.
    """
    out = {}

    # The solutions of theta 2 contain the corresponding elbow flag
    for idx2, (elbow_up, j2) in enumerate(j2_solutions.items()):
        # No common setup required
        try:
            # Calculate theta 3
            j3 = _ik_spherical_wrist_joint3(conf, elbow_up, p14)

            # Calculate transformation from joint 1 to joint 4
            tf24 = forward_kinematics(conf[1:3], [j2, j3], subtract_offset=True)
            tf14 = np.dot(tf12, tf24)

            # Calculate theta 5 (requires theta 1 - 3)
            j5_solutions = _ik_spherical_wrist_joint5(non_flip, tf14, zdir)

            # Add the flag summand of the current level
            flag_sum += 2 * elbow_up

            # Calculate all angles depending on the j5 solutions
            inner_out = _calc_j5_dependants(conf, flag_sum, [j1, j2, j3], j5_solutions, tf14, xdir, zdir)

            # Collect the solutions
            out.update(inner_out)
        except ValueError:
            if len(out) == 0 and idx2 == len(j2_solutions) - 1:
                # IK failed for all theta 2 solutions so failed for current theta 1 only
                raise
    return out


def _calc_j5_dependants(conf: List[BaseJoint], flag_sum: int, theta123: List[float], j5_solutions: Dict[bool, float],
                        tf14: np.ndarray, xdir: np.ndarray, zdir: np.ndarray) -> JointSolution:
    """
    Calculate the dependant last joint for given j5 solutions
    :param conf: Tuple of joints, offsets are considered
    :param flag_sum: Sum of previous levels. All solutions from here are have flag = flagsum + k, k=0..1
    :param theta123: List of angles j1-j3
    :param j5_solutions: Dict of J5 solutions. Boolean values of right flag are keys, angles are values.
    :param tf14: Transformation matrix from joint 1 to joint 4
    :param xdir: TCP frame x-direction unit vector
    :param zdir: TCP frame z-direction unit vector
    :return: Dictionary of complete IK solutions. Pose flags are key, lists of angles are values.
    """
    out = {}

    # The solutions of theta 5 contain the corresponding non flip flag
    for idx5, (non_flip, theta5) in enumerate(j5_solutions.items()):
        try:
            # Calculate theta 4
            theta4 = _ik_spherical_wrist_joint4(non_flip, tf14, zdir)

            # Calculate transformation from joint 1 to joint 6
            tf46 = forward_kinematics(conf[3:5], [theta4, theta5], subtract_offset=True)
            tf16 = np.dot(tf14, tf46)

            # Calculate theta 6 (requires theta 1 - 5)
            theta6 = _ik_spherical_wrist_joint6(tf16, xdir)
        except ValueError:
            # Solution failed
            if len(out) == 0 and idx5 == len(j5_solutions) - 1:
                # IK failed for all theta 5 solutions so failed for current theta 2 only
                raise
        else:
            # Bundle all the angles and wrap them to (-pi, pi]
            theta = theta123 + [theta4, theta5, theta6]
            theta = [wrap_to_pi(angle - joint.zero_offset) for angle, joint in zip(theta, conf)]

            # Append to solutions using the pose flags as key
            out[flag_sum + non_flip] = theta
    return out


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


def _ik_spherical_wrist_joint2(config: List[BaseJoint], right: bool, elbow_up: Optional[bool], tf12: np.ndarray,
                               p14_f2: np.ndarray) -> Dict[bool, float]:
    """
    Calculate the second joint for the spherical wrist robot type
    :param config: Robot configuration to access DH-parameters
    :param right: Pose flog (always known because tf12 is known)
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
    return phi_offset - aux_phi_2


def _ik_spherical_wrist_joint4(non_flip: bool, tjoint14: np.ndarray, zdir) -> float:
    """
    Calculate the fourth joint for the spherical wrist robot type
    :param non_flip: Pose flag
    :param tjoint14: Previously calculated transformation
    :param zdir: Unit vector for z-axis of flange frame given in base frame
    :return: Solution for theta 4
    :raises: WristSingularity if axes of theta 4 and theta 6 are collinear
    """
    # Get z-direction of joint 4 in base frame
    z3 = tjoint14[0:3, 2]
    y3 = tjoint14[0:3, 1]
    x3 = tjoint14[0:3, 0]

    # c needs to be a unit vector
    c = vec3_cross(zdir, z3)
    c_vector_len = np.linalg.norm(c)
    c /= c_vector_len

    if c_vector_len <= WRIST_SINGULARITY_THRESHOLD:
        # Singularity (J4 and J6 are collinear)
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


def _ik_spherical_wrist_joint5(non_flip, tjoint14, zdir) -> Dict[bool, float]:
    """
    Calculate the fifth joint for the spherical wrist robot type
    :param non_flip: Pose flog
    :param tjoint14: Previously calculated transformation
    :param zdir: Unit vector for z-axis of flange frame given in base frame
    :return:  Dict of solutions, non_flip is used as key.
    If a configuration is given by non_flip only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Theta 5 describes angle between z-dir of joint 4 and joint 6
    z3 = tjoint14[0:3, 2]
    theta5_abs = acos_safe(np.dot(z3, zdir))

    # Select the solution based on the pose flag
    if non_flip is not None:
        # Only one solution is chosen
        return {non_flip: (theta5_abs if non_flip else - theta5_abs)}
    # Both solutions are returned
    return {True: theta5_abs, False: - theta5_abs}


def _ik_spherical_wrist_joint6(tjoint16: np.ndarray, xdir: np.ndarray) -> float:
    """
    Calculate the sixth joint for the spherical wrist robot type
    :param tjoint16: Previously calculated transformation
    :param xdir: Unit vector for x-axis of flange frame given in base frame
    :return: List of solutions.
    If a configuration is given by non_flip only one solutions is returned.
    Otherwise both solutions are returned.
    """
    # Get frame of joint 5
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
    arg_clipped = int(1e10 * arg) / 1e10
    try:
        return acos(arg_clipped)
    except ValueError as e:
        raise OutOfReachError from e
