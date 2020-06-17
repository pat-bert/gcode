# Functional Redundancy Resolution
from math import pi
from typing import List

import numpy as np

from src.kinematics.forward_kinematics import geometric_jacobian, right_generalized_inverse_jacobian, forward_kinematics
from src.kinematics.joints import BaseJoint
from src.prechecks.configs import melfa_rv_4a


def frr(config: List[BaseJoint], joints: List[float], tool_vector: List[float], h: List[float]) -> List[float]:
    """
    Functional Redundancy Resolution
    :param config: Tuple of joints, offsets are considered
    :param joints: Tuple of joint coordinate values (either mm or radian)
    :param tool_vector:
    :param h:
    :return:
    """
    # Calculate jacobian and its righ-hand pseudo-inverse
    jac = geometric_jacobian(config, joints)
    pinv_jac = right_generalized_inverse_jacobian(jac)

    # Calculate tool vector, its transposed version and the projector
    total_tform = forward_kinematics(config, joints)
    e = np.asarray(tool_vector)
    e = total_tform[0:3, 2]
    e = e[:, None]

    if e.shape != (3, 1):
        raise TypeError(f'Tool vector must be (3,1) not {e.shape}.')

    t_proj = e @ e.T

    # Transform the manipulation vector and calculate the delta joints
    h = np.asarray(h)
    h = h[:, None]

    if h.shape != (len(joints), 1):
        raise TypeError(f'h must be a {len(joints)}x1 vector.')

    delta_joints = pinv_jac @ np.vstack([np.zeros((3, 1)), t_proj @ jac[3:, :] @ (0.1 * np.eye(6) @ h)])

    # Return flat list
    return delta_joints.T[0, :].tolist()


if __name__ == '__main__':
    robot = melfa_rv_4a()
    home = [0, 0, pi / 2, 0, pi / 2, 0]
    tool = [0, 0, 1]
    joint_limits = [
        -2.7925, 2.7925,
        -1.5708, 2.4435,
        +0.2618, 2.9496,
        -2.7925, 2.7925,
        -2.0944, 2.0944,
        -3.4907, 3.4907
    ]
    median = [(lower + upper) / 2 for lower, upper in zip(joint_limits[::2], joint_limits[1::2])]
    while True:
        manip = [qm - q for qm, q in zip(median, home)]
        print(f'Manip: {manip}')
        delta = frr(robot, home, tool, manip)
        print(f'Delta (deg): {[round(dq / pi * 180, ndigits=3) for dq in delta]}')
        home = [q + dq for q, dq in zip(home, delta)]
        print(f'Joints (deg): {[round(q / pi * 180, ndigits=3) for q in home]}')
        input('Continue?')
