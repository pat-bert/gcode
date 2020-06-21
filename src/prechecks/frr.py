# Functional Redundancy Resolution
from math import pi
from typing import List

import numpy as np

from src.kinematics.forward_kinematics import geometric_jacobian, right_generalized_inverse_jacobian, forward_kinematics
from src.kinematics.joints import BaseJoint
from src.prechecks.configs import melfa_rv_4a


def frr(config: List[BaseJoint], initial_joints: List[float], weights=None, stop_threshold: float = 1e-5) \
        -> List[float]:
    """
    Functional Redundancy Resolution
    :param config: Tuple of joints, offsets are considered
    :param initial_joints: Tuple of joint coordinate values (either mm or radian)
    :param weights:
    :param stop_threshold:
    :return:
    """
    # Obtain tool column vector as z-axis of endeffector frame
    total_tform = forward_kinematics(config, initial_joints)
    e = total_tform[0:3, 2]
    e = e[:, None]

    # Calculate projector onto tool axis
    t_proj = e @ e.T

    joint_limits = [
        -2.7925, 2.7925,
        -1.5708, 2.4435,
        +0.2618, 2.9496,
        -2.7925, 2.7925,
        -2.0944, 2.0944,
        -3.4907, 3.4907
    ]
    median = [(lower + upper) / 2 for lower, upper in zip(joint_limits[::2], joint_limits[1::2])]

    # Initialize current joint column vector
    current_joints = np.asarray(initial_joints)
    current_joints = current_joints[:, None]

    # Optimize the joint vector in the orthogonal space
    while True:
        # Calculate jacobian and its righ-hand pseudo-inverse for current joint values
        jac = geometric_jacobian(config, current_joints[:, 0].tolist())
        pinv_jac = right_generalized_inverse_jacobian(jac)

        # Create the manipulation column vector
        manip = [qm - q for qm, q in zip(median, current_joints[:, 0].tolist())]
        h = np.asarray(manip)
        h = h[:, None]

        if h.shape != (len(initial_joints), 1):
            raise TypeError(f'h must be a {len(initial_joints)}x1 vector.')

        # Set weights as diagonal matrix
        if weights is None:
            weights = [0.1] * len(initial_joints)
        elif any((i >= 1 for i in weights)):
            raise ValueError('Weights need to be < 1 for the algorithm to converge.')
        w = np.diag(weights)

        # Calculate delta for iteration
        effect = np.linalg.multi_dot([t_proj, jac[3:, :], w, h])
        delta_joints = pinv_jac @ np.vstack([np.zeros((3, 1)), effect])
        current_joints += delta_joints

        if np.linalg.norm(delta_joints) < stop_threshold:
            # Stop when the correcting effort is below a certain threshold
            break

    # Return flat list
    return current_joints[:, 0].tolist()


if __name__ == '__main__':
    # Define test data
    robot = melfa_rv_4a(rtoff=-50, atoff=200)
    home = [0, 0, pi / 2, 0, pi / 2, pi]
    tuning = [0, 0.1, 0.1, 0.1, 0.1, 0.01]

    # Do optimization and print joints
    new_joints = frr(robot, home, weights=tuning)
    print(f'Original joints:\t{[f"{i:+.3f}" for i in home]}')
    print(f'New joints:\t\t\t{[f"{i:+.3f}" for i in new_joints]}')

    # Validate new position
    tcp_pos = forward_kinematics(robot, home)
    tcp_pos_new = forward_kinematics(robot, new_joints)
    tcp_pos_dev = tcp_pos_new - tcp_pos
    print(f'TCP Pos Deviation:\t{[f"{i:+.3f}" for i in tcp_pos_dev[0:3, 3]]}')
    print(f'TCP ZDIR Deviation:\t{[f"{i:+.3f}" for i in tcp_pos_dev[0:3, 2]]}')
    print(f'Weights:\t\t\t{tuning}')
