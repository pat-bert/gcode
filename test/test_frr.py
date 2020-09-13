from math import pi

import numpy as np
import pytest

from src.kinematics.forward_kinematics import forward_kinematics
from src.prechecks.configs import melfa_rv_4a
from src.prechecks.frr import frr


@pytest.mark.parametrize("home",
                         [
                             [0, 0, 90, 0, 90, 180],
                             [90, 0, 90, 0, 90, -130],
                             [60, 45, 90, 0, 90, 160],
                             [-59, 7, 90, 43, 86, -45],
                         ]
                         )
def test_frr(home):
    robot = melfa_rv_4a(rtoff=50, atoff=200)
    tuning = [0.005, 0.1, 0.1, 0.1, 0.1, 0.01]
    home = [i / 180 * pi for i in home]
    new_joints = frr(robot, home, weights=tuning)

    tcp_pos = forward_kinematics(robot, home)
    tcp_pos_new = forward_kinematics(robot, new_joints)
    tcp_pos_dev = tcp_pos_new - tcp_pos

    print(f'\nOriginal joints:\t{[f"{i:+.3f}" for i in home]}')
    print(f'New joints:\t\t\t{[f"{i:+.3f}" for i in new_joints]}')
    print(f'TCP Pos Deviation:\t{[f"{i:+.3f}" for i in tcp_pos_dev[0:3, 3]]}')
    print(f'TCP ZDIR Deviation:\t{[f"{i:+.3f}" for i in tcp_pos_dev[0:3, 2]]}')

    # Compare rotation around tool axis / z-axis in base frame
    np.testing.assert_allclose(np.zeros((3,)), tcp_pos_dev[0:3, 2], atol=1e-3)

    # Compare TCP position
    np.testing.assert_allclose(np.zeros((3,)), tcp_pos_dev[0:3, 3], atol=1)
