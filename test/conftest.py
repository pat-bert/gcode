from math import pi

import pytest

from kinematics.joint_factories import BaseJointFactory


@pytest.fixture
def dh_melfa_rv_4a():
    """
    Provide the DH config for the Mitsubishi Melfa RV-4A
    :return:
    """
    rtoff = 0.0  # radial tool offset
    atoff = 0.0  # axial tool offset

    # Denavit-Hartenberg parameters: a - alpha - d - zero offset
    # Mitsubishi defines axis origins differently than resulting from DH-convention
    dh_parameters = [
        [100, -pi / 2, 350, 0],
        [250, 0.00000, 0.0, -pi / 2],
        [135, -pi / 2, 0.0, -pi / 2],
        [0.0, +pi / 2, 250, 0],
        [0.0, -pi / 2, 0.0, 0],
        [rtoff, 0.00000, 90 + atoff, pi]
    ]

    config = [BaseJointFactory.new(a=a, alpha=alpha, d=d, theta=None, offset=z) for a, alpha, d, z in dh_parameters]
    return config
