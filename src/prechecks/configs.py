from math import pi

from src.kinematics.joint_factories import BaseJointFactory


def melfa_rv_4a(rtoff=0, atoff=0):
    """

    :param rtoff: Radial tool offset (in x-direction of tool frame)
    :param atoff: Axial tool offset (in z-direction of tool frame)
    :return:
    """

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
