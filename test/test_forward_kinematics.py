from math import pi, cos, sin
from typing import List

import numpy as np
import pytest

from src.kinematics.forward_kinematics import forward_kinematics, tform2quat
from src.kinematics.joint_factories import BaseJointFactory


@pytest.fixture
def simple_translational_joint():
    return BaseJointFactory.new(a=0, alpha=0, d=None, theta=0)


@pytest.fixture
def simple_rotational_joint():
    return BaseJointFactory.new(a=0, alpha=0, d=0, theta=None)


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
        [0.100, -pi / 2, 0.350, 0],
        [0.250, 0.00000, 0.000, -pi / 2],
        [0.135, -pi / 2, 0.000, -pi / 2],
        [0.000, +pi / 2, 0.250, 0],
        [0.000, -pi / 2, 0.000, 0],
        [rtoff, 0.00000, 0.090 + atoff, pi]
    ]

    config = [BaseJointFactory.new(a=a, alpha=alpha, d=d, theta=None, offset=z) for a, alpha, d, z in dh_parameters]
    return config


def validate_vec(kind: str, actual, expected: List[float], atol=0.001):
    """
    Helper function to validate vectors of a homogenous matrix (4x4)
    :param kind:
    :param actual:
    :param expected:
    :param atol:
    :return:
    """
    if kind == 'X':
        np.testing.assert_allclose(actual[0:3, 0], np.array(expected), atol=atol)
    elif kind == 'Y':
        np.testing.assert_allclose(actual[0:3, 1], np.array(expected), atol=atol)
    elif kind == 'Z':
        np.testing.assert_allclose(actual[0:3, 2], np.array(expected), atol=atol)
    elif kind == 'pos':
        np.testing.assert_allclose(actual[0:3, 3], np.array(expected), atol=atol)
    else:
        raise AssertionError('Unknown vector kind.')


@pytest.mark.parametrize("joint_coordinate", [-10, 0, 3.7])
def test_forward_kinematics_two_translational_joints(simple_translational_joint, joint_coordinate):
    """
    Validate that the correct position and orientation is obtained for transformations for two translational joints.
    :param simple_translational_joint:
    :param joint_coordinate:
    :return:
    """
    # Calculate the forward kinematics
    transformation = forward_kinematics([simple_translational_joint] * 2, [joint_coordinate] * 2)

    validate_vec('X', transformation, [1, 0, 0])
    validate_vec('Y', transformation, [0, 1, 0])
    validate_vec('Z', transformation, [0, 0, 1])
    validate_vec('pos', transformation, [0, 0, 2 * joint_coordinate])


@pytest.mark.parametrize("joint_coordinate", [pi / 2, 0, 3.7, -2.1])
def test_forward_kinematics_rotational_joint(simple_rotational_joint, joint_coordinate):
    """
    Validate that the correct position and orientation is obtained for transformations for one rotational joint.
    :param simple_rotational_joint:
    :param joint_coordinate:
    :return:
    """
    # Calculate the forward kinematics
    transformation = forward_kinematics([simple_rotational_joint], [joint_coordinate])

    validate_vec('X', transformation, [cos(joint_coordinate), sin(joint_coordinate), 0])
    validate_vec('Y', transformation, [-sin(joint_coordinate), cos(joint_coordinate), 0])
    validate_vec('Z', transformation, [0, 0, 1])
    validate_vec('pos', transformation, [0, 0, 0])


@pytest.mark.parametrize("joints_deg,abcxyz,tcp_cs",
                         [
                             # Adjusted home position
                             (
                                     [0, 0, 90, 0, 0, 0],
                                     [-180.000, +90.000, +180.000, +439.641, +0.000, +734.911],
                                     # TCP coordinate system: x down, y left, z front
                                     [
                                         [0, 0, -1],
                                         [0, 1, 0],
                                         [1, 0, 0]
                                     ]
                             ),
                             # Adjusted home position moved to left end-stop of J1
                             (
                                     [-160, 0, 90, 0, 0, 0],
                                     [-180.000, +90.000, +20.000, -413.127, -150.366, +734.911],
                                     # TCP coordinate system:
                                     [
                                         [0, 0, -1],
                                         [sin(160 / 180 * pi), cos(160 / 180 * pi), 0],
                                         [cos(160 / 180 * pi), -sin(160 / 180 * pi), 0]
                                     ]
                             ),
                             # Adjusted home position moved to right end-stop of J1
                             (
                                     [+160, 0, 90, 0, 0, 0],
                                     [+0.000, +90.000, +160.000, -413.127, 150.366, +734.911],
                                     # TCP coordinate system:
                                     [
                                         [0, 0, -1],
                                         [-sin(160 / 180 * pi), cos(160 / 180 * pi), 0],
                                         [cos(160 / 180 * pi), sin(160 / 180 * pi), 0]
                                     ]
                             ),
                             # Arbitrary position
                             (
                                     [52.550, -42.990, 131.910, -43.210, 93.720, -103.650],
                                     [165.438, -41.185, -20.204, 153.391, 99.003, 606.813],
                                     None
                             )
                         ]
                         )
def test_forward_melfa_coordinates(dh_melfa_rv_4a, joints_deg, abcxyz, tcp_cs):
    """
    Validate some positions by XYZ-values obtained from the Mitsubishi simulator
    :param dh_melfa_rv_4a: Joint configurations based on DH-parameters for Mitsubishi Melfa RV-4A
    :param joints_deg: Joint angles in degrees
    :param abcxyz: Expected values for X,Y,Z and Euler-angles A, B, C
    """
    # Convert to radian
    joints_rad = [np.deg2rad(i) for i in joints_deg]

    # Calculate actual coordinates
    tcp_pose = forward_kinematics(dh_melfa_rv_4a, joints_rad)

    # Unpack expected coordinates
    a, b, c, *xyz = abcxyz

    # Convert mm to m
    xyz = [i / 1000 for i in xyz]

    print(tcp_pose)

    # Calculate quaternion
    quat = tform2quat(tcp_pose)
    print(f'Quaternion: {quat}')

    # Compare position vector in homogeneous matrix with expected value
    validate_vec('pos', tcp_pose, xyz)

    # Check TCP coordinate system
    if tcp_cs is not None:
        tcp_x, tcp_y, tcp_z = tcp_cs
        validate_vec('X', tcp_pose, tcp_x)
        validate_vec('Y', tcp_pose, tcp_y)
        validate_vec('Z', tcp_pose, tcp_z)
