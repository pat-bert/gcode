from math import pi, cos, sin
from typing import List

import numpy as np
import pytest

from src.kinematics.forward_kinematics import forward_kinematics, tform2quat, tform2euler, calculate_pose_flags
from src.kinematics.joint_factories import BaseJointFactory


@pytest.fixture
def simple_translational_joint():
    return BaseJointFactory.new(a=0, alpha=0, d=None, theta=0)


@pytest.fixture
def simple_rotational_joint():
    return BaseJointFactory.new(a=0, alpha=0, d=0, theta=None)


def validate_vec(kind: str, actual, expected: List[float], atol=1):
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
def test_forward_melfa_coordinates(dh_melfa_rv_4a, benchmark, joints_deg, abcxyz, tcp_cs):
    """
    Validate some positions by XYZ-values obtained from the Mitsubishi simulator
    :param dh_melfa_rv_4a: Joint configurations based on DH-parameters for Mitsubishi Melfa RV-4A
    :param joints_deg: Joint angles in degrees
    :param abcxyz: Expected values for X,Y,Z and Euler-angles A, B, C
    """
    # Convert to radian
    joints_rad = [np.deg2rad(i) for i in joints_deg]

    # Calculate actual coordinates
    tcp_pose = benchmark(forward_kinematics, dh_melfa_rv_4a, joints_rad)

    # Unpack expected coordinates
    a, b, c, *xyz = abcxyz

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


@pytest.mark.parametrize("joints_deg,a,b,c",
                         [
                             # Home position
                             ([0, 0, 90, 0, 90, 0], 180, 0, 180),
                             ([-110, 0, 90, 0, 90, 0], -180, 0, 70),
                             # Arbitrary positions
                             ([5.951, -23.864, 139.059, -153.690, 65.113, -51.627], 30, -50, 120),
                             ([37.200, 14.850, 74.505, -20.107, 88.542, -177.127], 160, -3, 35),
                             ([0, -54.748, 141.102, 0, 3.646, 42.34], 90, 47.66, 90),
                             # Degenerate beta = +90 deg
                             ([0, -54.748, 141.102, 0, 3.646, 0], -180, 90, 180),
                             ([-63.008, -18.080, 124.879, 98.374, 64.246, -108.716], 138.009, 89.999, 138.009),
                             ([22.835, -34.309, 134.587, -87.672, 102.626, 100.536], 95.896, 90, 15.896),
                             ([0, 29.017, 57.018, 0, 3.965, 0], -180, 90, 180),
                             # Degenerate beta = -90 deg
                             ([0, 52.521, 98.176, 0, 119.303, 0], 0, -90, 0),
                         ]
                         )
def test_tform2euler(dh_melfa_rv_4a, joints_deg, a, b, c):
    # Convert to radian
    joints_rad = [np.deg2rad(i) for i in joints_deg]

    # Calculate actual coordinates
    tcp_pose = forward_kinematics(dh_melfa_rv_4a, joints_rad)

    # Calculate euler angles
    actual_angles = tform2euler(tcp_pose)
    actual_angles = np.rad2deg(actual_angles)
    a_actual, b_actual, c_actual = actual_angles

    digits = 5
    print(f'Actual: A:{a_actual:.{digits}f}° B:{b_actual:.{digits}f}° C:{c_actual:.{digits}f}°')
    print(f'Expected: A:{a:.{digits}f}° B:{b:.{digits}f}° C:{c:.{digits}f}°')

    # Check
    diff_to_gimbal_lock = abs(abs(b) - 90)

    # Different tolerance depending on distance to gimbal lock
    if diff_to_gimbal_lock == 0.0:
        # Gimbal lock!
        assert round(b - b_actual, 3) % 360 == pytest.approx(0, abs=0.01)
        assert round(a - a_actual + c - c_actual, 3) % 360 == pytest.approx(0, abs=1.5)
    else:
        if diff_to_gimbal_lock > 0.1:
            # Normal
            atol = 0.01
        else:
            # Very close
            atol = 1.0

        # Rounding to many digits is still necessary to get the modulo right
        r = 10

        # Angles match or are off by 360 degrees
        assert (a == pytest.approx(a_actual, abs=atol)) or (round(a - a_actual, r) % 360 == pytest.approx(0, abs=atol))
        assert (a == pytest.approx(a_actual, abs=atol)) or (round(b - b_actual, r) % 360 == pytest.approx(0, abs=atol))
        assert (a == pytest.approx(a_actual, abs=atol)) or (round(c - c_actual, r) % 360 == pytest.approx(0, abs=atol))


@pytest.mark.parametrize("joints,ex_flags",
                         [
                             # RBN
                             [(-160, 0, 28, 0, 20, 0), 5],
                             # RAN
                             [(-160, 0, 29, 0, 20, 0), 7],
                             # RAF
                             [(-160, 0, 29, 0, -20, 0), 6],
                             # RBF
                             [(-160, 0, 28, 0, -20, 0), 4],
                             # LAN
                             [(0, -46, 90, 0, 90, 0), 3],
                             # RAN
                             [(0, -45, 90, 0, 90, 0), 7],
                             # LAF
                             [(0, -50, 90, 0, -30, 0), 2],
                             # LBF
                             [(0, -15, 28, 160, -5, 0), 0],
                             [(120, -15, 28, 160, -5, 0), 0],
                             # LBN
                             [(0, -15, 28, 160, 5, 0), 1]
                         ]
                         )
def test_calculate_pose_flags(joints, ex_flags, dh_melfa_rv_4a):
    """
    Test that for given angles the correct flags can be calculated
    :param joints:
    :param ex_flags:
    :param dh_melfa_rv_4a:
    :return:
    """
    # Collect angles
    joints = np.deg2rad(joints)

    # Calculate flags
    flags = calculate_pose_flags(dh_melfa_rv_4a, joints)

    assert flags == ex_flags
