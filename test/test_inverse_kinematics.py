from math import pi, cos, sin

import numpy as np
import pytest

from src.kinematics.forward_kinematics import forward_kinematics, get_tform, calculate_pose_flags
from src.kinematics.inverse_kinematics import ik_spherical_wrist, WristSingularity


def idfn(val):
    if isinstance(val, list) and len(val) == 6:
        joint_str = 'expected: '
        for joint in val:
            joint_str += f'{joint:+04}; '
        return joint_str[:-2]


@pytest.fixture
def dummy_tform():
    return get_tform([1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 2, 3])


@pytest.mark.parametrize("xdir,ydir,zdir,pos,flags,expected_joints,exc",
                         [
                             # Home position (RAN)
                             (
                                     [-1, 0, 0], [0, 1, 0], [0, 0, -1],
                                     [+349.519, +0.000, +644.789], 7,
                                     [0, 0, 90, 0, 90, 0], None
                             ),
                             # Wrist Singularity (theta 5 = 0)
                             pytest.param(
                                 [0, 0, -1],
                                 [sin(160 / 180 * pi), cos(160 / 180 * pi), 0],
                                 [cos(160 / 180 * pi), -sin(160 / 180 * pi), 0],
                                 [-413.127, -150.366, +734.911], 7,
                                 [-160, 0, 90, 0, 0, 0], WristSingularity
                             ),
                             # Arbitrary position (RAF)
                             (
                                     [0, 0, 1], [0, -1, 0], [1, 0, 0],
                                     [90.122, 239.937, 709.612], 6,
                                     [90, -26, 116, 90, -90, 90], None
                             ),
                             # Arbitrary position (RAN)
                             (
                                     [0, 0, 1], [0, 1, 0], [-1, 0, 0],
                                     [-90.122, 239.937, 709.612], 7,
                                     [90, -26, 116, 90, 90, 90], None
                             ),
                             # Arbitrary position (LAN)
                             (
                                     [0, 0, -1], [-1, 0, 0], [0, 1, 0],
                                     [0, 41.134, 719.298], 3,
                                     [90, -67, 112, 0, 45, 0], None
                             ),
                             # LAF
                             # LBF
                             # LBN
                             # RBN
                             # RBF
                             # Shoulder Singularity
                             # Elbow Singularity
                         ], ids=idfn
                         )
def test_ik_spherical_wrist(xdir, ydir, zdir, pos, expected_joints, flags, exc, dh_melfa_rv_4a):
    """
    Test that for a given TCP-pose the correct joint angles are determined.
    :param xdir: Unit vector of the x-axis of TCP coordinate system expressed in base system
    :param ydir: Unit vector of the y-axis of TCP coordinate system expressed in base system
    :param zdir: Unit vector of the z-axis of TCP coordinate system expressed in base system
    :param pos: TCP position expressed in base system
    :param expected_joints: List of expected joint angles in degrees
    :param dh_melfa_rv_4a: Joint configuration for Mitsubishi Melfa RV-4A
    :return:
    """
    # Convert the vectors to a homogeneous matrixy
    tform = get_tform(xdir, ydir, zdir, pos)

    # Calculate the inverse kinematics
    print('\n\nCalculating inverse kinematics...')
    if exc is None:
        # Regular solution should be possible
        actual_joints = ik_spherical_wrist(dh_melfa_rv_4a, tform, pose_flags=flags)

        # Test the solution
        expected_joints = np.deg2rad(expected_joints)
        print('\nChecking results...')
        print(f'Actual:  {[f"{actual:+.3f}" for actual in actual_joints]}')
        print(f'Expect:  {[f"{expect:+.3f}" for expect in expected_joints]}')

        non_flip = bool((flags & 1) == 1)
        up = bool((flags & 2) == 2)
        right = bool((flags & 4) == 4)

        print(f'EFlags:  {"R" if right else "L"},{"A" if up else "B"},{"N" if non_flip else "F"}')

        np.testing.assert_allclose(actual_joints, expected_joints, atol=0.01)

        # Reconvert the joint angles to a pose
        print('\n\nCalculating forward kinematics from solution...')
        tform_fk = forward_kinematics(dh_melfa_rv_4a, actual_joints)

        # Check that the matrices are the same
        print('\nChecking results...')
        np.testing.assert_allclose(tform_fk, tform, atol=0.1)
    else:
        # Singularity expected
        with pytest.raises(exc):
            ik_spherical_wrist(dh_melfa_rv_4a, tform, pose_flags=flags)
    print('All good!')


@pytest.mark.parametrize("pose_flag", [-1, 8])
def test_ik_spherical_wrist_bad_pose(pose_flag, dummy_tform, dh_melfa_rv_4a):
    """
    Test that bad pose flags are rejected
    :param pose_flag:
    :param dh_melfa_rv_4a:
    :return:
    """
    with pytest.raises(ValueError):
        ik_spherical_wrist(dh_melfa_rv_4a, dummy_tform, pose_flags=pose_flag)


def test_ik_spherical_wrist_bad_config(dummy_tform, dh_melfa_rv_4a):
    """
    Test that bad configs are rejected
    :param dh_melfa_rv_4a:
    :return:
    """
    with pytest.raises(ValueError):
        ik_spherical_wrist(dh_melfa_rv_4a[0:-1], dummy_tform, pose_flags=7)

    with pytest.raises(ValueError):
        ik_spherical_wrist(dh_melfa_rv_4a + [dh_melfa_rv_4a[-1]], dummy_tform, pose_flags=7)


@pytest.mark.parametrize("theta6", [-160, -90, 0, 90, 160])
@pytest.mark.parametrize("theta5", [-160, -90, 90, 160])
@pytest.mark.parametrize("theta4", [-160, -90, 0, 90, 160])
@pytest.mark.parametrize("theta3", [15, 20, 90, 130, 169])
@pytest.mark.parametrize("theta2", [-90, 0, 30, 140])
@pytest.mark.parametrize("theta1", [-160, -90, 0, 90, 160])
def test_ik_spherical_wrist_stress(theta1, theta2, theta3, theta4, theta5, theta6, dh_melfa_rv_4a):
    # Collect angles
    expected_joints = [theta1, theta2, theta3, theta4, theta5, theta6]
    expected_joints = np.deg2rad(expected_joints)

    # Calculate roboter pose
    tform_under_test = forward_kinematics(dh_melfa_rv_4a, expected_joints)

    # Calculate pose flags
    flags = calculate_pose_flags(dh_melfa_rv_4a, expected_joints)

    # Calculate inverse kinematics
    actual_joints = ik_spherical_wrist(dh_melfa_rv_4a, tform_under_test, pose_flags=flags)

    print(f'Actual:  {[f"{actual:+.3f}" for actual in actual_joints]}')
    print(f'Expect:  {[f"{expect:+.3f}" for expect in expected_joints]} Flags: {flags}')
    np.testing.assert_allclose(actual_joints, expected_joints, atol=0.01)
