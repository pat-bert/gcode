from math import pi, cos, sin
from typing import Optional

import numpy as np
import pytest

from src.kinematics.forward_kinematics import forward_kinematics, get_tform, calculate_pose_flags
from src.kinematics.inverse_kinematics import ik_spherical_wrist, WristSingularity, OutOfReachError, \
    ShoulderSingularity, ElbowSingularity

ONE_DEGREE = pi / 180
TENTH_DEGREE = 0.1 * ONE_DEGREE
HUNDREDTH_DEGREE = 0.01 * ONE_DEGREE

TENTH_MM = 0.1
HUNDREDTH_MM = 0.01


def idfn(val) -> Optional[str]:
    if val is None:
        return ''
    if isinstance(val, list):
        if len(val) == 6:
            joint_str = 'exp: '
            for joint in val:
                joint_str += f'{joint:+04}; '
            return joint_str[:-2]
        if len(val) == 0:
            return ''
        return None
    return None


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
                             # Arbitrary position (RAN)
                             (
                                     [0, 0, 1], [0, 1, 0], [-1, 0, 0],
                                     [-90.122, 239.937, 709.612], 7,
                                     [90, -26, 116, 90, 90, 90], None
                             ),
                             # Arbitrary position (RAF)
                             (
                                     [0, 0, 1], [0, -1, 0], [1, 0, 0],
                                     [90.122, 239.937, 709.612], 6,
                                     [90, -26, 116, 90, -90, 90], None
                             ),
                             # RBN
                             pytest.param(
                                 [-1, 0, 0], [0, 1, 0], [0, 0, -1],
                                 [493.107, 0.000, 615.287], 5,
                                 [0, 55, 15, 0, 110, 0], None, marks=pytest.mark.xfail(reason='Bad accuracy')
                             ),
                             # RBF
                             (
                                     [1, 0, 0], [0, 1, 0], [0, 0, 1],
                                     [493.107, 0.000, 795.532], 4,
                                     [0, 55, 15, 0, -70, 0], None
                             ),
                             # Arbitrary position (LAN)
                             (
                                     [0, 0, -1], [-1, 0, 0], [0, 1, 0],
                                     [0, 41.134, 719.298], 3,
                                     [90, -67, 112, 0, 45, 0], None
                             ),
                             # LAF
                             (
                                     [1, 0, 0], [0, 1, 0], [0, 0, 1],
                                     [-206.979, 0.000, 780.453], 2,
                                     [0, -75, 90, 0, -15, 0], None
                             ),
                             # LBN
                             pytest.param(
                                 [0, 0, -1], [-1, 0, 0], [0, 1, 0],
                                 [0, -63.224, 817.545], 1,
                                 [90, -24, 20, 0, 94, 0], None, marks=pytest.mark.xfail(reason='Bad accuracy')
                             ),
                             # LBF
                             pytest.param(
                                 [0, 0, 1], [-1, 0, 0], [0, -1, 0],
                                 [0, -243.468, 817.545], 0,
                                 [90, -24, 20, 0, -86, 0], None, marks=pytest.mark.xfail(reason='Bad accuracy')
                             ),
                             # Shoulder Singularity
                             (
                                     [1, 0, 0], [0, 1, 0], [0, 0, 1],
                                     [0.0, 0.0, 843.940], 6,
                                     [], ShoulderSingularity
                             ),
                             # Elbow Singularity
                             pytest.param(
                                 [0, 0, -1], [0, 1, 0], [1, 0, 0],
                                 [190.438, 0, 883.363], 5,
                                 [], ElbowSingularity, marks=pytest.mark.xfail(reason='Unclear specification.')
                             ),
                             # Wrist Singularity (theta 5 = 0)
                             (
                                     [0, 0, -1],
                                     [sin(160 / 180 * pi), cos(160 / 180 * pi), 0],
                                     [cos(160 / 180 * pi), -sin(160 / 180 * pi), 0],
                                     [-413.127, -150.366, +734.911], 7,
                                     [], WristSingularity
                             ),
                             # Point out of reach
                             (
                                     [1, 0, 0], [0, 1, 0], [0, 0, 1],
                                     [1000, 0, 650], 7,
                                     [], OutOfReachError
                             ),
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
        act_joints = ik_spherical_wrist(dh_melfa_rv_4a, tform, pose_flags=flags)

        # Test the solution
        expected_joints = np.deg2rad(expected_joints)
        print('\nChecking results...')
        print(f'Actual:  {[f"{actual:+.5f}" for actual in act_joints]}')
        print(f'Expect:  {[f"{expect:+.5f}" for expect in expected_joints]}')

        non_flip = bool((flags & 1) == 1)
        up = bool((flags & 2) == 2)
        right = bool((flags & 4) == 4)

        print(f'EFlags:  {"R" if right else "L"},{"A" if up else "B"},{"N" if non_flip else "F"}')

        # Reconvert the joint angles to a pose
        print('\n\nCalculating forward kinematics from solution...')
        tform_fk = forward_kinematics(dh_melfa_rv_4a, act_joints)

        # Check that the matrices are the same
        print('\nChecking results...')

        np.testing.assert_allclose(tform_fk, tform, atol=TENTH_MM, err_msg='FK result deviated by > 0.1 mm')
        np.testing.assert_allclose(act_joints, expected_joints, atol=ONE_DEGREE, err_msg='IK result off by > 1 deg')
    else:
        # Singularity expected
        with pytest.raises(exc):
            act_joints = ik_spherical_wrist(dh_melfa_rv_4a, tform, pose_flags=flags)
            print(f'Actual:  {[f"{actual:+.3f}" for actual in act_joints]}')
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


@pytest.mark.parametrize("expected_joints",
                         [
                             # 0
                             (-70, 0, 15, +160, -90, 90),
                             (-70, 0, 169, -70, -60, 120),
                             (30, 0, 15, -50, 30, 30),
                             (50, 0, 169, 30, 70, 45),
                             (30, -90, 15, -50, 30, 30),
                             # 5
                             (30, -90, 15, -50, -30, 80),
                             (30, -90, 30, -50, 30, -70),
                             (30, -90, 120, -50, -30, 30),
                             (0, -4, 30, 0, -116, 0),
                             (0, 55, 15, 0, 110, 0),
                             # 10
                             (90, -25, 25, 0, 90, 0),
                             (90, -25, 25, 0, -90, 0)
                         ]
                         )
def test_ik_spherical_wrist_fk_based(expected_joints, dh_melfa_rv_4a, benchmark):
    expected_joints = np.deg2rad(expected_joints)

    # Calculate roboter pose
    tform_under_test = forward_kinematics(dh_melfa_rv_4a, expected_joints)

    # Calculate pose flags
    flags = calculate_pose_flags(dh_melfa_rv_4a, expected_joints)

    # Calculate inverse kinematics
    actual_joints = benchmark(ik_spherical_wrist, dh_melfa_rv_4a, tform_under_test, pose_flags=flags)

    print(f'Actual:  {[f"{actual:+.5f}" for actual in actual_joints]}')
    print(f'Expect:  {[f"{expect:+.5f}" for expect in expected_joints]} Flags: {flags}')
    np.testing.assert_allclose(actual_joints, expected_joints, atol=HUNDREDTH_DEGREE)
