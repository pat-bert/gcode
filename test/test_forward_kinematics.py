from math import pi, cos, sin
from typing import List

import numpy as np
import pytest

from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joint_factories import BaseJointFactory


@pytest.fixture
def simple_translational_joint():
    return BaseJointFactory.new(a=0, alpha=0, d=None, theta=0)


@pytest.fixture
def simple_rotational_joint():
    return BaseJointFactory.new(a=0, alpha=0, d=0, theta=None)


@pytest.fixture
def dh_melfa_rv_4a():
    rtoff = 0.0  # radial tool offset
    atoff = 0.0  # axial tool offset

    # Denavit-Hartenberg parameters: a - alpha - d
    dh_parameters = [
        [0.100, -pi / 2, 0.350],
        [0.250, 0.00000, 0.000],
        [0.135, -pi / 2, 0.000],
        [0.000, +pi / 2, 0.250],
        [0.000, -pi / 2, 0.000],
        [rtoff, 0.00000, atoff]
    ]

    config = [BaseJointFactory.new(a=a, alpha=alpha, d=d, theta=None) for a, alpha, d in dh_parameters]
    return config


def validate_vec(kind: str, actual, expected: List[float]):
    if kind == 'X':
        np.testing.assert_allclose(actual[0:3, 0], np.array(expected))
    elif kind == 'Y':
        np.testing.assert_allclose(actual[0:3, 1], np.array(expected))
    elif kind == 'Z':
        np.testing.assert_allclose(actual[0:3, 2], np.array(expected))
    elif kind == 'pos':
        np.testing.assert_allclose(actual[0:3, 3], np.array(expected))
    else:
        raise AssertionError('Unknown vector kind.')


@pytest.mark.parametrize("joint_coordinate", [-10, 0, 3.7])
def test_forward_kinematics_two_translational_joints(simple_translational_joint, joint_coordinate):
    # Calculate the forward kinematics
    transformation = forward_kinematics([simple_translational_joint] * 2, [joint_coordinate] * 2)

    validate_vec('X', transformation, [1, 0, 0])
    validate_vec('Y', transformation, [0, 1, 0])
    validate_vec('Z', transformation, [0, 0, 1])
    validate_vec('pos', transformation, [0, 0, 2 * joint_coordinate])


@pytest.mark.parametrize("joint_coordinate", [pi / 2, 0, 3.7, -2.1])
def test_forward_kinematics_rotational_joint(simple_rotational_joint, joint_coordinate):
    # Calculate the forward kinematics
    transformation = forward_kinematics([simple_rotational_joint], [joint_coordinate])

    validate_vec('X', transformation, [cos(joint_coordinate), sin(joint_coordinate), 0])
    validate_vec('Y', transformation, [-sin(joint_coordinate), cos(joint_coordinate), 0])
    validate_vec('Z', transformation, [0, 0, 1])
    validate_vec('pos', transformation, [0, 0, 0])


@pytest.mark.skip(reason='Not implemented.')
def test_melfa_coordinates(dh_melfa_rv_4a):
    assert False
