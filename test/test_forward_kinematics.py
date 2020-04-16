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
def simple_translational_joint():
    return BaseJointFactory.new(a=0, alpha=0, d=0, theta=None)


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


@pytest.mark.parametrize("joint_coordinate", [pi / 4, 0, 3.7])
def test_forward_kinematics_rotational_joint(simple_translational_joint, joint_coordinate):
    # Calculate the forward kinematics
    transformation = forward_kinematics([simple_translational_joint] * 2, [joint_coordinate] * 2)

    validate_vec('X', transformation, [cos(joint_coordinate), sin(joint_coordinate), 0])
    validate_vec('Y', transformation, [0, 1, 0])
    validate_vec('Z', transformation, [0, 0, 1])
    validate_vec('pos', transformation, [0, 0, 0])
