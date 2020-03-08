from math import cos, sin, pi

import pytest

from src.kinematics.joint_factories import BaseJointFactory, BASE_TOLERANCE

# Factor schemes (1 = field should be calculated upon init, 0 = field needs to be determined at runtime/multiplication)
translational_joint_schema = [[1, 1, 1, 1], [1, 1, 1, 1], [1, 1, 1, 0], [1, 1, 1, 1]]
general_rotational_joint_schema = [[0, 0, 0, 0], [0, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]]
zero_twist_schema = [[0, 0, 1, 0], [0, 0, 1, 0], [1, 1, 1, 1], [1, 1, 1, 1]]
perpendicular_twist_schema = [[0, 1, 0, 0], [0, 1, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]]
no_offset_schema = [[0, 0, 0, 1], [0, 0, 0, 1], [1, 1, 1, 1], [1, 1, 1, 1]]
zero_twist_no_offset_schema = [[0, 0, 1, 1], [0, 0, 1, 1], [1, 1, 1, 1], [1, 1, 1, 1]]
perpendicular_twist_no_offset_schema = [[0, 1, 0, 1], [0, 1, 0, 1], [1, 1, 1, 1], [1, 1, 1, 1]]


def idfn(val):
    """
    Generate test annotations for better readability.
    :param val:
    :return:
    """
    if val == translational_joint_schema:
        return '<Simple TransJoint>'
    elif val == general_rotational_joint_schema:
        return '<GeneralRotJoint>'
    elif val == zero_twist_schema:
        return '<NoTwistRotJoint>'
    elif val == perpendicular_twist_schema:
        return '<PerpTwistRotJoint>'
    elif val == no_offset_schema:
        return '<NoOffsetRotJoint>'
    elif val == zero_twist_no_offset_schema:
        return '<NoTwistNoOffsetRotJoint>'
    elif val == perpendicular_twist_no_offset_schema:
        return '<PerpTwistNoOffsetRotJoint>'


def reference_dh(a, alpha, d, theta):
    s_alpha = sin(alpha)
    c_alpha = cos(alpha)

    if abs(s_alpha) <= BASE_TOLERANCE:
        s_alpha = 0
    if abs(c_alpha) <= BASE_TOLERANCE:
        c_alpha = 0

    return [
        [cos(theta), -sin(theta) * c_alpha, sin(theta) * s_alpha, a * cos(theta)],
        [sin(theta), cos(theta) * c_alpha, -cos(theta) * s_alpha, a * sin(theta)],
        [0, s_alpha, c_alpha, d],
        [0, 0, 0, 1]
    ]


class TestBaseJointFactory:
    @pytest.mark.parametrize("d,theta", [(None, None), (-4, 5)])
    def test_new_combined_fail(self, d, theta):
        with pytest.raises(ValueError) as excinfo:
            BaseJointFactory.new(a=0, alpha=0, d=d, theta=theta)
        assert str(excinfo.value) == "Cannot specify combined joint."

    @pytest.mark.parametrize("a,alpha,d,theta,actual_val,const_matrix_factors",
                             [
                                 # Translational joints:
                                 (30, 2.7, None, 0, 10, translational_joint_schema),
                                 # Rotational joints:
                                 (30, 2.7, -15, None, 2.7, general_rotational_joint_schema),
                                 (0, 2.7, -15, None, 2.7, no_offset_schema),
                                 (30, pi / 2, -15, None, 2.7, perpendicular_twist_schema),
                                 (30, -pi / 2, -15, None, 2.7, perpendicular_twist_schema),
                                 (30, 0, -15, None, 2.7, zero_twist_schema),
                                 (30, pi, -15, None, 2.7, zero_twist_schema),
                                 (30, -pi, -15, None, 2.7, zero_twist_schema),
                                 (0, pi, -15, None, 2.7, zero_twist_no_offset_schema),
                                 (0, pi / 2, -15, None, 2.7, perpendicular_twist_no_offset_schema),
                                 (0, -pi / 2, -15, None, 2.7, perpendicular_twist_no_offset_schema),
                             ], ids=idfn
                             )
    def test_new(self, a, alpha, d, theta, actual_val, const_matrix_factors):
        # Construct a joint
        joint = BaseJointFactory.new(a=a, alpha=alpha, d=d, theta=theta)

        # Print the class
        print(type(joint))

        # Check that all the parameters have been set
        assert joint.a == a
        assert joint.alpha == alpha
        assert joint.d == d
        assert joint.theta == theta

        # Set correct values
        if d is None:
            d = actual_val
        if theta is None:
            theta = actual_val

        # Calculate the whole matrix
        expected_full_matrix = reference_dh(a, alpha, d, theta)

        # Calculate expected constant matrix (apply factors)
        expected_const_matrix = []
        for row_factors, row in zip(const_matrix_factors, expected_full_matrix):
            expected_const_matrix.append([i * factor for i, factor in zip(row, row_factors)])

        actual_const_matrix = [list(i) for i in joint.matrix]

        # Test that the full matrix is calculated correctly
        for expected_row, actual_row in zip(expected_const_matrix, actual_const_matrix):
            for expected, actual in zip(expected_row, actual_row):
                assert pytest.approx(expected, abs=BASE_TOLERANCE) == actual

        # Calculate full matrix
        joint.mul(theta=theta, d=d)

        actual_full_matrix = [list(i) for i in joint.matrix]

        # Test that the full matrix is calculated correctly
        for expected_row, actual_row in zip(expected_full_matrix, actual_full_matrix):
            for expected, actual in zip(expected_row, actual_row):
                assert pytest.approx(expected, abs=BASE_TOLERANCE) == actual
