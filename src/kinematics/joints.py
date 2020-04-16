import abc
from enum import Enum, unique
from math import cos, sin

import numpy as np


@unique
class JointType(Enum):
    """
    Define enums for each available plane.
    """

    ROTATIONAL = 1
    TRANSLATIONAL = 2


class BaseJoint(metaclass=abc.ABCMeta):
    """
    Denavit-Hartenberg representation for any 1DOF joint.
    """

    def __init__(self, a=0, alpha=0, d=None, theta=None):
        """
        Saves all the parameters and creates the initial matrix.
        :param a: link length in m
        :param alpha: twist angle in rad
        :param d: joint distance in m
        :param theta: joint angle in radS
        """
        # Save parameters
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

        # Common parameter
        self.c_alpha = cos(alpha)
        self.s_alpha = sin(alpha)

        # Allocation is costly, so the variable parameters will be resolved at a later stage
        self.matrix = np.array(
            [
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, self.s_alpha, self.c_alpha, 0],
                [0, 0, 0, 1]
            ]
        )

    @abc.abstractmethod
    def mul(self, *, joint_value):
        """
        Abstract method with arbitrary interface
        :param joint_value: Can be either angle (rad) or translation (m)
        :return:
        """

    @property
    @abc.abstractmethod
    def joint_type(self) -> JointType:
        """
        Give the joint type of the joint.
        :return: Joint Type Enumeration Value
        """


class BaseRotationalJoint(BaseJoint):
    """
    Interface: Denavit-Hartenberg representation for any 1DOF rotational joint.
    """

    def __init__(self, *, a, alpha, d):
        """
        Common initialisation for all rotational joints.
        :param a: link length in m
        :param alpha: twist angle in rad
        :param d: Constant joint distance in m
        """
        super().__init__(a=a, alpha=alpha, d=d)
        self.matrix[2][-1] = d

    @abc.abstractmethod
    def mul(self, *, joint_value):
        """
        Abstract method with more specific interface
        :param joint_value: Current joint angle in rad
        :return:
        """

    @property
    def joint_type(self) -> JointType:
        """
        Specifies the joint type for all rotational joints.
        :return: Rotational Joint
        """
        return JointType.ROTATIONAL


class GeneralRotationalJoint(BaseRotationalJoint):
    """
    Denavit-Hartenberg representation for 1DOF rotational joint without simplifications.
    """

    def __init__(self, *, a, alpha, d):
        """
        Initialize the general joint
        :param a: link length in m
        :param alpha: twist angle in rad
        :param d: joint distance in m
        """
        super().__init__(a=a, alpha=alpha, d=d)

    def mul(self, *, joint_value):
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or m)
        :return:
        """
        theta = joint_value
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows
        self.matrix[0:2][:] = [
            [cos(theta), -self.c_alpha * s_theta, self.s_alpha * s_theta, self.a * c_theta],
            [sin(theta), self.c_alpha * c_theta, -self.s_alpha * c_theta, self.a * s_theta]
        ]


class NoOffsetRotationalJoint(BaseRotationalJoint):
    """
    Denavit-Hartenberg representation for 1DOF rotational joint with simplified link length
    """

    def __init__(self, *, alpha, d):
        """
        Creates a rotational joint with zero link length
        :param alpha: twist angle in rad
        :param d: joint distance in m
        """
        super().__init__(alpha=alpha, d=d, a=0)

    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or m)
        :return: None
        """
        theta = joint_value
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first three elements of the first two rows (last column = 0 due to a = 0)
        self.matrix[0:2][:] = [
            [cos(theta), -self.c_alpha * s_theta, self.s_alpha * s_theta, 0],
            [sin(theta), self.c_alpha * c_theta, -self.s_alpha * c_theta, 0]
        ]


class ParallelRotationalJoint(BaseRotationalJoint):
    """
    alpha = 0 => sin(alpha) = 0
    """

    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or m)
        :return: None
        """
        theta = joint_value
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows (third column = 0 due to sin(alpha)=0)
        self.matrix[0:2][:] = [
            [c_theta, -self.c_alpha * s_theta, 0, self.a * c_theta],
            [s_theta, self.c_alpha * c_theta, 0, self.a * s_theta]
        ]


class ParallelNoOffsetRotationalJoint(NoOffsetRotationalJoint):
    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or m)
        :return: None
        """
        # Calculate common values
        theta = joint_value
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows (third column = 0 due to sin(alpha)=0)
        self.matrix[0:2][:] = [
            [c_theta, -self.c_alpha * s_theta, 0, 0],
            [s_theta, self.c_alpha * c_theta, 0, 0]
        ]


class PerpendicularRotationalJoint(BaseRotationalJoint):
    """
    alpha = +- 90° => cos(alpha) = 0
    """

    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or m)
        :return: None
        """
        # Calculate common values
        theta = joint_value
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows (second column = 0 due to cos(alpha)=0)
        self.matrix[0:2][:] = [
            [c_theta, 0, self.s_alpha * s_theta, self.a * c_theta],
            [s_theta, 0, -self.s_alpha * c_theta, self.a * s_theta]
        ]


class PerpendicularNoOffsetRotationalJoint(NoOffsetRotationalJoint):
    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or m)
        :return: None
        """
        theta = joint_value
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows (last column = 0 due to a = 0, second column = 0 due to cos(alpha)=0)
        self.matrix[0:2][:] = [
            [cos(theta), 0, self.s_alpha * s_theta, 0],
            [sin(theta), 0, -self.s_alpha * c_theta, 0]
        ]


class TranslationalJoint(BaseJoint):
    """
    Representation of a purely translational joint.
    """

    def __init__(self, a, alpha, theta):
        """
        Creates a matrix with constant elements.
        :param a: link length in m
        :param alpha: twist angle in rad
        :param theta: joint angle in radS
        """
        super().__init__(a=a, alpha=alpha, theta=theta)

        # Calculate common values
        c_theta = cos(theta)
        s_theta = sin(theta)

        # Override the first two rows
        self.matrix[0:2][:] = [
            [c_theta, -s_theta * self.c_alpha, s_theta * self.s_alpha, a * c_theta],
            [s_theta, c_theta * self.c_alpha, -c_theta * self.s_alpha, a * s_theta]
        ]

    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint distance (rad or m)
        :return:
        """
        # Override the 4th element of the third row
        self.matrix[2][3] = joint_value

    @property
    def joint_type(self) -> JointType:
        """
        Specifies the joint type for all translational joints.
        :return: Translational Joint
        """
        return JointType.TRANSLATIONAL
