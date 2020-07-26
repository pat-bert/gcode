import abc
from enum import Enum, unique
from math import cos, sin
from typing import Optional

import numpy as np


@unique
class JointType(Enum):
    """ Define enums for each available plane.
    """

    ROTATIONAL = 1
    TRANSLATIONAL = 2


class BaseJoint(metaclass=abc.ABCMeta):
    """ Denavit-Hartenberg representation for any 1DOF joint.
    """

    def __init__(self, a: float = 0, alpha: float = 0, d: Optional[float] = None, theta: Optional[float] = None,
                 offset: float = 0):
        """
        Saves all the parameters and creates the initial matrix.
        :param a: link length in mm
        :param alpha: twist angle in rad
        :param d: joint distance in mm
        :param theta: joint angle in rad
        :param offset: Specifies an offset for the zero position of the joint coordinate to be applied to each joint
        coordinate given, defaults to zero.
        """
        # Save parameters
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta
        self.zero_offset = offset

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
            ], dtype=np.float64
        )

    @abc.abstractmethod
    def mul(self, *, joint_value):
        """
        Abstract method with arbitrary interface
        :param joint_value: Can be either angle (rad) or translation (mm)
        :return:
        """

    @property
    @abc.abstractmethod
    def joint_type(self) -> JointType:
        """
        Give the joint type of the joint.
        :return: Joint Type Enumeration Value
        """

    def __len__(self):
        return 1


class BaseRotationalJoint(BaseJoint):
    """ Interface: Denavit-Hartenberg representation for any 1DOF rotational joint.
    """

    def __init__(self, *, a, alpha, d, offset=0):
        """
        Common initialisation for all rotational joints.
        :param a: link length in mm
        :param alpha: twist angle in rad
        :param d: Constant joint distance in mm
        :param offset: Specifies an offset for the zero position of the joint coordinate to be applied to each joint
        coordinate given, defaults to zero.
        """
        super().__init__(a=a, alpha=alpha, d=d, offset=offset)
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
    """ Denavit-Hartenberg representation for 1DOF rotational joint without simplifications.
    """

    def __init__(self, *, a, alpha, d, offset=0):
        """
        Initialize the general joint
        :param a: link length in mm
        :param alpha: twist angle in rad
        :param d: joint distance in mm
        :param offset: Specifies an offset for the zero position of the joint coordinate to be applied to each joint
        coordinate given, defaults to zero.
        """
        super().__init__(a=a, alpha=alpha, d=d, offset=offset)

    def mul(self, *, joint_value):
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or mm)
        :return:
        """
        theta = joint_value + self.zero_offset
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows
        self.matrix[0:2][:] = [
            [cos(theta), -self.c_alpha * s_theta, self.s_alpha * s_theta, self.a * c_theta],
            [sin(theta), self.c_alpha * c_theta, -self.s_alpha * c_theta, self.a * s_theta]
        ]


class NoOffsetRotationalJoint(BaseRotationalJoint):
    """ Denavit-Hartenberg representation for 1DOF rotational joint with simplified link length
    """

    def __init__(self, *, alpha, d, offset=0):
        """
        Creates a rotational joint with zero link length
        :param alpha: twist angle in rad
        :param d: joint distance in mm
        :param offset: Specifies an offset for the zero position of the joint coordinate to be applied to each joint
        coordinate given, defaults to zero.
        """
        super().__init__(alpha=alpha, d=d, a=0, offset=offset)

    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or mm)
        :return: None
        """
        theta = joint_value + self.zero_offset
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first three elements of the first two rows (last column = 0 due to a = 0)
        self.matrix[0:2][:] = [
            [cos(theta), -self.c_alpha * s_theta, self.s_alpha * s_theta, 0],
            [sin(theta), self.c_alpha * c_theta, -self.s_alpha * c_theta, 0]
        ]


class ParallelRotationalJoint(BaseRotationalJoint):
    """ alpha = 0 => sin(alpha) = 0
    """

    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or mm)
        :return: None
        """
        theta = joint_value + self.zero_offset
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
        :param joint_value: Current joint angle (rad or mm)
        :return: None
        """
        # Calculate common values
        theta = joint_value + self.zero_offset
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows (third column = 0 due to sin(alpha)=0)
        self.matrix[0:2][:] = [
            [c_theta, -self.c_alpha * s_theta, 0, 0],
            [s_theta, self.c_alpha * c_theta, 0, 0]
        ]


class PerpendicularRotationalJoint(BaseRotationalJoint):
    """ alpha = +- 90° => cos(alpha) = 0
    """

    def mul(self, *, joint_value) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint angle (rad or mm)
        :return: None
        """
        # Calculate common values
        theta = joint_value + self.zero_offset
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
        :param joint_value: Current joint angle (rad or mm)
        :return: None
        """
        theta = joint_value + self.zero_offset
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows (last column = 0 due to a = 0, second column = 0 due to cos(alpha)=0)
        self.matrix[0:2][:] = [
            [cos(theta), 0, self.s_alpha * s_theta, 0],
            [sin(theta), 0, -self.s_alpha * c_theta, 0]
        ]


class TranslationalJoint(BaseJoint):
    """ Representation of a purely translational joint.
    """

    def __init__(self, a, alpha, theta, offset=0):
        """
        Creates a matrix with constant elements.
        :param a: link length in mm
        :param alpha: twist angle in rad
        :param theta: joint angle in rad
        :param offset: Specifies an offset for the zero position of the joint coordinate to be applied to each joint
        coordinate given, defaults to zero.
        """
        super().__init__(a=a, alpha=alpha, theta=theta, offset=offset)

        # Calculate common values
        c_theta = cos(theta)
        s_theta = sin(theta)

        # Override the first two rows
        self.matrix[0:2][:] = [
            [c_theta, -s_theta * self.c_alpha, s_theta * self.s_alpha, a * c_theta],
            [s_theta, c_theta * self.c_alpha, -c_theta * self.s_alpha, a * s_theta]
        ]

    def mul(self, *, joint_value: float) -> None:
        """
        Sets the variable elements of the matrix.
        :param joint_value: Current joint distance (rad or mm)
        :return:
        """
        # Override the 4th element of the third row
        self.matrix[2][3] = joint_value + self.zero_offset

    @property
    def joint_type(self) -> JointType:
        """
        Specifies the joint type for all translational joints.
        :return: Translational Joint
        """
        return JointType.TRANSLATIONAL


class Singularity(ValueError):
    pass


class ShoulderSingularity(Singularity):
    """
    Will be raised if wrist center point is on J1 axis
    """


class WristSingularity(Singularity):
    """
    Will be raised if J4 and J6 align (infinite solutions)
    """


class ElbowSingularity(Singularity):
    """
    Will be raised if the wrist center point is within the plane through J2 and J3
    """


WRIST_SINGULARITY_THRESHOLD = 1e-3
ELBOW_SINGULARITY_THRESHOLD = 1e-3
SHOULDER_SINGULARITY_THRESHOLD = 1e-3
