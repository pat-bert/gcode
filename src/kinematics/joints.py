import abc
from math import cos, sin

import numpy as np


class BaseJoint(metaclass=abc.ABCMeta):
    """
    Denavit-Hartenberg representation for any 1DOF joint.
    """

    def __init__(self, a=0, alpha=0, d=None, theta=None):
        """
        Saves all the parameters and creates the initial matrix.
        :param a: link length
        :param alpha: twist angle
        :param d: joint distance
        :param theta: joint angle
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
    def mul(self, **kwargs):
        """
        Abstract method with arbitrary interface
        :param kwargs: Denavit-Hartenberg parameters
        :return:
        """
        pass


class BaseRotationalJoint(BaseJoint, metaclass=abc.ABCMeta):
    """
    Denavit-Hartenberg representation for any 1DOF rotational joint.
    """

    def __init__(self, a, alpha, d):
        super().__init__(a=a, alpha=alpha, d=d)
        self.matrix[2][-1] = d

    @abc.abstractmethod
    def mul(self, *, theta):
        pass


class GeneralRotationalJoint(BaseRotationalJoint):
    def __init__(self, a, alpha, d):
        super().__init__(a=a, alpha=alpha, d=d)

    def mul(self, *, theta):
        s_theta = sin(theta)
        c_theta = cos(theta)

        self.matrix[2:][:] = [
            [cos(theta), -self.c_alpha * s_theta, self.s_alpha * s_theta, self.a * c_theta],
            [sin(theta), self.c_alpha * c_theta, -self.s_alpha * c_theta, self.a * s_theta]
        ]


class NoOffsetRotationalJoint(BaseRotationalJoint):
    def __init__(self, alpha, d):
        """
        Creates a rotational joint with zero link length
        :param alpha: Joint twist
        :param d: Joint distance
        """
        super().__init__(alpha=alpha, d=d, a=0)

    def mul(self, *, theta):
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first three elements of the first two rows (last column = 0 due to a = 0)
        self.matrix[0:2][0:3] = [
            [cos(theta), -self.c_alpha * s_theta, self.s_alpha * s_theta],
            [sin(theta), self.c_alpha * c_theta, -self.s_alpha * c_theta]
        ]


class ParallelRotationalJoint(BaseRotationalJoint):
    """
    alpha = 0 => sin(alpha) = 0
    """

    def __init__(self, a, d):
        """
        Creates a rotational joint with zero twist.
        :param a: Link length
        :param d: Joint distance
        """
        super().__init__(alpha=0, d=d, a=a)

    def mul(self, *, theta):
        # Calculate common values
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows (third column = 0 due to sin(alpha)=0)
        self.matrix[0:2][:] = [
            [c_theta, -self.c_alpha * s_theta, 0, self.a * c_theta],
            [s_theta, self.c_alpha * c_theta, 0, self.a * s_theta]
        ]


class PerpendicularRotationalJoint(BaseRotationalJoint):
    """
    alpha = +- 90° => cos(alpha) = 0
    """

    def mul(self, *, theta):
        # Calculate common values
        s_theta = sin(theta)
        c_theta = cos(theta)

        # Override the first two rows (second column = 0 due to cos(alpha)=0)
        self.matrix[0:2][:] = [
            [c_theta, 0, self.s_alpha * s_theta, self.a * c_theta],
            [s_theta, 0, -self.s_alpha * c_theta, self.a * s_theta]
        ]


class TranslationalJoint(BaseJoint):
    def __init__(self, a, alpha, theta):
        """
        Creates a matrix with constant elements.
        :param a:
        :param alpha:
        :param theta:
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

    def mul(self, *, d):
        """
        Sets the variable elements of the matrix.
        :param d:
        :return:
        """
        # Override the 4th element of the third row
        self.matrix[2][3] = d
