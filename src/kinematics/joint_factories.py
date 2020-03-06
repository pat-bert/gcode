from math import cos

from kinematics.joints import BaseJoint, TranslationalJoint, BaseRotationalJoint, NoOffsetRotationalJoint, \
    ParallelRotationalJoint, PerpendicularRotationalJoint, GeneralRotationalJoint


class BaseJointFactory:
    """
    Build the correct joint type.
    """

    @staticmethod
    def new(a, alpha, d=None, theta=None) -> BaseJoint:
        if d is None:
            theta = theta if theta is not None else 0
            return TranslationalJoint(a=a, alpha=alpha, theta=theta)
        elif theta is None:
            d = d if d is not None else 0
            return RotationalJointFactory.new(a=a, alpha=alpha, d=d)


class RotationalJointFactory:
    """
    Build optimized rotational joints.
    """

    @staticmethod
    def new(*, a, alpha, d) -> BaseRotationalJoint:
        """
        Uses the joint parameters to derive the optimal structure.
        :param a:
        :param alpha:
        :param d:
        :return:
        """
        if a == 0:
            return NoOffsetRotationalJoint(alpha=alpha, d=d)
        elif alpha == 0:
            return ParallelRotationalJoint(a=a, d=d)
        elif cos(alpha) == 0:
            return PerpendicularRotationalJoint(a=a, alpha=alpha, d=d)
        else:
            return GeneralRotationalJoint(a=a, alpha=alpha, d=d)
