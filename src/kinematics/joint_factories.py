from math import cos, sin
from typing import Optional

from src.kinematics.joints import BaseJoint, TranslationalJoint, BaseRotationalJoint, NoOffsetRotationalJoint, \
    ParallelRotationalJoint, PerpendicularRotationalJoint, GeneralRotationalJoint, ParallelNoOffsetRotationalJoint, \
    PerpendicularNoOffsetRotationalJoint

BASE_TOLERANCE = 1e-6


class BaseJointFactory:
    """ Build the correct joint type.
    """

    @staticmethod
    def new(*, a: float, alpha: float, d: Optional[float] = None, theta: Optional[float] = None,
            offset: float = 0) -> BaseJoint:
        """
        Creates a new joint object.
        :param a: link length
        :param alpha: twist angle
        :param d: joint distance, optional. Defaults to zero if static in joint.
        :param theta: joint angle, optional. Defaults to zero if static in joint.
        :param offset: Specifies an offset for the zero position of the joint coordinate to be applied to each joint
        coordinate given, defaults to zero.
        :return: Joint object implementing BaseJoint
        """
        if d is None:
            # d is variable -> translational joint
            if theta is None:
                raise ValueError("Cannot specify combined joint.")
            return TranslationalJoint(a=a, alpha=alpha, theta=theta, offset=offset)
        if theta is None:
            # theta is variable -> rotational joint
            return RotationalJointFactory.new(a=a, alpha=alpha, d=d, offset=offset)
        raise ValueError("Cannot specify combined joint.")


class RotationalJointFactory:
    """ Build optimized rotational joints.
    """
    tol = BASE_TOLERANCE

    @classmethod
    def new(cls, *, a: float, alpha: float, d: float, offset: float = 0) -> BaseRotationalJoint:
        """
        Uses the joint parameters to derive the optimal object for a rotational joint
        :param a: link length
        :param alpha: twist angle
        :param d: joint distance, optional. Defaults to zero if static in joint.
        :param offset: Specifies an offset for the zero position of the joint coordinate to be applied to each joint
        coordinate given, defaults to zero.
        :return: Joint object implementing BaseRotationalJoint
        """
        no_axis_offset = a == 0
        parallel = (-cls.tol <= sin(alpha) <= cls.tol)
        perpendicular = (-cls.tol <= cos(alpha) <= cls.tol)

        if no_axis_offset:
            if parallel:
                return ParallelNoOffsetRotationalJoint(alpha=alpha, d=d, offset=offset)
            if perpendicular:
                return PerpendicularNoOffsetRotationalJoint(alpha=alpha, d=d, offset=offset)
            return NoOffsetRotationalJoint(alpha=alpha, d=d, offset=offset)
        if parallel:
            return ParallelRotationalJoint(alpha=alpha, a=a, d=d, offset=offset)
        if perpendicular:
            return PerpendicularRotationalJoint(a=a, alpha=alpha, d=d, offset=offset)
        return GeneralRotationalJoint(a=a, alpha=alpha, d=d, offset=offset)
