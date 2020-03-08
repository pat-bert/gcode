from math import cos, sin

from kinematics.joints import BaseJoint, TranslationalJoint, BaseRotationalJoint, NoOffsetRotationalJoint, \
    ParallelRotationalJoint, PerpendicularRotationalJoint, GeneralRotationalJoint, ParallelNoOffsetRotationalJoint, \
    PerpendicularNoOffsetRotationalJoint

BASE_TOLERANCE = 1e-6


class BaseJointFactory:
    """
    Build the correct joint type.
    """

    @staticmethod
    def new(*, a, alpha, d=None, theta=None) -> BaseJoint:
        """
        Creates a new joint object.
        :param a: link length
        :param alpha: twist angle
        :param d: joint distance, optional. Defaults to zero if static in joint.
        :param theta: joint angle, optional. Defaults to zero if static in joint.
        :return:
        """
        if d is None:
            # d is variable -> translational joint
            if theta is None:
                raise ValueError("Cannot specify combined joint.")
            return TranslationalJoint(a=a, alpha=alpha, theta=theta)
        elif theta is None:
            # theta is variable -> rotational joint
            return RotationalJointFactory.new(a=a, alpha=alpha, d=d)
        else:
            raise ValueError("Cannot specify combined joint.")


class RotationalJointFactory:
    """
    Build optimized rotational joints.
    """
    tol = BASE_TOLERANCE

    @classmethod
    def new(cls, *, a, alpha, d) -> BaseRotationalJoint:
        """
        Uses the joint parameters to derive the optimal structure.
        :param a:
        :param alpha:
        :param d:
        :return:
        """
        no_offset = a == 0
        parallel = (-cls.tol <= sin(alpha) <= cls.tol)
        perpendicular = (-cls.tol <= cos(alpha) <= cls.tol)

        if no_offset:
            if parallel:
                return ParallelNoOffsetRotationalJoint(alpha=alpha, d=d)
            elif perpendicular:
                return PerpendicularNoOffsetRotationalJoint(alpha=alpha, d=d)
            else:
                return NoOffsetRotationalJoint(alpha=alpha, d=d)
        elif parallel:
            return ParallelRotationalJoint(alpha=alpha, a=a, d=d)
        elif perpendicular:
            return PerpendicularRotationalJoint(a=a, alpha=alpha, d=d)
        else:
            return GeneralRotationalJoint(a=a, alpha=alpha, d=d)
