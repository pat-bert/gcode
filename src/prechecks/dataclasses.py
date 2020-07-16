from typing import NamedTuple, List


class Constraints(NamedTuple):
    """
    pos_cartesian: List of user-defined cartesian workspace limitations [-x, +x, -y, +y, -z, +z]
    pos_joint: List of joint position limitations [min J1, max J1, .., min Jn, max Jn]
    vel_joint: List of joint velocity limitations [max v_J1, max v_J2, .., max v_Jn]
    """
    pos_cartesian: List[float]
    pos_joint: List[float]
    vel_joint: List[float]


class Increments(NamedTuple):
    ds: float
    dphi: float
