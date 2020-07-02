from math import atan2, pi
from typing import Optional

import numpy as np

from src import ApplicationExceptions
from src.ApplicationExceptions import UnknownPlaneError, IllegalAngleError
from src.MelfaCoordinateService import Plane

# Global coordinate system (positive axis direction)
RIGHTHAND_AXES = "XYZ"
_RIGHTHAND_CS = [
    np.array([1, 0, 0]),
    np.array([0, 1, 0]),
    np.array([0, 0, 1])
]


def get_circle_cs(veca: np.ndarray, vecb: np.ndarray, plane: Plane, normal_vec: Optional[np.ndarray] = None):
    """
    Calculates the coordinate system fitting into a given circle. All axes may be tilted so that the z-axis is
    perpendicular to the plane of the circle. However, the system will not be twisted around the z-axis.
    :param veca:
    :param vecb:
    :param plane:
    :param normal_vec:
    :return:
    """
    # Get the z-axis depending on the plane
    if plane is Plane.XY:
        z_axis = _RIGHTHAND_CS[2]
    elif plane is Plane.XZ:
        z_axis = _RIGHTHAND_CS[1]
    elif plane is Plane.YZ:
        z_axis = _RIGHTHAND_CS[0]
    elif plane is Plane.ANY:
        # Angle sign depends on direction of normal vector so it cannot be detected from the points alone
        if normal_vec is None:
            raise ApplicationExceptions.MelfaBaseException("Normal vector must be supplied if vectors are collinear.")
        if normal_vec.dot(veca) != 0 or normal_vec.dot(vecb) != 0:
            raise ApplicationExceptions.MelfaBaseException("Normal vector supplied is not normal to circle.")
        z_axis = normal_vec
    else:
        raise UnknownPlaneError

    # Normal vector of any circle can be tilted in two directions but does not need to be twisted
    x_axis = veca
    x_axis = np.divide(x_axis, np.linalg.norm(x_axis))

    # Get corresponding y-axis for right-hand system
    y_axis = np.cross(z_axis, x_axis)
    y_axis = np.divide(y_axis, np.linalg.norm(y_axis))

    return x_axis, y_axis, z_axis


def get_angle(start_pos: np.ndarray, target_pos: np.ndarray, center_pos: np.ndarray, plane: Plane,
              normal_vec: Optional[np.ndarray] = None) -> float:
    """
    Calculates the angle between three points in R^3
    :param start_pos: Starting point for arc
    :param target_pos: End point for arc
    :param center_pos: Center point for arc
    :param plane: Plane for the arc, if it is standard (XY, YZ, XZ)
    :param normal_vec:  Normal vector for the plane of the arc
    :return: Angle between the three points in rad, [-2 pi, +2 pi]
    """
    # Get the direction vectors
    cs = start_pos - center_pos
    ct = target_pos - center_pos

    # Get the new coordinate system
    x_axis_c, y_axis_c, _ = get_circle_cs(cs, ct, plane, normal_vec)

    # Projections onto a and y axis
    x_b = x_axis_c.dot(ct)
    y_b = y_axis_c.dot(ct)

    # Get the angle
    return atan2(y_b, x_b)


def get_intermediate_point(angle: float, start_pos: np.ndarray, target_pos: np.ndarray, center_pos: np.ndarray,
                           plane: Plane, normal_vec: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Calculates intermediate point on a given arc
    :param angle: Total angle described by the arc
    :param start_pos: Starting point (x,y,z)
    :param target_pos: Target point (x,y,z)
    :param center_pos: Center point (x,y,z)
    :param plane: Preferred plane as fallback
    :param normal_vec: Normal vector for unusual plane, only considered if plane is passed as plane.ANY
    :return: coordinates of an intermediate point (half the angle)
    """
    if abs(angle) > 2 * pi:
        raise IllegalAngleError("Angles with absolute value greater that 2 pi are not allowed.")

    if abs(np.linalg.norm(center_pos - start_pos) - np.linalg.norm(center_pos - target_pos)) > 0.0001:
        raise ValueError("Start and end point are not equidistant from center.")

    # Point in the middle of start and target on a direct line
    middle = 0.5 * (target_pos - start_pos) + start_pos

    # Full circle (just take opposite point)
    if abs(angle) == 2 * pi:
        intermediate = 2 * center_pos - middle
    # Half circle
    elif abs(angle) == pi:
        if plane is Plane.XY:
            normal_vec = _RIGHTHAND_CS[2]
        elif plane is Plane.XZ:
            normal_vec = _RIGHTHAND_CS[1]
        elif plane is Plane.YZ:
            normal_vec = _RIGHTHAND_CS[0]
        elif plane is Plane.ANY:
            # Use the supplied normal vector
            pass
        else:
            raise ApplicationExceptions.MelfaBaseException("Unknown plane supplied.")

        # Calculate and resize radial normal vector
        normal_r = np.cross(target_pos - center_pos, normal_vec)
        normal_r = np.divide(normal_r, np.linalg.norm(normal_r))

        # Orientation
        if angle < 0:
            normal_r *= -1

        # intermediate = center + ci
        intermediate = center_pos + normal_r * np.linalg.norm(center_pos - start_pos)
    else:
        cm = middle - center_pos
        cm_rescaled = np.divide(cm, np.linalg.norm(cm)) * np.linalg.norm(center_pos - start_pos)
        if pi > abs(angle) > 0:
            intermediate = center_pos + cm_rescaled
        else:
            intermediate = center_pos - cm_rescaled
    return intermediate
