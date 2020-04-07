from math import atan2, pi
from typing import Union

from src import ApplicationExceptions
from src.ApplicationExceptions import UnknownPlaneError, IllegalAngleError
from src.Coordinate import Coordinate
from src.MelfaCoordinateService import Plane

# Global coordinate system (positive axis direction)
RIGHTHAND_AXES = "XYZ"
_RIGHTHAND_CS = [
    Coordinate([1, 0, 0], RIGHTHAND_AXES),
    Coordinate([0, 1, 0], RIGHTHAND_AXES),
    Coordinate([0, 0, 1], RIGHTHAND_AXES)
]


def get_circle_cs(veca: Coordinate, vecb: Coordinate, plane: Plane, normal_vec: Union[Coordinate, None] = None):
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
            raise ApplicationExceptions.MelfaBaseException(
                "Normal vector must be supplied if vectors are collinear."
            )
        if normal_vec.dot(veca) != 0 or normal_vec.dot(vecb) != 0:
            raise ApplicationExceptions.MelfaBaseException("Normal vector supplied is not normal to circle.")
        z_axis = normal_vec
    else:
        raise UnknownPlaneError

    # Normal vector of any circle can be tilted in two directions but does not need to be twisted
    x_axis = veca
    x_axis /= x_axis.vector_len()

    # Get corresponding y-axis for right-hand system
    y_axis = z_axis.cross(x_axis)
    y_axis /= y_axis.vector_len()

    return x_axis, y_axis, z_axis


def project_vector(vec: Coordinate, *axes: Coordinate):
    return [axis.dot(vec) for axis in axes]


def get_angle(
        start: Coordinate,
        target: Coordinate,
        center: Coordinate,
        plane: Plane,
        normal_vec: Union[Coordinate, None] = None,
) -> float:
    """
    Calculates the angle between three points in R^3
    :param start:   Starting point for arc
    :param target:  End point for arc
    :param center:  Center point for arc
    :param plane:   Plane for the arc, if it is standard (XY, YZ, XZ)
    :param normal_vec:  Normal vector for the plane of the arc
    :return:    Angle between the three points in rad, [-2 pi, +2 pi]
    """

    # Remove any robot specific coordinates
    if normal_vec is not None:
        normal_vec = normal_vec.reduce_to_axes("XYZ")
    start = start.reduce_to_axes("XYZ")
    target = target.reduce_to_axes("XYZ")
    center = center.reduce_to_axes("XYZ")

    # Get the direction vectors
    cs = start - center
    ct = target - center

    # Get the new coordinate system
    x_axis_c, y_axis_c, _ = get_circle_cs(cs, ct, plane, normal_vec)

    # Projections onto a and y axis
    x_b, y_b = project_vector(ct, x_axis_c, y_axis_c)

    # Get the angle
    return atan2(y_b, x_b)


def get_intermediate_point(
        angle: float,
        start: Coordinate,
        target: Coordinate,
        center: Coordinate,
        plane: Plane,
        normal_vec: Union[Coordinate, None] = None,
) -> Coordinate:
    """
    Calculates intermediate point on a given arc
    :param angle: Total angle described by the arc
    :param start: Starting point
    :param target: Target point
    :param center: Center point
    :param plane: Preferred plane as fallback
    :param normal_vec: Normal vector for unusual plane, only considered if plane is passed as plane.ANY
    :return: coordinates of an intermediate point (half the angle)
    """
    if abs(angle) > 2 * pi:
        raise IllegalAngleError(
            "Angles with absolute value greater that 2 pi are not allowed."
        )

    start = start.reduce_to_axes("XYZ")
    target = target.reduce_to_axes("XYZ")
    center = center.reduce_to_axes("XYZ")

    if abs((center - start).vector_len() - (center - target).vector_len()) > 0.0001:
        raise ValueError("Start and end point are not equidistant from center.")

    # Point in the middle of start and target on a direct line
    middle = 0.5 * (target - start) + start

    # Full circle (just take opposite point)
    if abs(angle) == 2 * pi:
        intermediate = 2 * center - middle
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
        normal_r = (target - center).cross(normal_vec)
        normal_r /= normal_r.vector_len()

        # Orientation
        if angle < 0:
            normal_r *= -1

        # intermediate = center + ci
        intermediate = center + normal_r * (center - start).vector_len()
    else:
        cm = middle - center
        cm_rescaled = cm / cm.vector_len() * (center - start).vector_len()
        if pi > abs(angle) > 0:
            intermediate = center + cm_rescaled
        else:
            intermediate = center - cm_rescaled
    return intermediate
