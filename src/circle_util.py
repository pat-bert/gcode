from math import atan2, pi
from typing import *

import numpy

from src import ApplicationExceptions
from src.ApplicationExceptions import UnknownPlaneError, IllegalAngleError
from src.Coordinate import Coordinate
from src.MelfaCoordinateService import Plane

# Global coordinate system

_RIGHTHAND_CS = [numpy.array([1, 0, 0]), numpy.array([0, 1, 0]), numpy.array([0, 0, 1])]
_RIGHTHAND_AXES = "XYZ"


def get_circle_cs(veca, vecb, plane: Plane, normal_vec=None):
    """
    Calculates the coordinate system fitting into a given circle. All axes may be tilted so that the z-axis is
    perpendicular to the plane of the circle. However, the system will not be twisted around the z-axis.
    :param veca:
    :param vecb:
    :param plane:
    :param normal_vec:
    :return:
    """
    # Z-axis
    if plane is Plane.XY:
        z_axis = _RIGHTHAND_CS[2]
    elif plane is Plane.XZ:
        z_axis = _RIGHTHAND_CS[1]
    elif plane is Plane.YZ:
        z_axis = _RIGHTHAND_CS[0]
    elif plane is Plane.ANY:
        # Get z-axis perpendicular to plane of circle (can be any)
        z_axis = numpy.cross(veca, vecb)
        z_axis[2] = abs(z_axis[2])

        z_len = numpy.sqrt(numpy.sum(z_axis ** 2))

        # Normalise z-axis, fallback to preferred plane if angle is 180°
        if z_len != 0:
            z_axis = z_axis / z_len
        else:
            try:
                assert numpy.dot(normal_vec, veca) == 0 and numpy.dot(normal_vec, vecb)
            except TypeError:
                raise ApplicationExceptions.MelfaBaseException(
                    "Normal vector must be supplied if vectors are collinear."
                )
            except AssertionError:
                raise ApplicationExceptions.MelfaBaseException(
                    "Normal vector supplied is not normal to circle."
                )
            else:
                z_axis = normal_vec
    else:
        raise UnknownPlaneError

    # Normal vector of any circle can be tilted in two directions but does not need to be twisted
    # TODO Consider z axis parallel y axis
    y_axis_r = numpy.array([0, 1, 0])
    x_axis = numpy.cross(y_axis_r, z_axis)
    # Get corresponding y-axis for right-hand system
    y_axis = numpy.cross(z_axis, x_axis)
    y_axis = y_axis / numpy.sqrt(numpy.sum(y_axis ** 2))

    return x_axis, y_axis, z_axis


def get_np_vectors(
        start: Coordinate, target: Coordinate, center: Coordinate, normal_vec: Coordinate
):
    # Get vectors from center
    cs = start - center
    ct = target - center

    # Convert to numpy arrays
    veca = numpy.array(list(cs.coordinate.values()))
    vecb = numpy.array(list(ct.coordinate.values()))

    if normal_vec is not None:
        normal_vec = numpy.array(normal_vec.coordinate.values())

    return normal_vec, veca, vecb


def project_vector(vec, *axes):
    proj = []
    for axis in axes:
        proj.append(numpy.dot(axis, vec))
    return proj


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

    normal_vec, veca, vecb = get_np_vectors(start, target, center, normal_vec)

    # Get the new coordinate system
    x_axis_c, y_axis_c, _ = get_circle_cs(veca, vecb, plane, normal_vec)

    # Projections onto a and y axis
    x_b, y_b = project_vector(vecb, x_axis_c, y_axis_c)
    x_a, y_a = project_vector(veca, x_axis_c, y_axis_c)

    # Get the angle
    return atan2(y_b, x_b) - atan2(y_a, x_a)


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
    :param normal_vec: Normal vector for unusual plane
    :return: coordinates of an intermediate point (half the angle)
    """
    if abs(angle) > 2 * pi:
        raise IllegalAngleError(
            "Angles with absolute value greater that 2 pi are not allowed."
        )

    start = start.reduce_to_axes('XYZ')
    target = target.reduce_to_axes('XYZ')
    center = center.reduce_to_axes('XYZ')

    if abs((center - start).vector_len() - (center - target).vector_len()) > 0.0001:
        raise ValueError("Start and end point are not equidistant from center.")

    # Point in the middle of start and target on a direct line
    middle = 0.5 * (target - start) + start
    cm = middle - center

    # Full circle (just take opposite point)
    if abs(angle) == 2 * pi:
        intermediate = center - cm
    # Half circle
    elif abs(angle) == pi:
        if plane is Plane.XY:
            normal_vec = _RIGHTHAND_CS[2]
            # Convert to coordinate
            normal_vec = Coordinate(normal_vec, _RIGHTHAND_AXES)
        elif plane is Plane.XZ:
            normal_vec = _RIGHTHAND_CS[1]
            # Convert to coordinate
            normal_vec = Coordinate(normal_vec, _RIGHTHAND_AXES)
        elif plane is Plane.YZ:
            normal_vec = _RIGHTHAND_CS[0]
            # Convert to coordinate
            normal_vec = Coordinate(normal_vec, _RIGHTHAND_AXES)
        elif plane is Plane.ANY:
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
        cm_rescaled = cm / (cm.vector_len()) * (center - start).vector_len()
        if pi > abs(angle) > 0:
            intermediate = center + cm_rescaled
        else:
            intermediate = center - cm_rescaled
    return intermediate
