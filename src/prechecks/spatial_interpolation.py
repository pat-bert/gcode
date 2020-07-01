from math import floor, atan2, cos, sin, pi
from typing import List, Iterator

import numpy as np
from numpy import ndarray

from src.MelfaCoordinateService import Plane
from src.circle_util import get_circle_cs
from src.kinematics.forward_kinematics import get_tform


def linear_interpolation(start: ndarray, end: ndarray, *, ds: float) -> Iterator[ndarray]:
    """
    Calculate equidistant interpolated waypoints on a straight line.
    :param start: Start position (x,y,z,phi,theta,psi) given as 4x4 homogeneous matrix
    :param end: End position (x,y,z,phi,theta,psi) given as 4x4 homogeneous matrix
    :param ds: Constant distance between points in mm
    :return: Generator with all pose points (x,y,z,phi,theta,psi) on a straight line including start and end.
    """
    # Get the cartesian distance
    direction_vec = end[0:3, 3] - start[0:3, 3]
    total_way_len = np.linalg.norm(direction_vec)

    if total_way_len <= ds:
        # No interpolation is necessary, just return the start and end points.
        yield start
        yield end
    else:
        # Include the start point
        yield start

        # Direction vector along line
        total_increments = floor(total_way_len / ds)
        increment_vec = direction_vec / total_increments

        # Initialize the matrix components
        init_tform = np.array(start)
        xdir, ydir, zdir, pos = init_tform[0:3, 0], init_tform[0:3, 1], init_tform[0:3, 2], init_tform[0:3, 3]

        # Linear interpolation
        current_increment = 0
        while current_increment < total_increments - 1:
            pos += increment_vec
            current_tform = get_tform(xdir, ydir, zdir, pos)
            current_increment += 1
            yield current_tform

        # Finish with the end point
        yield end


def circular_interpolation(start: ndarray, target: ndarray, center: List[float], normal_vec: List[float],
                           is_clockwise: bool, *, ds: float) -> Iterator[ndarray]:
    """
    Calculate equidistant interpolated waypoints on a circle segment.
    :param start: Start pose (x,y,z,phi,theta,psi) given as 4x4 homogeneous matrix
    :param target: End pose (x,y,z,phi,theta,psi) given as 4x4 homogeneous matrix
    :param center: End position (x,y,z,phi,theta,psi)
    :param normal_vec: Normal vector given as list of three elements
    :param is_clockwise: Flag to indicate the direction around the given normal vector (right-hand-rule)
    :param ds: Constant distance between points in mm
    :return: Generator with all pose points (x,y,z,phi,theta,psi) on a straight line including start and end.
    """
    # Convert the list to an array
    center = np.array(center)
    nvec = np.array(normal_vec)
    start_pos = start[0:3, 3]
    target_pos = target[0:3, 3]

    # Get the direction vectors
    cs = start_pos - center
    ct = target_pos - center

    # Get the new coordinate system
    x_axis_c, y_axis_c, _ = get_circle_cs(cs, ct, Plane.ANY, nvec)
    x_b = x_axis_c.dot(ct)
    y_b = y_axis_c.dot(ct)

    # Get the radius, angle and arc length
    r = np.linalg.norm(target_pos - center)
    phi_total = atan2(y_b, x_b)

    # Use the conversion to get the correct angle
    if not is_clockwise:
        # Angle needs to be positive
        if phi_total <= 0:
            phi_total = phi_total + 2 * pi
    else:
        # Angle needs to be negative
        if phi_total >= 0:
            phi_total = phi_total - 2 * pi

    total_way_len = r * abs(phi_total)

    if total_way_len <= ds:
        yield start
        yield target
    else:
        yield start

        # Initialize the matrix components
        init_tform = np.array(start)
        xdir0, ydir0, zdir0, pos0 = init_tform[0:3, 0], init_tform[0:3, 1], init_tform[0:3, 2], init_tform[0:3, 3]

        # Calculate intermediate points for circular interpolation (orientation maintained)
        i_total = floor(total_way_len / ds)

        # Interpolate over the angle
        for i in range(1, i_total):
            phi_current = i / i_total * phi_total
            increment_vec = r * (cos(phi_current) * x_axis_c + sin(phi_current) * y_axis_c)
            current_tform = get_tform(xdir0, ydir0, zdir0, center + increment_vec)
            yield current_tform

        yield target
