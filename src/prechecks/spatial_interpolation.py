from math import floor
from typing import List, Iterator

import numpy as np
from numpy import ndarray

from Coordinate import Coordinate
from MelfaCoordinateService import Plane
from circle_util import get_angle
from src.kinematics.forward_kinematics import get_tform


def linear_interpolation(start: ndarray, end: ndarray, *, ds: float) -> Iterator[ndarray]:
    """
    Calculate equidistant interpolated waypoints on a straight line.
    :param start: Start pose (x,y,z,phi,theta,psi) given as 4x4 homogeneous matrix
    :param end: End pose (x,y,z,phi,theta,psi) given as 4x4 homogeneous matrix
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

        # TODO Orientation interpolation (SLERP)
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


def circular_interpolation(start: ndarray, end: ndarray, centre: List[float], *, ds: float) -> Iterator[ndarray]:
    """
    Calculate equidistant interpolated waypoints on a circle segment.
    :param start: Start pose (x,y,z,phi,theta,psi) given as 4x4 homogeneous matrix
    :param end: End pose (x,y,z,phi,theta,psi) given as 4x4 homogeneous matrix
    :param centre: End position (x,y,z,phi,theta,psi)
    :param ds: Constant distance between points in mm
    :return: Generator with all pose points (x,y,z,phi,theta,psi) on a straight line including start and end.
    """
    # Convert the arrays to coordinates
    start_c = Coordinate(start[0:3, 3], 'XYZ')
    end_c = Coordinate(end[0:3, 3], 'XYZ')
    centre_c = start = Coordinate(centre[0:3, 3], 'XYZ')

    # Get the radius and angle
    radius = np.linalg.norm(end[0:3, 3] - centre[0:3, 3])

    # TODO Use correct plane or normal vector
    plane = Plane.XY
    centri_angle = get_angle(start_c, end_c, centre_c, plane=plane)

    # Cartesian travel distance is length of arc
    total_way_len = radius * centri_angle

    if total_way_len <= ds:
        yield start
        yield end
    else:
        yield start

        # TODO Calculate intermediate points
        total_increments = floor(total_way_len / ds)

        yield end
