from math import floor
from typing import List, Iterator

import numpy as np
from numpy import ndarray

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

        # TODO Orientation interpolation (based on quaternions or euler?)
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
