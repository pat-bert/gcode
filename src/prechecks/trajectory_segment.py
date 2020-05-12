import abc
from typing import List, Generator

import numpy as np


def is_point_within_boundaries(point: np.ndarray, boundaries: List) -> bool:
    """
    A point is within a cuboid if all values are within a lower and an upper threshold.
    :param point: List of float values
    :param boundaries: List of boundaries,
    Boundary length must be equal and not more than twice as long as the point list.
    :return: Boolean to indicate whether the point is within
    :raises: ValueError if lists are not of required length
    """
    if len(boundaries) % 2 != 0 or len(boundaries) > 2 * len(point):
        raise ValueError('Boundary length must be equal and not more than twice as long as the point list.')
    return all(lower <= i <= upper for i, lower, upper in zip(point, boundaries[::2], boundaries[1::2]))


class TrajectorySegment(metaclass=abc.ABCMeta):
    def __init__(self, trajectory_points: Generator[np.ndarray]):
        self.trajectory_points = list(trajectory_points)

    @abc.abstractmethod
    def is_within_cartesian_boundaries(self, boundaries: List) -> bool:
        pass


class LinearSegment(TrajectorySegment):
    def is_within_cartesian_boundaries(self, boundaries: List) -> bool:
        """
        A straight linear segment is within a rectangular cuboid if start and end point are within.
        :param boundaries: List of boundaries with twice the length of the coordinates
        :return: Boolean to indicate whether the point is within
        """
        # Convert generator to list to access last element
        start_inside = is_point_within_boundaries(self.trajectory_points[0], boundaries)
        end_inside = is_point_within_boundaries(self.trajectory_points[-1], boundaries)
        return start_inside and end_inside


class CircularSegment(TrajectorySegment):
    def is_within_cartesian_boundaries(self, boundaries: List) -> bool:
        """
        A circular segment/sector is within a rectangular cuboid if all points are within.
        :param boundaries: List of boundaries with twice the length of the coordinates
        :return: Boolean to indicate whether the point is within
        """
        return all(is_point_within_boundaries(point, boundaries) for point in self.trajectory_points)
