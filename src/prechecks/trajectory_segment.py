import abc
from typing import List, Union, Iterator

import numpy as np

from src.kinematics.inverse_kinematics import JointSolution


def is_point_within_boundaries(point: Union[np.ndarray, List], boundaries: List) -> bool:
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


class CartesianTrajectorySegment(metaclass=abc.ABCMeta):
    """
    Base class for parts of a trajectory within the cartesian task space
    """

    def __init__(self, trajectory_points: Iterator[np.ndarray]):
        self.trajectory_points = list(trajectory_points)

        if len(self.trajectory_points) == 0:
            raise ValueError('Trajectory may not be empty.')

    @abc.abstractmethod
    def is_within_cartesian_boundaries(self, boundaries: List) -> bool:
        pass


class LinearSegment(CartesianTrajectorySegment):
    """
    Represents a straight line segment within the task space
    """

    def is_within_cartesian_boundaries(self, boundaries: List) -> bool:
        """
        A straight linear segment is within a rectangular cuboid if start and end point are within.
        :param boundaries: List of boundaries with twice the length of the coordinates
        :return: Boolean to indicate whether the point is within
        """
        # Convert generator to list to access last element
        start_inside = is_point_within_boundaries(self.trajectory_points[0], boundaries)
        end_inside = is_point_within_boundaries(self.trajectory_points[-1], boundaries)
        return start_inside and (len(self.trajectory_points) == 1 or end_inside)


class CircularSegment(CartesianTrajectorySegment):
    """
    Represents a circular sector within the task space
    """

    def is_within_cartesian_boundaries(self, boundaries: List) -> bool:
        """
        A circular segment/sector is within a rectangular cuboid if all points are within.
        :param boundaries: List of boundaries with twice the length of the coordinates
        :return: Boolean to indicate whether the point is within
        """
        return all(is_point_within_boundaries(point, boundaries) for point in self.trajectory_points)


class JointTrajectorySegment:
    """
    Represents a list of trajectory point solutions given in the joint space
    """

    def __init__(self, solutions: List[JointSolution]):
        self.solutions = solutions

    def is_within_joint_limits(self, limits: List) -> bool:
        """
        Remove solutions that are outside the joint limits for all points.
        :param limits: Joint limits given as list of floats, e.g. J1 min, J1 max, ..
        :return: Boolean to indicate whether there are solutions within the joint limits for all points.
        """
        # Iterate over all points
        within = True
        for point_number, point_solutions in enumerate(self.solutions):
            # Check all available configurations for the current point
            for configuration, joints in point_solutions.items():
                # Check all joint boundaries (solutions and boundaries are naturally in manufacturer's system)
                if not is_point_within_boundaries(joints, limits):
                    # Remove the solution
                    del point_solutions[configuration]
                    if len(point_solutions) == 0:
                        # No solution left for a point on the segment
                        within = False
        return within

    def get_common_configurations(self) -> List[float]:
        """
        Check whether there is a common configuration for all points of that segment
        :return: List of common configurations, can be empty
        """
        # Init with the configurations of the first point for that solutions exist
        common_configurations = set(self.solutions[0].keys())

        # Start comparison with second point
        idx = 1
        while common_configurations and idx < len(self.solutions):
            # Only keep the configurations that are also viable for the next point
            common_configurations &= set(self.solutions[idx].keys())
        return list(common_configurations)

    def get_least_configuration_changes(self, shoulder_cost, elbow_cost, wrist_cost) -> List[JointSolution]:
        """
        Find the path of solutions with the minimum cost of configuration changes
        :param shoulder_cost: Cost that a change of the shoulder configuration incurrs
        :param elbow_cost: Cost that a change of the elbow configuration incurrs
        :param wrist_cost: Cost that a change of the wrist configuration incurrs
        :return: List of selected joint solutions with minimum cost
        """
        current_configuration = 7
        next_configuration = 6

        cost = 0

        # Calculate the cost for a change
        cost += shoulder_cost * (current_configuration & 4 == next_configuration & 4)
        cost += elbow_cost * (current_configuration & 2 == next_configuration & 2)
        cost += wrist_cost * (current_configuration & 1 == next_configuration & 1)
