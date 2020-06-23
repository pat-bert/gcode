import abc
from typing import List, Union, Iterator

import numpy as np

from src.kinematics.inverse_kinematics import JointSolution
from src.prechecks.speed_profile import trapezoidal_speed_profile


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

    def __init__(self, trajectory_points: Iterator[np.ndarray], velocity=None, acceleration=None, ds=None):
        """
        Create a cartesian trajectory segment. Time points are calculated according to a trapezoidal speed profile.
        :param trajectory_points:
        :param velocity: Velocity in mm/min
        :param acceleration: Acceleration in mm/s^2
        :param ds: Way delta for discretizing the segment in mm
        """
        self.trajectory_points = list(trajectory_points)
        s_total = self.trajectory_points[-1] - self.trajectory_points[0]

        if s_total.shape == (4, 4,):
            # Homogeneous transformation matrix
            self.s_total = np.linalg.norm(s_total[0:3, 3])
        else:
            # Assume single element
            self.s_total = np.linalg.norm(s_total)

        self.ds = ds
        if velocity is not None and acceleration is not None and ds is not None:
            self.time_points = trapezoidal_speed_profile(velocity / 60, acceleration, self.s_total, self.ds)
        else:
            self.time_points = None

        if len(self.trajectory_points) == 0:
            raise ValueError('Trajectory may not be empty.')

    @abc.abstractmethod
    def is_within_cartesian_boundaries(self, boundaries: List) -> bool:
        pass

    @property
    def target(self):
        return self.trajectory_points[-1]


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
        # Distinguish element types
        if self.trajectory_points[0].shape == (4, 4,):
            # Homogeneous transformation matrix
            start = self.trajectory_points[0][0:3, 3]
            end = self.trajectory_points[-1][0:3, 3]
        else:
            # Assume single element
            start = self.trajectory_points[0]
            end = self.trajectory_points[-1]

        start_inside = is_point_within_boundaries(start, boundaries)
        end_inside = is_point_within_boundaries(end, boundaries)
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
        if self.trajectory_points[0].shape == (4, 4,):
            return all(is_point_within_boundaries(point[0:3, 3], boundaries) for point in self.trajectory_points)
        else:
            return all(is_point_within_boundaries(point, boundaries) for point in self.trajectory_points)


class JointTrajectorySegment:
    """
    Represents a list of trajectory point solutions given in the joint space
    """
    IDX = 0

    def __init__(self, solutions: List[JointSolution], time_points):
        self.solutions = solutions
        self.time_points = time_points
        self.idx = self.IDX
        JointTrajectorySegment.IDX = JointTrajectorySegment.IDX + 1

    def is_within_joint_limits(self, limits: List) -> bool:
        """
        Remove solutions that are outside the joint limits for all points.
        :param limits: Joint limits given as list of floats, e.g. J1 min, J1 max, ..
        :return: Boolean to indicate whether there are solutions within the joint limits for all points.
        """
        # Iterate over all points
        within = True
        for point_number, point_solutions in enumerate(self.solutions):
            # Dictionary to be populated with remaining solutions
            remaining_solutions = {}

            # Check all available configurations for the current point
            for configuration, joints in point_solutions.items():
                # Check all joint boundaries (solutions and boundaries are naturally in manufacturer's system)
                if is_point_within_boundaries(joints, limits):
                    # Remove the solution
                    remaining_solutions[configuration] = joints

            # Update solutions for point
            self.solutions[point_number] = remaining_solutions

            # Update return value
            if len(remaining_solutions) == 0:
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
        for current_point_solutions in self.solutions:
            if not common_configurations:
                break
            # Only keep the configurations that are also viable for the next point
            common_configurations &= set(current_point_solutions.keys())
        return list(common_configurations)