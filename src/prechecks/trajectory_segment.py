import abc
from collections import defaultdict
from typing import List, Union, Iterator, Set, Dict, Optional

import numpy as np

from src.prechecks.exceptions import ConfigurationChangesError, JointVelocityViolation
from src.kinematics.inverse_kinematics import JointSolution
from src.prechecks.exceptions import CartesianLimitViolation, JointLimitViolation, JOINT_SPEED_ALLOWABLE_RATIO
from src.prechecks.speed_profile import trapezoidal_speed_profile


def get_violated_boundaries(point: Union[np.ndarray, List], boundaries: List) -> Set[int]:
    """
    A point is within a cuboid if all values are within a lower and an upper threshold.
    :param point: List of float values
    :param boundaries: List of boundaries,
    Boundary length must be equal and not more than twice as long as the point list.
    :return: List of indices of the violated boundaries
    :raises: ValueError if lists are not of required length
    """
    if len(boundaries) % 2 != 0 or len(boundaries) > 2 * len(point):
        raise ValueError('Boundary length must be equal and not more than twice as long as the point list.')

    violation_indices = set()

    for (idx, val), lower, upper in zip(enumerate(point), boundaries[::2], boundaries[1::2]):
        if val < lower:
            violation_indices.add(2 * idx)
        if val > upper:
            violation_indices.add(2 * idx + 1)
    return violation_indices


class CartesianTrajSegment(metaclass=abc.ABCMeta):
    """
    Base class for parts of a trajectory within the cartesian task space
    """

    def __init__(self, trajectory_points: Iterator[np.ndarray], extrusion: Optional[bool] = False,
                 vel: Optional[float] = None, acc: Optional[float] = None, ds: Optional[float] = None):
        """
        Create a cartesian trajectory segment. Time points are calculated according to a trapezoidal speed profile.
        :param trajectory_points:
        :param vel: Velocity in mm/min
        :param acc: Acceleration in mm/s^2
        :param ds: Way delta for discretizing the segment in mm
        """
        self.trajectory_points = {0: list(trajectory_points)}
        self.has_extrusion = extrusion

        if len(self.trajectory_points[0]) == 0:
            raise ValueError('Trajectory may not be empty.')

        if ds is not None:
            self.s_total = ds * (len(self.trajectory_points[0]) - 1)
        else:
            self.s_total = None
        self.ds = ds
        if vel is not None and acc is not None and ds is not None and self.s_total > 0.0:
            self.time_points = trapezoidal_speed_profile(vel / 60, acc, self.s_total, self.ds)
        else:
            self.time_points = None

    @abc.abstractmethod
    def get_violated_boundaries(self, boundaries: List[float]) -> Set[int]:
        """
        Check whether the segment satisfies the cartesian boundaries
        :param boundaries: List of boundaries given as float values
        :return: Tuple of isValid flag and list of indices of violated boundaries
        """

    @property
    def target(self):
        return self.trajectory_points[0][-1]

    @property
    def unmodified_points(self):
        return self.trajectory_points[0]


class LinearSegment(CartesianTrajSegment):
    """
    Represents a straight line segment within the task space
    """

    def get_violated_boundaries(self, boundaries: List[float]) -> Set[int]:
        """
        A straight linear segment is within a rectangular cuboid if start and end point are within.
        :param boundaries: List of boundaries with twice the length of the coordinates
        :return: Boolean to indicate whether the point is within
        """
        # Distinguish element types
        if self.unmodified_points[0].shape == (4, 4,):
            # Homogeneous transformation matrix
            start = self.unmodified_points[0][0:3, 3]
            end = self.unmodified_points[-1][0:3, 3]
        else:
            # Assume single element
            start = self.unmodified_points[0]
            end = self.unmodified_points[-1]

        start_violations = get_violated_boundaries(start, boundaries)
        end_violations = get_violated_boundaries(end, boundaries)

        if len(self.unmodified_points) > 1:
            return start_violations | end_violations
        return start_violations


class CircularSegment(CartesianTrajSegment):
    """
    Represents a circular sector within the task space
    """

    def __init__(self, trajectory_points: Iterator[np.ndarray], centre: np.ndarray, extrusion: Optional[bool] = False,
                 vel: Optional[float] = None, acc: Optional[float] = None, ds: Optional[float] = None):
        super().__init__(trajectory_points, extrusion, vel, acc, ds)
        self.centre = centre

    def get_violated_boundaries(self, boundaries: List[float]) -> Set[int]:
        """
        A circular segment/sector is within a rectangular cuboid if all points are within.
        :param boundaries: List of boundaries with twice the length of the coordinates
        :return: Boolean to indicate whether the point is within
        """
        if self.unmodified_points[0].shape == (4, 4,):
            all_violations = [get_violated_boundaries(point[0:3, 3], boundaries) for point in self.unmodified_points]
        else:
            all_violations = [get_violated_boundaries(point, boundaries) for point in self.unmodified_points]
        return {element for idx_list in all_violations for element in idx_list}


class JointTrajSegment:
    """
    Represents a list of trajectory point solutions given in the joint space
    """
    IDX = 0

    def __init__(self, solutions: List[JointSolution], time_points=None):
        self.solutions = solutions
        self.time_points = time_points
        self.idx = self.IDX
        JointTrajSegment.IDX = JointTrajSegment.IDX + 1

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
                if not get_violated_boundaries(joints, limits):
                    # Remove the solution
                    remaining_solutions[configuration] = joints

            # Update solutions for point
            self.solutions[point_number] = remaining_solutions

            # Update return value but keep removing values
            if len(remaining_solutions) == 0:
                # No solution left for a point on the segment
                within = False
        return within

    def joints_exceeding_velocity_limits(self, config_path: List[int], qdlim: List[float]) -> Dict[int, float]:
        """
        Check the joint velocities of the segment for given robot configurations using the internal joint solutions and
        time points.
        :param config_path: List of robot configurations, one for each point on the segment
        :param qdlim: List of joint velocity limits
        :return: Set of indices for the joints that exceed the threshold on this segment
        :raises: TypeError if the time points are None
        :raises: ValueError if the number of configurations is unequal the number of segment points
        """
        if self.time_points is None:
            raise TypeError('Time points need to be specified.')

        if len(config_path) != len(self.solutions):
            raise ValueError('Number of configs must be equal to number of points on the segment.')

        exceeding_joints = defaultdict(lambda: 0)

        # Set the values for the first point
        tprev = self.time_points[0]
        prev_joint_values = self.solutions[0][config_path[0]]

        # Iterate over the remaining points
        for ik_solutions_point, tcurr, config in zip(self.solutions[1:], self.time_points[1:], config_path[1:]):
            curr_joint_values = ik_solutions_point[config]
            # Calculate the velocity ratio for each joint with regard to the previous joint position
            for (joint_idx, jprev), jcurr, j_vel_max in zip(enumerate(prev_joint_values), curr_joint_values, qdlim):
                joint_velocity = abs((jcurr - jprev) / (tcurr - tprev))
                velocity_ratio = joint_velocity / j_vel_max
                if velocity_ratio > JOINT_SPEED_ALLOWABLE_RATIO:
                    # Update the maximum velocity of the joint
                    # Joints are counted from 1
                    exceeding_joints[joint_idx + 1] = max(joint_velocity, exceeding_joints[joint_idx + 1])

            # Update the previous values
            tprev = tcurr
            prev_joint_values = curr_joint_values

        return dict(exceeding_joints)

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


def check_cartesian_limits(task_trajectory: List[CartesianTrajSegment], clim: List[float]) -> None:
    """
    Verify that a set of waypoints is within given cartesian limits.
    :param task_trajectory:
    :param clim: List of user-defined cartesian workspace limitations [-x, +x, -y, +y, -z, +z]
    :raises: CartesianLimitViolation if any point of a segment is violating the limits
    """
    print('Validating trajectory in task space...')
    boundary_violations = [segment.get_violated_boundaries(clim) for segment in task_trajectory]
    if any(boundary_violations):
        # At least one set is not empty and contains the indices of the violated boundaries
        error_msg = '\n'
        for segment_idx, violation_indices in enumerate(boundary_violations):
            if violation_indices:
                limits = ''
                for idx in violation_indices:
                    limit_type = 'min' if (idx % 2 == 0) else 'max'
                    limit_id = 'XYZ'[idx // 2]
                    limits += f'{limit_id}{limit_type} '
                error_msg += f'Segment #{segment_idx}: {limits}violated\n'
        raise CartesianLimitViolation(f'Found segments violating the cartesian limits ({clim}):\n{error_msg}')
    print('=>All segments are located within the cartesian limits.')


def filter_joint_limits(joint_trajectory: List[JointTrajSegment], qlim: List[float]) -> None:
    """
    Eliminates solutions in joint space that violate the joint limits.
    :param joint_trajectory:
    :param qlim: List of joint position limitations [min J1, max J1, .., min Jn, max Jn]
    :raises: JointLimitViolation if any point of a segment does not have a valid solution in joint space
    """
    print('Filtering positional joint limits...')
    within_joint = [segment.is_within_joint_limits(qlim) for segment in joint_trajectory]
    if not all(within_joint):
        violation_idx = [idx for idx, val in enumerate(within_joint) if not val]
        raise JointLimitViolation('Found segments violating the joint limits.', violation_idx)
    print('=>All segments have joint solutions within the limits.')


def check_common_configurations(joint_trajectory: List[JointTrajSegment]) -> None:
    """
    Search for common configurations for each segment of the trajectory.
    :param joint_trajectory:
    :raises: ConfigurationChangesError if any segment is found that does not have solutions with a common configuration.
    """
    print('\nChecking common configurations...')
    common_configurations = [segment.get_common_configurations() for segment in joint_trajectory]
    if not all(common_configurations):
        violation_idx = [idx for idx, val in enumerate(common_configurations) if not val]
        error_msg = ''
        for idx in violation_idx:
            configs = {i for point in joint_trajectory[idx].solutions for i in point.keys()}
            error_msg += f'Segment #{idx}: Points accessible in configurations {configs}\n'
        raise ConfigurationChangesError(f'Found segments without common configurations:\n{error_msg}')
    print('=>Each segment can be executed without configuration change.')
    print(f'=>Configurations in trajectory: {set(conf for conf_list in common_configurations for conf in conf_list)}')
    print(f'=>Configurations common to all segments: {set.intersection(*map(set, common_configurations))}')


def check_joint_velocities(joint_traj: List[JointTrajSegment], config_path: List[int], qdlim: List[float]):
    """
    Check the joint velocities along a trajectory for given robot configurations per segment.
    :param joint_traj: Joint trajectory as list of coherent segments
    :param config_path: List of robot configurations per point
    :param qdlim:
    :return:
    """
    if sum(len(seg.solutions) for seg in joint_traj) != len(config_path):
        raise ValueError('Number of segments is unequal to number of configs.')

    print('\nChecking joint velocities on selected path..')
    start = 0
    exceeding_indices = []

    error_msg = f"Joint velocity ratio {100 * JOINT_SPEED_ALLOWABLE_RATIO :.2f}% exceeded " \
                f"(max velocities by segment and joint):\n"
    for seg_idx, seg in enumerate(joint_traj):
        end = start + len(seg.solutions)
        current_exceeding = seg.joints_exceeding_velocity_limits(config_path[start:end], qdlim)
        exceeding_indices.append(current_exceeding)
        start += len(seg.solutions)

        if current_exceeding:
            joint_vel_str = [f'J{j_idx}: {vmax :.3f} rad/s' for j_idx, vmax in current_exceeding.items()]
            error_msg += f"Segment #{seg_idx}: {'; '.join(sorted(joint_vel_str))}\n"

    # Check the violations
    if any(exceeding_indices):
        raise JointVelocityViolation(error_msg)
    print('=>All movements are done within the defined percentage of maximum joint velocity.')
