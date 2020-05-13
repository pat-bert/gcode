from typing import List

import numpy as np

from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import forward_kinematics, pose2tform
from src.kinematics.inverse_kinematics import ik_spherical_wrist, OutOfReachError
from src.kinematics.joints import BaseJoint
from src.prechecks.spatial_interpolation import linear_interpolation, circular_interpolation
from src.prechecks.trajectory_segment import LinearSegment, CircularSegment, CartesianTrajectorySegment, \
    JointTrajectorySegment


class TrajectoryError(ValueError):
    """
    Indicate that a trajectory check has failed.
    """


class WorkspaceViolation(TrajectoryError):
    """
    Will be raised if no solution for the IK-problem can be found for a given pose.
    Cannot be recovered.
    """


class CartesianLimitViolation(TrajectoryError):
    """
    Will be raised if any position of the trajectory is outside of the specified cartesian cuboid.
    Cannot be recovered.
    """


class JointLimitViolation(TrajectoryError):
    """
    Will be raised if the positional limits of the joints are violated in each solution for a point.
    In that case the trajectory is outside of the reduced joint workspace.
    The reduced joint workspace is defined by the manufacturer's joint limits or by user-defined ones within.
    Cannot be recovered.
    """


class JointVelocityViolation(TrajectoryError):
    """
    Will be raised if the the joint velocity would be exceeded.
    In that case a singularity is present or the speed of the TCP has to be lowered.
    """


class ConfigurationChanges(TrajectoryError):
    """
    Will be raised if a segment is not accessible within at least one common configuration.
    Since start and end points are included in segments also continuity between segments
    is tested.
    In case of a configuration change a singularity needs to be crossed and a path with the least
    configuration changes might be found.
    """


class CommandFailureInfo:
    """
    Contains information of a command evaluation, e.g. reason for failed checks.
    """

    def __init__(self, line: int, command: GCmd, exc=None):
        self.line_number = line
        self.command_str = str(command)
        self.failure_reason = str(exc)
        self.error_level = 'critical'

    def __str__(self):
        return f'{self.error_level} on line {self.line_number}: {self.command_str} - {self.failure_reason}'


def check_trajectory(
        cmds: List[GCmd],
        config: List[BaseJoint],
        clim: List[float],
        qlim: List[float],
        qdlim: List[float],
        home_pos: List[float],
        ds: float,
        fail_first: bool = False
):
    """
    Validate a trajectory defined by a list of G-code commands.
    :param cmds: List of G-Code command objects.
    :param config: List of joints containing coordinate transformations.
    :param clim: List of user-defined cartesian workspace limitations [-x, +x, -y, +y, -z, +z]
    :param qlim: List of joint position limitations [min J1, max J1, .., min Jn, max Jn]
    :param qdlim: List of joint velocity limitations [max v_J1, max v_J2, .., max v_Jn]
    :param home_pos: Home position given as list of joint values
    :param ds: Float value for distance between pose points in mm
    :param fail_first: Indicate whether the validation should be aborted on the first failure, defaults to False
    :return: None

    The following checks are done in order:
    1.  Check that all points are within the user-specified task space
        This involves processing commands that change the coordinate system.

        A linear segment is within the cuboid if its start and end point are within.

    2.  Check that a valid joint configuration can be found for each point.
        Prerequisite: Fixed way interpolation (linear and circular)
        a.) The IK solver does not raise OutOfReach
            = Trajectory is partially outside of allowable joint space
            => Object needs to be placed elsewhere
        b.) The IK solver does not indicate passing directly through a singularity
            = Task-space interpolation will raise an alert during execution
            => Reposition object in workspace
            Optional Fix: Execute real-time joint interpolation suffering from lower TCP speed
        c.) At least one of the solutions is within the given joint limits.
            => Reposition object in workspace
        d.) The solutions are within the same configuration
            Optional Fix: Configuration changes are inserted

    3.  Check that the TCP can reach the specified speed within the joint velocities.
        Prerequisite: Speed profile for fixed time steps, inverse Jacobian (difference based on IK)

    4.  Check that the robot paths are free of collisions.
        Special care is required for tool changes.
    """
    # Initialize the current position with the home position given in joint space
    start_position = forward_kinematics(config, home_pos)

    # Iterate over command list and generate waypoints
    task_trajectory = generate_task_trajectory(cmds, start_position, ds)

    # 1.) Validate the trajectory in the task space
    within_cartesian = [segment.is_within_cartesian_boundaries(clim) for segment in task_trajectory]
    if not all(within_cartesian):
        violation_idx = [idx for idx, val in enumerate(within_cartesian) if not val]
        raise CartesianLimitViolation('Found segments violating the cartesian boundaries.', violation_idx)

    # 2.) Create the trajectory in the joint space
    joint_trajectory = generate_joint_trajectory(task_trajectory, config)

    # Filter for the joint limits and validate the trajectory
    within_joint = [segment.is_within_joint_limits(qlim) for segment in joint_trajectory]
    if not all(within_joint):
        violation_idx = [idx for idx, val in enumerate(within_joint) if not val]
        raise JointLimitViolation('Found segments violating the joint boundaries.', violation_idx)

    # Check the configurations of the solutions
    common_configurations = [segment.get_common_configurations() for segment in joint_trajectory]
    if not all(common_configurations):
        violation_idx = [idx for idx, val in enumerate(within_joint) if not val]
        raise ConfigurationChanges('Found segments without common configuration.', violation_idx)

    # TODO Check for collisions (requires joint values)


def generate_joint_trajectory(complete_trajectory, config) -> List[JointTrajectorySegment]:
    segments_joint_space = []
    for segment in complete_trajectory:
        segment_solutions = []
        for pose in segment.trajectory_points:
            try:
                # Calculate the inverse kinematics without specifying a specific pose flag
                # Only if no solution at all can be found exceptions will propagate out
                solutions = ik_spherical_wrist(config, pose, pose_flags=None)
                # Append the solutions for the current pose
                segment_solutions.append(solutions)
            except OutOfReachError as e:
                # Inverse kinematic cannot be solved for points outside the workspace
                raise WorkspaceViolation from e
        # Append the solutions for the current segment
        segments_joint_space.append(JointTrajectorySegment(segment_solutions))
    return segments_joint_space


def generate_task_trajectory(cmds: List[GCmd], current_pos: np.ndarray, ds: float) -> List[CartesianTrajectorySegment]:
    """
    Generates trajectory points for a list of G-code commands.
    :param cmds: List of G-Code command objects.
    :param current_pos: Start pose point given as 4x4 homogeneous matrix
    :param ds: Float value for distance between pose points in mm
    :return:
    """
    is_absolute = True
    all_trajectory_pose_points = []

    for line_number, command in enumerate(cmds):
        # Movement commands
        if command.id in ['G01', 'G1']:
            lin_segment, target_pos = linear_segment_from_gcode(command, current_pos, ds, is_absolute)
            all_trajectory_pose_points.append(lin_segment)
            current_pos = target_pos
        elif command.id in ['G02', 'G2']:
            circ_segment, target_pos = circular_segment_from_gcode(command, current_pos, ds, is_absolute)
            all_trajectory_pose_points.append(circ_segment)
            current_pos = target_pos
        # Consider relative coordinates
        elif command.id == 'G90':
            is_absolute = True
        elif command.id == 'G91':
            is_absolute = False
        # TODO Consider coordinate origin shifting
        elif command.id == 'G92':
            pass
        # TODO Consider tool changes
        elif command.id.startswith('T'):
            pass

    return all_trajectory_pose_points


def circular_segment_from_gcode(command, current_pos, ds, is_absolute):
    # Get end point and centre point
    target_pos = command.cartesian_abs
    centre_pos = target_pos + command.cartesian_rel
    if not is_absolute:
        target_pos += current_pos
        centre_pos += current_pos
    # Circular interpolation (constant way-interval)
    # TODO Ensure order
    target_position = target_pos.values
    # TODO Get angles
    x_angle = 0
    y_angle = 0
    z_angle = 0
    target_pose = pose2tform(target_position, x_angle=x_angle, y_angle=y_angle, z_angle=z_angle)
    centre_position = centre_pos.values
    trajectory_pose_points = circular_interpolation(current_pos, target_pose, centre_position, ds=ds)
    circ_segment = CircularSegment(trajectory_pose_points)
    return circ_segment, target_pos


def linear_segment_from_gcode(command, current_pos, ds, is_absolute):
    # Get end point
    target_pos = command.cartesian_abs
    if not is_absolute:
        target_pos += current_pos
    # Linear interpolation (constant way-interval)
    # TODO Ensure order
    target_position = target_pos.values
    # TODO Get angles
    x_angle = 0
    y_angle = 0
    z_angle = 0
    target_pose = pose2tform(target_position, x_angle=x_angle, y_angle=y_angle, z_angle=z_angle)
    trajectory_pose_points = linear_interpolation(current_pos, target_pose, ds=ds)
    lin_segment = LinearSegment(trajectory_pose_points)
    return lin_segment, target_pos
