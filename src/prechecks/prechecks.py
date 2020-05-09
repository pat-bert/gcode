from math import floor
from typing import List

import numpy as np

from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import forward_kinematics, get_tform
from src.kinematics.inverse_kinematics import ik_spherical_wrist, Singularity, OutOfReachError
from src.kinematics.joints import BaseJoint


class TrajectoryError(ValueError):
    """
    Indicate that a trajectory check has failed.
    """


class WorkspaceViolation(TrajectoryError):
    """
    Will be raised if no solution for the IK-problem can be found for a given pose.
    """


class JointLimitViolation(TrajectoryError):
    """
    Will be raised if the positional limits of the joints are violated.
    """


class JointVelocityViolation(TrajectoryError):
    """
    Will be raised if the
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


def linear_interpolation(start: np.ndarray, end: np.ndarray, *, ds: float):
    """
    Calculate linearly interpolated waypoints on a straight line.
    :param start: Start pose (x,y,z) given as 4x4 homogeneous matrix
    :param end: End pose (x,y,z) given as 4x4 homogeneous matrix
    :param ds: Constant distance between points in mm
    :return: Generator with all pose points on a straight line including start and end.
    """
    # Get the cartesian distance
    direction_vec = end[0:3, 3] - start[0:3, 3]
    total_way_len = np.linalg.norm(direction_vec)

    if total_way_len <= ds:
        # No interpolation is necessary, just return the start and end points.
        yield start
        yield end
    else:
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


def circular_interpolation():
    pass


def trapezoidal_speed_profile(v_max: float, a: float, dt: float):
    """

    :param v_max: Peak velocity in mm/s
    :param a: Constant acceleration in mm/s^2
    :param dt: Constant time sample in seconds
    :return:
    """
    pass


def check_trajectory(
        cmds: List[GCmd],
        config: List[BaseJoint],
        qlim: List[float],
        clim: List[float],
        qdlim: List[float],
        home_pos: List[float],
        fail_first: bool = False
):
    """
    Validate a trajectory defined by a list of G-code commands.
    :param cmds: List of G-Code command objects.
    :param config: List of joints containing coordinate transformations.
    :param qlim: List of joint position limitations [min J1, max J1, .., min Jn, max Jn]
    :param clim: List of user-defined cartesian workspace limitations [-x, +x, -y, +y, -z, +z]
    :param qdlim: List of joint velocity limitations [max v_J1, max v_J2, .., max v_Jn]
    :param home_pos:
    :param fail_first: Indicate whether the validation should be aborted on the first failure, defaults to False
    :return:

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
    current_pos = forward_kinematics(config, home_pos)

    # Iterate over command list and generate waypoints
    for line_number, command in enumerate(cmds):
        if command.id in ['G01', 'G1']:
            # Linear interpolation (constant way-interval)
            linear_interpolation(current_pos, command.cartesian_abs, ds=1)
            current_pos = command.cartesian_abs
        elif command.id in ['G02', 'G2']:
            # Circular interpolation (constant way-interval)
            pass
        # Consider relative coordinates
        # Consider coordinate origin shifting
        # Consider tool changes

    # Validate the trajectory in the task space

    # Validate the trajectory in the joint space
    try:
        # Calculate the inverse kinematics without specifying a specific pose flag
        tform = np.array([0])
        solutions = ik_spherical_wrist(config, tform, pose_flags=None)
    except OutOfReachError as e:
        # Inverse kinematic cannot be solved for points outside the workspace
        raise WorkspaceViolation from e
    except Singularity as e:
        # Task-space interpolation will fail for exact singularities
        pass

    # Check the configurations of the solutions

    # TODO Check for collisions (requires joint values)
