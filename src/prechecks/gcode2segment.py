from math import pi

import numpy as np

from Coordinate import Coordinate
from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import pose2tform
from src.prechecks.spatial_interpolation import circular_interpolation, linear_interpolation
from src.prechecks.trajectory_segment import CircularSegment, LinearSegment


def circular_segment_from_gcode(command: GCmd, current_pos, ds: float, is_absolute: bool, curr_vel: float,
                                curr_acc: float) -> CircularSegment:
    """
    Generates a circular trajectory segment in task space from a linear G-code command.
    :param command:
    :param current_pos:
    :param ds: Way delta for discretizing the segment in mm
    :param is_absolute: Flag to indicate whether the command specifies absolute or relative coordinates
    :param curr_vel: Velocity in mm/min
    :param curr_acc: Acceleration in mm/s^2
    :return:
    """
    # Get end point and centre point
    target_pos = command.cartesian_abs
    centre_pos = target_pos + command.cartesian_rel
    if not is_absolute:
        target_pos += current_pos
        centre_pos += current_pos

    # Consider velocity update
    curr_vel = command.speed or curr_vel

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
    circ_segment = CircularSegment(trajectory_pose_points, curr_vel, curr_acc, ds)
    return circ_segment


def linear_segment_from_gcode(command: GCmd, current_pose: np.ndarray, ds: float, is_absolute: bool, curr_vel: float,
                              curr_acc: float) -> LinearSegment:
    """
    Generates a linear trajectory segment in task space from a linear G-code command.
    :param command:
    :param current_pose:
    :param ds: Way delta for discretizing the segment in mm
    :param is_absolute: Flag to indicate whether the command specifies absolute or relative coordinates
    :param curr_vel: Velocity in mm/min
    :param curr_acc: Acceleration in mm/s^2
    :return:
    """
    current_position = Coordinate(current_pose[0:3, 3], 'XYZ')
    target_position = command.cartesian_abs

    # Consider missing elements
    if is_absolute:
        # Missing element means use value from previous position
        target_position.update_empty(current_position)
    else:
        # Missing element means zero offset in relative positioning
        zero = Coordinate([0] * 6, 'XYZABC')
        target_position.update_empty(zero)
        target_position += current_position

    # Consider velocity update
    curr_vel = command.speed or curr_vel

    # Get end point (TODO Ensure order)
    # TODO Get angles from tool vector (initial rotation)
    x_angle = -pi
    y_angle = 0
    z_angle = -pi
    target_pose = pose2tform(target_position.values, x_angle=x_angle, y_angle=y_angle, z_angle=z_angle)

    # Linear interpolation (constant way-interval)
    trajectory_pose_points = linear_interpolation(current_pose, target_pose, ds=ds)
    lin_segment = LinearSegment(trajectory_pose_points, curr_vel, curr_acc, ds)
    return lin_segment