import numpy as np

from Coordinate import Coordinate
from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import pose2tform
from src.prechecks.spatial_interpolation import circular_interpolation, linear_interpolation
from src.prechecks.trajectory_segment import CircularSegment, LinearSegment


def circular_segment_from_gcode(command: GCmd, current_pos, ds: float, is_absolute: bool) -> CircularSegment:
    """
    Generates a circular trajectory segment in task space from a linear G-code command.
    :param command:
    :param current_pos:
    :param ds:
    :param is_absolute:
    :return:
    """
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
    return circ_segment


def linear_segment_from_gcode(command: GCmd, current_pose: np.ndarray, ds: float, is_absolute: bool) -> LinearSegment:
    """
    Generates a linear trajectory segment in task space from a linear G-code command.
    :param command:
    :param current_pose:
    :param ds:
    :param is_absolute:
    :return:
    """
    # Consider relative positioning
    if not is_absolute:
        zero = Coordinate([0] * 6, 'XYZABC')
        command.cartesian_abs.update_empty(zero)
    else:
        none_values = Coordinate(current_pose[0:3, 3], 'XYZ')
        command.cartesian_abs.update_empty(none_values)

    # Get end point (TODO Ensure order)
    target_position = command.cartesian_abs.values
    # TODO Get angles
    x_angle = 0
    y_angle = 0
    z_angle = 0
    target_pose = pose2tform(target_position, x_angle=x_angle, y_angle=y_angle, z_angle=z_angle)

    if not is_absolute:
        target_pose = np.dot(current_pose, target_pose)

    # Linear interpolation (constant way-interval)
    trajectory_pose_points = linear_interpolation(current_pose, target_pose, ds=ds)
    lin_segment = LinearSegment(trajectory_pose_points)
    return lin_segment
