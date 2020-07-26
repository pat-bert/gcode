from math import pi
from typing import Optional

import numpy as np

from src.Coordinate import Coordinate
from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import pose2tform
from src.prechecks.spatial_interpolation import circular_interpolation, linear_interpolation
from src.prechecks.trajectory_segment import CircularSegment, LinearSegment


def normal2euler(normal_vec=None, rotation=None):
    # TODO Calculate angles from normal vector
    x_angle = -pi
    y_angle = 0
    z_angle = -pi
    return x_angle, y_angle, z_angle


def circ_segment_from_gcode(command: GCmd, current_pose: np.ndarray, ds: float, is_absolute: bool, curr_vel: float,
                            curr_acc: float, orig_offset: Optional[Coordinate] = None) -> CircularSegment:
    """
    Generates a circular trajectory segment in task space from a linear G-code command.
    :param command: G-Code command for circular interpolation to be converted
    :param current_pose: Start pose for the current segment (not contained within command) given as 4x4 matrix, in robot
    base coordinate
    :param ds: Way delta for discretizing the segment in mm
    :param is_absolute: Flag to indicate whether the command specifies absolute or relative coordinates
    :param curr_vel: Velocity in mm/min
    :param curr_acc: Acceleration in mm/s^2
    :param orig_offset:
    :return: Circular Segment containing points with distance ds
    """
    # Get end point and center point
    current_position = Coordinate(current_pose[0:3, 3], 'XYZ')
    target_position = command.cartesian_abs
    zero = Coordinate([0] * 6, 'XYZ')
    command.cartesian_rel.update_empty(zero)
    centre_pos = current_position + command.cartesian_rel

    if is_absolute:
        target_position.add_axis(current_position)
        target_position.update_empty(current_position)

        # Only absolute positioning needs to consider shifting between robot and work coordinate system
        if orig_offset is None:
            target_position.update_empty(current_position)
        else:
            # Target values are current values but need to be transformed into work coordinate system
            target_position.update_empty(current_position - orig_offset)
            target_position = target_position + orig_offset

    else:
        target_position.add_axis(zero)
        target_position.update_empty(zero)
        target_position += current_position

    # Consider velocity update
    curr_vel = command.speed or curr_vel
    is_clockwise = command.id in ['G02', 'G2']
    has_extr = command.extrude_len is not None

    # TODO Ensure correct order of coordinate values
    target_position = target_position.values

    normal_vec = command.normal_vec
    x_angle, y_angle, z_angle = normal2euler()
    target_pose = pose2tform(target_position, x_angle=x_angle, y_angle=y_angle, z_angle=z_angle)

    centre_position = centre_pos.values

    # Circular interpolation (constant way-interval)
    traj_poses = circular_interpolation(current_pose, target_pose, centre_position, normal_vec, is_clockwise, ds=ds)
    centre_position = np.array(centre_position)
    circ_segment = CircularSegment(traj_poses, centre_position, extrusion=has_extr, vel=curr_vel, acc=curr_acc, ds=ds)
    return circ_segment


def lin_segment_from_gcode(command: GCmd, current_pose: np.ndarray, ds: float, is_absolute: bool, curr_vel: float,
                           curr_acc: float, orig_offset: Optional[Coordinate] = None) -> LinearSegment:
    """
    Generates a linear trajectory segment in task space from a linear G-code command.
    :param command: G-Code command for circular interpolation to be converted
    :param current_pose: Start pose for the current segment (not contained within command), in robot base coordinate
    :param ds: Way delta for discretizing the segment in mm
    :param is_absolute: Flag to indicate whether the command specifies absolute or relative coordinates
    :param curr_vel: Velocity in mm/min
    :param curr_acc: Acceleration in mm/s^2
    :param orig_offset:
    :return: Linear Segment containing points with distance ds
    """
    current_position = Coordinate(current_pose[0:3, 3], 'XYZ')
    target_position = command.cartesian_abs

    # Consider missing elements
    if is_absolute:
        # Missing element means use value from previous position
        target_position.add_axis(current_position)

        # Only absolute positioning needs to consider shifting between robot and work coordinate system
        if orig_offset is None:
            target_position.update_empty(current_position)
        else:
            # Target values are current values but need to be transformed into work coordinate system
            target_position.update_empty(current_position - orig_offset)
            target_position = target_position + orig_offset
    else:
        # Missing element means zero offset in relative positioning
        zero = Coordinate([0] * 6, 'XYZ')
        target_position.add_axis(zero)
        target_position.update_empty(zero)
        target_position += current_position

    # Consider velocity update
    curr_vel = command.speed or curr_vel
    has_extr = command.extrude_len is not None

    x_angle, y_angle, z_angle = normal2euler()

    # Get end point (TODO Ensure order)
    target_pose = pose2tform(target_position.values, x_angle=x_angle, y_angle=y_angle, z_angle=z_angle)

    # Linear interpolation (constant way-interval)
    trajectory_pose_points = linear_interpolation(current_pose, target_pose, ds=ds)
    lin_segment = LinearSegment(trajectory_pose_points, extrusion=has_extr, vel=curr_vel, acc=curr_acc, ds=ds)
    return lin_segment
