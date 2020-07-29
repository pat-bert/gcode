from typing import List

import numpy as np

from src.Coordinate import Coordinate
from src.gcode.GCmd import GCmd
from src.kinematics.inverse_kinematics import ik_spherical_wrist, OutOfReachError
from src.kinematics.joints import BaseJoint
from src.prechecks.exceptions import WorkspaceViolation
from src.prechecks.gcode2segment import lin_segment_from_gcode, circ_segment_from_gcode
from src.prechecks.trajectory_segment import CartesianTrajSegment, JointTrajSegment
from src.prechecks.utils import print_progress


def generate_task_trajectory(cmds: List[GCmd], curr_pos: np.ndarray, ds: float, acc: float, hb_offset: Coordinate) \
        -> List[CartesianTrajSegment]:
    """
    Generates trajectory points for a list of G-code commands.
    :param cmds: List of G-Code command objects.
    :param curr_pos: Start pose point given as 4x4 homogeneous matrix
    :param ds: Float value for distance between pose points in mm
    :param acc: Float value for the default robot acceleration in mm/s^2
    :param hb_offset:
    :return: Task trajectory given as list of CartesianTrajectorySegments
    """
    is_abs = True
    all_trajectory_pose_points = []

    # Initialize default values
    curr_vel = 0
    curr_acc = acc

    print('Generating task trajectory...')
    total_len = len(cmds)

    for line_number, cmd in enumerate(cmds):
        prefix = f'Creating task trajectory for command line #{line_number} ...'
        print_progress(line_number + 1, total_len, prefix=prefix)

        # Movement commands
        if cmd.id in ['G01', 'G1']:
            if len(cmd.cartesian_abs.values) > 0:
                # G-Code can also be used only to retract filament or change speed
                lin_segment = lin_segment_from_gcode(cmd, curr_pos, ds, is_abs, curr_vel, curr_acc,
                                                     orig_offset=hb_offset)
                all_trajectory_pose_points.append(lin_segment)
                curr_pos = lin_segment.target
            curr_vel = cmd.speed or curr_vel
        elif cmd.id in ['G02', 'G2', 'G03', 'G3']:
            if len(cmd.cartesian_abs.values) > 0:
                # G-Code can also be used only to retract filament or change speed
                circ_segment = circ_segment_from_gcode(cmd, curr_pos, ds, is_abs, curr_vel, curr_acc,
                                                       orig_offset=hb_offset)
                all_trajectory_pose_points.append(circ_segment)
                curr_pos = circ_segment.target
            curr_vel = cmd.speed or curr_vel
        elif cmd.id == 'G28':
            curr_pos = np.array(
                [
                    [-1.0, 0.0, +0.0, 0.0],
                    [+0.0, 1.0, +0.0, 0.0],
                    [+0.0, 0.0, -1.0, 0.0],
                    [+0.0, 0.0, +0.0, 0.0]
                ]
            )
            curr_pos[0:3, 3] = hb_offset.values
        # Consider relative coordinates
        elif cmd.id == 'G90':
            is_abs = True
        elif cmd.id == 'G91':
            is_abs = False
        elif cmd.id == 'G92':
            # Consider coordinate origin shifting
            if cmd.cartesian_abs.values:
                raise NotImplementedError(f'Origin shifting of XYZ is not supported: #{line_number} {cmd}')
        elif cmd.id[0] == 'T' and cmd.id[1:] != '0':
            # Consider tool changes
            raise NotImplementedError(f'Tool changes are not supported: #{line_number} {cmd}')

    return all_trajectory_pose_points


def generate_joint_trajectory(task_traj: List[CartesianTrajSegment], config: List[BaseJoint]) -> List[JointTrajSegment]:
    """
    Generates the trajectory in joint space
    :param task_traj: Trajectory in task space (cartesian), given as list of CartesianTrajectorySegments
    :param config: List of joints implementing the BaseJoint interface
    :return: Joint trajectory given as list of JointTrajectorySegments
    """
    print('Generating joint trajectory...')

    joint_segments = []

    total_len = len(task_traj)

    for seg_idx, cartesian_segment in enumerate(task_traj):
        current_segment_solutions = []

        prefix = f'Calculating joint solutions for segment #{seg_idx} ...'
        print_progress(seg_idx + 1, total_len, prefix=prefix)

        for pt_idx, pose in enumerate(cartesian_segment.unmodified_points):
            try:
                # Calculate the inverse kinematics without specifying a specific pose flag
                # Only if no solution at all can be found exceptions will propagate out
                current_point_solutions = ik_spherical_wrist(config, pose, pose_flags=None)
                # Append the solutions for the current pose
                current_segment_solutions.append(current_point_solutions)
            except OutOfReachError:
                # Inverse kinematic cannot be solved for points outside the workspace
                x_descr = 'XDir {}'.format(', '.join(f'{i:.2f}' for i in pose[0:3, 0]))
                y_descr = 'YDir {}'.format(', '.join(f'{i:.2f}' for i in pose[0:3, 1]))
                z_descr = 'ZDir {}'.format(', '.join(f'{i:.2f}' for i in pose[0:3, 2]))
                pos_descr = 'Pos {}'.format(', '.join(f'{i:.2f}' for i in pose[0:3, 3]))
                pose_descr = f'{x_descr}; {y_descr}; {z_descr}; {pos_descr}'
                raise WorkspaceViolation(
                    f'Cannot reach position #{pt_idx} in segment #{seg_idx}:\n{pose_descr}') from None
        # Append the solutions for the current segment
        joint_segments.append(JointTrajSegment(current_segment_solutions, cartesian_segment.time_points))
    return joint_segments
