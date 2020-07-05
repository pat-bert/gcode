from typing import List

import numpy as np

from src.gcode.GCmd import GCmd
from src.kinematics.inverse_kinematics import ik_spherical_wrist, OutOfReachError
from src.kinematics.joints import BaseJoint
from src.prechecks.exceptions import WorkspaceViolation
from src.prechecks.gcode2segment import linear_segment_from_gcode, circular_segment_from_gcode
from src.prechecks.trajectory_segment import CartesianTrajSegment, JointTrajSegment


def generate_task_trajectory(cmds: List[GCmd], current_pos: np.ndarray, ds: float, acc: float) \
        -> List[CartesianTrajSegment]:
    """
    Generates trajectory points for a list of G-code commands.
    :param cmds: List of G-Code command objects.
    :param current_pos: Start pose point given as 4x4 homogeneous matrix
    :param ds: Float value for distance between pose points in mm
    :param acc: Float value for the default robot acceleration in mm/s^2
    :return: Task trajectory given as list of CartesianTrajectorySegments
    """
    is_absolute = True
    all_trajectory_pose_points = []

    # Initialize default values
    current_vel = 0
    current_acc = acc

    for line_number, command in enumerate(cmds):
        # Movement commands
        if command.id in ['G01', 'G1']:
            lin_segment = linear_segment_from_gcode(command, current_pos, ds, is_absolute, current_vel, current_acc)
            all_trajectory_pose_points.append(lin_segment)
            current_pos = lin_segment.target
            current_vel = command.speed or current_vel
        elif command.id in ['G02', 'G2', 'G03', 'G3']:
            circ_segment = circular_segment_from_gcode(command, current_pos, ds, is_absolute, current_vel, current_acc)
            all_trajectory_pose_points.append(circ_segment)
            current_pos = circ_segment.target
            current_vel = command.speed or current_vel
        # Consider relative coordinates
        elif command.id == 'G90':
            is_absolute = True
        elif command.id == 'G91':
            is_absolute = False
        elif command.id == 'G92':
            # Consider coordinate origin shifting
            raise NotImplementedError('Origin shifting is not supported.')
        elif command.id[0] == 'T' and command.id[1:] != 0:
            # Consider tool changes
            raise NotImplementedError('Tool changes are not supported.')

    return all_trajectory_pose_points


def generate_joint_trajectory(task_trajectory: List[CartesianTrajSegment], config: List[BaseJoint]) \
        -> List[JointTrajSegment]:
    """
    Generates the trajectory in joint space
    :param task_trajectory: Trajectory in task space (cartesian), given as list of CartesianTrajectorySegments
    :param config: List of joints implementing the BaseJoint interface
    :return: Joint trajectory given as list of JointTrajectorySegments
    """
    print('Generating joint trajectory...')

    joint_segments = []
    for seg_idx, cartesian_segment in enumerate(task_trajectory):
        current_segment_solutions = []
        for pt_idx, pose in enumerate(cartesian_segment.trajectory_points):
            try:
                # Calculate the inverse kinematics without specifying a specific pose flag
                # Only if no solution at all can be found exceptions will propagate out
                current_point_solutions = ik_spherical_wrist(config, pose, pose_flags=None)
                # Append the solutions for the current pose
                current_segment_solutions.append(current_point_solutions)
            except OutOfReachError as e:
                # Inverse kinematic cannot be solved for points outside the workspace
                x_descr = 'XDir {}'.format(', '.join(f'{i:.2f}' for i in pose[0:3, 0]))
                y_descr = 'YDir {}'.format(', '.join(f'{i:.2f}' for i in pose[0:3, 1]))
                z_descr = 'ZDir {}'.format(', '.join(f'{i:.2f}' for i in pose[0:3, 2]))
                pos_descr = 'Pos {}'.format(', '.join(f'{i:.2f}' for i in pose[0:3, 3]))
                pose_descr = f'{x_descr}; {y_descr}; {z_descr}; {pos_descr}'
                raise WorkspaceViolation(f'Cannot reach position #{pt_idx} in segment #{seg_idx}:\n{pose_descr}') from e
        # Append the solutions for the current segment
        joint_segments.append(JointTrajSegment(current_segment_solutions, cartesian_segment.time_points))
    return joint_segments
