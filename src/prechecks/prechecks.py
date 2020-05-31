from math import pi
from typing import List, Iterator

from src.collisions.collision_checking import MatlabCollisionChecker
from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joints import BaseJoint
from src.prechecks.configs import melfa_rv_4a
from src.prechecks.trajectory_generation import generate_task_trajectory, generate_joint_trajectory
from src.prechecks.trajectory_segment import JointTrajectorySegment
from src.prechecks.utils import print_progress, time_func_call


class TrajectoryError(ValueError):
    """
    Indicate that a trajectory check has failed.
    """


class CollisionViolation(TrajectoryError):
    """

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


@time_func_call
def check_trajectory(
        cmds: List[GCmd],
        config: List[BaseJoint],
        clim: List[float],
        qlim: List[float],
        qdlim: List[float],
        home_pos: List[float],
        ds: float,
        urdf_path: str,
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
    :param urdf_path:
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
    print('Generating task trajectory...')
    task_trajectory = generate_task_trajectory(cmds, start_position, ds)

    # 1.) Validate the trajectory in the task space
    print('Validating trajectory in task space...')
    within_cartesian = [segment.is_within_cartesian_boundaries(clim) for segment in task_trajectory]
    if not all(within_cartesian):
        violation_idx = [idx for idx, val in enumerate(within_cartesian) if not val]
        raise CartesianLimitViolation('Found segments violating the cartesian limits.', violation_idx)
    print('All segments are located within the cartesian limits.')

    # 2.) Create the trajectory in the joint space
    print('Generating joint trajectory...')
    joint_trajectory = generate_joint_trajectory(task_trajectory, config)

    # Filter for the joint limits and validate the trajectory
    print('Filtering positional joint limits...')
    within_joint = [segment.is_within_joint_limits(qlim) for segment in joint_trajectory]
    if not all(within_joint):
        violation_idx = [idx for idx, val in enumerate(within_joint) if not val]
        raise JointLimitViolation('Found segments violating the joint limits.', violation_idx)
    print('All segments have joint solutions within the limits.')

    # Check the configurations of the solutions
    print('Checking common configurations...')
    common_configurations = [segment.get_common_configurations() for segment in joint_trajectory]
    if not all(common_configurations):
        violation_idx = [idx for idx, val in enumerate(common_configurations) if not common_configurations]
        raise ConfigurationChanges('Found segments without common configuration.', violation_idx)
    print('Each segment can be executed without configuration change.')

    print('Creating collision scene...')
    # TODO Create collision scene from task trajectory segments
    all_collision_scenes = list(range(10))

    # Init Matlab Runtime
    collider = MatlabCollisionChecker()
    collisions = collider.check_collisions(home_pos, path=urdf_path)
    if collisions[0]:
        raise CollisionViolation('Home position is in collision.')
    print('Home position is not in collision.')

    # Check paths sorted by minimum configuration change cost for collisions
    for least_cost_config in get_min_cost_paths(common_configurations, joint_trajectory):
        # TODO Paralellize collision checking (segment-wise)
        for seg, seg_conf, current_collision_scene in zip(joint_trajectory, least_cost_config, all_collision_scenes):
            # Get joint coordinates at each point for the determined arm onfiguration
            total_len = len(seg.solutions)
            print_progress(0, total_len, prefix=f'Checking collisions for segment #{seg.idx}...')
            for point_idx, point_solutions in enumerate(seg.solutions):
                print_progress(point_idx + 1, total_len, prefix=f'Checking collisions for segment #{seg.idx}...')
                joint_values = point_solutions[seg_conf]
                collisions = collider.check_collisions(joint_values)
                if collisions[0]:
                    # The current segment is not valid
                    print(f'Found collisions for point #{point_idx} on segment #{seg.idx}.')
                    break
            else:
                continue
            break
        else:
            # The configuration list is valid for all segments, no need to calculate the next best path
            print('All segments were collision-free with the current configuration path.')
            break
        print('Some segments were in collision. Querying next-best configuration path.')
    else:
        raise CollisionViolation('No collision-free trajectory could be determined.')


def get_min_cost_paths(common_conf: List[List[float]], joint_traj: List[JointTrajectorySegment]) -> Iterator[List[int]]:
    # TODO Yield results of Dijkstra algorithm for minimum cost in ascending order
    best_conf = [i[0] for i in common_conf]
    yield best_conf * len(joint_traj)


if __name__ == '__main__':
    cmd_raw = 'G91\n' \
              'G01 X100 Z-50\n' \
              'G01 Y50 Z-50\n' \
              'G01 Z-200\n' \
              'G01 Y300\n' \
              'G01 Y-700'
    commands = [GCmd.read_cmd_str(cmd_str) for cmd_str in cmd_raw.splitlines()]
    robot_config = melfa_rv_4a()
    cartesian_limits = [0, 1000, -500, 500, 0, 700]
    joint_limits = [
        -2.7925, 2.7925,
        -1.5708, 2.4435,
        +0.2618, 2.9496,
        -2.7925, 2.7925,
        -2.0944, 2.0944,
        -3.4907, 3.4907
    ]
    joint_velocity_limits = [3.7699, 4.7124, 4.7124, 4.7124, 4.7124, 7.5398]
    home_position = [0, 0, pi / 2, 0, pi / 2, 0]
    inc_distance = 1
    urdf_file_path = './../../ressource/robot.urdf'

    check_trajectory(commands, robot_config, cartesian_limits, joint_limits, joint_velocity_limits, home_position,
                     inc_distance, urdf_file_path)
