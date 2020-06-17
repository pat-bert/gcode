from math import pi
from typing import List, Iterator

from dijkstar import find_path

from src.collisions.collision_checking import MatlabCollisionChecker
from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joints import BaseJoint
from src.prechecks.configs import melfa_rv_4a
from src.prechecks.exceptions import CollisionViolation, CartesianLimitViolation, JointLimitViolation, \
    ConfigurationChangesError
from src.prechecks.graph_search import create_graph
from src.prechecks.trajectory_generation import generate_task_trajectory, generate_joint_trajectory
from src.prechecks.trajectory_segment import JointTrajectorySegment
from src.prechecks.utils import print_progress, time_func_call


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

    3.  TODO Check that the TCP can reach the specified speed within the joint velocities.
        Prerequisite: Speed profile for fixed time steps, inverse Jacobian (difference based on IK)

    4.  Check that the robot paths are free of collisions.
    """
    # Initialize the current position with the home position given in joint space
    start_position = forward_kinematics(config, home_pos)

    # Generate cartesian waypoints from command list and validate limits
    print('Generating task trajectory...')
    task_trajectory = generate_task_trajectory(cmds, start_position, ds)
    check_cartesian_limits(task_trajectory, clim)

    # Convert to joint space and filter out solutions exceeding the limits
    joint_trajectory = generate_joint_trajectory(task_trajectory, config)
    filter_joint_limits(joint_trajectory, qlim)

    # Check the configurations of the solutions
    common_configurations = get_common_configurations(joint_trajectory)

    # Create a unidirectional graph of the solutions. For each point a set of nodes is created according to viable robot
    # configurations. The nodes of the same points stay unconnected but all nodes of adjacent points are connected at
    # the beginning. The edges between the nodes represent the cost required. The cost is calculated based on the joint
    # coordinates and the robot configurations of the connected nodes.
    graph, start_node, stop_node = create_graph(joint_trajectory, qlim)

    # Find the initially shortest path to be checked for collisions. Nodes in collision can be removed to query for the
    # next best path.
    path_info = find_path(graph, start_node, stop_node)
    print(f'Total cost for the minimum cost path: {path_info.total_cost}')

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


def get_common_configurations(joint_trajectory) -> List[List[int]]:
    """
    Search for common configurations for each segment of the trajectory.
    :param joint_trajectory:
    :return: For each segment a list with common robot configurations for points of that segments is added to a total
    list.
    :raises: ConfigurationChangesError if any segment is found that does not have solutions with a common configuration.
    """
    print('Checking common configurations...')
    common_configurations = [segment.get_common_configurations() for segment in joint_trajectory]
    if not all(common_configurations):
        violation_idx = [idx for idx, val in enumerate(common_configurations) if not val]
        # TODO Attempt different tool orientation
        raise ConfigurationChangesError('Execution of segments without common configuration is not supported',
                                        violation_idx)
    print('Each segment can be executed without configuration change.')
    return common_configurations


def check_cartesian_limits(task_trajectory, clim: List[float]):
    """
    Verify that a set of waypoints is within given cartesian limits.
    :param task_trajectory:
    :param clim: List of user-defined cartesian workspace limitations [-x, +x, -y, +y, -z, +z]
    :raises: CartesianLimitViolation if any point of a segment is violating the limits
    """
    print('Validating trajectory in task space...')
    within_cartesian = [segment.is_within_cartesian_boundaries(clim) for segment in task_trajectory]
    if not all(within_cartesian):
        violation_idx = [idx for idx, val in enumerate(within_cartesian) if not val]
        raise CartesianLimitViolation('Found segments violating the cartesian limits.', violation_idx)
    print('All segments are located within the cartesian limits.')


def filter_joint_limits(joint_trajectory, qlim: List[float]):
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
    print('All segments have joint solutions within the limits.')


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
              'G01 Y-700\n' \
              'G90\n' \
              'G01 Y0\n' \
              'G01 X500 Z-50'
    commands = [GCmd.read_cmd_str(cmd_str) for cmd_str in cmd_raw.splitlines()]
    robot_config = melfa_rv_4a()
    cartesian_limits = [0, 1000, -500, 500, -100, 700]
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
