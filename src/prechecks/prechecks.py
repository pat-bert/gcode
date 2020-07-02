from collections import namedtuple
from typing import List

from dijkstar import find_path

from src.collisions.collision_checking import MatlabCollisionChecker
from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joints import BaseJoint
from src.prechecks.exceptions import CollisionViolation, ConfigurationChangesError
from src.prechecks.graph_search import create_graph, calc_conf_from_node
from src.prechecks.trajectory_generation import generate_task_trajectory, generate_joint_trajectory
from src.prechecks.trajectory_segment import check_cartesian_limits, filter_joint_limits
from src.prechecks.utils import print_progress, time_func_call

Constraints = namedtuple('Constraints', 'pos_cartesian pos_joint vel_joint')
"""
:param pos_cartesian: List of user-defined cartesian workspace limitations [-x, +x, -y, +y, -z, +z]
:param pos_joint: List of joint position limitations [min J1, max J1, .., min Jn, max Jn]
:param vel_joint: List of joint velocity limitations [max v_J1, max v_J2, .., max v_Jn]
"""


@time_func_call
def check_traj(cmds: List[GCmd], config: List[BaseJoint], limits: Constraints, home: List[float], ds: float,
               default_acc: float, urdf: str):
    """
    Validate a trajectory defined by a list of G-code commands.
    :param cmds: List of G-Code command objects.
    :param config: List of joints containing coordinate transformations.
    :param limits: Namedtuple containing all relevant trajectory constraints
    :param home: Home position given as list of joint values
    :param ds: Float value for distance between pose points in mm
    :param default_acc: Float value for the default robot acceleration in mm/s^2
    :param urdf:
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
    """
    # Initialize the current position with the home position given in joint space
    start_position = forward_kinematics(config, home)

    # Generate cartesian waypoints from command list and validate limits
    print('Generating task trajectory...')
    task_trajectory = generate_task_trajectory(cmds, start_position, ds, default_acc)
    check_cartesian_limits(task_trajectory, limits.pos_cartesian)

    # Convert to joint space and filter out solutions exceeding the limits
    joint_traj = generate_joint_trajectory(task_trajectory, config)
    filter_joint_limits(joint_traj, limits.pos_joint)

    # Check the configurations of the solutions
    common_configurations = get_common_configurations(joint_traj)
    print(f'Configurations in trajectory: {set(conf for conf_list in common_configurations for conf in conf_list)}')
    print(f'Configurations common to all segments: {set.intersection(*map(set, common_configurations))}')

    # Create a unidirectional graph of the solutions. For each point a set of nodes is created according to viable robot
    # configurations. The nodes of the same points stay unconnected but all nodes of adjacent points are connected at
    # the beginning. The edges between the nodes represent the cost required. The cost is calculated based on the joint
    # coordinates and the robot configurations of the connected nodes.
    graph, start_node, stop_node = create_graph(joint_traj, limits.pos_joint, limits.vel_joint)

    # Find the initially shortest path to be checked for collisions. Nodes in collision can be removed to query for the
    # next best path.
    path = find_path(graph, start_node, stop_node)
    print(f'Total cost for the minimum cost path: {path.total_cost}')
    robot_conf_per_point = [calc_conf_from_node(node_idx, pt_idx) for pt_idx, node_idx in enumerate(path.nodes[1:-1])]
    print(f'Configurations in shortest path: {set(robot_conf_per_point)}')

    print('Creating collision scene...')
    # TODO Create collision scene from task trajectory segments
    all_collision_scenes = [1] * len(joint_traj)

    # Init Matlab Runtime
    collider = MatlabCollisionChecker()
    collisions = collider.check_collisions(home, path=urdf)
    if collisions[0]:
        raise CollisionViolation('Home position is in collision.')
    print('Home position is not in collision.')

    # Check paths sorted by minimum configuration change cost for collisions
    for best_config_path in [robot_conf_per_point]:
        is_collision_free = []
        for seg, seg_conf, current_collision_scene in zip(joint_traj, best_config_path, all_collision_scenes):
            free = is_segment_collision_free(collider, seg, seg_conf, current_collision_scene)
            is_collision_free.append(free)
        if all(is_collision_free):
            # The configuration list is valid for all segments, no need to calculate the next best path
            print('All segments were collision-free with the current configuration path.')
            break
        print('Some segments were in collision. Querying next-best configuration path.')
    else:
        raise CollisionViolation('No collision-free trajectory could be determined.')


def is_segment_collision_free(collider, seg, seg_conf, current_collision_scene) -> bool:
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
            return False
    return True


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
