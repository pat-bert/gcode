from math import ceil, pi
from typing import List, Optional

import numpy as np
from dijkstar import find_path, NoPathError

from src.collisions.collision_checking import MatlabCollisionChecker
from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joints import BaseJoint
from src.prechecks.dataclasses import Constraints, Increments
from src.prechecks.exceptions import CollisionViolation, ConfigurationChangesError, JointVelocityViolation, \
    JOINT_SPEED_ALLOWABLE_RATIO, NoValidPathFound
from src.prechecks.graph_search import create_graph, calc_conf_from_node, calc_node_idx
from src.prechecks.trajectory_generation import generate_task_trajectory, generate_joint_trajectory
from src.prechecks.trajectory_segment import check_cartesian_limits, JointTrajSegment, filter_joint_limits, \
    CartesianTrajSegment
from src.prechecks.utils import print_progress, time_func_call


def axang2tform(nvec, angle: float):
    # TODO Rodrigues equation
    return np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
    )


def expand_task_trajectory(task_trajectory: List[CartesianTrajSegment], dphi: float) -> List[CartesianTrajSegment]:
    """
    Expand the task trajectory by adding equivalent points obtained by applying constant per segment rotation around
    the tool axis.
    :param task_trajectory:
    :param dphi:
    :return:
    """
    # Create the angles around the tool axis that need to be sampled
    samples = ceil(pi / dphi)
    # Start is included but stop is not included
    angles = [(i, dphi * i,) for i in range(-samples + 1, samples) if i != 0]

    # Iterate over all segments
    for segment in task_trajectory:
        # Get the z-axis of the current segment (constant orientation)
        nvec = segment.unmodified_points[0][0:3, 2]
        orig_points = tuple(segment.unmodified_points)

        for angle_idx, angle in angles:
            # Calculate the transformation matrix around the tool axis
            modified_tform = axang2tform(nvec, angle)
            segment.trajectory_points[angle_idx] = [np.dot(tform, modified_tform) for tform in orig_points]

    return task_trajectory


@time_func_call
def check_traj(cmds: List[GCmd], config: List[BaseJoint], limits: Constraints, home: List[float], incs: Increments,
               default_acc: float, urdf: str):
    """
    Validate a trajectory defined by a list of G-code commands.
    :param cmds: List of G-Code command objects.
    :param config: List of joints containing coordinate transformations.
    :param limits: Namedtuple containing all relevant trajectory constraints
    :param home: Home position given as list of joint values
    :param incs: Float value for distance between pose points in mm
    :param default_acc: Float value for the default robot acceleration in mm/s^2
    :param urdf:
    :return: None

    The following checks are done in order:
    1.  Check that all points are within the user-specified task space
        This involves processing commands that change the coordinate system.

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
    task_trajectory = generate_task_trajectory(cmds, start_position, incs.ds, default_acc)
    check_cartesian_limits(task_trajectory, limits.pos_cartesian)

    # Expand task trajectory by additional degree of freedom
    task_trajectory = expand_task_trajectory(task_trajectory, incs.dphi)

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

    print('Creating collision scene...')
    # TODO Create collision scene from task trajectory segments
    all_collision_scenes = [1] * len(joint_traj)

    # Init Matlab Runtime
    collider = MatlabCollisionChecker()
    collisions = collider.check_collisions(home, path=urdf)
    if collisions[0]:
        raise CollisionViolation('Home position is in collision.')
    print('Home position is not in collision.')

    # Calculate the end index of each point
    seg_pt_end = list(np.cumsum([len(seg.solutions) for seg in joint_traj]))
    seg_pt_start = [0] + seg_pt_end[:-1]

    # Check paths sorted by minimum configuration change cost for collisions
    max_iterations = 100
    for _ in range(max_iterations):
        # Find the initially shortest path to be checked for collisions.
        try:
            path = find_path(graph, start_node, stop_node)
        except NoPathError as e:
            raise NoValidPathFound from e

        print(f'Total cost for the current minimum cost path: {path.total_cost}')

        if path.total_cost == float('inf'):
            raise NoValidPathFound('Path cost is infinite (invalid transitions).')

        pt_configurations = [calc_conf_from_node(node_idx, pt_idx) for pt_idx, node_idx in enumerate(path.nodes[1:-1])]
        print(f'Configurations in current shortest path: {set(pt_configurations)}')
        colliding_points = []
        seg_configs = []

        # Iterate over the segments
        for (i, seg), curr_coll_scene in zip(enumerate(joint_traj), all_collision_scenes):
            # Check the current segment for collisions using the relevant slice of all point configurations
            start, end = seg_pt_start[i], seg_pt_end[i]
            curr_seg_conf = list(set(pt_configurations[start:end]))
            if len(curr_seg_conf) > 1:
                raise ValueError('Only common configurations are supported per segment')

            # Calculate the collisions
            colliding_point_idx = get_first_colliding_point(collider, seg, curr_seg_conf[0], curr_coll_scene)
            if i > 0 and colliding_point_idx is not None:
                # Point index is internal to segment and needs to be offset by the end of the previous segment
                colliding_point_idx += seg_pt_end[i - 1]

            # Update the collision array and the config array
            colliding_points.append(colliding_point_idx)
            seg_configs.append(curr_seg_conf[0])

        if all(i is None for i in colliding_points):
            # The configuration list is valid for all segments, no need to calculate the next best path
            print('All segments were collision-free with the current configuration path.')
            break

        # Calculate indices of nodes to be removed
        node_indices = [calc_node_idx(pt, conf) for conf, pt in zip(seg_configs, colliding_points) if pt is not None]

        # Remove the nodes from the graph
        print('Removing first colliding point of each segment in collision.')
        for node in node_indices:
            graph.remove_node(node)

        # Nodes in collision can be removed to query for the next best path.
        print('Querying next-best configuration path.')
    else:
        raise CollisionViolation('No collision-free trajectory could be determined.')

    # Finally, check the joint velocities
    check_joint_velocities(joint_traj, pt_configurations, limits.vel_joint)


def check_joint_velocities(joint_traj: List[JointTrajSegment], config_path: List[int], qdlim: List[float]):
    """
    Check the joint velocities along a trajectory for given robot configurations per segment.
    :param joint_traj: Joint trajectory as list of coherent segments
    :param config_path: List of robot configurations per point
    :param qdlim:
    :return:
    """
    if sum(len(seg.solutions) for seg in joint_traj) != len(config_path):
        raise ValueError('Number of segments is unequal to number of configs.')

    print('Checking joint velocities on selected path..')
    start = 0
    exceeding_indices = []

    error_msg = f"Joint velocity ratio {100 * JOINT_SPEED_ALLOWABLE_RATIO :.2f}% exceeded " \
                f"(max velocities by segment and joint):\n"
    for seg_idx, seg in enumerate(joint_traj):
        end = start + len(seg.solutions)
        current_exceeding = seg.joints_exceeding_velocity_limits(config_path[start:end], qdlim)
        exceeding_indices.append(current_exceeding)
        start += len(seg.solutions)

        if current_exceeding:
            joint_vel_str = [f'J{j_idx}: {vmax :.3f} rad/s' for j_idx, vmax in current_exceeding.items()]
            error_msg += f"Segment #{seg_idx}: {'; '.join(sorted(joint_vel_str))}\n"

    # Check the violations
    if any(exceeding_indices):
        raise JointVelocityViolation(error_msg)
    print('All movements are done within the defined percentage of maximum joint velocity.')


def get_first_colliding_point(collider, seg: JointTrajSegment, seg_conf: int, current_collision_scene) -> Optional[int]:
    """
    Check whether a segment has any collisions
    :param collider:
    :param seg: Segment of a joint trajectory
    :param seg_conf: Configuration for the whole segment
    :param current_collision_scene:
    :return:
    """
    # Get joint coordinates at each point for the determined arm onfiguration
    total_len = len(seg.solutions)
    prefix = f'Checking collisions for segment #{seg.idx} in robot config {seg_conf}...'
    print_progress(0, total_len, prefix=prefix)
    for point_idx, point_solutions in enumerate(seg.solutions):
        print_progress(point_idx + 1, total_len, prefix=prefix)
        joint_values = point_solutions[seg_conf]
        collisions = collider.check_collisions(joint_values)
        if collisions[0]:
            # The current segment is not valid
            print(f'Found collisions for point #{point_idx} on segment #{seg.idx}.')
            return point_idx
    return None


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
        error_msg = ''
        for idx in violation_idx:
            configs = {i for point in joint_trajectory[idx].solutions for i in point.keys()}
            error_msg += f'Segment #{idx}: Points accessible in configurations {configs}\n'
        # TODO Attempt different tool orientation
        raise ConfigurationChangesError(f'Found segments without common configurations:\n{error_msg}')
    print('Each segment can be executed without configuration change.')
    return common_configurations
