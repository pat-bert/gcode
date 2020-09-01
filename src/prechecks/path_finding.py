import logging
from typing import List

import numpy as np
from dijkstar import find_path, NoPathError

from src.prechecks.utils import print_progress
from src.prechecks.collision_checking import get_first_colliding_point
from src.prechecks.exceptions import CollisionViolation, NoValidPathFound
from src.prechecks.graph_creation import calc_node_idx, calc_conf_from_node
from src.prechecks.trajectory_segment import JointTrajSegment


def get_best_valid_path(collider, graph, joint_traj: List[JointTrajSegment], start_node: int, stop_node: int) \
        -> List[int]:
    """
    Iterate over all paths in order of cost to find the best valid path
    :param collider:
    :param graph:
    :param joint_traj:
    :param start_node: Node index of the start node
    :param stop_node: Node index of the final node
    :return: List of configurations for the best valid path
    :raises: CollisionViolation if all valid paths are in collision
    """
    # Calculate the end index of each point
    seg_pt_end = list(np.cumsum([len(seg.solutions) for seg in joint_traj]))
    seg_pt_start = [0] + seg_pt_end[:-1]

    max_iterations = 100
    for _ in range(max_iterations):
        # Get the configurations of the path with the least cost
        pt_configurations = get_best_path_configs(graph, start_node, stop_node)

        colliding_points = []
        seg_configs = []

        # Iterate over the segments
        total_len = len(joint_traj)
        print('\n')
        for segment_idx, seg in enumerate(joint_traj):
            # Check the current segment for collisions using the relevant slice of all point configurations
            start, end = seg_pt_start[segment_idx], seg_pt_end[segment_idx]
            curr_seg_conf = list(set(pt_configurations[start:end]))
            if len(curr_seg_conf) > 1:
                raise ValueError('Only common configurations are supported per segment')

            # Progress bar
            prefix = f'Checking collisions for segment #{seg.idx} in robot config {curr_seg_conf}...'
            print_progress(segment_idx, total_len, prefix=prefix)

            # Calculate the collisions
            colliding_point_idx = get_first_colliding_point(collider, seg, curr_seg_conf[0])
            if segment_idx > 0 and colliding_point_idx is not None:
                # Point index is internal to segment and needs to be offset by the end of the previous segment
                colliding_point_idx += seg_pt_end[segment_idx - 1]

            # Update the collision array and the config array
            colliding_points.append(colliding_point_idx)
            seg_configs.append(curr_seg_conf[0])

        if all(i is None for i in colliding_points):
            # The configuration list is valid for all segments, no need to calculate the next best path
            logging.info('\nAll segments were collision-free with the current configuration path.')
            break

        # Calculate indices of nodes to be removed
        node_indices = [calc_node_idx(pt, conf) for conf, pt in zip(seg_configs, colliding_points) if pt is not None]

        # Remove the nodes from the graph
        logging.warning('Removing first colliding point of each segment in collision.')
        for node in node_indices:
            graph.remove_node(node)

        # Nodes in collision can be removed to query for the next best path.
        logging.info('Querying next-best configuration path.')
    else:
        raise CollisionViolation('No collision-free trajectory could be determined.')
    return pt_configurations


def get_best_path_configs(graph, start_node: int, stop_node: int) -> List[int]:
    """
    Get the shortest path and calculate the nodes from it. Also print out some helpful information.
    :param graph:
    :param start_node: Node index of the start node
    :param stop_node: Node index of the final node
    :return:
    :raises: NoValidPathFound if no path with finite cost is found
    """
    # Find the initially shortest path to be checked for collisions.
    try:
        print('\nSearching minimum cost path...')
        path = find_path(graph, start_node, stop_node)
    except NoPathError as e:
        raise NoValidPathFound from e
    print(f'=>Total cost for the current minimum cost path: {path.total_cost :.4f}')
    if path.total_cost == float('inf'):
        raise NoValidPathFound('Path cost is infinite (invalid transitions).')
    pt_configurations = [calc_conf_from_node(node_idx, pt_idx) for pt_idx, node_idx in enumerate(path.nodes[1:-1])]
    print(f'=>Configurations in current shortest path: {set(pt_configurations)}')
    return pt_configurations
