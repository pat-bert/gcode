import time
from random import uniform
from typing import List, Optional, Iterator, Tuple

import numpy as np
from dijkstar import Graph, NoPathError, find_path
from dijkstar.algorithm import PathInfo

from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joints import BaseJoint
from src.prechecks.collision_checking import MatlabCollisionChecker
from src.prechecks.dataclasses import Constraints
from src.prechecks.trajectory_segment import get_violated_boundaries
from src.prechecks.utils import print_progress

MAX_ITERATIONS = 1000


def is_node_free_and_within(config: List[BaseJoint], collider: MatlabCollisionChecker, jcurr: List[float],
                            clim: List[float]) -> bool:
    """
    Check whether a node is not in collision and within the cartesian boundaries
    :param config: List of joints containing coordinate transformations.
    :param collider: Matlab collision checking interface
    :param jcurr: Joint coordinates for the given node
    :param clim: Cartesian boundaries
    :return: Flag to indicate whether the node is free and within the cartesian boundaries
    """
    # Check cartesian position
    pose = forward_kinematics(config, jcurr)
    cviolation = get_violated_boundaries(pose[0:3, 3], clim)
    if cviolation:
        # Point is outside of allowed cuboid, generate new node
        return False

    # Check node for collisions
    collisions = collider.check_collisions(jcurr, visual=False)
    return not collisions[0]


def generate_rand_free_node(config: List[BaseJoint], collider: MatlabCollisionChecker, clim: List[float],
                            qlim: List[float], q_margin: Optional[float] = 0.1) -> List[float]:
    """
    Create a randomly selected node within the cartesian and joint limits that is not in collision
    :param config: List of joints containing coordinate transformations.
    :param collider: Matlab collision checking interface
    :param clim: Cartesian boundaries
    :param qlim: Joint limits
    :param q_margin:
    :return: List of joint coordinates for a random node
    """
    for _ in range(MAX_ITERATIONS):
        # Generate random node
        jcurr = [uniform(jmin * (1 + q_margin), jmax * (1 - q_margin)) for jmin, jmax in zip(qlim[0::2], qlim[1::2])]

        if is_node_free_and_within(config, collider, jcurr, clim):
            return jcurr
    raise ValueError('Could not generate more nodes.')


def generate_joint_path(j0: List[float], j1: List[float], dj: float) -> Iterator[List[float]]:
    """
    Interpolate a path between joint values
    :param j0: Initial joint coordinates
    :param j1: Final joint coordinates
    :param dj: Joint increment
    :return: Iterator for joint values along the path
    """
    j0_np = np.array(j0)
    j1_np = np.array(j1)

    jdiff = j1_np - j0_np
    n = int(jdiff.max() // dj)

    yield j0_np.tolist()

    for i in range(1, n):
        yield (j0_np + i / n * jdiff).tolist()

    yield j1_np.tolist()


def joint_node_distance(j0: List[float], j1: List[float]) -> float:
    return sum((j1_i - j0_i) ** 2 for j0_i, j1_i in zip(j0, j1))


def create_prm(config: List[BaseJoint], collider: MatlabCollisionChecker, constr: Constraints,
               dj: float, dmax: float, max_neighbors: Optional[int] = 30,
               max_time_s: Optional[int] = 60) -> Tuple[Graph, Tuple[List[float]]]:
    """
    Create a probabilistic road map
    :param config: List of joints containing coordinate transformations.
    :param collider: Matlab collision checking interface
    :param constr: Namedtuple containing all relevant trajectory constraints
    :param dj:
    :param dmax: Maximum allowable distance value
    :param max_neighbors: Maximum number of neighbors to be checked for a connection, defaults to 30.
    :param max_time_s: Time in seconds to be spent on creating a probabilistic roadmap
    :return:
    """
    if dj <= 0:
        raise ValueError('Joint increment must be positive.')

    # Create a new undirected graph and a list of the nodes to store the joint coordinates
    graph = Graph(undirected=True)
    nodes = []

    # Initialize the time
    t0 = time.time()
    current_time = t0
    print('\n')
    prefix = 'Creating probabilistic roadmap ...'

    while current_time - t0 < max_time_s:
        # Update the progress bar
        print_progress(int(current_time - t0), max_time_s, prefix=prefix)

        # Create a new node
        current_node_idx = len(nodes)
        try:
            j0 = generate_rand_free_node(config, collider, constr.pos_cartesian, constr.pos_joint)
        except ValueError:
            # Maximum number of iterations reached, retry if some time is left
            break

        # Log the data of the new node
        graph.add_node(current_node_idx)
        nodes.append(j0)

        # Calculate distance for all nodes
        neighbors = [(joint_node_distance(j0, jn), jn, idx) for idx, jn in enumerate(nodes[:current_node_idx])]

        # Attempt to connect neighbors (smallest distance first)
        is_connected = False
        for distance, jn, n_idx in sorted(neighbors, key=lambda x: x[0]):
            # Stop searching if distance or neigbor count exceeding thresholds
            if (distance > dmax or n_idx >= max_neighbors) and is_connected:
                break

            # Node selection logic
            try:
                # Check connectivity with rest of the graph
                path_info: PathInfo = find_path(graph, current_node_idx, n_idx)
            except NoPathError:
                # Connect neighbors that are not in the same tree
                pass
            else:
                if len(path_info.nodes) <= 3:
                    # Skip nodes that have paths via one intermittent node
                    continue

            # Generate the path and check discrete nodes along it, path does not need to be saved
            path = generate_joint_path(j0, jn, dj=5)
            if all(is_node_free_and_within(config, collider, ji, constr.pos_cartesian) for ji in path):
                # Connect the nodes with edge 1 so that number of intermediate steps can be minimized
                graph.add_edge(current_node_idx, n_idx, edge=1)
                is_connected = True

        # Update time
        current_time = time.time()

    print_progress(max_time_s, max_time_s, prefix=prefix)
    return graph, tuple(nodes)
