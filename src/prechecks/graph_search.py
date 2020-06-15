from typing import List, Tuple

from dijkstar import Graph

from src.prechecks.trajectory_segment import JointTrajectorySegment

START_NODE = -1
STOP_NODE = -2


def calc_cost(curr_conf, curr_joints, prev_conf, prev_joints, curr_seg_idx=None, prev_seg_idx=None):
    # Check whether the configuration changes
    if curr_conf != prev_conf:
        if curr_seg_idx == prev_seg_idx:
            # Currently, configuration changes within a segment are not allowed.
            return float('Inf')
        else:
            # Currently, configuration changes between segments are not allowed.
            # TODO Study different configuration changes, apply penalty instead.
            return float('Inf')
    else:
        # Common configurations are fine
        cost = 1

        # TODO Apply additional cost depending on proximity to joint limits, singularitites or high velocities
        return cost


def calculate_node_idx(point_idx, configuration) -> int:
    """
    Calculates a unique node index.
    :param point_idx: Index of the point that the node belongs to within the trajectory.
    :param configuration: Robot configuration for the node
    :return: Integer value for the node index
    """
    if 0 <= configuration <= 7:
        if point_idx >= 0:
            return 8 * point_idx + configuration
        raise ValueError('Point index must be positive.')
    raise ValueError('Only configurations from 0-7 are allowed.')


def create_graph(joint_traj: List[JointTrajectorySegment]) -> Tuple[Graph, int, int]:
    """
    Constructs a graph for a given joint trajectory.
    The graph is unidirectional and has one common start and one common end node.
    The graph consists of n layers with n being the total count of points in the joint trajectory.
    The nodes within a layer are not connected but the nodes of adjacent layers are all connected initially.
    For each point a node is created for each viable robot configuration/joint solution.
    :param joint_traj: List of JointTrajectorySegments
    :return: Graph that can be used to determine shortest paths, start node, stop node.
    """
    joint_network = Graph()
    current_parents = {}
    total_point_count = sum((len(segment.solutions) for segment in joint_traj))

    point_idx = 0
    for segment in joint_traj:
        for point_nodes in segment.solutions:
            for current_conf, current_joints in point_nodes.items():
                # Iterate over all nodes of the current point
                node_idx = calculate_node_idx(point_idx, current_conf)
                if point_idx == 0:
                    # First point nodes are all connected to start node (zero cost) and do not have distinct parent
                    # nodes.
                    joint_network.add_edge(START_NODE, node_idx, edge=0)
                else:
                    # Following point nodes are connected to all nodes of the previous point
                    for previous_conf, previous_joints in current_parents.items():
                        # Calculate the index of the previous node
                        previous_node_idx = calculate_node_idx(point_idx - 1, previous_conf)
                        # Call a cost function to determine the transition cost between the nodes based on the robot
                        # configurations and the joint values.
                        cost = calc_cost(current_conf, current_joints, previous_conf, previous_joints)
                        # Add the edge to the graph
                        joint_network.add_edge(previous_node_idx, node_idx, edge=cost)

            # Connect all last point nodes to a common last point (zero cost). This is useful to find the shortest path
            # simply as path from START_NODE to STOP_NODE. The nodes are connected with zero cost.
            if point_idx >= total_point_count - 1:
                for current_conf in point_nodes.keys():
                    node_idx = calculate_node_idx(point_idx, current_conf)
                    joint_network.add_edge(node_idx, STOP_NODE, edge=0)

            # Move forward to the next point and save the nodes of the current point as parents for the next point
            current_parents = point_nodes
            point_idx += 1

    return joint_network, START_NODE, STOP_NODE,
