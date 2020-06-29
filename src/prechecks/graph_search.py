from collections import namedtuple
from typing import List, Tuple, Optional

from dijkstar import Graph

from src.prechecks.exceptions import JointVelocityViolation, JOINT_SPEED_ALLOWABLE_RATIO
from src.prechecks.trajectory_segment import JointTrajSegment

START_NODE = -1
STOP_NODE = -2

NodeInfo = namedtuple('NodeInfo', 'conf joints seg_idx t')


def joint_limit_cost(joints: List[float], qlim: List[float], w: Optional[List[float]] = None) -> float:
    """
    Measure to drive joints away from their limits.
    :param joints: Joint coordinates to be evaluated
    :param qlim: Joint limits in order [J1 min, J1 max, J2 min, J2 max, Jn min, Jn max]
    :param w: Weights for the individual joints.
    :return: Non-negative cost value for the given joint coordinates. Best is zero.

    [1] B. Siciliano, L. Sciavicco, L. Villani und G. Oriolo, Robotics : Modelling, Planning and Control, London:
    Springer, 2009.
    """
    val = 0
    if w is None:
        for jmin, jmax, j in zip(qlim[::2], qlim[1::2], joints):
            # Use distance from mid-point relative to total range
            val += ((j - 0.5 * (jmin + jmax)) / (jmax - jmin)) ** 2
    else:
        if len(w) != len(joints):
            raise ValueError('Need to supply as many weight factors as joint coordinates.')

        for jmin, jmax, j, jw in zip(qlim[::2], qlim[1::2], joints, w):
            val += jw * ((j - 0.5 * (jmin + jmax)) / (jmax - jmin)) ** 2

    # Normalize with regard to number of joints
    return val / (2 * len(joints))


def joint_velocity_cost(prev_j: List[float], curr_j: List[float], qdlim: List[float], dt: float,
                        w: Optional[List[float]] = None) -> float:
    """
    Measure to penalize large joint velocities.
    :param prev_j: Joint coordinates of the previous node
    :param curr_j: Joint coordinate of the current node
    :param qdlim: Maximum joint velocities in order
    :param dt: Time Delta between the points in seconds
    :param w: Weights for the individual joints.
    :return: Non-negative cost value for the given joint coordinates. Best is zero.
    """
    if w is not None:
        if len(w) != len(curr_j):
            raise ValueError('Need to supply as many weight factors as joint coordinates.')
    else:
        w = [1] * len(curr_j)

    val = 0
    for jprev, jcurr, j_vel_max, jw in zip(prev_j, curr_j, qdlim, w):
        velocity_ratio = (jcurr - jprev) / (dt * j_vel_max)
        if velocity_ratio >= JOINT_SPEED_ALLOWABLE_RATIO:
            # Notify calling function about invalid connection
            raise JointVelocityViolation
        val += jw * velocity_ratio ** 2

    # Normalize with regard to number of joints
    return val / (2 * len(curr_j))


def singularity_proximity(curr_j: List[float]) -> float:
    # TODO Apply additional cost depending on proximity to singularitites
    return 0.0


def calc_cost(curr: NodeInfo, prev: NodeInfo, qlim: List[float], qdlim: List[float]) -> float:
    """
    Calculate the edge cost between two nodes.
    :param curr: Info about the current node
    :param prev: Info about the previous node
    :param qlim: Joint limits in order [J1 min, J1 max, J2 min, J2 max, Jn min, Jn max]
    :param qdlim: Maximum joint velocities in order
    :return: Non-negative cost value for the given joint coordinates. Best is zero.
    """
    # Check whether the configuration changes
    if curr.conf != prev.conf:
        if curr.seg_idx == prev.seg_idx:
            # Currently, configuration changes within a segment are not allowed.
            return float('Inf')

        # Currently, configuration changes between segments are not allowed either.
        # TODO Study different configuration changes, apply penalty instead.
        return float('Inf')

    # Common configurations are fine
    cost = joint_limit_cost(curr.joints, qlim)

    # Proportional to joint delta
    cost += joint_velocity_cost(prev.joints, curr.joints, qdlim, dt=curr.t - prev.t)

    # Cost with regard to singularity proximity
    cost += singularity_proximity(curr.joints)

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


def calc_conf_from_node(node_idx, point_idx) -> int:
    """
    Calculates the configuration from a node index
    :param node_idx: Integer value for the node index
    :param point_idx: Index of the point that the node belongs to within the trajectory.
    :return: Integer value for the robot configuration of the node
    """
    if point_idx >= 0:
        return node_idx - 8 * point_idx
    raise ValueError('Point index must be positive.')


def create_graph(joint_traj: List[JointTrajSegment], qlim: List[float], qdlim: List[float]) \
        -> Tuple[Graph, int, int]:
    """
    Constructs a graph for a given joint trajectory.
    The graph is unidirectional and has one common start and one common end node.
    The graph consists of n layers with n being the total count of points in the joint trajectory.
    The nodes within a layer are not connected but the nodes of adjacent layers are all connected initially.
    For each point a node is created for each viable robot configuration/joint solution.
    :param joint_traj: List of JointTrajectorySegments
    :param qlim: Joint limits in order [J1 min, J1 max, J2 min, J2 max, Jn min, Jn max]
    :param qdlim: Maximum joint velocities in order
    :return: Graph that can be used to determine shortest paths, start node, stop node.
    """
    joint_network = Graph()
    current_parents = {}
    prev_seg_idx = 0
    t_prev_point = 0
    total_point_count = sum((len(segment.solutions) for segment in joint_traj))

    point_idx = 0
    # Iterate over all segments
    for seg_idx, joint_segment in enumerate(joint_traj):
        # Iterate over points in task space and corresponding points in time per segment
        for ik_solutions_point, t_curr_point in zip(joint_segment.solutions, joint_segment.time_points):
            # Iterate over all nodes of the current point
            for curr_conf, curr_j in ik_solutions_point.items():
                node_idx = calculate_node_idx(point_idx, curr_conf)
                if point_idx == 0:
                    # First point nodes are all connected to start node (zero cost) and do not have distinct parent
                    # nodes.
                    joint_network.add_edge(START_NODE, node_idx, edge=0)
                else:
                    # Following point nodes are connected to all nodes of the previous point
                    for prev_conf, prev_j in current_parents.items():
                        # Calculate the index of the previous node
                        previous_node_idx = calculate_node_idx(point_idx - 1, prev_conf)
                        # Call a cost function to determine the transition cost between the nodes based on the robot
                        # configurations and the joint values.
                        curr_node_info = NodeInfo(conf=curr_conf, joints=curr_j, seg_idx=seg_idx, t=t_curr_point)
                        prev_node_info = NodeInfo(conf=prev_conf, joints=prev_j, seg_idx=prev_seg_idx, t=t_prev_point)
                        cost = calc_cost(curr_node_info, prev_node_info, qlim, qdlim)
                        # Add the edge to the graph
                        joint_network.add_edge(previous_node_idx, node_idx, edge=cost)

            # Connect all last point nodes to a common last point (zero cost). This is useful to find the shortest path
            # simply as path from START_NODE to STOP_NODE. The nodes are connected with zero cost.
            if point_idx >= total_point_count - 1:
                for curr_conf in ik_solutions_point.keys():
                    node_idx = calculate_node_idx(point_idx, curr_conf)
                    joint_network.add_edge(node_idx, STOP_NODE, edge=0)

            # Move forward to the next point and save the nodes of the current point as parents for the next point
            current_parents = ik_solutions_point
            t_prev_point = t_curr_point
            point_idx += 1

        # Move forward to the next segment
        prev_seg_idx = seg_idx

    return joint_network, START_NODE, STOP_NODE,
