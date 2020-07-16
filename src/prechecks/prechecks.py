from math import ceil, pi
from typing import List

import numpy as np

from prechecks.trajectory_segment import check_joint_velocities
from src.gcode.GCmd import GCmd
from src.kinematics.forward_kinematics import axang2tform
from src.kinematics.forward_kinematics import forward_kinematics
from src.kinematics.joints import BaseJoint
from src.prechecks.collision_checking import MatlabCollisionChecker
from src.prechecks.dataclasses import Constraints, Increments
from src.prechecks.exceptions import CollisionViolation
from src.prechecks.graph_search import create_graph
from src.prechecks.path_finding import get_best_valid_path
from src.prechecks.trajectory_generation import generate_task_trajectory, generate_joint_trajectory
from src.prechecks.trajectory_segment import check_cartesian_limits, filter_joint_limits, CartesianTrajSegment
from src.prechecks.trajectory_segment import check_common_configurations
from src.prechecks.utils import time_func_call


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
    check_common_configurations(joint_traj)

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

    # Get the best path that is valid
    pt_configurations = get_best_valid_path(all_collision_scenes, collider, graph, joint_traj, start_node, stop_node)

    # Finally, check the joint velocities
    check_joint_velocities(joint_traj, pt_configurations, limits.vel_joint)
