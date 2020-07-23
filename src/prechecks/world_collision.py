from typing import List

import numpy as np

from src.prechecks.utils import print_progress
from src.prechecks.trajectory_segment import CartesianTrajSegment
from src.prechecks.dataclasses import Extrusion
from src.prechecks.trajectory_segment import LinearSegment, CircularSegment


def create_cuboid_from_path(line: LinearSegment, nvec: np.ndarray, extr: Extrusion) -> np.ndarray:
    """
    Create vertices of a cuboid for a linear segment.
    :param line: Linear segment
    :param nvec: Normal vector for build direction
    :param extr: Extrusion parameters
    :return: (8x3) array
    """
    upper_mid_start = line.unmodified_points[0][0:3, 3]
    upper_mid_end = line.unmodified_points[-1][0:3, 3]

    # Get the lateral direction
    lateral_vec = np.cross(nvec, upper_mid_end - upper_mid_start)
    try:
        ydir = extr.width * lateral_vec / np.linalg.norm(lateral_vec)
    except ZeroDivisionError:
        raise ValueError('Invalid normal vector. Cross product had zero length.')

    # Get the height vector
    zdir = extr.height * nvec

    cuboid = np.array(
        [
            # Lower left start
            upper_mid_start - zdir + 0.5 * ydir,
            # Lower right start
            upper_mid_start - zdir - 0.5 * ydir,
            # Upper left start
            upper_mid_start + 0.5 * ydir,
            # Upper right start
            upper_mid_start - 0.5 * ydir,
            # Lower left end
            upper_mid_end - zdir + 0.5 * ydir,
            # Lower right end
            upper_mid_end - zdir - 0.5 * ydir,
            # Upper left end
            upper_mid_end + 0.5 * ydir,
            # Upper right end
            upper_mid_end - 0.5 * ydir,
        ]
    )
    return cuboid


def create_vertices_from_arc(arc: CircularSegment, nvec: np.ndarray, extr: Extrusion) -> np.ndarray:
    """
    Create vertices of a circular cuboid.
    :param arc: Circular segment
    :param nvec: Normal vector for build direction
    :param extr: Extrusion parameters
    :return: (nx3) array with n = 4k
    """
    # Get the height vector
    zdir = extr.height * nvec

    vertices = []

    # Iterate over all points in the arc
    for pose in arc.unmodified_points:
        upper_mid = pose[0:3, 3]
        lateral_vec = arc.centre - upper_mid

        try:
            ydir = extr.width * lateral_vec / np.linalg.norm(lateral_vec)
        except ZeroDivisionError:
            raise ValueError('Arc with zero radius detected.')

        # Create the corners of the cross section
        vertices.append(upper_mid - zdir - 0.5 * ydir)
        vertices.append(upper_mid - zdir + 0.5 * ydir)
        vertices.append(upper_mid - 0.5 * ydir)
        vertices.append(upper_mid + 0.5 * ydir)

    return np.array(vertices)


def create_collision_objects(task_trajectory: List[CartesianTrajSegment], extr: Extrusion) -> List[np.ndarray]:
    print('Creating collision objects...')

    # Create individual collision objects
    collision_vertices_list = []
    total_len = len(task_trajectory)

    for seg_idx, seg in enumerate(task_trajectory):
        prefix = f'Creating collision object for segment #{seg_idx} ...'
        print_progress(seg_idx + 1, total_len, prefix=prefix)

        if seg.has_extrusion:
            # Get normal vector from first point (orientation is constant)
            nvec = seg.unmodified_points[0][0:3, 2]
            if isinstance(seg, LinearSegment):
                cuboid = create_cuboid_from_path(seg, nvec, extr)
                collision_vertices_list.append(cuboid)
            elif isinstance(seg, CircularSegment):
                vertices = create_vertices_from_arc(seg, nvec, extr)
                collision_vertices_list.append(vertices)
            else:
                raise ValueError('Unsupported segment for creation of collision object.')
        else:
            cuboid = None
            collision_vertices_list.append(cuboid)

    return collision_vertices_list
