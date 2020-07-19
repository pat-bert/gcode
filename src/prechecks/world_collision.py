import numpy as np

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
