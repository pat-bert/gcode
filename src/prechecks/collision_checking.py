from typing import Union, List, Optional, Tuple

import src_matlab.checkCollision.for_redistribution_files_only.CollisionChecking as CollisionChecking
import matlab
import numpy as np

from src.prechecks.trajectory_segment import JointTrajSegment
from src.prechecks.utils import print_progress


class MatlabCollisionChecker:
    def __init__(self):
        self.checker = CollisionChecking.initialize()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.checker.wait_for_figures_to_close()
        self.checker.terminate()

    def check_collisions(self, joint_coord: Union[List, np.ndarray], visual: Optional[bool] = True,
                         path: Optional[str] = None, collision_objects: Optional[List[np.ndarray]] = None,
                         scene_idx: Optional[int] = 0) -> Tuple[bool, np.ndarray, np.ndarray, np.ndarray]:
        """

        :param joint_coord: Joint coordinates in manufacturer system and in rad
        :param visual: Flag to indicate whether a plot will be created
        :param path: Path to the URDF file
        :param collision_objects:
        :param scene_idx:
        :return:
        """
        config = matlab.double(list(joint_coord), size=[len(joint_coord), 1])

        # Conversion from list of nx3 numpy arrays to matlab
        extr_vertices = []
        if collision_objects is not None:
            for vertices in collision_objects:
                if vertices is None:
                    extr_vertices.append(matlab.double([]))
                else:
                    if vertices.shape[1] != 3:
                        raise ValueError(f'Vertices are not nx3 but {vertices.shape}!')
                    extr_vertices.append(matlab.double(vertices.tolist(), size=vertices.shape))

        # Call Matlab function
        if path is not None:
            a, b, c, d = self.checker.validate_config(config, scene_idx, visual, path, extr_vertices, nargout=4)
        else:
            a, b, c, d = self.checker.validate_config(config, scene_idx, visual, nargout=4)

        # Rename results
        is_coll = a
        self_coll_pair = np.asarray(b, dtype=np.uint64)
        world_coll_pair = np.asarray(c, dtype=np.uint64)
        extr_coll_pair = np.asarray(d, dtype=np.uint64)

        if is_coll == -1:
            raise ValueError('Need to supply path at least once to initialize.')

        return is_coll, self_coll_pair, world_coll_pair, extr_coll_pair


def get_first_colliding_point(collider, seg: JointTrajSegment, seg_conf: int) -> Optional[int]:
    """
    Check whether a segment has any collisions
    :param collider:
    :param seg: Segment of a joint trajectory
    :param seg_conf: Configuration for the whole segment
    :return:
    """
    # Get joint coordinates at each point for the determined arm onfiguration
    for point_idx, point_solutions in enumerate(seg.solutions):
        joint_values = point_solutions[seg_conf]
        # Matlab starts counting from one, so included extrusions are [1,current segment in python indexing]
        collisions = collider.check_collisions(joint_values, scene_idx=seg.idx)
        if collisions[0]:
            # The current segment is not valid
            print(f'Found collisions for point #{point_idx} on segment #{seg.idx}: {collisions[1:]}')
            return point_idx
    return None
