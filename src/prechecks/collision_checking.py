from typing import Union, List, Optional, Tuple

import src_matlab.CollisionChecking.for_redistribution_files_only.CollisionChecking as CollisionChecking
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
        self.checker.terminate()

    def check_collisions(self, joint_coord: Union[List, np.ndarray], visual: Optional[bool] = False,
                         path: Optional[str] = None) -> Tuple[bool, np.ndarray, np.ndarray]:
        """

        :param joint_coord: Joint coordinates in manufacturer system and in rad
        :param visual:
        :param path: Path to the URDF file
        :return:
        """
        config = matlab.double(list(joint_coord), size=[len(joint_coord), 1])

        if path is not None:
            is_coll, self_coll_pair, world_coll_pair = self.checker.validate_config(config, visual, path, nargout=3)
        else:
            is_coll, self_coll_pair, world_coll_pair = self.checker.validate_config(config, visual, nargout=3)

        if is_coll == -1:
            raise ValueError('Need to supply path at least once to initialize.')

        return is_coll, np.asarray(self_coll_pair, dtype=np.uint64), np.asarray(world_coll_pair, dtype=np.uint64)


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
