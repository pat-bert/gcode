from typing import Union, List, Optional, Tuple

import CollisionChecking
import matlab
import numpy as np


# Checker.terminate()
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
