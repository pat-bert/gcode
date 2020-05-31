from math import pi
from typing import Union, List, Optional

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
                         path: Optional[str] = None):
        """

        :param joint_coord:
        :param visual:
        :param path:
        :return:
        """
        config = matlab.double(list(joint_coord), size=[len(joint_coord), 1])

        if path is not None:
            is_coll, self_coll_pair, world_coll_pair = self.checker.validate_config(config, visual, path, nargout=3)
        else:
            is_coll, self_coll_pair, world_coll_pair = self.checker.validate_config(config, nargout=3)

        if is_coll == -1:
            raise ValueError('Need to supply path at least once to initialize.')

        return is_coll, np.asarray(self_coll_pair, dtype=np.uint64), np.asarray(world_coll_pair, dtype=np.uint64)


if __name__ == '__main__':
    with MatlabCollisionChecker() as col:
        try:
            col.check_collisions([0, 3, 4], path=None)
        except ValueError:
            # Expected
            pass
        urdf_path = r'D:\Nutzer\Documents\PycharmProjects\gcode\ressource\robot.urdf'
        import time

        r = list()
        start = time.process_time()
        for i in range(1, 100):
            r.append(col.check_collisions([-pi / 2, 0, pi / 2, 0, pi / 2, 0 + i * pi / 100], path=urdf_path))
        print(time.process_time() - start)
        print(r)
