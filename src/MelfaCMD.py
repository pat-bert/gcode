from math import ceil
import numpy as np
from BaseCmd import BaseCmd
from typing import *
from Coordinate import Coordinate


class MelfaCMD(BaseCmd):
    """
    This class implements a command for the Mitsubishi Melfa series.
    """
    COMMENT = ''
    SUPPORTED_CMDS = [
        'MVS', 'MOV'
    ]

    def __init__(self, code_id: str):
        self.id = code_id

        # Validate input
        if not self._is_valid():
            raise ValueError('Unsupported or unknown command passed: ' + self.id)

    def _is_valid(self) -> bool:
        return self.id in self.SUPPORTED_CMDS

    def __str__(self):
        pass

    @classmethod
    def read_cmd_str(cls, command_str) -> Union['MelfaCMD', None]:
        return cls('0')


class InterpolationMov:
    DIR_CLOCKWISE = 0
    DIR_COUNTERCLOCKWISE = 1

    @staticmethod
    def mvs(start: Coordinate, target: Coordinate, velocity: float, delta_t: float = 7.1e-3) -> List[Coordinate]:
        """
        Linear interpolation from start coordinates to target coordinates
        :param start: Start coordinates
        :param target: Target coordinates
        :param velocity: Speed for motion
        :param delta_t: Time interval
        :return: List of coordinates
        """
        vector = target - start
        distance = vector.vector_len()
        t_total = distance / velocity
        count_segments = ceil(t_total / delta_t)
        inc_vector = vector / count_segments
        return [start + i * inc_vector for i in range(count_segments)]

    @classmethod
    def mv_crl(cls, start: Coordinate, target: Coordinate, offset: Coordinate, velocity: float, delta_t: float = 7.1e-3,
               mv_dir=DIR_CLOCKWISE) -> List[Coordinate]:
        """
        Circular interpolation from start coordinates to target coordinates
        :param start: Start coordinates
        :param target: Target coordinates
        :param offset: Centre offset
        :param velocity: Speed for motion
        :param delta_t: Time interval
        :param mv_dir: Direction of circular movement
        :return: List of coordinates
        """
        centre = start + offset
        cs = offset
        ct = centre - target

        if cs.vector_len() != ct.vector_len():
            raise ValueError("Centre of arc is not equidistant from initial and final position.")
        else:
            radius = cs
            # Get angle between vectors
            phi = np.rad2deg(cs.vector_angle_rad(ct))
            # Get correct angle for direction
            if mv_dir == cls.DIR_CLOCKWISE:
                phi = 360 - phi
            # Get phi increments
            arc_len = phi * radius
            t_total = arc_len / velocity
            count_segments = ceil(t_total / delta_t)
            delta_phi = phi / count_segments
            # Generate intermediate positions
