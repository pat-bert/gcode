from enum import unique, Enum
from typing import *

from AM_IR.Coordinate import Coordinate


@unique
class Plane(Enum):
    XY = 1
    XZ = 2
    YZ = 3


class MelfaCoordinateService:
    @staticmethod
    def to_melfa_response(c: Coordinate):
        pass

    @staticmethod
    def from_melfa_response(melfa_str: str, number_axes: int):
        segments = melfa_str.split(';')
        values = [float(i) for i in segments[1:2 * number_axes:2]]
        axes = segments[0:2 * number_axes:2]
        return Coordinate(values, axes)

    @staticmethod
    def to_melfa_point(c: Coordinate, plane: Plane):
        angles = 'ABC'
        values = MelfaCoordinateService.melfa_orientation_plane(plane)
        for angle, val in zip(angles, values):
            if angle not in c.coordinate.keys():
                c.coordinate[angle] = val

        txt = ['{:.{d}f}'.format(i, d=c.digits) if i is not None else '' for i in c.coordinate.values()]
        return '(' + ','.join(txt) + ')' + '(7,0)'

    @staticmethod
    def melfa_orientation_plane(plane: Plane) -> Tuple[float]:
        # TODO Determine reliable ABC angles
        if plane is Plane.XY:
            return tuple([180.0, 0.0, 0.0])
        elif plane is Plane.XZ:
            raise NotImplementedError
        elif plane is Plane.YZ:
            raise NotImplementedError
