from enum import unique, Enum
from typing import Dict

from src.Coordinate import Coordinate


@unique
class Plane(Enum):
    """
    Define enums for each available plane.
    """

    XY = 1
    XZ = 2
    YZ = 3
    ANY = 4


class MelfaCoordinateService:
    @staticmethod
    def to_melfa_response(c: Coordinate):
        pass

    @staticmethod
    def from_response(melfa_str: str, number_axes: int) -> Coordinate:
        segments = melfa_str.split(";")
        values = [float(i) for i in segments[1: 2 * number_axes: 2]]
        axes = segments[0: 2 * number_axes: 2]
        return Coordinate(values, axes)

    @staticmethod
    def to_cmd(c: Coordinate):
        """
        Convert a coordinate to the point format used in R3 protocol
        :param c:
        :return:
        """
        txt = (
            "{:.{d}f}".format(i, d=c.digits) if i is not None else "" for i in c.values
        )
        return "(" + ",".join(txt) + ")" + "(7,0)"

    @staticmethod
    def melfa_orientation_plane(plane: Plane) -> Dict[str, float]:
        """
        Calculates the angles for the standard planes
        :param plane: Standard planes (XY, XZ, YZ)
        :return:
        """
        # TODO Determine reliable ABC angles
        if plane is Plane.XY:
            return {"A": 180.0, "B": 0.0, "C": 0.0}
        elif plane is Plane.XZ:
            raise NotImplementedError
        elif plane is Plane.YZ:
            raise NotImplementedError
