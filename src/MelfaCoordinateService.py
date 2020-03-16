from enum import unique, Enum
from typing import Tuple

from src.Coordinate import Coordinate
from src.protocols.IProtocol import CoordinateAdapter


@unique
class Plane(Enum):
    """
    Define enums for each available plane.
    """

    XY = 1
    XZ = 2
    YZ = 3
    ANY = 4


class MelfaCoordinateService(CoordinateAdapter):
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
    def to_cmd(c: Coordinate, plane: Plane):
        """
        Convert a coordinate to the point format used in R3 protocol
        :param c:
        :param plane:
        :return:
        """
        angles = "ABC"
        values = MelfaCoordinateService.melfa_orientation_plane(plane)
        existing_values = [val for val in c.coordinate.values()]

        for angle, val in zip(angles, values):
            if angle not in c.coordinate.keys():
                existing_values.append(val)

        txt = [
            "{:.{d}f}".format(i, d=c.digits) if i is not None else ""
            for i in existing_values
        ]
        return "(" + ",".join(txt) + ")" + "(7,0)"

    @staticmethod
    def melfa_orientation_plane(plane: Plane) -> Tuple[float]:
        """
        Calculates the angles for the standard planes
        :param plane: Standard planes (XY, XZ, YZ)
        :return:
        """
        # TODO Determine reliable ABC angles
        if plane is Plane.XY:
            return tuple([180.0, 0.0, 0.0])
        elif plane is Plane.XZ:
            raise NotImplementedError
        elif plane is Plane.YZ:
            raise NotImplementedError
