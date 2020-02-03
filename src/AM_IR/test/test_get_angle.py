from math import pi

import pytest

from AM_IR.ApplicationExceptions import UnknownPlaneError
from AM_IR.Coordinate import Coordinate
from AM_IR.MelfaCoordinateService import Plane
from AM_IR.circle_util import get_angle


@pytest.mark.parametrize("s,t,c,expected_angle",
                         [
                             # a-axis to y-axis in XY-plane, first quadrant
                             [(10, 0, 0), (0, 10, 0), (0, 0, 0), pi / 2],
                             # 45 degrees
                             [(10, 0, 0), (10, 10, 0), (0, 0, 0), pi / 4],
                             # y-axis to a-axis in XY-plane, first quadrant
                             [(0, 10, 0), (5, 0, 0), (0, 0, 0), -pi / 2],
                             # 180 degrees, second quadrant
                             [(10, 0, 0), (-10, +0, 0), (0, 0, 0), pi],
                             # -90 degrees, fourth quadrant
                             [(10, 0, 0), (0, -10, 0), (0, 0, 0), -pi / 2],
                             # -90 degrees, third quadrant
                             [(10, 0, 0), (-10, -10, 0), (0, 0, 0), -0.75 * pi],
                             # Start in second quadrant, finish in third quadrant
                             [(10, 10, 0), (-10, 10, 0), (0, 0, 0), pi / 2],
                         ]
                         )
def test_get_angle_standard_planes(s, t, c, expected_angle):
    axes = 'XYZ'
    tol = 0.001
    actual_angle = get_angle(Coordinate(s, axes), Coordinate(t, axes), Coordinate(c, axes), Plane.XY, None)

    assert actual_angle == pytest.approx(expected_angle, abs=tol)


def test_get_angle_illegal_plane():
    zero = Coordinate((0, 0, 0), 'XYZ')
    with pytest.raises(UnknownPlaneError):
        # noinspection PyTypeChecker
        get_angle(zero, zero, zero, -1, None)
