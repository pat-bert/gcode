from math import pi, sqrt

import pytest

from AM_IR.ApplicationExceptions import UnknownPlaneError, IllegalAngleError
from AM_IR.Coordinate import Coordinate
from AM_IR.MelfaCoordinateService import Plane
from AM_IR.circle_util import get_angle, get_intermediate_point


@pytest.mark.parametrize("plane,normal_v", [(Plane.XY, None)])
@pytest.mark.parametrize("s,t,c,expected_angle",
                         [
                             # x-axis to y-axis in XY-plane, first quadrant
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
def test_get_angle_standard_planes(plane, normal_v, s, t, c, expected_angle):
    axes = 'XYZ'
    tol = 0.001

    actual_angle = get_angle(Coordinate(s, axes), Coordinate(t, axes), Coordinate(c, axes), plane, normal_vec=normal_v)

    assert actual_angle == pytest.approx(expected_angle, abs=tol)


def test_get_angle_illegal_plane():
    zero = Coordinate((0, 0, 0), 'XYZ')
    with pytest.raises(UnknownPlaneError):
        # noinspection PyTypeChecker
        get_angle(zero, zero, zero, -1, None)


@pytest.mark.parametrize("axes,plane", [('XYZ', Plane.XY), ('YZX', Plane.YZ), ('XZY', Plane.XZ)])
@pytest.mark.parametrize("start,target,center,angle,expected_intermediate",
                         [
                             # -pi/2, XY-Plane (will work in any plane since coordinates are used instead)
                             [(0, 0, 0), (5, 5, 0), (5, 0, 0), -pi / 2, (5 - 0.5 * sqrt(2) * 5, 0.5 * sqrt(2) * 5, 0)],
                             # +3/2*pi, XY-Plane (will work in any plane since coordinates are used instead)
                             [(0, 0, 0), (5, 5, 0), (5, 0, 0), 1.5 * pi, (5 + 2.5 * sqrt(2), -0.5 * sqrt(2) * 5, 0)],
                             # -pi, XY-Plane (requires plane or normal vector)
                             [(0, 0, 0), (10, 0, 0), (5, 0, 0), -pi, (5, 5, 0)],
                             # +pi, XY-Plane (requires plane or normal vector)
                             [(0, 0, 0), (10, 0, 0), (5, 0, 0), +pi, (5, -5, 0)],
                         ]
                         )
def test_get_intermediate_point_standard(axes, plane, start, target, center, angle: float, expected_intermediate):
    """
    Test that for any given arc the correct intermediate point is calculated
    :param start:
    :param target:
    :param center:
    :param angle:
    :param expected_intermediate:
    :return:
    """
    tol = 0.001
    s = Coordinate(start, axes)
    t = Coordinate(target, axes)
    c = Coordinate(center, axes)

    actual_intermediate = get_intermediate_point(angle, s, t, c, plane, None)

    assert list(actual_intermediate.values) == pytest.approx(list(expected_intermediate), abs=tol)


@pytest.mark.parametrize("start,target,center,angle,normal_v,expected_intermediate",
                         [
                             [(-3, -3, -3), (5, 5, 5), (1, 1, 1), -pi, (0, 1, -1), (6.656, -1.828, -1.828)],
                             [(-3, -3, -3), (5, 5, 5), (1, 1, 1), +pi, (0, 1, -1), (-4.656, 3.828, 3.828)],
                             [(-3, -3, -3), (5, 5, 5), (1, 1, 1), -pi, (3, 0, -3), (3.828, -4.656, 3.828)],
                             [(-3, -3, -3), (5, 5, 5), (1, 1, 1), +pi, (3, 0, -3), (-1.828, 6.656, -1.828)],
                             [(0, 0, 0), (10, 0, 0), (5, 0, 0), +pi, (0, 0, 1), (5, -5, 0)],
                         ]
                         )
def test_get_intermediate_point_free_plane(start, target, center, angle: float, normal_v, expected_intermediate):
    """
    Test that for any given arc the correct intermediate point is calculated
    :param start:
    :param target:
    :param center:
    :param angle:
    :param expected_intermediate:
    :return:
    """
    tol = 0.001
    s = Coordinate(start, 'XYZ')
    t = Coordinate(target, 'XYZ')
    c = Coordinate(center, 'XYZ')
    n = Coordinate(normal_v, 'XYZ')

    actual_intermediate = get_intermediate_point(angle, s, t, c, Plane.ANY, n)

    a1 = list(actual_intermediate.values) == pytest.approx(list(expected_intermediate), abs=tol)
    a2 = (s - actual_intermediate).vector_len() == pytest.approx((t - actual_intermediate).vector_len(), abs=tol)
    a3 = (s - c).vector_len() == pytest.approx((actual_intermediate - c).vector_len(), abs=tol)
    assert a1 and a2 and a3


@pytest.mark.parametrize("angle", [3 * pi, -3 * pi])
def test_get_intermediate_point_illegal_angle(angle):
    """
    abs(angle) <= 2*pi is required
    :return:
    """
    s = Coordinate((0, 0, 0), 'XYZ')
    t = Coordinate((5, 5, 0), 'XYZ')
    c = Coordinate((5, 0, 0), 'XYZ')

    with pytest.raises(IllegalAngleError):
        get_intermediate_point(angle, s, t, c, Plane.XY, normal_vec=None)
