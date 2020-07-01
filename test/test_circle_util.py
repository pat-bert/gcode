from math import pi, sqrt

import numpy as np
import pytest

from src.ApplicationExceptions import UnknownPlaneError, IllegalAngleError
from src.Coordinate import Coordinate
from src.MelfaCoordinateService import Plane
from src.circle_util import get_angle, get_intermediate_point, RIGHTHAND_AXES

# Rotate axes through, e.g. XYZ->YZX->ZXY
RIGHTHAND_AXES_ROTATED = [RIGHTHAND_AXES[i:] + RIGHTHAND_AXES[:i] for i in range(0, 3)]


@pytest.mark.parametrize("plane,axes",
                         [
                             (Plane.XY, RIGHTHAND_AXES_ROTATED[0]),
                             (Plane.YZ, RIGHTHAND_AXES_ROTATED[1]),
                             (Plane.XZ, RIGHTHAND_AXES_ROTATED[2])])
@pytest.mark.parametrize(
    "s,t,c,expected_angle",
    [
        # x-axis to y-axis in XY-plane, first quadrant
        [(10, 0, 0), (0, 10, 0), (0, 0, 0), pi / 2],
        # Full circle
        [(0, 10, 0), (0, 10, 0), (0, 0, 0), 0],
        [(25, 10, 0), (25, 10, 0), (0, 0, 0), 0],
        # 45 degrees
        [(10, 0, 0), (10, 10, 0), (0, 0, 0), pi / 4],
        # 135 degrees, second quadrant
        [(10, 0, 0), (-10, 10, 0), (0, 0, 0), 3 / 4 * pi],
        # y-axis to x-axis in XY-plane, first quadrant
        [(0, 10, 0), (5, 0, 0), (0, 0, 0), -pi / 2],
        # 180 degrees, second quadrant
        [(10, 0, 0), (-10, +0, 0), (0, 0, 0), pi],
        # -90 degrees, fourth quadrant
        [(10, 0, 0), (0, -10, 0), (0, 0, 0), -pi / 2],
        # -45 degrees, fourth quadrant
        [(10, 0, 0), (10, -10, 0), (0, 0, 0), -pi / 4],
        # -135 degrees, third quadrant
        [(10, 0, 0), (-10, -10, 0), (0, 0, 0), -3 / 4 * pi],
        # Start in second quadrant, finish in third quadrant
        [(10, 10, 0), (-10, 10, 0), (0, 0, 0), pi / 2],
    ],
)
def test_get_angle_standard_planes(plane, s, t, c, expected_angle, axes):
    """
    Test that the function calculates the correct angle on the standard planes.
    :param plane: Planes are rotated through by rotating the passed axes
    :param s:
    :param t:
    :param c:
    :param expected_angle:
    :param axes:
    :return:
    """
    tol = 0.001

    s_sorted = np.array([0.0, 0.0, 0.0])
    t_sorted = np.array([0.0, 0.0, 0.0])
    c_sorted = np.array([0.0, 0.0, 0.0])

    for idx, axis in enumerate(axes):
        if axis == 'X':
            s_sorted[0] = s[idx]
            t_sorted[0] = t[idx]
            c_sorted[0] = c[idx]
        elif axis == 'Y':
            s_sorted[1] = s[idx]
            t_sorted[1] = t[idx]
            c_sorted[1] = c[idx]
        else:
            s_sorted[2] = s[idx]
            t_sorted[2] = t[idx]
            c_sorted[2] = c[idx]

    actual_angle = get_angle(s_sorted, t_sorted, c_sorted, plane, normal_vec=None)

    assert abs(actual_angle) <= pi
    assert actual_angle == pytest.approx(expected_angle, abs=tol)


@pytest.mark.parametrize(
    "s,t,c,n,expected_angle",
    [
        # Simply tilted around x axis
        [(5, 0, 0), (0, 3, 4), (0, 0, 0), (0, -4, 3), pi / 2],
        [(5, 0, 0), (0, -3, -4), (0, 0, 0), (0, -4, 3), -pi / 2],
        [(5, 0, 0), (-5, 0, 0), (0, 0, 0), (0, -4, 3), pi],
        [(5, 0, 0), (5, 0, 0), (0, 0, 0), (0, -4, 3), 0],
    ]
)
def test_get_angle_free_plane(s, t, c, n, expected_angle):
    tol = 0.001

    start = np.array(s)
    target = np.array(t)
    center = np.array(c)
    normal = np.array(n)

    assert np.linalg.norm(start - center) == pytest.approx(np.linalg.norm(target - center), abs=tol)

    actual_angle = get_angle(start, target, center, Plane.ANY, normal_vec=normal, )

    assert abs(actual_angle) <= pi
    assert actual_angle == pytest.approx(expected_angle, abs=tol)


def test_get_angle_illegal_plane():
    """
    Test that an exception is raised if an unknown plane is passed.
    :return:
    """
    zero = Coordinate((0, 0, 0), RIGHTHAND_AXES)
    with pytest.raises(UnknownPlaneError):
        # noinspection PyTypeChecker
        get_angle(zero, zero, zero, -1, None)


@pytest.mark.parametrize("plane,axes",
                         [
                             (Plane.XY, RIGHTHAND_AXES_ROTATED[0]),
                             (Plane.YZ, RIGHTHAND_AXES_ROTATED[1]),
                             (Plane.XZ, RIGHTHAND_AXES_ROTATED[2])
                         ]
                         )
@pytest.mark.parametrize(
    "s,t,c,angle,expected_intermediate",
    [
        # -pi/2, XY-Plane (will work in any plane since coordinates are used instead), clockwise
        [(0, 0, 0), (5, 5, 0), (5, 0, 0), -1 * pi / 2, (5 - 0.5 * sqrt(2) * 5, 0.5 * sqrt(2) * 5, 0)],
        # +3/2*pi, XY-Plane (will work in any plane since coordinates are used instead), counter-clockwise
        [(0, 0, 0), (5, 5, 0), (5, 0, 0), +3 * pi / 2, (5 + 0.5 * sqrt(2) * 5, -0.5 * sqrt(2) * 5, 0)],
        # -pi, XY-Plane (requires plane or normal vector)
        [(0, 0, 0), (10, 0, 0), (5, 0, 0), -pi, (5, 5, 0)],
        # +pi, XY-Plane (requires plane or normal vector)
        [(0, 0, 0), (10, 0, 0), (5, 0, 0), +pi, (5, -5, 0)],
    ],
)
def test_get_intermediate_point_standard(plane, axes, s, t, c, angle: float, expected_intermediate):
    """
    Test that for any given arc the correct intermediate point is calculated
    :param plane:
    :param axes:
    :param s:
    :param t:
    :param c:
    :param angle:
    :param expected_intermediate:
    :return:
    """
    tol = 0.001

    s_sorted = np.array([0.0, 0.0, 0.0])
    t_sorted = np.array([0.0, 0.0, 0.0])
    c_sorted = np.array([0.0, 0.0, 0.0])
    ex_sorted = np.array([0.0, 0.0, 0.0])

    for idx, axis in enumerate(axes):
        if axis == 'X':
            s_sorted[0] = s[idx]
            t_sorted[0] = t[idx]
            c_sorted[0] = c[idx]
            ex_sorted[0] = expected_intermediate[idx]
        elif axis == 'Y':
            s_sorted[1] = s[idx]
            t_sorted[1] = t[idx]
            c_sorted[1] = c[idx]
            ex_sorted[1] = expected_intermediate[idx]
        else:
            s_sorted[2] = s[idx]
            t_sorted[2] = t[idx]
            c_sorted[2] = c[idx]
            ex_sorted[2] = expected_intermediate[idx]

    actual_intermediate = get_intermediate_point(angle, s_sorted, t_sorted, c_sorted, plane, None)

    assert list(actual_intermediate) == pytest.approx(list(ex_sorted), abs=tol)


@pytest.mark.parametrize(
    "start,target,center,angle,normal_v,expected_intermediate",
    [
        [(-3, -3, -3), (5, 5, 5), (1, 1, 1), -pi, (0, 1, -1), (6.656, -1.828, -1.828)],
        [(-3, -3, -3), (5, 5, 5), (1, 1, 1), +pi, (0, 1, -1), (-4.656, 3.828, 3.828)],
        [(-3, -3, -3), (5, 5, 5), (1, 1, 1), -pi, (3, 0, -3), (3.828, -4.656, 3.828)],
        [(-3, -3, -3), (5, 5, 5), (1, 1, 1), +pi, (3, 0, -3), (-1.828, 6.656, -1.828)],
        [(0, 0, 0), (10, 0, 0), (5, 0, 0), +pi, (0, 0, 1), (5, -5, 0)],
    ],
)
def test_get_intermediate_point_free_plane(
        start, target, center, angle: float, normal_v, expected_intermediate
):
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
    s = np.array(start)
    t = np.array(target)
    c = np.array(center)
    n = np.array(normal_v)

    actual_intermediate = get_intermediate_point(angle, s, t, c, Plane.ANY, n)

    assert list(actual_intermediate) == pytest.approx(list(expected_intermediate), abs=tol)
    assert np.linalg.norm(s - actual_intermediate) == pytest.approx(np.linalg.norm(t - actual_intermediate), abs=tol)
    assert np.linalg.norm(s - c) == pytest.approx(np.linalg.norm(actual_intermediate - c), abs=tol)


@pytest.mark.parametrize("angle", [3 * pi, -3 * pi])
def test_get_intermediate_point_illegal_angle(angle):
    """
    abs(angle) <= 2*pi is required
    :return:
    """
    s = np.array([0.0, 0.0, 0.0])
    t = np.array([5, 5, 0])
    c = np.array([5, 0, 0])

    with pytest.raises(IllegalAngleError):
        get_intermediate_point(angle, s, t, c, Plane.XY, normal_vec=None)
