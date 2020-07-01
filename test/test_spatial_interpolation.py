from math import pi, sqrt
from typing import List

import numpy as np
import pytest

from src.kinematics.forward_kinematics import get_tform
from src.prechecks.spatial_interpolation import linear_interpolation, circular_interpolation


@pytest.mark.parametrize("start,end,ds,expected_points",
                         [
                             (
                                     [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                     [[0, 0, 0], [0, 0, 0], [0, 0, 0], [300, 0, 0]],
                                     50,
                                     7
                             ),
                             (
                                     [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                     [[0, 0, 0], [0, 0, 0], [0, 0, 0], [50, 0, 0]],
                                     50,
                                     2
                             )
                         ]
                         )
def test_linear_interpolation(start, end, ds, expected_points):
    # Create the start and end point matrices
    start = get_tform(*start)
    end = get_tform(*end)

    # Calculate the interpolated tforms
    interpolated_tforms = list(linear_interpolation(start, end, ds=ds))
    helper_spatial_interpolation_test(interpolated_tforms, start, end, expected_points)

    # Check that the points are equidistant
    if expected_points > 2:
        for i in range(expected_points - 1):
            ds_actual = np.linalg.norm(interpolated_tforms[i + 1][0:3, 3] - interpolated_tforms[i][0:3, 3])
            assert pytest.approx(ds, rel=0.1) == ds_actual


@pytest.mark.parametrize("start,end,nvec,cw,ds,expected_points",
                         [
                             # XY plane half circle (start, intermediate, end)
                             (
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0]],
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0]],
                                     [0, 0, 1],
                                     True,
                                     pi / 2,
                                     3
                             ),
                             # XY plane half circle (start, end)
                             (
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0]],
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0]],
                                     [0, 0, 1],
                                     True,
                                     pi,
                                     2
                             ),
                             # XY plane half circle (start, end) rounded
                             (
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0]],
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0]],
                                     [0, 0, 1],
                                     True,
                                     pi / 2 * 1.1,
                                     2
                             ),
                             # XY plane half circle (start, end) rounded
                             (
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0]],
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0]],
                                     [0, 0, 1],
                                     False,
                                     pi / 5,
                                     6
                             ),
                             # XY plane 3/4 circle, five points
                             (
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0]],
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, -1, 0]],
                                     [0, 0, 1],
                                     True,
                                     6 / 16 * pi,
                                     5
                             ),
                             # XY plane full circle, five points
                             (
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0]],
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0]],
                                     [0, 0, 1],
                                     False,
                                     2 / 3 * pi,
                                     4
                             ),
                             # YZ plane 3/4 circle, five points
                             (
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, -1, 0]],
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, -1]],
                                     [1, 0, 0],
                                     True,
                                     6 / 16 * pi,
                                     5
                             ),
                             # XY plane half circle (start, end) rounded
                             (
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, -0.5 * sqrt(2), 0.5 * sqrt(2)]],
                                     [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0.5 * sqrt(2), -0.5 * sqrt(2)]],
                                     [0, 1, 1],
                                     False,
                                     pi / 5,
                                     6
                             )
                         ]
                         )
def test_circular_interpolation(start, end, nvec, cw, ds, expected_points):
    # Create the start and end point matrices
    start = get_tform(*start)
    end = get_tform(*end)

    # Calculate the interpolated tforms
    interpolated_tforms = list(circular_interpolation(start, end, [0, 0, 0], nvec, cw, ds=ds))
    print(interpolated_tforms)

    helper_spatial_interpolation_test(interpolated_tforms, start, end, expected_points)

    # Check that the points all have distance of the radius to the center point
    r = np.linalg.norm(start[0:3, 3])
    for tform in interpolated_tforms:
        assert pytest.approx(r, rel=0.01) == np.linalg.norm(tform[0:3, 3])

    # Check that the points are equidistant
    if expected_points > 3:
        ds_straight_line_ref = np.linalg.norm(interpolated_tforms[1][0:3, 3] - interpolated_tforms[0][0:3, 3])
        for i in range(1, expected_points - 1):
            ds_actual = np.linalg.norm(interpolated_tforms[i + 1][0:3, 3] - interpolated_tforms[i][0:3, 3])
            assert pytest.approx(ds_straight_line_ref, rel=0.1) == ds_actual


def helper_spatial_interpolation_test(interpolated_tforms: List[np.ndarray], start, end, expected_points):
    # Test that the number of interpolated points is correct
    assert len(interpolated_tforms) == expected_points

    # Test that the start and end points are included
    np.testing.assert_allclose(interpolated_tforms[0], start)
    np.testing.assert_allclose(interpolated_tforms[-1], end)
