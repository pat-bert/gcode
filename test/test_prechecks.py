import numpy as np
import pytest

from src.kinematics.forward_kinematics import get_tform
from prechecks.spatial_interpolation import linear_interpolation


@pytest.mark.parametrize("start,end,ds,expected_points",
                         [
                             (
                                     [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
                                     [[0, 0, 0], [0, 0, 0], [0, 0, 0], [300, 0, 0]],
                                     50,
                                     7
                             )
                         ]
                         )
def test_linear_interpolation(start, end, ds, expected_points):
    # Create the start and end point matrices
    start = get_tform(*start)
    end = get_tform(*end)

    # Calculate the interpolated tforms
    interpolated_tforms = list(linear_interpolation(start, end, ds=ds))

    # Test that the number of interpolated points is correct
    assert len(interpolated_tforms) == expected_points

    # Test that the start and end points are included
    np.testing.assert_allclose(interpolated_tforms[0], start)
    np.testing.assert_allclose(interpolated_tforms[-1], end)

    # Test that no point is included twice
    unique_tforms = np.unique(interpolated_tforms, axis=0)
    assert unique_tforms.shape[0] == expected_points
