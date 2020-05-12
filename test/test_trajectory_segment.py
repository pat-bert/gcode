import pytest

from prechecks.trajectory_segment import is_point_within_boundaries


@pytest.mark.parametrize("point,boundaries,within,exc",
                         [
                             # Boundary length = 2 * point length, within
                             (
                                     [0.1, -0.1, 0],
                                     3 * [-0.1, 0.1],
                                     True,
                                     None
                             ),
                             # Boundary length = 2 * point length, outside
                             (
                                     [0, 1, -0.1],
                                     3 * [-0.1, 0.1],
                                     False,
                                     None
                             ),
                             # Odd boundary length
                             (
                                     [0, 1, -0.1],
                                     [-0.1, 0.1, 0.3],
                                     False,
                                     ValueError
                             ),
                             # Excess boundaries
                             (
                                     [0, 1, -0.1],
                                     4 * [-0.1, 0.1],
                                     False,
                                     ValueError
                             ),
                             # Less boundaries than possible, within
                             (
                                     [0, 1, -0.1],
                                     [-0.1, 0.1, 1, 2],
                                     True,
                                     None
                             ),
                             # Less boundaries than possible, outside
                             (
                                     [0, 1, -0.1],
                                     [-0.1, 0.1, 1.1, 2],
                                     False,
                                     None
                             )
                         ]
                         )
def test_is_point_within_boundaries(point, boundaries, within, exc):
    if exc is None:
        actual_within = is_point_within_boundaries(point, boundaries)
        assert actual_within == within
    else:
        with pytest.raises(exc):
            is_point_within_boundaries(point, boundaries)
