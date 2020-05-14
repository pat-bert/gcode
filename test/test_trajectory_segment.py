import numpy as np
import pytest

from src.prechecks.trajectory_segment import is_point_within_boundaries, LinearSegment, CircularSegment


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


class TestLinearSegment:
    @pytest.mark.parametrize("traj,boundaries,within,exc",
                             [
                                 # Start and end within
                                 (range(0, 4), [0, 3], True, None),
                                 # End outside
                                 (range(0, 5), [0, 3], False, None),
                                 # Start outside
                                 (range(-1, 4), [0, 3], False, None),
                                 # Both outside
                                 (range(4, 6), [0, 3], False, None),
                                 # Only one element but within
                                 ([0], [0, 3], True, None),
                                 # Only one element but outside
                                 ([-1], [0, 3], False, None),
                                 # Empty initializer
                                 ([], [0, 3], None, ValueError)
                             ]
                             )
    def test_is_within_cartesian_boundaries(self, traj, boundaries, within, exc):
        # Demonstrate this in 1D
        traj = [np.array([point]) for point in traj]
        if exc is None:
            linear_segment = LinearSegment(traj)
            actual_within = linear_segment.is_within_cartesian_boundaries(boundaries)
            assert actual_within == within
        else:
            with pytest.raises(exc):
                LinearSegment(traj)


class TestCircularSegment:
    @pytest.mark.parametrize("traj,boundaries,within,exc",
                             [
                                 # All within
                                 (range(0, 4), [0, 3], True, None),
                                 # One outside
                                 ([0, 4, 2], [0, 3], False, None),
                                 # All outside
                                 (range(4, 6), [0, 3], False, None),
                                 # Only one element but within
                                 ([0], [0, 3], True, None),
                                 # Only one element but outside
                                 ([-1], [0, 3], False, None),
                                 # Empty trajectory
                                 ([], [0, 3], None, ValueError)
                             ]
                             )
    def test_is_within_cartesian_boundaries(self, traj, boundaries, within, exc):
        # Demonstrate this in 1D
        traj = [np.array([point]) for point in traj]
        if exc is None:
            circular_segment = CircularSegment(traj)
            actual_within = circular_segment.is_within_cartesian_boundaries(boundaries)
            assert actual_within == within
        else:
            with pytest.raises(exc):
                CircularSegment(traj)
