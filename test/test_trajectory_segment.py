import numpy as np
import pytest

from src.prechecks.exceptions import CartesianLimitViolation, JointLimitViolation
from src.prechecks.trajectory_segment import get_violated_boundaries, LinearSegment, CircularSegment, \
    JointTrajSegment, check_cartesian_limits, filter_joint_limits


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
        actual_within = get_violated_boundaries(point, boundaries)
        assert (len(actual_within) == 0) == within
    else:
        with pytest.raises(exc):
            get_violated_boundaries(point, boundaries)


class TestLinearSegment:
    @pytest.mark.parametrize("traj,boundaries,within,exc",
                             [
                                 # Start and end within
                                 ([[0], [1], [2], [3]], [0, 3], True, None),
                                 # End outside
                                 ([[0], [1], [2], [3]], [0, 2], False, None),
                                 # Start outside
                                 ([[-1], [0], [1]], [0, 1], False, None),
                                 # Both outside
                                 ([[4], [5]], [0, 3], False, None),
                                 # Only one element but within
                                 ([[0]], [0, 3], True, None),
                                 # Only one element but outside
                                 ([[-1]], [0, 3], False, None),
                                 # Empty initializer
                                 ([], [0, 3], None, ValueError),
                                 # 4x4 matrix
                                 (
                                         [
                                             [
                                                 [0, 0, 0, 0],
                                                 [0, 0, 0, 10],
                                                 [0, 0, 0, -15],
                                                 [0, 0, 0, 0],
                                             ],
                                             [
                                                 [0, 0, 0, -5],
                                                 [0, 0, 0, -2],
                                                 [0, 0, 0, 3],
                                                 [0, 0, 0, 0],
                                             ],
                                         ],
                                         [-5, 0, -2, 10, -15, 3], True, None
                                 ),
                             ]
                             )
    def test_is_within_cartesian_boundaries(self, traj, boundaries, within, exc):
        # Demonstrate this in 1D
        traj = [np.array(point) for point in traj]
        if exc is None:
            linear_segment = LinearSegment(traj)
            actual_within = linear_segment.get_violated_boundaries(boundaries)
            assert (len(actual_within) == 0) == within
        else:
            with pytest.raises(exc):
                LinearSegment(traj)


class TestCircularSegment:
    @pytest.mark.parametrize("traj,boundaries,within,exc",
                             [
                                 # All within
                                 ([[0], [1], [2]], [0, 2], True, None),
                                 # One outside
                                 ([[0], [4], [2]], [0, 3], False, None),
                                 # All outside
                                 ([[4], [5]], [0, 3], False, None),
                                 # Only one element but within
                                 ([[0]], [0, 3], True, None),
                                 # Only one element but outside
                                 ([[-1]], [0, 3], False, None),
                                 # Empty trajectory
                                 ([], [0, 3], None, ValueError),
                                 # 4x4 matrix
                                 (
                                         [
                                             [
                                                 [0, 0, 0, 0],
                                                 [0, 0, 0, 10],
                                                 [0, 0, 0, -15],
                                                 [0, 0, 0, 0],
                                             ],
                                             [
                                                 [0, 0, 0, -5],
                                                 [0, 0, 0, -2],
                                                 [0, 0, 0, 3],
                                                 [0, 0, 0, 0],
                                             ],
                                         ],
                                         [-5, 0, -2, 10, -15, 2], False, None
                                 ),
                             ]
                             )
    def test_is_within_cartesian_boundaries(self, traj, boundaries, within, exc):
        # Demonstrate this in 1D
        traj = [np.array(point) for point in traj]
        if exc is None:
            circular_segment = CircularSegment(traj, np.array([0, 0, 0]))
            actual_within = circular_segment.get_violated_boundaries(boundaries)
            assert (len(actual_within) == 0) == within
        else:
            with pytest.raises(exc):
                CircularSegment(traj, np.array([0, 0, 0]))


valid_solution = [-3, 0, 1, -2, 3, 0.2]
invalid_solution = [-4, 0, 2, -2, 3, 1]
jlimits = [-3, 3, -3, 3, -3, 3, -3, 3, -3, 3, -3, 3]


class TestJointSegment:
    @pytest.mark.parametrize("solutions,exp_within",
                             [
                                 # All within, all common
                                 ([{0: valid_solution, 1: valid_solution}], True),
                                 # Some deleted, all common
                                 ([{0: valid_solution, 1: invalid_solution}], True),
                                 # All deleted, all common
                                 ([{0: invalid_solution, 1: invalid_solution}], False),
                                 # Some deleted, one common
                                 ([{0: invalid_solution, 1: valid_solution}, {2: valid_solution, 1: valid_solution}],
                                  True),
                                 # Some deleted, no common
                                 (
                                         [{0: invalid_solution, 1: valid_solution},
                                          {2: invalid_solution, 3: invalid_solution}],
                                         False
                                 )
                             ]
                             )
    def test_is_within_joint_limits(self, solutions, exp_within):
        jseg = JointTrajSegment(solutions)
        assert jseg.solutions == solutions

        act_within = jseg.is_within_joint_limits(jlimits)
        assert act_within == exp_within

    @pytest.mark.parametrize("solutions,commons",
                             [
                                 # All within, all common
                                 ([{0: valid_solution, 1: valid_solution}], [0, 1]),
                                 # Some deleted, all common
                                 ([{0: valid_solution, 1: invalid_solution}], [0, 1]),
                                 # All deleted, all common
                                 ([{0: invalid_solution, 1: invalid_solution}], [0, 1]),
                                 # Some deleted, one common
                                 ([{0: invalid_solution, 1: valid_solution}, {2: valid_solution, 1: valid_solution}],
                                  [1]),
                                 # Some deleted, no common
                                 (
                                         [{0: invalid_solution, 1: valid_solution},
                                          {2: invalid_solution, 3: invalid_solution}],
                                         []
                                 ),
                                 # Empty from the start
                                 (
                                         [{}], []
                                 )
                             ]
                             )
    def test_get_common_configurations(self, solutions, commons):
        jseg = JointTrajSegment(solutions)
        act_common = jseg.get_common_configurations()
        assert act_common == commons


@pytest.mark.parametrize("points,clim,exc",
                         [
                             (
                                     [-3, 4, 6], [-10, 10], None
                             ),
                             (
                                     [5, 6, 11, 4], [-10, 10], CartesianLimitViolation
                             )
                         ]
                         )
def test_check_cartesian_limits(points, clim, exc):
    points = [np.array([point]) for point in points]
    traj = [LinearSegment(points), CircularSegment(points, np.array([0, 0, 0]))]
    if exc is None:
        check_cartesian_limits(traj, clim)
    else:
        with pytest.raises(exc):
            check_cartesian_limits(traj, clim)


@pytest.mark.parametrize("points,qlim,exc",
                         [
                             (
                                     [{1: [0]}, {2: [10], 3: [4]}], [-1, 5], None
                             ),
                             (
                                     [{1: [0]}, {2: [10], 3: [4]}], [-1, 3], JointLimitViolation
                             )
                         ]
                         )
def test_filter_joint_limits(points, qlim, exc):
    traj = [JointTrajSegment(points), JointTrajSegment(points)]
    if exc is None:
        filter_joint_limits(traj, qlim)
    else:
        with pytest.raises(exc):
            filter_joint_limits(traj, qlim)
