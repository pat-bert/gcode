from math import pi

import numpy as np
import pytest

from src.GCmd import GCmd
from src.kinematics.forward_kinematics import get_tform
from src.prechecks.gcode2segment import lin_segment_from_gcode, circ_segment_from_gcode

CURRENT_POSE = get_tform([1, 0, 0], [0, 1, 0], [0, 0, 1], [50, 2, 3])
DEFAULT_VELOCITY = 100
DEFAULT_ACCELERATION = 500


@pytest.mark.parametrize("cmd,ds,is_absolute,curr_vel,curr_acc,ex_points,target",
                         [
                             ('G02 X52 I1', pi / 4, True, DEFAULT_VELOCITY, DEFAULT_ACCELERATION, 5, [52, 2, 3]),
                             ('G02 X2 I1', pi / 4, False, DEFAULT_VELOCITY, DEFAULT_ACCELERATION, 5, [52, 2, 3]),
                         ]
                         )
def test_circular_segment_from_gcode(cmd, ds, is_absolute, curr_vel, curr_acc, ex_points, target):
    gcode = GCmd.read_cmd_str(cmd)
    circ_segment = circ_segment_from_gcode(gcode, CURRENT_POSE, ds, is_absolute, curr_vel, curr_acc)
    common_properties(curr_acc, curr_vel, ds, ex_points, circ_segment, target)


@pytest.mark.parametrize("cmd,ds,is_absolute,curr_vel,curr_acc,ex_points,target",
                         [
                             # Relative movement
                             ('G01 X100', 1, False, DEFAULT_VELOCITY, DEFAULT_ACCELERATION, 101, [150, 2, 3]),
                             ('G01 X100', 1, False, None, DEFAULT_ACCELERATION, 101, [150, 2, 3]),
                             # Absolute movement
                             ('G01 X100', 1, True, DEFAULT_VELOCITY, DEFAULT_ACCELERATION, 51, [100, 2, 3]),
                             # Absolute movement, speed changed
                             ('G01 X100 F100', 1, True, DEFAULT_VELOCITY, DEFAULT_ACCELERATION, 51, [100, 2, 3]),
                         ]
                         )
def test_linear_segment_from_gcode(cmd, ds, is_absolute, curr_vel, curr_acc, ex_points, target):
    gcode = GCmd.read_cmd_str(cmd)
    lin_segment = lin_segment_from_gcode(gcode, CURRENT_POSE, ds, is_absolute, curr_vel, curr_acc)
    common_properties(curr_acc, curr_vel, ds, ex_points, lin_segment, target)


def common_properties(curr_acc, curr_vel, ds, ex_points, segment, target):
    assert segment.ds == ds
    assert len(segment.unmodified_points) == ex_points
    if curr_vel is not None and curr_acc is not None:
        assert len(segment.time_points) == len(segment.unmodified_points)
        # Time must be monotonically increasing
        assert all(segment.time_points[i] - segment.time_points[i - 1] > 0 for i in
                   range(1, len(segment.time_points)))
    else:
        assert segment.time_points is None
    assert segment.s_total == ds * (ex_points - 1)

    if target is not None:
        np.testing.assert_allclose(segment.target[0:3, 3], target)
