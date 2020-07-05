import pytest

from src.cli_commands.check_trajectory import check_trajectory
from src.prechecks.exceptions import WorkspaceViolation, CartesianLimitViolation


def test_check_trajectory_out_of_reach():
    with pytest.raises(WorkspaceViolation) as e:
        check_trajectory(config_f='config.ini', gcode_f='./test/joint_violation.gcode')

    assert 'segment #0' in str(e.value)
    assert 'Pos 471' in str(e.value)


def test_check_trajectory_cartesian_violation():
    with pytest.raises(CartesianLimitViolation) as e:
        check_trajectory(config_f='config.ini', gcode_f='./test/cartesian_violation.gcode')
    assert 'segment #1' in str(e.value).lower()
    assert 'ymax violated' in str(e.value).lower()
