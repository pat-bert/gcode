import pytest

from src.prechecks.exceptions import WorkspaceViolation
from src.cli_commands.check_trajectory import check_trajectory


def test_check_trajectory_out_of_reach():
    with pytest.raises(WorkspaceViolation) as e:
        check_trajectory(config_f='config.ini', gcode_f='joint_violation.gcode')

    assert 'segment #0' in str(e.value)
    assert 'Pos 471' in str(e.value)
