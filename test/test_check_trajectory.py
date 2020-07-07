import pytest

from src.cli_commands.check_trajectory import check_trajectory
from src.prechecks.exceptions import WorkspaceViolation, CartesianLimitViolation, ConfigurationChangesError


def test_check_trajectory_out_of_reach():
    with pytest.raises(WorkspaceViolation) as e:
        check_trajectory(config_f='./test/config.ini', gcode_f='./test/joint_violation.gcode')

    assert 'segment #0' in str(e.value).lower()
    assert 'pos 471' in str(e.value).lower()


def test_check_trajectory_pass_through_singularity():
    with pytest.raises(ConfigurationChangesError) as e:
        check_trajectory(config_f='./test/config.ini', gcode_f='./test/pass_through_singularity.gcode')

    assert 'segment #1' in str(e.value).lower()
    assert 'segment #2' in str(e.value).lower()
    assert '{6, 7}' in str(e.value).lower()


def test_check_trajectory_cartesian_violation():
    with pytest.raises(CartesianLimitViolation) as e:
        check_trajectory(config_f='./test/config.ini', gcode_f='./test/cartesian_violation.gcode')
    assert 'segment #1' in str(e.value).lower()
    assert 'ymax zmin violated' in str(e.value).lower()


def test_check_trajectory_cartesian_violation_circle():
    with pytest.raises(CartesianLimitViolation) as e:
        check_trajectory(config_f='./test/config.ini', gcode_f='./test/cartesian_violation_circle.gcode')
    assert 'segment #1' in str(e.value).lower()
    assert 'ymax violated' in str(e.value).lower()
