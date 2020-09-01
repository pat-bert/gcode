import os
from pathlib import Path

import pytest

from src.cli_commands.check_trajectory import check_trajectory
from src.prechecks.exceptions import WorkspaceViolation, CartesianLimitViolation, ConfigurationChangesError


def make_paths_absolute(*f):
    current_file_path = Path(os.path.dirname(os.path.realpath(__file__)))
    return [str(Path(current_file_path, file)) for file in f]


def test_check_trajectory_out_of_reach():
    conf_f, gcode_f = make_paths_absolute('./config.ini', './joint_violation.gcode')
    with pytest.raises(WorkspaceViolation) as e:
        check_trajectory(config_f=conf_f, gcode_f=gcode_f)

    assert 'segment #0' in str(e.value).lower()
    assert 'pos 471' in str(e.value).lower()


def test_check_trajectory_pass_through_singularity():
    conf_f, gcode_f = make_paths_absolute('./config.ini', './pass_through_singularity.gcode')
    with pytest.raises(ConfigurationChangesError) as e:
        check_trajectory(config_f=conf_f, gcode_f=gcode_f)

    assert 'segment #1' in str(e.value).lower()
    assert 'segment #2' in str(e.value).lower()
    assert '{6, 7}' in str(e.value).lower()


def test_check_trajectory_cartesian_violation():
    conf_f, gcode_f = make_paths_absolute('./config.ini', './cartesian_violation.gcode')
    with pytest.raises(CartesianLimitViolation) as e:
        check_trajectory(config_f=conf_f, gcode_f=gcode_f)
    assert 'segment #1' in str(e.value).lower()
    assert 'ymax zmin violated' in str(e.value).lower()


def test_check_trajectory_cartesian_violation_circle():
    conf_f, gcode_f = make_paths_absolute('./config.ini', './cartesian_violation_circle.gcode')
    with pytest.raises(CartesianLimitViolation) as e:
        check_trajectory(config_f=conf_f, gcode_f=gcode_f)
    assert 'segment #1' in str(e.value).lower()
    assert 'ymax violated' in str(e.value).lower()
