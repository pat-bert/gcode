import unittest.mock as mock
from unittest.mock import MagicMock

import pytest

from printer_components.MelfaRobot import MelfaRobot, IllegalAxesCount


@pytest.fixture
def tcp():
    return MagicMock()


@pytest.fixture
def safe_robot(tcp):
    return MelfaRobot(tcp, safe_return=True)


@pytest.fixture
def no_safe_robot(tcp):
    return MelfaRobot(tcp, safe_return=False)


class TestMelfaRobot:
    def test_axes_init(self, tcp):
        with pytest.raises(IllegalAxesCount):
            MelfaRobot(tcp, number_axes=0)

        a = MelfaRobot(tcp, number_axes=1)
        assert a.joints == ['J1']

        b = MelfaRobot(tcp, number_axes=2)
        assert b.joints == ['J1', 'J2']

    def test_boot_no_safe(self, no_safe_robot):
        with mock.patch('printer_components.MelfaRobot.MelfaRobot.go_safe_pos', autospec=True) as mock_func:
            no_safe_robot.boot()
        assert not mock_func.called

    def test_boot_safe(self, safe_robot):
        with mock.patch('printer_components.MelfaRobot.MelfaRobot.go_safe_pos', autospec=True) as mock_func:
            safe_robot.boot()
        assert mock_func.called

    def test_shutdown_no_safe(self, no_safe_robot):
        with mock.patch('printer_components.MelfaRobot.MelfaRobot.go_safe_pos', autospec=True) as mock_func:
            no_safe_robot.shutdown()
        assert not mock_func.called

    def test_shutdown_safe(self, safe_robot):
        with mock.patch('printer_components.MelfaRobot.MelfaRobot.go_safe_pos', autospec=True) as mock_func:
            safe_robot.shutdown()
        assert mock_func.called

    def test_activate_work_coordinate(self):
        assert False

    def test_handle_gcode(self):
        assert False

    def test__prepare_circle(self):
        assert False

    def test_maintenance(self):
        assert False

    def test__change_communication_state(self):
        assert False

    def test__change_servo_state(self):
        assert False

    def test_read_parameter(self):
        assert False

    def test_set_speed(self):
        assert False

    def test_reset_linear_speed_factor(self):
        assert False

    def test_go_home(self):
        assert False

    def test_go_safe_pos(self):
        assert False

    def test_linear_move_poll(self):
        assert False

    def test_circular_move_poll(self):
        assert False

    def test_set_global_positions(self):
        assert False

    def test_get_pos(self):
        assert False

    def test__check_speed_threshold(self):
        assert False

    def test__set_ovrd(self):
        assert False

    def test__get_ovrd_speed(self):
        assert False

    def test_wait(self):
        assert False

    def test__zero(self):
        assert False
