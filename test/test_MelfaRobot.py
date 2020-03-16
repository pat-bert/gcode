import unittest.mock as mock
from unittest.mock import MagicMock

import pytest

from src.Coordinate import Coordinate
from src.gcode.GCmd import GCmd
from src.ApplicationExceptions import MelfaBaseException
import src.protocols.R3Protocol as R3Protocol
from src.clients.TcpClientR3 import TcpClientR3
from src.printer_components.MelfaRobot import MelfaRobot, IllegalAxesCount, SpeedBelowMinimum


@pytest.fixture
def tcp():
    a = MagicMock()
    a.mock_add_spec(TcpClientR3())
    return a


@pytest.fixture
def safe_robot(tcp):
    """
    Test ressource with safe point return activated.
    :param tcp:
    :return:
    """
    return MelfaRobot(tcp, safe_return=True)


@pytest.fixture
def no_safe_robot(tcp):
    """
    Test ressource with safe point return deactivated.
    :param tcp:
    :return:
    """
    return MelfaRobot(tcp, safe_return=False)


class TestMelfaRobot:
    def test_axes_init(self, tcp):
        """
        Test that the initialisation for the number of axes is working.
        :param tcp:
        :return:
        """
        with pytest.raises(IllegalAxesCount):
            MelfaRobot(tcp, number_axes=0)

        a = MelfaRobot(tcp, number_axes=1)
        assert a.joints == ['J1']

        b = MelfaRobot(tcp, number_axes=2)
        assert b.joints == ['J1', 'J2']

    def test_boot_no_safe(self, no_safe_robot):
        """
        Test that for no safe config the safe return is not called during booting.
        :param no_safe_robot:
        :return:
        """
        with mock.patch.object(no_safe_robot, 'go_safe_pos', autospec=True) as mock_func:
            with mock.patch('src.printer_components.MelfaRobot.sleep', return_value=None):
                no_safe_robot.boot()
        assert not mock_func.called

    def test_boot_safe(self, safe_robot):
        """
        Test that for safe config the safe return is called during booting.
        :param safe_robot:
        :return:
        """
        with mock.patch.object(safe_robot, 'go_safe_pos', autospec=True) as mock_func:
            with mock.patch('src.printer_components.MelfaRobot.sleep', return_value=None):
                safe_robot.boot()
        assert mock_func.called

    def test_shutdown_no_safe(self, no_safe_robot):
        """
        Test that for no safe config the safe return is not called during shutdown.
        :param no_safe_robot:
        :return:
        """
        with mock.patch.object(no_safe_robot, 'go_safe_pos', autospec=True) as mock_func:
            no_safe_robot.shutdown()
        assert not mock_func.called

    def test_shutdown_safe(self, safe_robot):
        """
        Test that for safe config the safe return is called during shutdown.
        :param safe_robot:
        :return:
        """
        with mock.patch.object(safe_robot, 'go_safe_pos', autospec=True) as mock_func:
            with mock.patch('src.printer_components.MelfaRobot.sleep', return_value=None):
                safe_robot.shutdown()
        assert mock_func.called

    def test_activate_work_coordinate(self, no_safe_robot):
        """
        Test that the state variable can be changed accordingly and that the correct commands are delegated/not
        delegated to the tcp.
        :param no_safe_robot:
        :return:
        """
        # Activate
        with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
            no_safe_robot.activate_work_coordinate(True)
        assert no_safe_robot.work_coordinate_active
        # TODO Decouple this test from the implementation of zero
        mock_func.assert_any_call(R3Protocol.SET_BASE_COORDINATES + "(-500,0,-250,0,0,0)")

        # Deactivate
        with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
            no_safe_robot.activate_work_coordinate(False)
        assert not no_safe_robot.work_coordinate_active
        mock_func.assert_any_call(R3Protocol.RESET_BASE_COORDINATES)

    def test_handle_gcode(self):
        assert True

    def test_handle_gcode_inch_switch(self, no_safe_robot):
        """
        Test that the inch mode switch works correctly.
        :param no_safe_robot:
        :return:
        """
        activate_inch = GCmd.read_cmd_str('G20')
        deactivate_inch = GCmd.read_cmd_str('G21')

        no_safe_robot.handle_gcode(activate_inch)
        assert no_safe_robot.inch_active

        no_safe_robot.handle_gcode(deactivate_inch)
        assert not no_safe_robot.inch_active

    def test__prepare_circle(self):
        assert True

    def test_maintenance(self):
        assert True

    def test__change_communication_state(self, no_safe_robot):
        """
        Test that the state variable can be changed accordingly and that the correct commands are delegated/not
        delegated to the tcp.
        :param no_safe_robot:
        :return:
        """
        # Activate
        with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
            no_safe_robot._change_communication_state(True)
        assert no_safe_robot.com_ctrl
        mock_func.assert_any_call(R3Protocol.CNTL_ON)
        with pytest.raises(AssertionError):
            mock_func.assert_any_call(R3Protocol.CNTL_OFF)

        # Deactivate
        with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
            no_safe_robot._change_communication_state(False)
        assert not no_safe_robot.servo
        mock_func.assert_any_call(R3Protocol.CNTL_OFF)
        with pytest.raises(AssertionError):
            mock_func.assert_any_call(R3Protocol.CNTL_ON)

    def test__change_servo_state(self, no_safe_robot):
        """
        Test that the state variable can be changed accordingly and that the correct commands are delegated/not
        delegated to the tcp.
        :param no_safe_robot:
        :return:
        """
        with mock.patch('src.printer_components.MelfaRobot.sleep', return_value=None):
            # Activate
            with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
                no_safe_robot._change_servo_state(True)
            assert no_safe_robot.servo
            mock_func.assert_any_call(R3Protocol.SRV_ON)
            with pytest.raises(AssertionError):
                mock_func.assert_any_call(R3Protocol.SRV_OFF)

            # Deactivate
            with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
                no_safe_robot._change_servo_state(False)
            assert not no_safe_robot.servo
            mock_func.assert_any_call(R3Protocol.SRV_OFF)
            with pytest.raises(AssertionError):
                mock_func.assert_any_call(R3Protocol.SRV_ON)

    def test_read_parameter(self):
        assert True

    def test_set_speed(self, no_safe_robot):
        """
        Check that the speed commands are sent as expected.
        :param no_safe_robot:
        :return:
        """
        with mock.patch.object(no_safe_robot, '_get_ovrd_speed') as ovrd:
            # Error
            with pytest.raises(SpeedBelowMinimum):
                no_safe_robot.set_speed(0, 'linear')

            # Regular setting
            ovrd.return_value = 100
            with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
                no_safe_robot.set_speed(1, 'linear')
            mock_func.assert_called_with(R3Protocol.MVS_SPEED + '1.00')

            # Regular setting with different override
            ovrd.return_value = 30
            with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
                no_safe_robot.set_speed(12, 'linear')
            mock_func.assert_called_with(R3Protocol.MVS_SPEED + '40.00')

    def test_reset_linear_speed_factor(self):
        assert True

    def test_go_home(self):
        assert True

    def test_go_safe_pos(self):
        assert True

    @pytest.mark.parametrize("speed,expected_speed", [(None, False), (100, True)])
    @pytest.mark.parametrize("target,expected_move",
                             [
                                 (Coordinate((10, -20, 0), 'XYZ'), True),
                                 (Coordinate((None, None, None), 'XYZ'), False)
                             ])
    def test_linear_move_poll(self, target, speed, expected_speed, expected_move, no_safe_robot):
        with mock.patch.object(no_safe_robot, 'set_speed', spec=mock.Mock()) as mock_set_speed:
            with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_send:
                no_safe_robot.linear_move_poll(target, speed, track_speed=False)

        # Assert speed setting
        if expected_speed:
            mock_set_speed.assert_called_with(speed, 'linear')

        # Assert movement
        if expected_move:
            mock_send.assert_called()
        else:
            mock_send.assert_not_called()

    def test_circular_move_poll(self):
        assert True

    def test_set_global_positions(self):
        assert True

    def test_get_pos(self):
        assert True

    def test__check_speed_threshold(self):
        assert True

    def test__get_ovrd(self):
        assert True

    def test__set_ovrd_speed_false(self, no_safe_robot):
        """
        Test that invalid override values raise an exception.
        :param no_safe_robot:
        :return:
        """
        with pytest.raises(MelfaBaseException):
            no_safe_robot._set_ovrd(101)
        with pytest.raises(MelfaBaseException):
            no_safe_robot._set_ovrd(0)

    @pytest.mark.parametrize("factor", [1, 53, 100])
    def test__set_ovrd_speed_okay(self, factor, no_safe_robot):
        """
        Test that the override can be set to valid values.
        :param factor:
        :param no_safe_robot:
        :return:
        """
        with mock.patch.object(no_safe_robot.tcp, 'send', spec=mock.Mock()) as mock_func:
            no_safe_robot._set_ovrd(factor)
        mock_func.assert_called_with(R3Protocol.OVERRIDE_CMD + "=" + str(factor))

    def test_wait(self):
        assert True

    def test__zero(self):
        assert True
