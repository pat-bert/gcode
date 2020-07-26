import pytest

from src.gcode.GCmd import GCmd


class TestGCmd:
    @pytest.mark.parametrize(
        "test_input",
        ["G28 X Y", "G02 I5.0 K5.3", "G01", "G28", "G28 X Y Z", "M104 S3.0", "T0"],
    )
    def test_str_recurrent(self, test_input):
        """
        Tests the creation of objects from a command string and their conversion back to the string.
        :return:
        """
        # Define digits to have outcomes as expected.
        GCmd.DIGITS = 1
        GCmd.CMD_REMOVE_LEAD_ZERO = False

        out_act = str(GCmd.read_cmd_str(test_input))
        assert out_act == test_input

    def test_str_remove_lead_zero(self):
        """
        Test that lead zeroes in the command can be removed
        :return:
        """
        GCmd.DIGITS = 1
        GCmd.CMD_REMOVE_LEAD_ZERO = True

        out_act = str(GCmd.read_cmd_str("G01 X10.3 Z-10.3"))
        assert out_act == "G1 X10.3 Z-10.3"

    def test_str_g28_remove_coordinates(self):
        """
        Test that for G28 (homing) only the axes are considered
        :return:
        """
        GCmd.DIGITS = 1
        GCmd.CMD_REMOVE_LEAD_ZERO = False

        out_act = str(GCmd.read_cmd_str("G28 Y10.0 Z"))
        assert out_act == "G28 Y Z"

    def test_str_convert_seconds(self):
        """
        Test that seconds are converted to milliseconds
        :return:
        """
        GCmd.DIGITS = 1
        GCmd.CMD_REMOVE_LEAD_ZERO = False

        out_act = str(GCmd.read_cmd_str("G01 X-50.0 Y30.0 F30.2 E17.1 S20.3"))
        assert out_act == "G01 X-50.0 Y30.0 F30.2 E17.1 P20300.0"

    def test_str_comment(self):
        """
        Test that a comment is turned to None
        :return:
        """
        GCmd.DIGITS = 1
        GCmd.CMD_REMOVE_LEAD_ZERO = False

        out_act = GCmd.read_cmd_str("; I am a comment.")
        assert out_act is None

    def test_invalid_command(self):
        with pytest.raises(ValueError):
            GCmd("G-1")

    @pytest.mark.parametrize("cmd", ["GK"])
    def test_invalid_command_from_str(self, cmd):
        GCmd.CMD_REMOVE_LEAD_ZERO = True
        with pytest.raises(ValueError):
            GCmd.read_cmd_str(cmd)
