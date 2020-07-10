import unittest.mock as mock

import pytest

from src.exit_codes import EXIT_SUCCESS
from src.main import __doc__ as cli_doc
from src.main import main


class TestMain:
    @mock.patch("src.main.interactive_gcode")
    def test_gcode_interactive(self, mock_func):
        """
        Test that the flag calls the function with the right argument
        :param mock_func:
        :return:
        """
        with pytest.raises(SystemExit) as cm:
            main(["--gi"])
        assert mock_func.called
        assert cm.value.code == EXIT_SUCCESS

    @mock.patch("src.main.interactive_melfa")
    def test_melfa_interactive(self, mock_func):
        """
        Test that the flag calls the function with the right argument
        :param mock_func:
        :return:
        """
        with pytest.raises(SystemExit) as cm:
            main(["--mi"])
        assert mock_func.called
        assert cm.value.code == EXIT_SUCCESS

    @pytest.mark.parametrize("cmd", ["-V", "--validate"])
    @mock.patch("src.main.check_trajectory")
    def test_validate_gcode(self, mock_func, cmd, tmpdir):
        """
        Test that the flags call the function with the right arguments
        :param mock_func:
        :param cmd:
        :param tmpdir:
        :return:
        """
        path = tmpdir.join("In_File")
        with pytest.raises(SystemExit) as cm:
            main([cmd, str(path)])
        # assert mock_func.called
        # assert cm.value.code == EXIT_SUCCESS

    @mock.patch("src.main.demo_mode")
    def test_demo_mode(self, mock_func):
        """
        Test that the flag --demo calls the function with the right arguments
        :param mock_func:
        :return:
        """
        with pytest.raises(SystemExit) as cm:
            main(["--demo"])
        assert mock_func.called
        assert cm.value.code == EXIT_SUCCESS

    @pytest.mark.parametrize("cmd", ["--help", "-h"])
    def test_help(self, capsys, cmd):
        """
        Test that the different help options print the doc string of main
        :param capsys:
        :param cmd:
        :return:
        """
        with pytest.raises(SystemExit):
            main([cmd])
        captured = capsys.readouterr().out.strip("\n")
        stripped_doc = cli_doc.strip("\n")
        assert stripped_doc in captured
