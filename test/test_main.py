import unittest.mock as mock

import pytest

from src.exit_codes import EXIT_SUCCESS, EXIT_BAD_INPUT
from src.main import __doc__ as cli_doc
from src.main import main


class TestMain:
    def test_gcode_interactive(self):
        """
        Test that the flag calls the function with the right argument
        :return:
        """
        with mock.patch('src.main.interactive_gcode') as mock_func:
            with pytest.raises(SystemExit) as cm:
                main(["--gi", "--ip=192.168.0.1", "--port=10002"])
            assert cm.value.code != EXIT_BAD_INPUT
            assert cm.value.code == EXIT_SUCCESS
            assert mock_func.called

    def test_melfa_interactive(self):
        """
        Test that the flag calls the function with the right argument
        :return:
        """
        with mock.patch('src.main.interactive_melfa') as mock_func:
            with pytest.raises(SystemExit) as cm:
                main(["--mi", "--ip=192.168.0.1", "--port=10002"])
            assert cm.value.code != EXIT_BAD_INPUT
            assert cm.value.code == EXIT_SUCCESS
            assert mock_func.called

    @pytest.mark.parametrize("cmd", ["-V", "--validate"])
    def test_validate_gcode(self, cmd):
        """
        Test that the flags call the function with the right arguments
        :param cmd:
        :return:
        """
        with mock.patch('src.main.check_trajectory') as mock_func:
            with pytest.raises(SystemExit) as cm:
                main([cmd, 'cartesian_violation.gcode', 'config.ini'])
            print(cm)
            assert cm.value.code != EXIT_BAD_INPUT
            assert cm.value.code == EXIT_SUCCESS
            assert mock_func.called

    def test_demo_mode(self):
        """
        Test that the flag --demo calls the function with the right arguments
        :return:
        """
        with mock.patch('src.main.demo_mode') as mock_func:
            with pytest.raises(SystemExit) as cm:
                main(["--demo", "--ip=192.168.0.1", "--port=10002"])
            assert cm.value.code != EXIT_BAD_INPUT
            assert cm.value.code == EXIT_SUCCESS
            assert mock_func.called

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
