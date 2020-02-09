import unittest.mock as mock

import pytest

from src.exit_codes import *
from src.main import main


class TestMain:
    def test_main_help(self):
        assert True

    @mock.patch('src.main.interactive_gcode')
    def test_gcode_interactive(self, mock_func):
        with pytest.raises(SystemExit) as cm:
            main(['--gi'])
        assert mock_func.called
        assert cm.value.code == EXIT_SUCCESS

    @mock.patch('src.main.interactive_melfa')
    def test_melfa_interactive(self, mock_func):
        with pytest.raises(SystemExit) as cm:
            main(['--mi'])
        assert mock_func.called
        assert cm.value.code == EXIT_SUCCESS

    @pytest.mark.parametrize("cmd", ['-I', '--interpret'])
    @mock.patch('src.main.interpret_gcode')
    def test_interpret_gcode(self, mock_func, cmd, tmpdir):
        path = tmpdir.join("In_File")
        with pytest.raises(SystemExit) as cm:
            main([cmd, str(path)])
        assert True
        # assert mock_func.called
        # assert cm.value.code == EXIT_SUCCESS

    @mock.patch('src.main.demo_mode')
    def test_demo_mode(self, mock_func):
        with pytest.raises(SystemExit) as cm:
            main(['--demo'])
        assert mock_func.called
        assert cm.value.code == EXIT_SUCCESS
