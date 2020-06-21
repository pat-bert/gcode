import pytest

from src.clients.TcpEchoServer import TcpEchoServer
from src.gcode.GCmd import GCmd
from src.printer_components.GPrinter import GPrinter

ROBOT_IP, ROBOT_PORT = 'localhost', 10010


@pytest.fixture
def printer(tcp_server):
    # Starting up server as dummy for printer
    with tcp_server:
        printer = GPrinter.default_init(ROBOT_IP, ROBOT_PORT)
        yield printer
        printer.shutdown()


@pytest.fixture
def tcp_server():
    return TcpEchoServer('localhost', 10010)


class TestGPrinter:
    @pytest.mark.parametrize('cmd_str', ['G91', 'G20'])
    def test_execute(self, printer, cmd_str):
        """
        Test that commands are sent to all corresponding components
        :param printer:
        :return:
        """
        cmd = GCmd.read_cmd_str(cmd_str)
        printer.execute(cmd)

    def test_shutdown(self, printer):
        """
        Test that all components are shutdown properly
        :return:
        """
        printer.shutdown()
