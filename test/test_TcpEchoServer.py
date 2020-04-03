import pytest

from src.clients.TcpClientR3 import TcpClientR3
from src.clients.TcpEchoServer import TcpEchoServer

PORT = 10009


@pytest.fixture
def tcp_server():
    global PORT
    PORT += 1
    return TcpEchoServer('127.0.0.1', PORT)


@pytest.fixture
def tcp_client():
    return TcpClientR3(host='127.0.0.1', port=PORT)


class TestTcpEchoServer:
    @pytest.mark.timeout(10)
    def test_listen(self, tcp_server):
        """
        Test that the server correctly enters and leaves the listening state
        :param tcp_server:
        :return:
        """
        assert not tcp_server.is_listening
        tcp_server.listen()

        try:
            # This should not be an issue
            assert tcp_server.is_listening
            tcp_server.listen()
            assert tcp_server.is_listening
        finally:
            # Shutdown in any case
            tcp_server.shutdown()
            assert not tcp_server.is_listening

    @pytest.mark.timeout(10)
    def test_successive_connections(self, tcp_server, tcp_client):
        """
        Test that the server can accept multiple successive connections.
        :param tcp_server:
        :param tcp_client:
        :return:
        """
        assert not tcp_server.is_listening
        tcp_server.listen()
        # Successive connections
        try:
            assert tcp_server.is_listening
            # First connection opened and closed
            tcp_client.connect()
            tcp_client.close()

            # Second connection without listening
            tcp_client.connect()
            tcp_client.close()
            assert tcp_server.is_listening
        finally:
            # Shutdown in any case
            tcp_server.shutdown()
            assert not tcp_server.is_listening

    @pytest.mark.timeout(10)
    def test_reboot(self, tcp_server, tcp_client):
        """
        Test that the server can be rebooted
        :param tcp_server:
        :param tcp_client:
        :return:
        """
        for _ in range(2):
            assert not tcp_server.is_listening
            tcp_server.listen()

            # First connection opened and closed
            try:
                assert tcp_server.is_listening
                tcp_client.connect()
                tcp_client.close()
            finally:
                tcp_server.shutdown()
                assert not tcp_server.is_listening
