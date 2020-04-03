from time import sleep

import pytest

from clients.TcpClientR3 import TcpClientR3
from clients.TcpEchoServer import TcpEchoServer


@pytest.fixture(scope='module')
def tcp_server():
    return TcpEchoServer('127.0.0.2', 10009)


@pytest.fixture
def tcp_client():
    return TcpClientR3(host='127.0.0.2', port=10009)


class TestTcpEchoServer:
    def test_listen(self, tcp_server):
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

    def test_successive_connections(self, tcp_server, tcp_client):
        assert not tcp_server.is_listening
        tcp_server.listen()
        sleep(1)
        # Successive connections
        try:
            # TODO This is not really working
            assert tcp_server.is_listening
            # First connection opened and closed
            tcp_client.connect()
            tcp_client.close()

            # Second connection without listening
            tcp_client.connect()
            tcp_client.close()
        finally:
            # Shutdown in any case
            tcp_server.shutdown()
