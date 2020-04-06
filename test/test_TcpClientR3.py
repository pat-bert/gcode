import pytest

from src.ApplicationExceptions import ErrorDispatch
from src.clients.TcpClientR3 import validate_ip, validate_port, TcpClientR3, TcpError
from src.clients.TcpEchoServer import ConfigurableEchoServer

VALID_HOST, VALID_PORT = 'localhost', 10002
INVALID_HOST, INVALID_PORT = '192.168.0.1', 10002


@pytest.fixture
def simple_tcp_echo():
    # Using the context manager to open and shutdown the communication for each test
    with ConfigurableEchoServer(VALID_HOST, VALID_PORT, 'utf-8') as new_server:
        yield new_server


@pytest.fixture
def valid_tcp_client():
    return TcpClientR3(host=VALID_HOST, port=VALID_PORT, timeout=3)


@pytest.fixture
def non_existing_tcp_client():
    return TcpClientR3(host=INVALID_HOST, port=INVALID_PORT, timeout=0.1)


@pytest.mark.parametrize(
    "ip,valid",
    [
        ("127.16.254.1", True),
        # 3 blocks
        ("127.254.1", False),
        # 5 blocks
        ("127.254.1.3.5", False),
        # upper edge
        ("255.255.255.255", True),
        # beyond upper edge
        ("255.255.255.256", False),
        ("255.255.256.255", False),
        ("255.256.255.255", False),
        ("256.255.255.255", False),
        # lower edge
        ("0.0.0.0", True),
        # below lower edge
        ("0.0.0.-1", False),
        ("0.0.-1.0", False),
        ("0.-1.0.0", False),
        ("-1.0.0.0", False),
        # simple int
        ("0", False),
        # not convertable
        ("Try.3.Not.4", False),
    ],
)
def test_validate_ip(ip, valid):
    assert validate_ip(ip) == valid


@pytest.mark.parametrize(
    "ip",
    [
        # wrong delimiter
        ("127.16,254.1", False),
        # simple text
        ("Text", False),
    ],
)
def test_validate_ip_exc(ip):
    with pytest.raises(Exception):
        validate_ip(ip)


@pytest.mark.parametrize(
    "port,valid", [(0, False), (1, True), (65535, True), (65536, False)]
)
def test_validate_port(port, valid):
    assert validate_port(port) == valid


class TestTcpClientR3:
    def test_connect_timeout(self, non_existing_tcp_client):
        """
        Test that a connection attemp to a non existing adress results in an exception.
        :param non_existing_tcp_client: TCP-Client configured with a non-existing server adress
        :return:
        """
        with pytest.raises(TcpError):
            non_existing_tcp_client.connect()

    def test_valid_connection(self, valid_tcp_client, simple_tcp_echo):
        """
        Test that the TCP client can connect and close properly to a simple TCP echo server.
        :param valid_tcp_client: TCP-Client configured with a valid server adress
        :param simple_tcp_echo: Simple TCP-Servo echoing back all messages
        :return:
        """
        # Try the connection
        valid_tcp_client.connect()
        assert valid_tcp_client.is_connected
        valid_tcp_client.close()
        assert not valid_tcp_client.is_connected

        # Context manager
        with valid_tcp_client as tcp_client:
            assert tcp_client.is_connected
        assert not tcp_client.is_connected

    @pytest.mark.timeout(10)
    @pytest.mark.parametrize("msg_list", [['Test', 'message']])
    def test_send_and_receive(self, msg_list, valid_tcp_client, simple_tcp_echo):
        """
        Test that multiple messages and their responses can be sent and received one by one.
        :param msg_list: List of messages to be sent
        :param valid_tcp_client: TCP-Client configured with a valid server adress
        :param simple_tcp_echo: Simple TCP-Servo echoing back all messages
        :return:
        """
        # Check the connection
        with valid_tcp_client as tcp_client:
            for msg in msg_list:
                tcp_client.send(msg)
                response = tcp_client.receive()
                assert response == msg

    def test_send_before_connect(self, valid_tcp_client):
        """
        A connection should be attempted first so that the message flow is as expected.
        :param valid_tcp_client:
        :return:
        """
        with pytest.raises(TcpError):
            valid_tcp_client.send('Test')

    @pytest.mark.timeout(10)
    def test_send_message_too_long(self, valid_tcp_client, simple_tcp_echo):
        """
        Test that messages longer than 128 characters are refused but that the client remains alive.
        :param valid_tcp_client:
        :param simple_tcp_echo:
        :return:
        """
        with valid_tcp_client as tcp_client:
            # Send a valid message
            tcp_client.send('T' * 127)

            # Send some invalid messages
            for _ in range(2):
                with pytest.raises(ValueError):
                    tcp_client.send('T' * 128)

            # Send a valid message
            tcp_client.send('E' * 127)

    @pytest.mark.timeout(10)
    def test_recv_exception(self, valid_tcp_client, simple_tcp_echo):
        """
        Test that the correct exceptions are raised if necessary.
        :param valid_tcp_client:
        :return:
        """
        message = 'Test'

        # Can also be used as context manager (ensures that connection is closed)
        with valid_tcp_client as tcp_client:
            # Iterate over all possible exceptions
            for prefix, exc in ErrorDispatch.items():
                # Configure the server to send the current error prefix
                simple_tcp_echo.reconfigure(prefix=prefix)
                tcp_client.send(message)

                if exc is not None:
                    # Check that the correct exception is raised and that it contains the rest of the message
                    with pytest.raises(exc, match=message):
                        tcp_client.receive()
                else:
                    # Check that the response is equal to the message without the prefix
                    response = tcp_client.receive()
                    assert response == message
