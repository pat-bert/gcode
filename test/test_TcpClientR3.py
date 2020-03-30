import pytest

from src.clients.TcpClientR3 import validate_ip, validate_port, TcpClientR3, TcpError
from test.util import SkipIfNotConditionWrapper


@pytest.fixture
def valid_tcp_client():
    return TestTcpClientR3()


@pytest.fixture
def non_existing_tcp_client():
    return TestTcpClientR3()


# TODO Use this for integration tests
hardware = SkipIfNotConditionWrapper(lambda: False,
                                     'For now this requires a physical port.')


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
    def test_connect_timeout(self):
        actual_tcp = TcpClientR3(timeout=0.1)
        with pytest.raises(TcpError):
            actual_tcp.connect()

    @hardware.required
    def test_close(self):
        assert True

    @hardware.required
    def test_send(self):
        assert True

    @hardware.required
    def test_send_before_connect(self):
        assert True

    @hardware.required
    def test_send_message_too_long(self):
        assert True

    @hardware.required
    def test_wait_send(self):
        assert True

    @hardware.required
    def test_receive(self):
        assert True

    @hardware.required
    def test_mainloop(self):
        assert True
