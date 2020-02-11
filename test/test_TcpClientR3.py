import pytest

from src.melfa.TcpClientR3 import validate_ip, validate_port


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
    "port,valid", [(0, True), (-1, False), (65535, True), (65536, False)]
)
def test_validate_port(port, valid):
    assert validate_port(port) == valid


class TestTcpClientR3:
    def test_connect(self):
        assert True

    def test_close(self):
        assert True

    def test_send(self):
        assert True

    def test_send_before_connect(self):
        assert True

    def test_send_message_too_long(self):
        assert True

    def test_wait_send(self):
        assert True

    def test_receive(self):
        assert True

    def test_mainloop(self):
        assert True
