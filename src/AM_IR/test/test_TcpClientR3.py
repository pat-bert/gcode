import pytest

from AM_IR.melfa.TcpClientR3 import TcpClientR3


class TestTcpClientR3:
    def test_connect(self):
        assert False

    def test_close(self):
        assert False

    def test_send(self):
        assert False

    def test_send_before_connect(self):
        assert False

    def test_send_message_too_long(self):
        tcp = TcpClientR3()
        tcp.connect()

        with pytest.raises(ValueError):
            tcp.send("T" * 128)

    def test_wait_send(self):
        assert False

    def test_receive(self):
        assert False

    def test_mainloop(self):
        assert False
