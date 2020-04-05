import pytest


def check_mitsubishi_available() -> bool:
    return False


pytestmark = pytest.mark.skipif(not check_mitsubishi_available(), reason='Mitsubishi controller not found.')


class TestR3Protocol:
    def test_dummy(self):
        assert False
