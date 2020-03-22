import pytest

import src.clients.IClient as IClient
from src.clients.ComClient import ComClient
from test.util import SkipIfConditionWrapper


@pytest.fixture
def valid_com_client():
    return ComClient((0x0403, 0x6001))


@pytest.fixture
def non_existing_com_client():
    return ComClient((-1, -1))


hardware = SkipIfConditionWrapper(lambda: ComClient((0x0403, 0x6001)).is_available(),
                                  'For now this requires a physical port.')


class TestComClient:
    def test_connect_client_not_found(self, non_existing_com_client):
        """
        Test that an exception if raised in case the client cannot be found.
        :param non_existing_com_client:
        :return: None
        """
        with pytest.raises(IClient.ClientNotAvailableError):
            non_existing_com_client.connect()

    @hardware.required
    def test_connect_twice(self, valid_com_client):
        """
        Test that an exception is raised on subsequent connection attempts.
        :param valid_com_client: Fixture object for an available client.
        :return: None
        """
        valid_com_client.connect()
        with pytest.raises(IClient.ClientOpenError):
            # Second try should not work
            valid_com_client.connect()

    def test_close(self):
        assert False

    def test_receive(self):
        assert False

    def test_send(self):
        assert False
