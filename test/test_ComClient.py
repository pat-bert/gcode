import pytest

import src.clients.IClient as IClient
from src.clients.ComClient import ComClient, validate_id
from test.util import SkipIfConditionWrapper


@pytest.fixture
def valid_com_client():
    return ComClient((0x0403, 0x6001))


@pytest.fixture
def non_existing_com_client():
    return ComClient((-1, -1))


# TODO Use this for integration tests
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

    @hardware.required
    def test_close(self, valid_com_client):
        """
        Test that closing does not raise an exception and closes the COM client properly.
        :param valid_com_client:
        :return:
        """
        valid_com_client.connect()

        # TODO Actual sending or receiving once the host is clean

        valid_com_client.close()
        # Receiving is not possible afterwards
        with pytest.raises(IClient.ClientError):
            valid_com_client.receive()

    def test_close_without_open(self, valid_com_client, non_existing_com_client):
        """
        Test that repeatedly closing does not cause an issue.
        :param valid_com_client:
        :return:
        """
        valid_com_client.close()
        non_existing_com_client.close()

    @hardware.required
    def test_receive(self):
        assert False

    @hardware.required
    def test_send(self):
        assert False

    def test_is_available_not(self, non_existing_com_client):
        """
        Test that for a non existing COM client the result is not available.
        :param non_existing_com_client:
        :return: None
        """
        assert not non_existing_com_client.is_available()

    @hardware.required
    def test_is_available(self, valid_com_client):
        """
        Test that the valid COM client is available.
        :param valid_com_client:
        :return: None
        """
        assert valid_com_client.is_available()


@pytest.mark.parametrize("value,valid", [(0, True), (-1, False), (2 ** 16 - 1, True), (2 ** 16, False)])
def test_validate_id(value, valid):
    assert validate_id(value) == valid
