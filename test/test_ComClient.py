import unittest.mock as mock

import pytest

import src.clients.IClient as IClient
from src.clients.ComClient import ComClient, validate_id
from test.util import SkipIfNotConditionWrapper


@pytest.fixture
def valid_com_client():
    return ComClient((0x0403, 0x6001))


@pytest.fixture
def duplicate_com_client():
    return ComClient((0x0001, 0x0001))


@pytest.fixture
def non_existing_com_client():
    return ComClient((-1, -1))


# TODO Use this for integration tests
hardware = SkipIfNotConditionWrapper(lambda: ComClient((0x0403, 0x6001)).is_available(),
                                     'For now this requires a physical port.')


@pytest.mark.flaky(reruns=3)
class TestComClient:
    """
    Tests for the client layer in case of using a COM client
    """

    def test_connect_client_not_found(self, non_existing_com_client):
        """
        Test that an exception if raised in case the client cannot be found.
        :param non_existing_com_client:
        :return: None
        """
        with pytest.raises(IClient.ClientNotAvailableError):
            try:
                non_existing_com_client.connect()
            finally:
                # In case the test fails release the COM client immediately
                non_existing_com_client.close()

    @hardware.required
    def test_connect_twice(self, valid_com_client):
        """
        Test that an exception is raised on subsequent connection attempts.
        :param valid_com_client: Fixture object for an available client.
        :return: None
        """
        try:
            valid_com_client.connect()
            with pytest.raises(IClient.ClientOpenError):
                # Second try should not work
                valid_com_client.connect()
        finally:
            valid_com_client.close()

    def test_connect_duplicate(self, duplicate_com_client):
        """
        Test that an exception is raised if multiple devices are found
        :param duplicate_com_client: Fixture object for an available client.
        :return: None
        """
        # This implementation is strongly tied to the list ports functionality but rather unlikely to ever change
        with mock.patch('serial.tools.list_ports.comports') as mockfunc:
            # Create some fake devices
            fake_device = mock.MagicMock()
            fake_device.pid = duplicate_com_client.pid
            fake_device.vid = duplicate_com_client.vid

            # Create an arbitrary device that is not searched for connection
            fake_device_2 = mock.MagicMock()
            fake_device_2.pid = duplicate_com_client.pid + 1
            fake_device_2.vid = duplicate_com_client.vid + 1

            # Check that duplicate clients are reported
            mockfunc.return_value = [fake_device, fake_device_2, fake_device]
            with pytest.raises(IClient.AmbiguousHardwareError):
                try:
                    duplicate_com_client.connect()
                finally:
                    duplicate_com_client.close()

            # Validate that the mock is working as expected
            mockfunc.return_value = [fake_device_2]
            with pytest.raises(IClient.ClientNotAvailableError):
                try:
                    duplicate_com_client.connect()
                finally:
                    duplicate_com_client.close()

    @hardware.required
    def test_connect_properly(self, valid_com_client):
        """
        Test that no exception is raised on a valid connect.
        :param valid_com_client: Fixture object for an available client.
        :return: None
        """
        try:
            valid_com_client.connect()
        finally:
            valid_com_client.close()

    @hardware.required
    def test_close(self, valid_com_client):
        """
        Test that closing does not raise an exception and closes the COM client properly.
        :param valid_com_client:
        :return:
        """
        try:
            valid_com_client.connect()
            # TODO Actual sending or receiving once the host is clean
        finally:
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

    # @hardware.required
    @pytest.mark.skip
    def test_receive(self):
        assert False

    @hardware.required
    def test_send(self, valid_com_client, capsys):
        msg = 'This is a message.'

        valid_com_client.connect()

        try:
            # This time it should not be logged
            valid_com_client.send(msg, silent_send=True)
            captured = capsys.readouterr()
            assert msg not in captured.out and msg not in captured.err

            # This time it should be logged somehow
            valid_com_client.send(msg, silent_send=False)
            captured = capsys.readouterr()
            assert msg in captured.out or msg in captured.err
        finally:
            valid_com_client.close()

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
