import unittest.mock as mock
from unittest.mock import MagicMock

import pytest

from src.clients.TcpClientR3 import TcpClientR3
from src.protocols.R3Protocol import R3Resetter, R3Protocol, R3Setter


@pytest.fixture
def fake_tcp():
    a = MagicMock()
    a.mock_add_spec(TcpClientR3())
    return a


whole_api = R3Protocol(MagicMock(), MagicMock(), ['J1'], digits=2)
reset_api, partial_reset_api = R3Resetter(MagicMock()), whole_api.resetter
set_api, partial_set_api = R3Setter(MagicMock(), digits=2), whole_api.setter
partial_reader_api = whole_api.reader
partial_pos_api = whole_api.pos
partial_util_api = whole_api.util


@pytest.mark.parametrize("agent", [whole_api, reset_api, partial_reset_api])
class TestR3ProtocolResetter:
    def test_reset_base_coordinate_system(self, agent: R3Resetter, fake_tcp):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.reset_base_coordinate_system()
        mock_func.assert_called_with('1;1;EXECBASE P_NBASE')

    def test_reset_override(self, agent: R3Resetter, fake_tcp):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.reset_override()
        mock_func.assert_called_with('1;1;OVRD M_NOVRD')

    def test_reset_linear_speed(self, agent: R3Resetter, fake_tcp):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.reset_linear_speed()
        mock_func.assert_called_with('1;1;EXECSPD M_NSPD')

    def test_reset_joint_speed(self, agent: R3Resetter, fake_tcp):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.reset_joint_speed()
        mock_func.assert_called_with('1;1;EXECJOVRD M_NJOVRD')

    def test_reset_all_speeds(self, agent: R3Resetter, fake_tcp):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.reset_all_speeds()
        mock_func.assert_any_call('1;1;EXECSPD M_NSPD')
        mock_func.assert_any_call('1;1;EXECJOVRD M_NJOVRD')


@pytest.mark.parametrize("agent", [whole_api, set_api, partial_set_api])
class TestR3ProtocolSetter:
    def test_set_work_coordinate(self, agent: R3Setter, fake_tcp):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.set_work_coordinate('Test')
        mock_func.assert_called_with('1;1;EXECBASE Test')

    @pytest.mark.parametrize("val", [1.0, 100.0])
    def test_set_override(self, agent: R3Setter, fake_tcp, val):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.set_override(val)
        mock_func.assert_called_with('1;1;OVRD={:.{d}f}'.format(val, d=agent.digits))

    @pytest.mark.parametrize("val", [1.0, 100.0])
    def test_set_linear_speed(self, agent: R3Setter, fake_tcp, val):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.set_linear_speed(val)
        mock_func.assert_called_with('1;1;EXECSPD {:.{d}f}'.format(val, d=agent.digits))

    @pytest.mark.parametrize("val", [1.0, 100.0])
    def test_set_joint_speed(self, agent: R3Setter, fake_tcp, val):
        agent.client = fake_tcp
        with mock.patch.object(agent.client, "send", spec=mock.Mock()) as mock_func:
            agent.set_joint_speed(val)
        mock_func.assert_called_with('1;1;EXECJOVRD {:.{d}f}'.format(val, d=agent.digits))
