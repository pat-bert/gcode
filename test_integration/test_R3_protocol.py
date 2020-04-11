import pytest

from src.ApplicationExceptions import ErrorDispatch
from src.Coordinate import Coordinate
from src.MelfaCoordinateService import MelfaCoordinateService
from src.clients.TcpClientR3 import TcpClientR3
from src.clients.TcpEchoServer import ConfigurableEchoServer
from src.protocols.R3Protocol import R3Protocol

# Parameters
VALID_HOST, VALID_PORT = 'localhost', 10002
JOINTS = ['J{}'.format(i) for i in range(1, 7)]


@pytest.fixture
def echo_server() -> ConfigurableEchoServer:
    # Using the context manager to open and shutdown the communication for each test
    with ConfigurableEchoServer(VALID_HOST, VALID_PORT, 'utf-8') as new_server:
        yield new_server


@pytest.fixture
def valid_client():
    # Using the context manager to open and close the communication for each test
    with TcpClientR3(host=VALID_HOST, port=VALID_PORT, timeout=3) as client:
        yield client


@pytest.fixture
def protocol(valid_client):
    return R3Protocol(valid_client, MelfaCoordinateService(), joints=JOINTS, digits=2)


@pytest.mark.skip(reason='Not implemented.')
@pytest.mark.usefixtures('echo_server')
class TestR3ProtocolUtil:
    def test_reset_alarm(self, protocol):
        protocol.reset_alarm()

    def test_activate_servo(self, protocol):
        protocol.activate_servo()

    def test_deactivate_servo(self, protocol):
        protocol.deactivate_servo()

    def test_obtain_control(self, protocol):
        protocol.obtain_control()

    def test_release_control(self, protocol):
        protocol.release_control()

    def test_open_communication(self, protocol):
        protocol.open_communication()

    def test_close_communication(self, protocol):
        protocol.close_communication()


@pytest.mark.skip(reason='Not implemented.')
@pytest.mark.usefixtures('echo_server')
class TestR3ProtocolPositions:
    @pytest.mark.parametrize("n,p", [])
    def test_set_position(self, protocol, n: str, p: Coordinate):
        protocol.set_position(n, p)

    @pytest.mark.parametrize("n,t", [])
    def test_define_variable(self, protocol, n: str, t: str):
        protocol.define_variable(n, var_type=t)

    @pytest.mark.parametrize("t", [])
    def test_linear_move(self, protocol, t: Coordinate):
        protocol.linear_move(t)

    @pytest.mark.parametrize("t", [])
    def test_joint_move(self, protocol, t: Coordinate):
        protocol.joint_move(t)

    @pytest.mark.parametrize("s,t,c", [])
    def test_circular_move_centre(self, protocol, s: str, t: str, c: str):
        protocol.circular_move_centre(s, t, c)

    @pytest.mark.parametrize("s,im,t", [])
    def test_circular_move_intermediate(self, protocol, s: str, im: str, t: str):
        protocol.circular_move_intermediate(s, im, t)

    @pytest.mark.parametrize("s,im1,im2", [])
    def test_circular_move_full(self, protocol, s: str, im1: str, im2: str):
        protocol.circular_move_full(s, im1, im2)

    def test_go_safe_pos(self, protocol):
        protocol.go_safe_pos()


@pytest.mark.parametrize("prefix,exc", [(p, e) for p, e in ErrorDispatch.items()])
@pytest.mark.usefixtures('echo_server')
class TestR3ProtocolReader:
    @staticmethod
    def float_value(simple_echo_server, func, prefix, exc, *, response):
        """
        Test macro for getting float responses
        """
        if exc is None:
            simple_echo_server.reconfigure(prefix=prefix, msg=response)
            assert func() == float(response)
        else:
            simple_echo_server.reconfigure(prefix=prefix, msg='')
            with pytest.raises(exc):
                func()

    def test_get_override(self, protocol, echo_server, prefix, exc):
        self.float_value(echo_server, protocol.get_override, prefix, exc, response='50.0')

    def test_get_current_linear_speed(self, protocol, echo_server, prefix, exc):
        self.float_value(echo_server, protocol.get_current_linear_speed, prefix, exc, response='50.0')

    def test_get_current_joint_speed(self, protocol, echo_server, prefix, exc):
        self.float_value(echo_server, protocol.get_joint_speed, prefix, exc, response='50.0')

    @pytest.mark.skip(reason='Not implemented.')
    def test_get_joint_borders(self, protocol, echo_server, prefix, exc):
        protocol.get_joint_borders()

    @pytest.mark.skip(reason='Not implemented.')
    def test_get_xyz_borders(self, protocol, echo_server, prefix, exc):
        protocol.get_xyz_borders()

    def test_get_current_xyzabc(self, protocol, echo_server, prefix, exc):
        """
        Test that the response string can be converted correctly
        :param protocol:
        :param echo_server:
        :param prefix:
        :param exc:
        :return:
        """
        response = 'X;290.62;Y;-0.09;Z;11.26;A;-179.94;B;-0.26;C;179.93;L1;0.00;;6,0;100;0.00;00000000'
        expected = Coordinate((290.62, -0.09, 11.26, -179.94, -0.26, 179.93), 'XYZABC')

        # Test
        if exc is None:
            echo_server.reconfigure(prefix=prefix, msg=response)
            actual = protocol.get_current_xyzabc()
            assert str(actual) == str(expected)
        else:
            echo_server.reconfigure(prefix=prefix, msg='')
            with pytest.raises(exc):
                protocol.get_current_xyzabc()

    def test_get_current_joint(self, protocol, echo_server, prefix, exc):
        """
        Test that the response string can be converted correctly
        :param protocol:
        :param echo_server:
        :param prefix:
        :param exc:
        :return:
        """
        response = 'J1;290.62;J2;-0.09;J3;11.26;J4;-179.94;J5;-0.26;J6;179.93;L1;0.00;;6,0;100;0.00;00000000'
        expected = Coordinate((290.62, -0.09, 11.26, -179.94, -0.26, 179.93), JOINTS)

        # Test
        if exc is None:
            echo_server.reconfigure(prefix=prefix, msg=response)
            actual = protocol.get_current_joint()
            assert str(actual) == str(expected)
        else:
            echo_server.reconfigure(prefix=prefix, msg='')
            with pytest.raises(exc):
                protocol.get_current_joint()

    @pytest.mark.skip(reason='Not implemented.')
    def test_get_safe_pos(self, protocol, prefix, exc):
        protocol.get_safe_pos()

    @pytest.mark.skip(reason='Not implemented.')
    def test_get_servo_state(self, protocol, prefix, exc):
        protocol.get_servo_state()

    @pytest.mark.skip(reason='Not implemented.')
    @pytest.mark.parametrize("var", [])
    def test_read_variable(self, protocol, var: str, prefix, exc):
        protocol.read_variable(var)


@pytest.mark.parametrize("prefix,exc", [(p, e) for p, e in ErrorDispatch.items()])
@pytest.mark.usefixtures('echo_server')
class TestR3ProtocolSetter:
    @staticmethod
    def limited_set(simple_echo_server, func, *, prefix, exc, lbound, ubound):
        """
        Test macro
        :param simple_echo_server:
        :param func:
        :param prefix:
        :param exc:
        :param lbound:
        :param ubound:
        :return:
        """
        valid_interval = [lbound, ubound]
        outside = [lbound - 1, ubound + 1]

        if exc is None:
            # Valid return configured, return value is not checked
            simple_echo_server.reconfigure(prefix=prefix, msg='garbage')

            # Should be possible to set
            for i in valid_interval:
                func(i)

            # Should raise an exception
            for i in outside:
                with pytest.raises(ValueError):
                    func(i)
        else:
            # Error return configured
            simple_echo_server.reconfigure(prefix=prefix, msg='more garbage')

            # Impossible to set due to server error
            for i in valid_interval:
                with pytest.raises(exc):
                    func(i)

            # Impossible to set due to input limitation
            for i in outside:
                with pytest.raises(ValueError):
                    func(i)

    @pytest.mark.parametrize("wc", [])
    def test_set_work_coordinate(self, protocol, wc: str, prefix, exc):
        protocol.set_work_coordinate(wc)

    def test_set_override(self, echo_server, protocol, prefix, exc):
        self.limited_set(echo_server, protocol.set_override, prefix=prefix, exc=exc, lbound=1.0, ubound=100.0)

    def test_set_linear_speed(self, echo_server, protocol, prefix, exc):
        self.limited_set(echo_server, protocol.set_linear_speed, prefix=prefix, exc=exc, lbound=1.0, ubound=1000.0)

    def test_set_joint_speed(self, echo_server, protocol, prefix, exc):
        self.limited_set(echo_server, protocol.set_joint_speed, prefix=prefix, exc=exc, lbound=1.0, ubound=100.0)


@pytest.mark.skip(reason='Not implemented.')
@pytest.mark.usefixtures('echo_server')
class TestR3ProtocolResetter:
    def test_reset_base_coordinate_system(self, protocol):
        protocol.reset_base_coordinate_system()

    def test_reset_override(self, protocol):
        protocol.reset_override()

    def test_reset_linear_speed(self, protocol):
        protocol.reset_linear_speed()

    def test_reset_joint_speed(self, protocol):
        protocol.reset_joint_speed()

    def test_reset_all_speeds(self, protocol):
        protocol.reset_all_speeds()
