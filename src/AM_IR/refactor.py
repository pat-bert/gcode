import time
from time import sleep

from AM_IR.ApplicationExceptions import TcpError
from AM_IR.melfa import MelfaCmd
from AM_IR.melfa.TcpClientR3 import TcpClientR3


def cmd_coordinate_response(tcp_client, command):
    tcp_client.send(command)
    response = tcp_client.receive()
    coordinate_str = response.split(MelfaCmd.DELIMITER)[1]
    coordinates = coordinate_str.split(', ')
    return [float(i) for i in coordinates]


def joint_borders(tcp_client):
    return cmd_coordinate_response(tcp_client, MelfaCmd.PARAMETER_READ + MelfaCmd.JOINT_BORDERS)


def xyz_borders(tcp_client):
    return cmd_coordinate_response(tcp_client, MelfaCmd.PARAMETER_READ + MelfaCmd.XYZ_BORDERS)


def cmp_response(poll_cmd: str, response_t: str, tcp_client: TcpClientR3, poll_rate_ms: int = 5, timeout_s: int = 60,
                 track_speed=False):
    """
    Uses a given command to poll for a given response.
    :param poll_cmd: Command used to execute the poll
    :param response_t: Target response string
    :param tcp_client:
    :param poll_rate_ms: Poll rate in milliseconds
    :param timeout_s: Time until timeout in seconds
    :param track_speed:
    :return:
    """
    t = 0
    timeout_ms = timeout_s * 1000
    response_act = ''

    time_samples = []
    speed_samples = []
    start_time = None

    # Iterate until timeout occurs or expected response is received
    while t < timeout_ms:
        # Handle communication

        if track_speed:
            current_time = time.clock()

            tcp_client.send(MelfaCmd.VAR_READ + MelfaCmd.CURRENT_SPEED_VAR)
            speed_response = tcp_client.receive()
            try:
                speed = float(speed_response.split('=')[-1])
            except ValueError:
                speed = 0

            if start_time is None:
                start_time = current_time

            time_samples.append(float(current_time - start_time))
            speed_samples.append(float(speed))

        tcp_client.send(poll_cmd, silent_send=True, silent_recv=True)
        response_act = tcp_client.receive()

        # Check response
        if response_act.startswith(response_t):
            break

        # Delay
        sleep(poll_rate_ms / 1000)
        t += poll_rate_ms
    else:
        raise TcpError(
            "Timeout after {} seconds. Expected: '{}' but got '{}'".format(timeout_s, response_t, response_act))

    if track_speed:
        return time_samples, speed_samples
    else:
        return None, None
