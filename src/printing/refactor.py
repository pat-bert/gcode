from time import sleep

from printing import MelfaCmd
from printing.ApplicationExceptions import TcpError
from printing.Coordinate import Coordinate
from printing.TcpClientR3 import TcpClientR3


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


def go_safe_pos(tcp_client):
    # Read safe position
    tcp_client.send(MelfaCmd.PARAMETER_READ + MelfaCmd.PARAMETER_SAFE_POSITION)
    safe_pos = tcp_client.receive()
    axes = ['J' + str(i) for i in range(1, 7)]
    safe_pos_values = safe_pos.split(';')[1]
    safe_pos_values = [float(i) for i in safe_pos_values.split(', ')]
    safe_pos = Coordinate(safe_pos_values, axes)

    # Return to safe position
    tcp_client.send(MelfaCmd.MOVE_SAFE_POSITION)
    tcp_client.receive()
    cmp_response(MelfaCmd.CURRENT_JOINT, safe_pos.to_melfa_response(), tcp_client)


def get_ovrd_speed(tcp_client):
    tcp_client.wait_send(MelfaCmd.OVERWRITE_CMD)
    speed = tcp_client.receive()
    return float(speed)


def reset_speeds(tcp_client):
    tcp_client.send(MelfaCmd.MVS_SPEED + MelfaCmd.MVS_MAX_SPEED)
    tcp_client.receive()
    # TODO Reset MOV Speed
    # tcp_client.send(MelfaCmd.MOV_SPEED + MelfaCmd.MOV_MAX_SPEED)
    # tcp_client.receive()


def cmp_response(poll_cmd: str, response_t: str, tcp_client: TcpClientR3, poll_rate_ms: int = 5, timeout_s: int = 60):
    """
    Uses a given command to poll for a given response.
    :param tcp_client:
    :param poll_cmd: Command used to execute the poll
    :param response_t: Target response string
    :param poll_rate_ms: Poll rate in milliseconds
    :param timeout_s: Time until timeout in seconds
    :return:
    """
    t = 0
    timeout_ms = timeout_s * 1000
    response_act = ''

    # Iterate until timeout occurs or expected response is received
    while t < timeout_ms:
        # Handle communication
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
