from time import sleep

import ApplicationExceptions
import MelfaCmd
from ApplicationExceptions import TcpError
from Coordinate import Coordinate


def joint_borders(tcp_client):
    tcp_client.send(MelfaCmd.PARAMETER_READ + MelfaCmd.JOINT_BORDERS)
    response = tcp_client.receive()
    coordinate_str = response.split(MelfaCmd.DELIMITER)[1]
    coordinates = coordinate_str.split(', ')
    return [float(i) for i in coordinates]


def xyz_borders(tcp_client):
    tcp_client.send(MelfaCmd.PARAMETER_READ + MelfaCmd.XYZ_BORDERS)
    response = tcp_client.receive()
    coordinate_str = response.split(MelfaCmd.DELIMITER)[1]
    coordinates = coordinate_str.split(', ')
    return [float(i) for i in coordinates]


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


def check_speed_threshold(tcp_client, speed_threshold=10):
    reset_speeds(tcp_client)
    # Check for low speed
    speed = get_ovrd_speed(tcp_client)
    if speed > speed_threshold:
        try:
            raise ApplicationExceptions.MelfaBaseException
            # speed_correct_value = speed_threshold / speed * 100
            # tcp_client.send(MelfaCmd.MVS_SPEED + str(speed_correct_value))
            # tcp_client.receive()
            # tcp_client.send(MelfaCmd.MOV_SPEED + str(speed_correct_value / 10))
            # tcp_client.receive()
        except ApplicationExceptions.MelfaBaseException:
            raise ApplicationExceptions.MelfaBaseException(
                "Please ensure a speed lower or equal 10% in interactive mode!")
    else:
        print("Speed of " + str(speed) + "%. Okay!")


def cmp_response(poll_cmd: str, response_t: str, tcp_client, poll_rate_ms: int = 3, timeout_s: int = 300):
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
    while t < timeout_ms:
        # Handle communication
        tcp_client.send(poll_cmd)
        response_act = tcp_client.receive()

        # Check response
        if response_act.startswith(response_t):
            break

        # Delay
        sleep(poll_rate_ms / 1000)
        t += poll_rate_ms
    else:
        raise TcpError("Timeout for check.")
