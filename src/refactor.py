import time
from time import sleep

from src.ApplicationExceptions import TcpError
from src.protocols.R3Protocol import R3Reader


def cmp_response(poll_cmd: str, response_t: str, protocol: R3Reader, poll_rate_ms: int = 5, timeout_s: int = 60,
                 track_speed=False, ):
    """
    Uses a given cmd to poll for a given response.
    :param poll_cmd: Command used to execute the poll
    :param response_t: Target response string
    :param protocol:
    :param poll_rate_ms: Poll rate in milliseconds
    :param timeout_s: Time until timeout in seconds
    :param track_speed:
    :return:
    """
    t = 0
    timeout_ms = timeout_s * 1000
    response_act = ""

    response_t = response_t.split(";A")[0]

    time_samples = []
    speed_samples = []
    start_time = None

    # Iterate until timeout occurs or expected response is received
    while t < timeout_ms:
        # Handle communication

        if track_speed:
            current_time = time.clock()

            try:
                speed = protocol.get_current_linear_speed()
            except ValueError:
                speed = 0

            if start_time is None:
                start_time = current_time

            time_samples.append(float(current_time - start_time))
            speed_samples.append(float(speed))

        protocol._protocol_send(poll_cmd, silent_send=True, silent_recv=True)
        response_act = protocol.client.receive()

        # Check response
        if response_act.startswith(response_t):
            break

        # Delay
        sleep(poll_rate_ms / 1000)
        t += poll_rate_ms
    else:
        raise TcpError(f"Timeout after {timeout_s} seconds. Expected: '{response_t}' but got '{response_act}'")

    if track_speed:
        return time_samples, speed_samples
    else:
        return None, None
