import ApplicationExceptions
from MelfaRobot import MelfaRobot
from TcpClientR3 import TcpClientR3


def interactive_gcode(ip, port, log_file=None) -> None:
    """
    Launches an interactive shell accepting G-code.
    :param ip: IPv4 network address of the robot
    :param port: Port of the robot
    :param log_file: Log file handle
    :return:
    """
    print("Launching interactive G-code shell...")

    if log_file is not None:
        # TODO Implement logging
        pass

    # Create TCP client
    tcp_client = TcpClientR3(host=ip, port=port)
    tcp_client.connect()

    # Create robot object
    MelfaRobot(tcp_client, number_axes=6)

    # Executing communication
    try:
        tcp_client.start(speed_threshold=10, internal=False)
        while True:
            usr_msg = input("G-Code>")
            if usr_msg.lower() in ['quit']:
                raise KeyboardInterrupt
            elif len(usr_msg) > 0:
                # TODO G-Code interactive mode
                print("Not implemented")
    except KeyboardInterrupt:
        pass
    except ApplicationExceptions.MelfaBaseException as e:
        print(str(e))
    finally:
        # Cleaning up
        tcp_client.close(internal=False)
