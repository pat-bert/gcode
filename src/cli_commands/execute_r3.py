from src.ApplicationExceptions import TcpError, PreCheckError, ApiException
from src.printer_components.MelfaRobot import MelfaRobot
from src.clients.TcpClientR3 import TcpClientR3


def execute_r3(f_input: str, ip, port) -> None:
    """
    Execute a R3 protocol command text file.
    :param ip:
    :param port:
    :param f_input: Input file path
    :return: None
    """
    # Read commands from file
    print("Reading command file.")
    try:
        with open(f_input, "r") as f:
            r3_commands = f.readlines()
    except OSError:
        print("Error reading command file.")
        raise
    else:
        print("Done.")

    # Pre-checks
    print("Running compliance checks...")
    try:
        # TODO Add start up checks and communication
        tcp = TcpClientR3(host=ip, port=port)
        tcp.connect()
        robot = MelfaRobot(tcp, number_axes=6, speed_threshold=10, safe_return=True)
        robot.boot()
    except TcpError:
        raise
    except PreCheckError:
        raise
    else:
        print("Done.")

    # Execute commands
    print("Executing commands...")
    try:
        for r3_cmd in r3_commands:
            # TODO Send cmd
            pass
    except OSError as file_exception:
        print(file_exception)
        raise
    except ApiException:
        # TODO Replace by specific exception (communication, cmd, ...) and print something
        raise
    else:
        print("Done.")
