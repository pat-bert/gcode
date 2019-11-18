from time import sleep

import ApplicationExceptions
import MelfaCmd
from ApplicationExceptions import *
from Coordinate import Coordinate
from GCmd import GCmd
from MelfaCmd import translate_cmd
from MelfaRobot import MelfaRobot
from TcpClientR3 import TcpClientR3


def demo_mode(ip=None, port=None):
    # Create TCP client
    if ip is not None and port is not None:
        tcp_client = TcpClientR3(host=ip, port=port)
    else:
        tcp_client = TcpClientR3()
    tcp_client.connect()

    # Executing communication
    try:
        tcp_client.start(speed_threshold=10)
        robot = MelfaRobot(tcp_client, number_axes=6)
        selection = input("Please choose a mode (1=cube, 2=flat circle): ")
        if selection == '1':
            cube(robot)
        else:
            raise NotImplementedError
    except KeyboardInterrupt:
        pass
    except NotImplementedError:
        pass
    except ApplicationExceptions.MelfaBaseException as e:
        print(str(e))
    finally:
        # Cleaning up
        tcp_client.close()


def interpret_gcode(f_input: str, f_output: str = 'out.txt') -> None:
    """
    Interpret a G-code file and translate it to R3 protocol commands.
    :param f_input: Input file path
    :param f_output: Output file path
    :return: None
    :raises: OSError
    """
    # Read input file and translate commands
    print("Parsing G-Code...")
    try:
        with open(f_input, 'r') as f:
            gcode_list = [GCmd.read_cmd_str(line) for line in f.readlines()]
    except OSError:
        print("Error reading file.")
        raise
    except GCmdError:
        print("Error reading G-code.")
        raise
    else:
        print("Done.")

    # Start translation to MELFA commands
    print("Translating commands to R3 protocol commands...")
    try:
        r3_code_list = [translate_cmd(gcode) for gcode in gcode_list]
    except MelfaBaseException:
        print("Error translating G-code.")
        raise
    else:
        print("Done.")

    # Write R3 commands to output file
    print("Writing R3 commands...")
    try:
        with open(f_output, 'w') as f:
            f.writelines([str(r3_code) for r3_code in r3_code_list])
    except OSError:
        print("Error writing file.")
        raise
    else:
        print("Done.")


def execute_r3(f_input: str, ip, port, f_log: str = 'run.log') -> None:
    """
    Execute a R3 protocol command text file.
    :param ip:
    :param port:
    :param f_input: Input file path
    :param f_log: Log file path
    :return: None
    """
    # Read commands from file
    print("Reading command file.")
    try:
        with open(f_input, 'r') as f:
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
        # tcp.start(speed_threshold=10)
    except TcpError:
        raise
    except PreCheckError:
        raise
    else:
        print("Done.")

    # Execute commands
    print("Executing commands...")
    try:
        with open(f_log, 'w') as fid_log:
            for r3_cmd in r3_commands:
                # TODO Send command
                pass
    except OSError as file_exception:
        print(file_exception)
        raise
    except ApiException:
        # TODO Replace by specific exception (communication, command, ...) and print something
        raise
    else:
        print("Done.")


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
                print(usr_msg)
    except KeyboardInterrupt:
        pass
    except ApplicationExceptions.MelfaBaseException as e:
        print(str(e))
    finally:
        # Cleaning up
        tcp_client.close(internal=False)


def interactive_melfa(ip, port, log_file=None) -> None:
    print("Launching interactive R3 protocol shell...")

    if log_file is not None:
        # TODO Implement logging
        pass

    # Create robot object
    robot = MelfaRobot(TcpClientR3(host=ip, port=port), number_axes=6)
    robot.tcp.connect()
    # Executing communication
    try:
        robot.tcp.start(speed_threshold=10)
        while True:
            usr_msg = input("Melfa>")
            if usr_msg.lower() in ['quit']:
                raise KeyboardInterrupt
            elif len(usr_msg) > 0:
                robot.tcp.wait_send(usr_msg.upper())
                try:
                    robot.tcp.receive()
                except ApplicationExceptions.MelfaBaseException as ex:
                    # Print error message
                    if len(ex.status) > 0:
                        print(str(ex))
                    else:
                        # Resolve empty status codes
                        print("Empty status code. Trying to resolve.")
                        robot.tcp.wait_send("ERROR")
                        try:
                            robot.tcp.receive()
                        except ApplicationExceptions.MelfaBaseException as ex_res:
                            print(str(ex_res))
                    # Reset alarm
                    sleep(1)
                    print("Error Reset")
                    robot.tcp.send(MelfaCmd.ALARM_RESET_CMD)
                    robot.tcp.receive(silence_errors=True)
    except KeyboardInterrupt:
        pass
    except ApplicationExceptions.MelfaBaseException as e:
        print(str(e))
    finally:
        # Cleaning up
        robot.tcp.close()


def cube(robot: MelfaRobot):
    # Base coordinates
    z_vector = Coordinate([0, 0, 5, 0, 0, 0], robot.axes)
    square_corners = [
        Coordinate([500, 50, 200, 180, 0, 0], robot.axes),
        Coordinate([500, -50, 200, 180, 0, 0], robot.axes),
        Coordinate([600, -50, 200, 180, 0, 0], robot.axes),
        Coordinate([600, 50, 200, 180, 0, 0], robot.axes)
    ]

    # Go to points
    for _ in range(10):
        # Square
        for point in square_corners:
            robot.linear_move_poll(point, 0)
        # Back to first point
        robot.linear_move_poll(square_corners[0], 0)
        # Increment z
        square_corners = [point + z_vector for point in square_corners]
