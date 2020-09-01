import logging
from typing import Optional

from src.printer_components.MelfaRobot import MelfaRobot
from src.ApplicationExceptions import MelfaBaseException
from src.clients.TcpClientR3 import TcpClientR3
from src.GCmd import GCmd


def interactive_gcode_robot_only(ip: str, port: int, safe_return: Optional[bool] = False) -> None:
    """
    Launches an interactive shell accepting G-code.
    :param ip: IPv4 network address of the robot
    :param port: Port of the robot
    :param safe_return:
    """
    logging.info("Launching interactive G-code shell (robot only)...")

    # Create clients and connect
    tcp_client = TcpClientR3(host=ip, port=port)
    tcp_client.connect()

    # Create printer object
    printer = MelfaRobot(tcp_client, number_axes=6, speed_threshold=10, safe_return=safe_return)
    printer.boot()

    # Executing communication
    try:
        while True:
            usr_msg = input("G-Code>")
            if usr_msg.lower() in ["quit"]:
                raise KeyboardInterrupt
            if len(usr_msg) > 0:
                # Parse G-code
                gcode = GCmd.read_cmd_str(usr_msg)
                print(str(gcode))
                printer.assign_task(gcode)
    except MelfaBaseException as e:
        print(str(e))
    except KeyboardInterrupt:
        print('Program terminated by user.')
    finally:
        printer.shutdown()
