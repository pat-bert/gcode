from typing import Optional

from src.ApplicationExceptions import MelfaBaseException
from src.GPrinter import GPrinter
from src.gcode.GCmd import GCmd


def interactive_gcode(ip: str, port: int, safe_return: Optional[bool] = False) -> None:
    """
    Launches an interactive shell accepting G-code.
    :param ip: IPv4 network address of the robot
    :param port: Port of the robot
    :param safe_return:
    """
    print("Launching interactive G-code shell (robot only)...")

    # Create printer object
    printer = GPrinter.default_init(ip, port, safe_return)

    # Executing communication
    try:
        while True:
            usr_msg = input("G-Code>")
            if usr_msg.lower() in ["quit"]:
                raise KeyboardInterrupt
            elif len(usr_msg) > 0:
                # Parse G-code
                gcode = GCmd.read_cmd_str(usr_msg)
                print(str(gcode))
                printer.execute(gcode)
    except MelfaBaseException as e:
        print(str(e))
    except KeyboardInterrupt:
        print('Program terminated by user.')
    finally:
        printer.shutdown()
