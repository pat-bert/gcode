import logging
from typing import Optional, Tuple

from src.ApplicationExceptions import MelfaBaseException
from src.gcode.GCmd import GCmd
from src.printer_components.GPrinter import GPrinter


def interactive_gcode(ip: str, port: int, serial_ids: Tuple[int, int], safe_return: Optional[bool] = False) -> None:
    """
    Launches an interactive shell accepting G-code.
    :param ip: IPv4 network address of the robot
    :param port: Port of the robot
    :param serial_ids:
    :param safe_return:
    """
    logging.info("Launching interactive G-code shell...")

    # Create printer object
    printer = GPrinter.default_init(ip, port, serial_ids=serial_ids, safe_return=safe_return)

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
                printer.execute(gcode)
    except MelfaBaseException as e:
        print(str(e))
    except KeyboardInterrupt:
        print('Program terminated by user.')
    finally:
        printer.shutdown()
