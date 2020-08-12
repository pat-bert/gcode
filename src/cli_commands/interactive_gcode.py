import logging
from typing import Optional, Tuple

from src.ApplicationExceptions import MelfaBaseException
from src.GCmd import GCmd
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

    print(f'Reading G-Code from file.')
    with open('xyzCalibration_cube.gcode', 'r') as f:
        cmd_raw = f.readlines()

    commands = [GCmd.read_cmd_str(cmd_str.strip()) for cmd_str in cmd_raw if not cmd_str.startswith(GCmd.COMMENT)]
    commands = [cmd for cmd in commands if cmd is not None]
    total_commands = len(commands)

    # Create printer object
    printer = GPrinter.default_init(ip, port, serial_ids=serial_ids, safe_return=safe_return)

    # Executing communication
    try:
        for idx, cmd in enumerate(commands):
            logging.info(f'Executing next ({idx + 1}/{total_commands}): {cmd}')
            printer.execute(cmd)
    except MelfaBaseException as e:
        print(str(e))
    except KeyboardInterrupt:
        print('Program terminated by user.')
    finally:
        printer.shutdown()

# Interactive:
# while True:
#     usr_msg = input("G-Code>")
#     if usr_msg.lower() in ["quit"]:
#         raise KeyboardInterrupt
#     if len(usr_msg) > 0:
#         # Parse G-code
#         gcode = GCmd.read_cmd_str(usr_msg)
#         print(str(gcode))
#         printer.execute(gcode)
