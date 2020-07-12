from typing import Optional, Tuple

from ApplicationExceptions import MelfaBaseException
from src.clients.ComClient import ComClient


def interactive_gcode_printer_only(usb_ids: Optional[Tuple[int, int]]) -> None:
    """
    Launches an interactive shell accepting G-code to send to a printer PCB.
    :param usb_ids: USB VID and PID to identify device
    """
    print("Launching interactive G-code shell (printer only)...")

    # Create com client
    with ComClient(usb_ids) as client:
        # Executing communication
        try:
            while True:
                usr_msg = input("G-Code>")
                if usr_msg.lower() in ["quit"]:
                    raise KeyboardInterrupt
                elif len(usr_msg) > 0:
                    client.send(usr_msg)
                    client.receive()
        except KeyboardInterrupt:
            print('Program terminated by user.')
        except MelfaBaseException as e:
            print(str(e))

# Commands to filter:
# G28 (endstops do not report anything)

# Deactivate software endstops: M211 S0 ?
