import logging
from threading import Barrier
from typing import Tuple, Optional

from src.clients.ComClient import ComClient
from src.clients.TcpClientR3 import TcpClientR3
from src.GCmd import GCmd
from src.printer_components.MelfaRobot import MelfaRobot
from src.printer_components.Peripherals import Peripherals
from src.printer_components.PrinterComponent import PrinterComponent


class GPrinter:
    """
    Bundles absolutely all components necessary to control the 3D printer.
    """

    def __init__(self, *components: PrinterComponent):
        """
        Initializes the printer with a set of components.
        :param components: Usable list of printer components
        """
        self.barrier = Barrier(len(components))
        self.components = components
        # Boot component
        for component in components:
            component.boot()

    def execute(self, gcode: GCmd) -> None:
        """
        Execute a G-code command on the printer.
        :param gcode: Command object
        """
        logging.info(f'Printer executing command: {gcode}')
        for component in self.components:
            component.assign_task(gcode, barrier=self.barrier)

        for component in self.components:
            result = component.get_result()
            logging.info(f'{component.name}: {str(result).strip()}')

    def shutdown(self) -> None:
        """
        Shutdown for all unique components.
        """
        logging.info('Shutting down all components.')
        for component in self.components:
            component.shutdown()

    @classmethod
    def default_init(cls, ip: str, port: int, serial_port: Optional[str] = None,
                     serial_ids: Optional[Tuple[int, int]] = None, safe_return=False) -> "GPrinter":
        """
        Create a printer for the default setup.
        :param ip: IPv4 address for the TCP/IP-communication with the Melfa robot
        :param port: Port for the TCP/IP-communication with the Melfa robot
        :param serial_ids: Tuple, containing vendor id and product id to be used for device identification (VID, PID).
        :param serial_port: Serial port, if given it takes precedence
        Both IDs are usually 16-bit integers.
        :param safe_return: Flag to specify whether the robot should start and stop at its safe position, defaults to
        false.
        :return: Printer object to be used to control the printer
        """
        # Validate COM client data
        if serial_port is None:
            if serial_ids is None:
                raise ValueError('Need to specifiy either serial port or serial IDs.')
            com_client = ComClient(ids=serial_ids)
        else:
            com_client = ComClient(port=serial_port)

        # Create clients and connect
        tcp_client = TcpClientR3(host=ip, port=port)
        tcp_client.connect()

        com_client.connect()

        # Create mover object
        mover = MelfaRobot(tcp_client, number_axes=6, speed_threshold=10, safe_return=safe_return)

        # Create object for remaining components
        perip = Peripherals(com_client)

        # Create printer object
        return cls(mover, perip)
