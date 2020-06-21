from typing import Dict, Union, Set, Tuple

from src.clients.ComClient import ComClient
from src.clients.TcpClientR3 import TcpClientR3
from src.gcode.GCmd import GCmd
from src.printer_components.GRedirect import RedirectionTargets, GRedirect
from src.printer_components.MelfaRobot import MelfaRobot
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
        self.components: Union[
            Dict[RedirectionTargets, Set[PrinterComponent]], Dict
        ] = {}

        # Initialize components and register
        for printer_component in components:
            # Link component to all its redirection targets
            for red in printer_component.redirector:
                if red not in self.components.keys():
                    self.components[red] = {printer_component}
                else:
                    self.components[red].add(printer_component)
            # Boot component
            printer_component.boot()

    def execute(self, gcode: GCmd) -> None:
        """
        Execute a G-code command on the printer.
        :param gcode: Command object
        """
        try:
            target = GRedirect.redirect_cmd(gcode)
        except ValueError as e:
            print(e)
        else:
            try:
                # Queue the commands for all required components
                for responsible_component in self.components[target]:
                    responsible_component.handle_gcode(gcode)

                # TODO Fire the communication
            except KeyError:
                raise ValueError("Unsupported redirection target: {}".format(target))

    def shutdown(self) -> None:
        """
        Shutdown for all unique components.
        """
        for comp in self.unique_components:
            comp.shutdown()

    @property
    def unique_components(self) -> Set[PrinterComponent]:
        """
        Acquires a list of unique components registered in all redirection targets.
        :return: Set of PrinterComponent objects.
        """
        comp = set()
        for val_set in self.components.values():
            comp.update(val_set)
        return comp

    @classmethod
    def default_init(cls, ip: str, port: int, serial_ids: Tuple[int, int] = (0x0403, 0x6001),
                     safe_return=False) -> "GPrinter":
        """
        Create a printer for the default setup.
        :param ip: IPv4 address for the TCP/IP-communication with the Melfa robot
        :param port: Port for the TCP/IP-communication with the Melfa robot
        :param serial_ids: Tuple, containing vendor id and product id to be used for device identification.
        Both IDs are usually 16-bit integers.
        :param safe_return: Flag to specify whether the robot should start and stop at its safe position, defaults to
        false.
        :return: Printer object to be used to control the printer
        """
        # Create clients and connect
        tcp_client = TcpClientR3(host=ip, port=port)
        tcp_client.connect()
        # com_client = ComClient(serial_ids)
        # com_client.connect()

        # Create mover object
        mover = MelfaRobot(tcp_client, number_axes=6, speed_threshold=10, safe_return=safe_return)

        # Create object for remaining components

        # Create printer object
        return cls(mover)
