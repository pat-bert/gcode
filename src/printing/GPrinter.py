from typing import *

from printing.Extruder import Extruder
from printing.GCmd import GCmd
from printing.GRedirect import RedirectionTargets, GRedirect
from printing.Heater import Heater
from printing.MelfaRobot import MelfaRobot
from printing.PrinterComponent import PrinterComponent
from printing.TcpClientR3 import TcpClientR3
from printing.UxHandler import UxHandler


class GPrinter(object):
    """
    Bundles absolutely all components necessary to control the 3D printer.
    """

    def __init__(self, *components: PrinterComponent):
        """
        Initializes the printer with a set of components.
        :param components: Usable list of printer components
        """
        self.components: Union[Dict[RedirectionTargets, Set[PrinterComponent]], Dict] = {}

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

    def execute(self, gcode: GCmd):
        try:
            target = GRedirect.redirect_cmd(gcode)
        except ValueError as e:
            print(e)
        else:
            try:
                for responsible_component in self.components[target]:
                    responsible_component.handle_gcode(gcode)
            except KeyError:
                raise ValueError("Unsupported redirection target: {}".format(target))

    def shutdown(self) -> None:
        """
        Shutdown for all unique components.
        :return:
        """
        for comp in self.unique_components:
            comp.shutdown(safe_return=False)

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
    def default_init(cls, ip, port) -> 'GPrinter':
        # Create TCP client
        tcp_client = TcpClientR3(host=ip, port=port)
        tcp_client.connect()

        # Create mover object
        mover = MelfaRobot(tcp_client, number_axes=6, speed_threshold=10)

        # Create extruder object
        extruder = Extruder()

        # Create heater object
        heater = Heater()

        # Create UX object
        user_output = UxHandler()

        # Create printer object
        return cls(mover, extruder, heater, user_output)
