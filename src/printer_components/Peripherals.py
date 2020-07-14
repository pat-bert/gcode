from typing import Iterable

from src.clients.IClient import IClient
from src.gcode.GCmd import GCmd
from src.printer_components.GRedirect import RedirectionTargets
from src.printer_components.PrinterComponent import PrinterComponent


class Peripherals(PrinterComponent):
    def __init__(self, io_client: IClient):
        self.client = io_client

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        self.client.close()

    @property
    def redirector(self) -> Iterable[RedirectionTargets]:
        return [RedirectionTargets.EXTRUDER]

    def handle_gcode(self, gcode: GCmd, *args, **kwargs):
        self.client.send(str(gcode))
        response = self.client.receive()
