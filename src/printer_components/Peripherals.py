import threading
from typing import Optional

from src.clients.IClient import IClient
from src.gcode.GCmd import GCmd
from src.printer_components.PrinterComponent import PrinterComponent


class Peripherals(PrinterComponent):
    def __init__(self, io_client: IClient):
        super().__init__(name='Peripherals')
        self.client = io_client

    def hook_boot(self, *args, **kwargs):
        pass

    def hook_shutdown(self, *args, **kwargs):
        self.client.close()

    def hook_handle_gcode(self, gcode: GCmd, barrier: Optional[threading.Barrier]):
        """
        Implements the G-Code execution.
        :param gcode: G-Code object
        :param barrier: Synchronization primitive
        :return:
        """
        # Wait for other components if barrier is passed
        if barrier is not None:
            barrier.wait()
        # Send the command and get the response
        self.client.send(str(gcode))
        response = self.client.receive()
