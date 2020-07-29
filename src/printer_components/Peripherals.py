import logging
import threading
from typing import Optional, Union

from src.clients.IClient import IClient, ClientError
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

    def hook_handle_gcode(self, gcode: GCmd, barrier: Optional[threading.Barrier]) -> Union[str, Exception]:
        """
        Implements the G-Code execution.
        :param gcode: G-Code object
        :param barrier: Synchronization primitive
        :return:
        """
        # Wait for other components if barrier is passed
        if barrier is not None:
            logging.info(f'{self.name} reached barrier')
            barrier.wait()
            logging.info(f'{self.name} passed barrier')
        # Send the command and get the response
        try:
            self.client.send(str(gcode))
            response = self.client.receive()
        except ClientError as e:
            return e
        else:
            return response
