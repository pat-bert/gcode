import logging
import threading
from typing import Optional, Union

from src.clients.IClient import IClient, ClientError
from src.GCmd import GCmd
from src.printer_components.PrinterComponent import PrinterComponent


class Peripherals(PrinterComponent):
    """
    Class representing all peripherals connected to the printer PCB (e.g. Megatronics v3)
    """
    def __init__(self, io_client: IClient):
        """
        Link the I/O-client to be used.
        :param io_client: Client implementing the IClient interface
        """
        super().__init__(name='Peripherals')
        self.client = io_client

    def hook_boot(self, *args, **kwargs) -> None:
        pass

    def hook_shutdown(self, *args, **kwargs) -> None:
        """
        Shuts down the component by disconnecting the client.
        :param args: -
        :param kwargs: -
        :return: None
        """
        self.client.close()

    def hook_handle_gcode(self, gcode: GCmd, barrier: Optional[threading.Barrier]) -> Union[str, Exception]:
        """
        Implements the G-Code execution.
        :param gcode: G-Code object
        :param barrier: Synchronization primitive
        :return: Response string or exception thrown by the client
        """
        # Wait for other components if barrier is passed
        if barrier is not None:
            logging.debug(f'{self.name} reached barrier')
            barrier.wait()
            logging.debug(f'{self.name} passed barrier')

        # Skip the homing since the endstops are disconnected
        if gcode.id == 'G28':
            return ''

        try:
            # Send the command and get the response
            self.client.send(str(gcode))
            response = self.client.receive()
        except ClientError as e:
            return e
        else:
            return response
