from typing import Tuple

import serial
import serial.tools.list_ports

from src.clients.IClient import IClient, ClientNotAvailableError, ClientOpenError, ClientError


class ComClient(IClient):
    """
    Client for serial communication.
    """
    BAUD_RATE = 115200
    PARITY = serial.PARITY_EVEN
    STOP_BIT = serial.STOPBITS_ONE
    BYTE_SIZE = serial.EIGHTBITS
    DEFAULT_ENCODING = 'utf-8'

    def __init__(self, ids: Tuple[int, int], encoding=DEFAULT_ENCODING, baud=BAUD_RATE, parity=PARITY,
                 stopbits=STOP_BIT, byte=BYTE_SIZE):
        """
        Create a client for the serial communication.

        :param ids: Tuple, containing vendor id and product id to be used for device identification.
                    Both IDs are usually 16-bit integers.
        :param encoding: Encoding to be used for data transformations.
        :param baud: Baudrate to be used for the communication, defaults to 115200.
        :param parity: Paritiy bit, defaults to none.
        :param stopbits: Number of stop bits, defaults to one.
        :param byte: Byte size, defaults to eight bits.
        :raises: ValueError if the IDs cannot be converted to int.
        """

        # Create port object but do not connect
        self._ser = serial.Serial(baudrate=baud, parity=parity, stopbits=stopbits, bytesize=byte)
        # Unpack and transform IDs
        self.vid, self.pid = map(int, ids)
        self.encoding = encoding

    def connect(self) -> None:
        """
        Attempt to open the serial port connected to the device with the specified IDs.
        :return: None
        :raises:
        """
        available_devices = serial.tools.list_ports.comports()
        # Search for port matching the desired IDs
        for usb_device in available_devices:
            if usb_device.vid == self.vid and usb_device.pid == self.pid:
                # Get the info necessary to open the port
                self._ser.port = usb_device.device
                # Attempt to open the port
                try:
                    self._ser.open()
                except serial.SerialException as e:
                    raise ClientOpenError(e) from e
                else:
                    print('Connected to {}.'.format(usb_device.description))
                    break
        else:
            # Could not find desired client
            raise ClientNotAvailableError(self.vid, self.pid)

    def close(self) -> None:
        """
        Attempt to close the serial port.
        :return: None
        :raises: IOError if port could not be closed.
        """
        # Just call it. This should not raise an exception.
        self._ser.close()

        # Ensure that the port is closed
        if self._ser.is_open:
            raise IOError("Could not close serial port.")

    def receive(self, silence_errors=False):
        pass

    def send(self, msg: str, silent_send: bool = False, silent_recv: bool = False) -> int:
        """
        :param msg:
        :param silent_recv:
        :param silent_send:
        :return: Number of bytes written
        :raises: IOError when a specified timeout occurs
        :raises: UnicodeError if the message cannot be encoded in the specified encoding
        """
        data = msg.encode(self.encoding)
        return self._ser.write(data)

    def is_available(self) -> bool:
        """
        Check whether a serial port can currently be opened.
        :return: Boolean to indicate whether it can be opened.
        """
        # Check whether it is open currently
        if self._ser.isOpen():
            return True
        # Otherwise check whether it can be opened
        try:
            self.connect()
        except ClientError:
            return False
        else:
            return True
        finally:
            # Closing should always be done
            self.close()


if __name__ == '__main__':
    client = ComClient((0x0403, 0x6001))
    client.connect()
    client.connect()
