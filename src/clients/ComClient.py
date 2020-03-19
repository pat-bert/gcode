import serial
from typing import Union


class ComClient:
    BAUD_RATE = 115200
    PARITY = serial.PARITY_EVEN
    STOP_BIT = serial.STOPBITS_ONE

    def __init__(self, baud=BAUD_RATE):
        """
        Create a client for the serial communication.
        :param baud:
        """

        # Create port object but do not connect
        try:
            self._ser = serial.Serial()
        except IOError:
            print("Could not open serial port.")
            raise

        # Configure serial port
        self._ser.baudrate = baud

    def open(self):
        self._ser.open()

    def close(self):
        """
        Closes the serial port.
        :return:
        """
        self._ser.close()

        # Ensure that the port is closed
        if self._ser.is_open:
            raise IOError("Could not close serial port.")

    def read(self):
        pass

    def write(self, data: Union[bytes, str], encoding=None) -> int:
        """
        :param data:
        :param encoding: Specify an encoding if the passed data is not bytes
        :return: Number of bytes written
        :raises: IOError when a specified timeout occurs
        """
        if encoding is not None:
            data = data.encode(encoding)
        return self._ser.write(data)


if __name__ == '__main__':
    ComClient()
