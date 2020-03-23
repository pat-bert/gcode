from time import sleep
from typing import Tuple

import serial
import serial.tools.list_ports

from src.clients.IClient import IClient, ClientNotAvailableError, ClientOpenError, ClientError


def validate_id(id_number: int) -> bool:
    """
    Validates the USB IDs individually.
    :param id_number: Can be any of vendor ID or product ID.
    :return: Flag to indicate whether the ID is valid.
    """
    return 0 <= id_number < 65536


class ComClient(IClient):
    """
    Client for serial communication.
    """
    BAUD_RATE = 115200
    PARITY = serial.PARITY_NONE
    STOP_BIT = serial.STOPBITS_ONE
    BYTE_SIZE = serial.EIGHTBITS
    DEFAULT_WRITE_ENCODING = 'ascii'
    DEFAULT_READ_ENCODING = 'utf-8'

    def __init__(self, ids: Tuple[int, int], encodings: Tuple[str, str] = None, baud=BAUD_RATE, parity=PARITY,
                 stopbits=STOP_BIT, byte=BYTE_SIZE):
        """
        Create a client for the serial communication.

        :param ids: Tuple, containing vendor id and product id to be used for device identification.
                    Both IDs are usually 16-bit integers.
        :param encodings: Encoding to be used for data transformations, defaults to ASCII (write) | UTF-8 (read).
        :param baud: Baudrate to be used for the communication, defaults to 115200.
        :param parity: Paritiy bit, defaults to none.
        :param stopbits: Number of stop bits, defaults to one.
        :param byte: Byte size, defaults to eight bits.
        :raises: ValueError if the IDs cannot be converted to int.
        """

        # Create port object but do not connect
        self._ser = serial.Serial(baudrate=baud, parity=parity, stopbits=stopbits, bytesize=byte, timeout=1)
        # Unpack and transform IDs
        self.vid, self.pid = map(int, ids)
        self.send_encoding, self.read_encoding = encodings or (self.DEFAULT_WRITE_ENCODING, self.DEFAULT_READ_ENCODING)

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
                    print('Connected to {} - {}.'.format(usb_device.description, usb_device.hwid))
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

    def receive(self, silence_errors=False) -> str:
        """
        Receive data sent on the serial port.
        :param silence_errors:
        :return:
        """
        prev_count, curr_count = 0, 0  # Keep track of the number of bytes waiting in ingoing buffer for each poll
        response_buffer = []  # Collect the decoded responses
        changing = False  # Flag to indicate whether there was a change between two poll cycles
        initial = True  # Flag to indicate that there were never more than zero bytes available

        i = 0
        # Loop until any bytes were received and the number of waiting byte stays constant
        while 'ok' not in response_buffer and (initial or changing):
            # Update counters and flags
            try:
                curr_count = self._ser.in_waiting
            except serial.SerialException as e:
                raise ClientError from e

            changing = prev_count != curr_count
            prev_count = curr_count
            if curr_count > 0:
                initial = False

            # Attempt to read the current byte buffer
            try:
                raw_response = self._ser.read_all()
            except serial.SerialException as e:
                raise ClientError from e

            # Decode the current content and append it to the string buffer (including newline characters)
            response = raw_response.decode(encoding=self.read_encoding)
            response_buffer.append(response)

            # Wait for 100x the time for one character to arrive
            sleep(200 / self._ser.baudrate)
            i += 1

        # Combine the response buffer and show each line on stdout
        response = ''.join(response_buffer)
        response_lines = ['<<: {}'.format(i) for i in response.splitlines()]
        print('\n'.join(response_lines))

        return response

    def send(self, msg: str, silent_send: bool = False, silent_recv: bool = False) -> int:
        """
        :param msg:
        :param silent_recv:
        :param silent_send:
        :return: Number of bytes written
        :raises: IOError when a specified timeout occurs
        :raises: UnicodeError if the message cannot be encoded in the specified encoding
        """
        print('>>: {}'.format(msg.strip()))

        if not msg.endswith('\n'):
            msg += '\n'
        data = msg.encode(encoding=self.send_encoding)
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
    sleep(10)
    client.receive()

    while True:
        cmd = input('Command: ')

        if cmd == 'quit':
            break

        client.send(cmd)
        client.receive()

    client.close()
