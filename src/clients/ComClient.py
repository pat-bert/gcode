from time import sleep
from typing import Tuple, Optional

import serial
import serial.tools.list_ports

from clients.ThreadedClient import ThreadedClient
from src.clients.IClient import ClientNotAvailableError, ClientOpenError, ClientError, AmbiguousHardwareError


def validate_id(id_number: int) -> bool:
    """
    Validates the USB IDs individually.
    :param id_number: Can be any of vendor ID or product ID.
    :return: Flag to indicate whether the ID is valid.
    """
    return 0 <= id_number < 65536


class ComClient(ThreadedClient):
    """
    Implements a reusable client side for serial communication.
    """

    BAUD_RATE = 115200
    PARITY = serial.PARITY_NONE
    STOP_BIT = serial.STOPBITS_ONE
    BYTE_SIZE = serial.EIGHTBITS
    DEFAULT_WRITE_ENCODING = 'ascii'
    DEFAULT_READ_ENCODING = 'utf-8'

    def __init__(self, ids: Optional[Tuple[int, int]] = None, encodings: Tuple[str, str] = None, baud=BAUD_RATE,
                 parity=PARITY, stopbits=STOP_BIT, byte=BYTE_SIZE, port: str = None):
        """
        Create a client for the serial communication.

        :param ids: Tuple, containing vendor id and product id to be used for device identification.
                    Both IDs are usually 16-bit integers.
        :param encodings: Encoding to be used for data transformations, defaults to ASCII (write) | UTF-8 (read).
        :param baud: Baudrate to be used for the communication, defaults to 115200.
        :param parity: Paritiy bit, defaults to none.
        :param stopbits: Number of stop bits, defaults to one.
        :param byte: Byte size, defaults to eight bits.
        :param port: Pass the actual port if it is known and static.
        :raises: ValueError if the IDs cannot be converted to int.
        """
        # Get features of threaded client
        super().__init__()

        # Serial port parameters
        # TODO Check timeout setting, for sockets blocking is used when it is accessed in a separate thread
        self._ser = serial.Serial(baudrate=baud, parity=parity, stopbits=stopbits, bytesize=byte, timeout=1)
        self.port = port
        self.send_encoding, self.read_encoding = encodings or (self.DEFAULT_WRITE_ENCODING, self.DEFAULT_READ_ENCODING)
        self.terminator = '\n'

        # Unpack and transform IDs (port has precedence)
        if self.port is None:
            self.vid, self.pid = map(int, ids)
        else:
            if ids is None:
                self.vid, self.pid = None, None
            else:
                self.vid, self.pid = ids

    def hook_thread_name(self) -> Optional[str]:
        """
        Client-specific thread-naming. Can be overriden. Defaults to standard thread naming.
        :return: Thread name mentioning the port and the client type
        """
        return f'Serial Client ({self._ser.port})'

    def hook_pre_connect(self) -> None:
        """
        Identify the port to be used.
        :return: None
        """
        # Choose manner of port identification
        if self.port is not None:
            self._ser.setPort(self.port)
            print('Attempting connection to port {}.'.format(self.port))
        else:
            self._resolve_ids()

    def hook_connect(self) -> None:
        """
        Attempt to open the serial port connected to the device.
        :return: None
        :raises: ClientOpenError if there was any error while opening the port, e.g. port already open
        """
        # Attempt to open the port
        try:
            self._ser.open()
        except serial.SerialException as e:
            raise ClientOpenError(e) from e

    def _resolve_ids(self) -> None:
        """
        Attempts to find the correct device by given PID and VID. On success the serial port is configured accordingly.
        :return: None
        :raises: AmbiguousHardwareError if multiple devices have the same VID:PID.
        :raises ClientNotAvailableError if the specified VID:PID was not found.
        """
        # Search for ports matching the desired IDs
        matches = [usb for usb in serial.tools.list_ports.comports() if usb.vid == self.vid and usb.pid == self.pid]
        # Cases ordered by estimated occurence
        if len(matches) == 0:
            # Could not find desired client
            raise ClientNotAvailableError(self.vid, self.pid)
        if len(matches) == 1:
            # Configure the serial port accordingly
            self._ser.port = matches[0].device
            print('Attempting connection to {} - {}.'.format(matches[0].description, matches[0].hwid))
        else:
            # Found multiple clients
            raise AmbiguousHardwareError

    def hook_close(self) -> None:
        """
        Close the serial port.
        :return: None
        """
        # Client specific closing
        self._ser.close()

    def hook_handle_msg(self, msg: str) -> str:
        self._send(msg)
        return self._receive()

    def _receive(self) -> str:
        """
        Receive data sent on the serial port.
        :return: Message string
        :raises:
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

    def _send(self, msg: str):
        """
        Sends data on the serial port.
        :param msg: Message to be sent, will be terminated by newline character if not present.
        :return: Number of bytes written
        :raises: IOError when a specified timeout occurs
        :raises: UnicodeError if the message cannot be encoded in the specified encoding
        """
        # Send the message
        data = msg.encode(encoding=self.send_encoding)
        total_sent_bytes = 0
        total_bytes = len(data)

        # Ensure that all the data is sent
        while total_sent_bytes < total_bytes:
            # Send only the remaining data
            sent_bytes = self._ser.write(data[total_sent_bytes:])
            total_sent_bytes += sent_bytes

    def hook_pre_send(self, msg: str) -> str:
        """
        Ensure that the message is terminated properly
        :return: Processed response string
        """
        return msg if msg.endswith(self.terminator) else (msg + self.terminator)


if __name__ == '__main__':
    with ComClient((0x0403, 0x6001)) as client:
        sleep(10)
        # client.receive()

        while True:
            cmd = input('Command: ')

            if cmd == 'quit':
                break

            client.send(cmd)
            client.receive()
