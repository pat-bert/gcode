import threading
from queue import Queue
from time import sleep
from typing import Tuple, Union

import serial
import serial.tools.list_ports

from src.clients.IClient import IClient, ClientNotAvailableError, ClientOpenError, ClientError, AmbiguousHardwareError, \
    ClientCloseError


def validate_id(id_number: int) -> bool:
    """
    Validates the USB IDs individually.
    :param id_number: Can be any of vendor ID or product ID.
    :return: Flag to indicate whether the ID is valid.
    """
    return 0 <= id_number < 65536


class ComClient(IClient):
    """
    Implements the client side of the serial communication.
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
        # TODO Check timeout setting, for sockets blocking is used when it is accessed in a separate thread
        self._ser = serial.Serial(baudrate=baud, parity=parity, stopbits=stopbits, bytesize=byte, timeout=1)

        # Unpack and transform IDs
        self.vid, self.pid = map(int, ids)
        self.send_encoding, self.read_encoding = encodings or (self.DEFAULT_WRITE_ENCODING, self.DEFAULT_READ_ENCODING)

        # Queues and thread for communication from and to worker tread
        self.t: Union[threading.Thread, None] = None
        self.send_q = Queue()
        self.recv_q = Queue()

    def connect(self) -> None:
        """
        Attempt to open the serial port connected to the device with the specified IDs.
        :return: None
        :raises: AmbiguousHardwareError if multiple devices have the same VID:PID.
        :raises: ClientOpenError if there was any error while opening the port, e.g. port already open
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

            # Attempt to open the port
            try:
                self._ser.open()
            except serial.SerialException as e:
                raise ClientOpenError(e) from e
            else:
                # Successful attempt, worker thread can now be started and the search for the port is done
                print('Connected to {} - {}.'.format(matches[0].description, matches[0].hwid))
                self.t = threading.Thread(target=self.mainloop)
        else:
            # Found multiple clients
            raise AmbiguousHardwareError

    def close(self) -> None:
        """
        Attempt to close the serial port.
        :return: None
        :raises: ClientCloseError if port could not be closed.
        """

        # self.t.join()
        # Just call it. This should not raise an exception.
        self._ser.close()

        # Ensure that the port is closed
        if self._ser.is_open:
            raise ClientCloseError('Could not close serial port {}.'.format(self._ser.port))

    def receive(self, silence_errors=False) -> str:
        """
        Receive data sent on the serial port.
        :param silence_errors:
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

    def send(self, msg: str, silent_send: bool = False, silent_recv: bool = False):
        """
        Sends data on the serial port.
        :param msg: Message to be sent, will be terminated by newline character if not present.
        :param silent_recv:
        :param silent_send:
        :return: Number of bytes written
        :raises: IOError when a specified timeout occurs
        :raises: UnicodeError if the message cannot be encoded in the specified encoding
        """
        # Display the message to the user
        if not silent_send:
            print('>>: {}'.format(msg.strip()))

        # Ensure that the message is terminated properly
        if not msg.endswith('\n'):
            msg += '\n'
        data = msg.encode(encoding=self.send_encoding)

        # Send the message
        total_sent_bytes = 0
        total_bytes = len(data)

        # Ensure that all the data is sent
        while total_sent_bytes < total_bytes:
            # Send only the remaining data
            sent_bytes = self._ser.write(data[total_sent_bytes:])
            total_sent_bytes += sent_bytes

    def mainloop(self):
        pass

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

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
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
