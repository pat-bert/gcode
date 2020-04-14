"""
File:       TCPClient.py
Author:     Patrick Bertsch
Content:    Implement TCP/IP communication to robot
"""
import socket
from typing import AnyStr, Union, Optional

from clients.ThreadedClient import ThreadedClient
from src import ApplicationExceptions
from src.ApplicationExceptions import TcpError


def validate_ip(ip: AnyStr) -> bool:
    try:
        return (
                all(((int(i) in range(0, 256)) for i in ip.split(".")))
                and len(ip.split(".")) == 4
        )
    except ValueError:
        return False


def validate_port(port: int) -> bool:
    return 0 < port < 65536


class TcpClientR3(ThreadedClient):
    """
    Implements the client side of the TCP/IP communication.
    The implementation uses a single blocking socket which is run in permanently in a separate process.
    Communication is achieved by putting messages to a sending queue and getting messages from a receiving queue.
    """

    # Network parameters
    HOST = "192.168.0.1"
    PORT = 10002
    BUFSIZE = 4096
    ENCODING = "utf-8"

    def __init__(self, host: str = HOST, port: int = PORT, bufsize: int = BUFSIZE, timeout: float = 60) -> None:
        """
        Initialises the objects for the TCP communication
        :param host: Host IP-Address
        :param port: Connection port
        :param bufsize: Buffer size for communication
        :param timeout: Specifies a timeout in seconds to be applied during connection phase, defaults to 60 s.
        """
        # Get features of threaded client
        super().__init__()

        # Socket parameters (blocking)
        self.s: Union[socket.socket, None] = None
        self.host = host
        self.port = port
        self.bufsize = bufsize
        self.timeout = timeout
        self._cnt_conn = 0

    def hook_thread_name(self) -> str:
        """
        Create a name for the worker thread.
        :return: Name string
        """
        self._cnt_conn += 1
        return 'TCP-Client ({}:{}) #{}'.format(self.host, self.port, self._cnt_conn)

    def hook_connect(self) -> None:
        """
        Connect to the robot via a worker thread for the protocol communication
        :raises: ValueError if invalid connection parameters are passed.
        :raises: TcpError for network related issues.
        :return: None
        """
        # Create new socket (unusable after closed)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Attemp to open a connection
        try:
            # Set a timeout to make this deterministic and testable on all machines
            self.s.settimeout(self.timeout)
            print('Attempting TCP connection to {}:{}.'.format(self.host, self.port))
            self.s.connect((self.host, self.port))
            print('Connected.')

            # Reset the socket to blocking mode
            self.s.settimeout(None)
        except socket.gaierror as e:
            # Specifically bad connection parameters
            raise TcpError('Invalid connection parameters.') from e
        except socket.timeout as e:
            # Connection timeout
            raise TcpError('Connection attempt timed out.') from e
        except socket.error as e:
            # Any other socket error
            raise TcpError('No connection possible.') from e
        # Otherwise return from the hook

    def hook_close(self) -> None:
        """
        Closing procedure of the TCP-Client.
        :return: None
        """
        # Close socket, no mutex required since the worker thread will be closed already
        self.s.close()

    def hook_pre_send(self, msg: Optional[str]) -> Optional[str]:
        """
        Pre-processes the message before sending.
        :param msg: Message to be sent. Needs to be shorter than 128 characters.
        :return: None
        :raises: ValueError if the message consists of >= 128 characters.
        """
        # Valid message can be queued
        if len(msg) <= 127:
            return msg
        else:
            raise ValueError("The message cannot be longer than 127 characters.")

    def hook_post_receive(self, response: str, silence_errors: bool) -> str:
        """
        Pre-processes the message after receving.
        :param response:
        :param silence_errors: Specify whether exceptions should be silenced.
        :return: Message string without status code
        :raises: Exception if error code is found in the response.
        """
        # Dispatch the status code part of the response
        try:
            exception = ApplicationExceptions.ErrorDispatch[response[:3]]
        except KeyError:
            return response

        # Raise an exception if something went wrong
        if exception is not None and not silence_errors:
            # Something went wrong
            raise exception(response[3:])
        # Return the rest of the response without the status code
        return response[3:]

    def hook_handle_msg(self, msg: str) -> str:
        """
        Handle the message in the worker thread and get a response.
        :param msg: Outgoing message string
        :return: Incoming response string
        """
        # Send the message
        msg_b = bytes(msg, encoding=self.ENCODING)
        self._send_all_bytes(msg_b)

        # Receive the message
        response_b: bytes = self.s.recv(self.bufsize)
        return str(response_b, encoding=self.ENCODING)

    def _send_all_bytes(self, msg_b: bytes) -> None:
        """
        Helper function repeatedly sending o ensure that all the data is sent.
        :param msg_b: Message in byte format
        :return: None
        """
        # Send the message
        total_sent_bytes = 0
        total_bytes = len(msg_b)
        # Ensure that all the data is sent
        while total_sent_bytes < total_bytes:
            # Send only the remaining data
            sent_bytes = self.s.send(msg_b[total_sent_bytes:])
            total_sent_bytes += sent_bytes
