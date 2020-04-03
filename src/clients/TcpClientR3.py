"""
File:       TCPClient.py
Author:     Patrick Bertsch
Content:    Implement TCP/IP communication to robot
"""
import socket
import threading
from queue import Queue, Empty
from typing import AnyStr, Union

import src.protocols.R3Protocol as R3Protocol
from src import ApplicationExceptions
from src.ApplicationExceptions import TcpError
from src.clients.IClient import IClient, ServerClosedConnectionError
from src.clients.IClient import Msg


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


class TcpClientR3(IClient):
    """
    Implements the client side of the TCP/IP communication.
    The implementation uses a single blocking socket which is run in permanently in a separate process.
    Communication is achieved by putting messages to a sending queue and getting messages from a receiving queue.
    """

    # Network parameters
    HOST = "192.168.0.1"
    PORT = 10002
    BUFSIZE = 4096

    # Delimiter
    DELIMITER = R3Protocol.DELIMITER
    ENCODING = "utf-8"

    # Parameters for R3 protocol
    ROBOT_NO = 1
    PROGRAM_NO = 1

    def __init__(
            self, host: str = HOST, port: int = PORT, bufsize: int = BUFSIZE, timeout: float = 60
    ) -> None:
        """
        Initialises the objects for the TCP communication
        :param host: Host IP-Address
        :param port: Connection port
        :param bufsize: Buffer size for communication
        :param timeout: Specifies a timeout in seconds to be applied during connection phase, defaults to 60 s.
        """
        # Socket parameters (blocking)
        self.s: Union[socket.socket, None] = None
        self.host = host
        self.port = port
        self.bufsize = bufsize
        self.timeout = timeout

        # Queues and thread for communication from and to worker tread
        self.t: Union[threading.Thread, None] = None
        self.send_q = Queue()
        self.recv_q = Queue()
        self.alive = threading.Event()
        self._cnt_conn = 0

        # Status Flags
        self._is_connected = False

    def connect(self) -> None:
        """
        Connect to the robot via a worker thread for the protocol communication
        :raises: ValueError if invalid connection parameters are passed.
        :raises: TcpError for network related issues.
        :return: None
        """
        # TODO How to handle repeated calls and reconnects?
        # Create new thread
        self._cnt_conn += 1
        thread_name = 'TCP-Client ({}:{}) #{}'.format(self.host, self.port, self._cnt_conn)
        self.t = threading.Thread(target=self.mainloop, name=thread_name)

        # Create new socket
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error as e:
            raise TcpError('Failed to create new socket.') from e

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
        else:
            # Start thread
            self.alive.set()
            self.t.start()
            self._is_connected = True

    def close(self) -> None:
        """
        Terminate the worker thread for the protocol communication.
        :return: None
        """
        # Put close object
        self.send_q.put(None)
        # Wait for queues to finish
        self.recv_q.join()
        self.send_q.join()
        # Wait for task to finish, this can be done multiple times
        self.alive.clear()
        self.t.join()
        # Close socket, no mutex required since the worker thread will be closed
        self.s.close()
        self._is_connected = False
        print('Closed communication.')

    def send(self, msg: str, silent_send: bool = False, silent_recv: bool = False) -> None:
        """
        Sends a message via the worker thread for the protocol communication.
        :param msg: Message to be sent. Needs to be shorter than 128 characters.
        :param silent_send:
        :param silent_recv:
        :return: None
        :raises: TcpError if client has not been connected successfully. Otherwise all messages passed to send would be
        sent out once a connection is established.
        """
        # Put the message to the outgoing queue of the protocol, None is used to end the communication
        if self._is_connected:
            if msg is not None:
                if len(msg) <= 127:
                    msg = Msg(msg, silent_send, silent_recv)
                    self.send_q.put(msg)
                else:
                    raise ValueError("The message cannot be longer than 127 characters.")
        else:
            raise TcpError('Client needs to be connected before sending since this could lead to unexpected behavior.')

    def wait_send(self, msg: str) -> None:
        """
        Sends a message and waits until all messages are processed.
        :param msg:
        :return: None
        """
        self.send(msg)
        self.send_q.join()

    def receive(self, silence_errors=False) -> str:
        """
        Get the last response received by the worker thread.
        :param silence_errors: Specify whether exceptions should be silenced.
        :return: Message string without status code
        """
        # Get the last response from the queue
        response = self.recv_q.get()
        self.recv_q.task_done()

        # Dispatch the status code part of the response
        try:
            exception = ApplicationExceptions.ErrorDispatch[response[:3]]
        except KeyError:
            return response

        # Raise an exception if something went wrong
        if exception is not None and not silence_errors:
            # Something went wrong
            raise exception(response[3:])
        return response[3:]

    def mainloop(self) -> None:
        """
        Main loop function for the worker thread for the network communication.
        :return: None
        :raises: ServerClosedConnectionError if an empty string is received
        """
        # Event to indicate that the thread should terminate
        while self.alive.isSet():
            # Get an item from the outgoing queue of the protocol
            try:
                # Using a timeout ensures that the loop condition is checked if no item is present
                msg = self.send_q.get(timeout=0.01)
            except Empty:
                # Restart the loop
                continue
            else:
                if msg is None:
                    # End of queue
                    self.send_q.task_done()
                    break
                else:
                    # Regular message
                    msg, silent_send, silent_recv = msg.unpack()
                    response = self._handle_msg(msg, silent_recv, silent_send)

                    # Put the response and indicate that the task is done
                    self.recv_q.put(response)
                    self.send_q.task_done()

                    # Server closed down connection
                    if len(response) == 0:
                        # TODO Can this be recovered by reopening the connection on a higher level?
                        raise ServerClosedConnectionError

    def _handle_msg(self, msg: str, silent_recv, silent_send) -> str:
        # Robot message
        # TODO This is specific and should be done by the protocol layer
        # msg = '{}{d}{}{d}{}'.format(self.ROBOT_NO, self.PROGRAM_NO, msg, d=self.DELIMITER)

        # Log the message
        if not silent_send:
            print('>>: {}'.format(msg.strip()))

        # Send the message
        msg_b = bytes(msg, encoding=self.ENCODING)
        self._send_all_bytes(msg_b)

        # Receive the message
        response_b: bytes = self.s.recv(self.bufsize)
        response_str = str(response_b, encoding=self.ENCODING)

        # Log the message
        if not silent_recv:
            print("<<: {} ".format(response_str.strip()))

        return response_str

    def _send_all_bytes(self, msg_b: bytes) -> None:
        """
        Repeatedly calls the send function to ensure that all the data is sent.
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


if __name__ == '__main__':
    a = TcpClientR3(host='127.0.0.1', port=10001)
    a.connect()
