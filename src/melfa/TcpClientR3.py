"""
File:       TCPClient.py
Author:     Patrick Bertsch
Content:    Implement TCP/IP communication to robot
"""
import socket
import threading
from queue import Queue
from time import sleep

from src import ApplicationExceptions
from src.melfa import MelfaCmd
from src.ApplicationExceptions import TcpError
from src.Coordinate import *


class Msg(object):
    def __init__(self, msg, silent_send, silent_recv):
        self.msg = msg
        self.ss = silent_send
        self.sr = silent_recv

    def unpack(self):
        return self.msg, self.ss, self.sr


class AbstractTcp(object):
    def send(self, *args, **kwargs):
        pass

    def receive(self, *args, **kwargs):
        pass


class TcpClientR3(AbstractTcp):
    """
    Implements the PC-side of the TCP/IP connection.
    """

    # Network parameters
    HOST = "192.168.0.1"
    PORT = 10002
    BUFSIZE = 1024

    # Delimiter
    DELIMITER = MelfaCmd.DELIMITER
    ENCODING = "utf-8"

    # Parameters for R3 protocol
    ROBOT_NO = 1
    PROGRAM_NO = 1

    def __init__(
            self, host: str = HOST, port: int = PORT, bufsize: int = BUFSIZE
    ) -> None:
        """
        Initialises the objects for the TCP communication
        :param host: Host IP-Address
        :param port: Connection port
        :param bufsize: Buffer size for communication
        """
        # Socket parameters
        self.host = host
        self.port = port
        self.bufsize = bufsize

        # Thread and socket properties
        self.t: Union[threading.Thread, None] = None
        self.s: Union[socket.socket, None] = None

        # Queues for communication from and to worker tread
        self.send_q = Queue()
        self.recv_q = Queue()

    def connect(self) -> None:
        """
        Connect to the robot via a worker thread for the protocol communication
        :raises: ConnectionError
        :raises: TimeoutError
        :return:
        """
        # Create new thread
        self.t = threading.Thread(target=self.mainloop)

        try:
            # Create new socket
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Open connection to controller
            print("Initialising TCP connection.")
            self.s.connect((self.host, self.port))
            print("Connected.")
        except ConnectionError as e:
            print(e)
            print("No connection possible.")
            raise TcpError
        except TimeoutError:
            print("Connection could not be established within time.")
            raise TcpError
        else:
            # Start thread
            self.t.start()

    def close(self) -> None:
        """
        Terminate the worker thread for the protocol communication.
        :return:
        """
        # Put close object
        self.send_q.put(None)
        # Wait for queue to finish
        self.recv_q.join()
        self.send_q.join()
        # Wait for task to finish
        self.t.join()
        # Close socket
        self.s.close()
        print("Done.")

    def send(
            self, msg: str, silent_send: bool = False, silent_recv: bool = False
    ) -> None:
        """
        Sends a message via the worker thread for the protocol communication.
        :param msg:
        :param silent_send:
        :param silent_recv:
        :return:
        """
        # Put the message to the outgoing queue of the protocol
        if msg is not None:
            if len(msg) <= 127:
                msg = Msg(msg, silent_send, silent_recv)
                self.send_q.put(msg)
            else:
                raise ValueError("The message cannot be longer than 127 characters.")

    def wait_send(self, msg: str) -> None:
        """
        Sends a message and waits until all messages are processed.
        :param msg:
        :return:
        """
        self.send(msg)
        self.send_q.join()

        if msg == MelfaCmd.SRV_ON:
            sleep(MelfaCmd.SERVO_INIT_SEC)

    def receive(self, silence_errors=False) -> str:
        response = self.recv_q.get()
        exception = ApplicationExceptions.ErrorDispatch[response[:3]]
        self.recv_q.task_done()
        if exception is not None and not silence_errors:
            # Something went wrong
            raise exception(response[3:])
        return response[3:]

    def mainloop(self) -> None:
        """
        Main loop function for the worker thread for the protocol communication.
        :return:
        """
        while True:
            # Get an item from the outgoing queue of the protocol
            msg = self.send_q.get()

            # Check for end of queue
            if msg is None:
                self.send_q.task_done()
                break

            # Unpack message
            msg, silent_send, silent_recv = msg.unpack()

            # Robot message
            msg_str = (
                    str(self.ROBOT_NO)
                    + self.DELIMITER
                    + str(self.PROGRAM_NO)
                    + self.DELIMITER
                    + str(msg)
            )
            if not silent_send:
                print("Sending:\t " + msg_str)
            msg_b = bytes(msg_str, encoding=self.ENCODING)

            # Send the message
            self.s.send(msg_b)

            # Handle the receive
            response: bytes = self.s.recv(self.bufsize)
            response_str = str(response, encoding=self.ENCODING)
            if not silent_recv:
                print("Received:\t " + response_str)
            self.recv_q.put(response_str)

            # Indicate that the task is done
            self.send_q.task_done()
