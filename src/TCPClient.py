"""
File:       TCPClient.py
Author:     Patrick Bertsch
Content:    Implement TCP/IP communication to robot
"""
import socket
import threading
from time import sleep
from typing import *
from queue import Queue


class TCPClient:
    """
    Implements the PC-side of the TCP/IP connection.
    """
    # Network parameters
    HOST = '127.0.0.1'
    PORT = 10001
    BUFSIZE = 1024

    # Basic commands for R3 protocol
    SRV_ON_MSG = ''
    SRV_OFF_MSG = ''
    CNTL_ON_MSG = ''
    CNTL_OFF_MSG = ''

    # Parameters for R3 protocol
    ROBOT_NO = 1
    PROGRAM_NO = 1

    # Status codes from robot controller
    STATUS_DONE = "QoK"
    COMMAND_OK_MINOR_ISSUE = "Qok"
    COMMAND_OK_INVALID = "QeR"
    NOT_UNDERSTOOD = "Qer"

    def __init__(self, host: str = HOST, port: int = PORT, bufsize: int = BUFSIZE) -> None:
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
            self.s.connect((self.host, self.port))
        except ConnectionError as e:
            print(e)
            print("No connection possible.")
            raise
        except TimeoutError:
            print("Connection could not be established within time.")
            raise
        else:
            # Start thread
            self.t.start()

    def start(self) -> None:
        """
        Initialises the robot communication
        :return:
        """
        self.send(self.CNTL_ON_MSG)
        self.send(self.SRV_ON_MSG)

    def send(self, msg: str) -> None:
        """
        Sends a message via the worker thread for the protocol communication.
        :param msg:
        :return:
        """
        # Put the message to the outgoing queue of the protocol
        self.send_q.put(msg)

    def close(self) -> None:
        """
        Terminate the worker thread for the protocol communication.
        :return:
        """
        # Finish robot communication
        self.send(self.SRV_OFF_MSG)
        self.send(self.CNTL_OFF_MSG)
        # Put close object
        self.send_q.put(None)
        # Wait for task to finish
        self.t.join()
        # Close socket
        self.s.close()

    def mainloop(self) -> None:
        """
        Main loop function for the worker thread for the protocol communication.
        :return:
        """
        while True:
            # Get an item from the outgoing queue of the protocol
            msg = self.send_q.get()
            # Process the message
            self.process_msg(msg)
            # Indicate that the task is done
            self.send_q.task_done()

    def process_msg(self, msg: bytes) -> None:
        """
        Process the message and send it using the protocol. Method used for worker thread.
        :param msg: Message to be sent
        :return:
        """
        # TODO Implement R3 protocol and message transformation
        self.s.send(msg)
        ans = self.s.recv(self.bufsize)
        sleep(1)
