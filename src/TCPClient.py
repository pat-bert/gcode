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
    HOST = '192.168.0.1'
    PORT = 65432
    BUFSIZE = 1024

    def __init__(self, host: str = HOST, port: int = PORT, bufsize: int = BUFSIZE) -> None:
        self.host = host
        self.port = port
        self.bufsize = bufsize
        self.t: Union[threading.Thread, None] = None
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

        # Create socket using context manager to omit explicitly calling close()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            # Open connection to controller
            try:
                s.connect((self.host, self.port))
            except ConnectionError as e:
                print("No connection possible.")
                raise
            except TimeoutError:
                print("Connection could not be established within time.")
                raise
            else:
                # Start thread
                self.t.start()

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
        # Put close object
        self.send_q.put(None)
        # Wait for task to finish
        self.t.join()

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

    @staticmethod
    def process_msg(_):
        """
        Process the message and send it using the protocol. Method used for worker thread.
        :param _:
        :return:
        """
        # TODO Implement protocol and message transformation
        sleep(1)
