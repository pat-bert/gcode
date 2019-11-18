"""
File:       TCPClient.py
Author:     Patrick Bertsch
Content:    Implement TCP/IP communication to robot
"""
import socket
import threading
from time import sleep
from queue import Queue

import ApplicationExceptions
import MelfaCmd
from ApplicationExceptions import TcpError
from Coordinate import *
from MelfaRobot import joint_borders, xyz_borders, go_safe_pos, reset_speeds, check_speed_threshold


class TcpClientR3(object):
    """
    Implements the PC-side of the TCP/IP connection.
    """
    # Network parameters
    HOST = '192.168.0.1'
    PORT = 10002
    BUFSIZE = 1024

    # Delimiter
    DELIMITER = MelfaCmd.DELIMITER
    ENCODING = 'utf-8'

    # Parameters for R3 protocol
    ROBOT_NO = 1
    PROGRAM_NO = 1

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

    def start(self, speed_threshold, internal=True) -> None:
        """
        Initialises the robot communication
        :return:
        """
        print("Initialising control...")

        # Open communication and obtain control
        self.send(MelfaCmd.COM_OPEN)
        self.receive()
        self.send(MelfaCmd.CNTL_ON)
        self.receive()

        # Switch servos on
        self.send(MelfaCmd.SRV_ON)
        self.receive()
        sleep(MelfaCmd.SERVO_INIT_SEC)

        if internal:
            # Check speed
            check_speed_threshold(self, speed_threshold=speed_threshold)

            # Go safe position
            go_safe_pos(self)
            # Print movement borders
            xyz_borders(self)
            joint_borders(self)

        # Wait until setup is executed
        self.recv_q.join()
        self.send_q.join()
        print("Done.")

    def close(self, internal=True) -> None:
        """
        Terminate the worker thread for the protocol communication.
        :return:
        """
        # Finish robot communication
        print("Finishing control...")
        if internal:
            go_safe_pos(self)
            # Reset speed
            reset_speeds(self)
        self.send(MelfaCmd.SRV_OFF)
        self.receive()
        self.send(MelfaCmd.CNTL_OFF)
        self.receive()
        self.send(MelfaCmd.COM_CLOSE)
        self.receive()
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

    def send(self, msg: str) -> None:
        """
        Sends a message via the worker thread for the protocol communication.
        :param msg:
        :return:
        """
        # Put the message to the outgoing queue of the protocol
        if msg is not None:
            self.send_q.put(msg)

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

            # Robot message
            msg_str = str(self.ROBOT_NO) + self.DELIMITER + str(self.PROGRAM_NO) + self.DELIMITER + str(msg)
            print("Sending:\t " + msg_str)
            msg_b = bytes(msg_str, encoding=self.ENCODING)

            # Send the message
            self.s.send(msg_b)

            # Handle the receive
            response: bytes = self.s.recv(self.bufsize)
            response_str = str(response, encoding=self.ENCODING)
            print("Received:\t " + response_str)
            self.recv_q.put(response_str)

            # Indicate that the task is done
            self.send_q.task_done()


def cmp_response(poll_cmd: str, response_t: str, tcp_client, poll_rate_ms: int = 3, timeout_s: int = 300):
    """
    Uses a given command to poll for a given response.
    :param tcp_client:
    :param poll_cmd: Command used to execute the poll
    :param response_t: Target response string
    :param poll_rate_ms: Poll rate in milliseconds
    :param timeout_s: Time until timeout in seconds
    :return:
    """
    t = 0
    timeout_ms = timeout_s * 1000
    while t < timeout_ms:
        # Handle communication
        tcp_client.send(poll_cmd)
        response_act = tcp_client.receive()

        # Check response
        if response_act.startswith(response_t):
            break

        # Delay
        sleep(poll_rate_ms / 1000)
        t += poll_rate_ms
    else:
        raise TcpError("Timeout for check.")
