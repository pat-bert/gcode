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
import MelfaError
import MelfaCmd
from ApplicationExceptions import TcpError
from Coordinate import *


class TCPClient(object):
    """
    Implements the PC-side of the TCP/IP connection.
    """
    # Network parameters
    HOST = '192.168.0.1'
    PORT = 10002
    BUFSIZE = 1024

    # Delimiter
    DELIM = MelfaCmd.DELIMITER
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

    def start(self, speed_threshold) -> None:
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

        # Check speed
        check_speed_threshold(speed_threshold)

        # Go safe position
        go_safe_pos()

        # Print movement borders
        xyz_borders()
        joint_borders()

        # Wait until setup is executed
        self.recv_q.join()
        self.send_q.join()
        print("Done.")

    def close(self) -> None:
        """
        Terminate the worker thread for the protocol communication.
        :return:
        """
        # Finish robot communication
        print("Finishing control...")
        go_safe_pos()
        # Reset speed
        reset_speeds()
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
        exception = MelfaError.ErrorDispatch[response[:3]]
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
            msg_str = str(self.ROBOT_NO) + self.DELIM + str(self.PROGRAM_NO) + self.DELIM + str(msg)
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


def go_safe_pos():
    # Read safe position
    tcp_client.send(MelfaCmd.PARAMETER_READ + MelfaCmd.PARAMETER_SAFE_POSITION)
    safe_pos = tcp_client.receive()
    axes = ['J' + str(i) for i in range(1, 7)]
    safe_pos_values = safe_pos.split(';')[1]
    safe_pos_values = [float(i) for i in safe_pos_values.split(', ')]
    safe_pos = Coordinate(safe_pos_values, axes)

    # Return to safe position
    tcp_client.send(MelfaCmd.MOVE_SAFE_POSITION)
    tcp_client.receive()
    cmp_response(MelfaCmd.CURRENT_JOINT, safe_pos.to_melfa_response())


def get_ovrd_speed():
    tcp_client.wait_send(MelfaCmd.OVERWRITE_CMD)
    speed = tcp_client.receive()
    return float(speed)


def reset_speeds():
    tcp_client.send(MelfaCmd.MVS_SPEED + MelfaCmd.MVS_MAX_SPEED)
    tcp_client.receive()
    # TODO Reset MOV Speed
    # tcp_client.send(MelfaCmd.MOV_SPEED + MelfaCmd.MOV_MAX_SPEED)
    # tcp_client.receive()


def check_speed_threshold(speed_threshold=10):
    reset_speeds()
    # Check for low speed
    speed = get_ovrd_speed()
    if speed > speed_threshold:
        try:
            speed_correct_value = speed_threshold / speed * 100
            tcp_client.send(MelfaCmd.MVS_SPEED + str(speed_correct_value))
            tcp_client.receive()
            tcp_client.send(MelfaCmd.MOV_SPEED + str(speed_correct_value / 10))
            tcp_client.receive()
        except MelfaError.MelfaBaseException:
            raise MelfaError.MelfaBaseException("Please ensure a speed lower or equal 10% in interactive mode!")
    else:
        print("Speed of " + str(speed) + "%. Okay!")


def interactive_shell():
    while True:
        usr_msg = input("Melfa>")
        if usr_msg.lower() in ['quit']:
            raise KeyboardInterrupt
        elif len(usr_msg) > 0:
            tcp_client.wait_send(usr_msg.upper())
            try:
                tcp_client.receive()
            except MelfaError.MelfaBaseException as ex:
                # Print error message
                if len(ex.status) > 0:
                    print(str(ex))
                else:
                    # Resolve empty status codes
                    print("Empty status code. Trying to resolve.")
                    tcp_client.wait_send("ERROR")
                    try:
                        tcp_client.receive()
                    except MelfaError.MelfaBaseException as ex_res:
                        print(str(ex_res))
                # Reset alarm
                sleep(1)
                print("Error Reset")
                tcp_client.send(MelfaCmd.ALARM_RESET_CMD)
                tcp_client.receive(silence_errors=True)


def cmp_response(poll_cmd: str, response_t: str, poll_rate_ms: int = 3, timeout_s: int = 300):
    """
    Uses a given command to poll for a given response.
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


def flat_square():
    axes = 'XYZABC'

    increment = Coordinate([0, 0, 15, 0, 0, 0], axes)
    p_l = [
        Coordinate([500, 100, 200, 180, 0, 0], axes),
        Coordinate([500, -100, 200, 180, 0, 0], axes),
        Coordinate([600, -100, 200, 180, 0, 0], axes),
        Coordinate([600, 100, 200, 180, 0, 0], axes)
    ]

    for _ in range(10):
        # Square
        for point in p_l:
            tcp_client.send(MelfaCmd.LINEAR_INTRP + point.to_melfa_point())
            tcp_client.receive()
            cmp_response(MelfaCmd.CURRENT_XYZABC, point.to_melfa_response())

        # Back to first point
        tcp_client.send(MelfaCmd.LINEAR_INTRP + p_l[0].to_melfa_point())
        tcp_client.receive()
        cmp_response(MelfaCmd.CURRENT_XYZABC, p_l[0].to_melfa_response())

        p_l = [i + increment for i in p_l]


def xyz_borders():
    tcp_client.send(MelfaCmd.PARAMETER_READ + MelfaCmd.XYZ_BORDERS)
    response = tcp_client.receive()
    coordinate_str = response.split(MelfaCmd.DELIMITER)[1]
    coordinates = coordinate_str.split(', ')
    return [float(i) for i in coordinates]


def joint_borders():
    tcp_client.send(MelfaCmd.PARAMETER_READ + MelfaCmd.JOINT_BORDERS)
    response = tcp_client.receive()
    coordinate_str = response.split(MelfaCmd.DELIMITER)[1]
    coordinates = coordinate_str.split(', ')
    return [float(i) for i in coordinates]


if __name__ == '__main__':
    # Create TCP client
    tcp_client = TCPClient()
    tcp_client.connect()

    # Executing communication
    try:
        tcp_client.start(speed_threshold=10)
        selection = input("Please choose a mode (1=interactive, 2=flat square): ")
        if selection == '1':
            interactive_shell()
        elif selection == '2':
            flat_square()
        else:
            raise NotImplementedError
    except KeyboardInterrupt:
        pass
    except NotImplementedError:
        pass
    except MelfaError.MelfaBaseException as e:
        print(str(e))
    finally:
        # Cleaning up
        tcp_client.close()

# TODO Check safe pos is reached
