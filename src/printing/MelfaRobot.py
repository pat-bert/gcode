from math import sin, cos
from time import sleep
from typing import *

from printing import MelfaCmd
from printing.Coordinate import Coordinate
from printing.GRedirect import RedirectionTargets
from printing.PrinterComponent import PrinterComponent
from printing.TcpClientR3 import TcpClientR3
from printing.refactor import cmp_response, xyz_borders, joint_borders


class MelfaRobot(PrinterComponent):
    """
    Class representing the physical robots with its unique routines, properties and actions.
    """

    redirector = RedirectionTargets.MOVER
    axes = 'XYZABC'

    def __init__(self, tcp_client, number_axes: int = 6):
        """
        Initialises the robot.
        :param tcp_client: Communication object for TCP/IP-protocol
        :param number_axes: Number of robot axes, declared by 'J[n]', n>=1
        """
        if not hasattr(tcp_client, 'send') or not hasattr(tcp_client, 'receive'):
            raise TypeError('TCP-client does not implement required methods.')
        if not number_axes > 0:
            raise TypeError('Illegal number of axes.')

        self.tcp: TcpClientR3 = tcp_client
        self.joints: Sized[AnyStr] = set(['J' + str(i) for i in range(1, number_axes + 1)])
        self.servo: bool = False
        self.com_ctrl: bool = False

    # Administration functions
    def boot(self):
        pass

    def start(self, safe_return: bool = True) -> None:
        """
        Starts the robot and initialises it.
        :param safe_return: Flag, whether the robot should return to its safe position
        :return: None
        """
        # Communication & Control on
        self.change_communication_state(True)
        # Servos on
        self.change_servo_state(True)
        # Safe position
        if safe_return:
            self.go_safe_pos()

    def shutdown(self, safe_return: bool = True) -> None:
        """
        Safely shuts down the robot.
        :param safe_return: Flag, whether the robot should return to its safe position
        :return: None
        """
        # Safe position
        if safe_return:
            self.go_safe_pos()
        # Servos off
        self.change_servo_state(False)
        # Communication & Control off
        self.change_communication_state(False)

    def maintenance(self):
        # Communication & Control on
        self.change_communication_state(True)

    # Utility functions
    def change_communication_state(self, activate: bool) -> None:
        """
        Obtain/release communication and control.
        :param activate: Boolean
        :return: None
        """
        if activate:
            # Open communication and obtain control
            self.tcp.send(MelfaCmd.COM_OPEN)
            self.tcp.receive()
            self.tcp.send(MelfaCmd.CNTL_ON)
            self.tcp.receive()
        else:
            # Open communication and obtain control
            self.tcp.send(MelfaCmd.CNTL_OFF)
            self.tcp.receive()
            self.tcp.send(MelfaCmd.COM_CLOSE)
            self.tcp.receive()

        self.com_ctrl = activate

    def change_servo_state(self, activate: bool) -> None:
        """
        Switch the servos on/off.
        :param activate: Boolean
        :return: None
        """
        if activate:
            self.tcp.send(MelfaCmd.SRV_ON)
            self.tcp.receive()
        else:
            self.tcp.send(MelfaCmd.SRV_OFF)
            self.tcp.receive()

        # Wait for servos to finish
        sleep(MelfaCmd.SERVO_INIT_SEC)
        self.servo = activate

    def read_parameter(self, parameter: AnyStr):
        self.tcp.send(MelfaCmd.PARAMETER_READ + parameter)
        response = self.tcp.receive()
        return response

    # Speed functions
    def set_speed_factors(self, factor):
        """
        Set the speed modification factors for joint and interpolation movement.
        :param factor:
        :return:
        """
        pass

    def reset_speed_factors(self):
        """
        Reset the speed modification factors to maximum speed.
        :return:
        """
        # TODO Check whether OVRD can be set as well
        pass

    # Movement functions
    def go_safe_pos(self) -> None:
        """
        Moves the robot to its safe position.
        :return:
        """
        # Read safe position
        self.tcp.send(MelfaCmd.PARAMETER_READ + MelfaCmd.PARAMETER_SAFE_POSITION)
        safe_pos = self.tcp.receive()
        safe_pos_values = [float(i) for i in safe_pos.split(';')[1].split(', ')]
        safe_pos = Coordinate(safe_pos_values, self.joints)

        # Return to safe position
        self.tcp.send(MelfaCmd.MOVE_SAFE_POSITION)
        self.tcp.receive()

        # Wait until position is reached
        cmp_response(MelfaCmd.CURRENT_JOINT, safe_pos.to_melfa_response(), self.tcp)

    def linear_move_poll(self, target_pos: Coordinate, speed: float) -> None:
        """
        Moves the robot linearly to a coordinate.
        :param target_pos: Coordinate for the target position.
        :param speed: Movement speed for tool.
        :return:
        """
        # TODO Set speed accordingly

        # Send move command
        self.tcp.send(MelfaCmd.LINEAR_INTRP + target_pos.to_melfa_point())
        self.tcp.receive()

        # Wait until position is reached
        cmp_response(MelfaCmd.CURRENT_XYZABC, target_pos.to_melfa_response(), self.tcp)

    def circular_move_poll(self, target_pos: Coordinate, center_pos: Coordinate, is_clockwise: bool,
                           speed: float) -> None:
        """
        Moves the robot on a (counter-)clockwise arc around a center position to a target position.
        :param target_pos: Coordinate for the target position.
        :param center_pos: Coordinate for the center of the arc.
        :param is_clockwise: Flag to indicate clockwise|counter-clockwise direction.
        :param speed: Movement speed for tool.
        :return:
        """
        # TODO Set speed accordingly

        # Set direct/indirect movement flag
        if is_clockwise:
            pass
        else:
            pass

        # Current position
        self.tcp.send(MelfaCmd.CURRENT_XYZABC)
        response = self.tcp.receive()

        # Reconstruct coordinate
        start_pos = Coordinate.from_melfa_response(response, len(self.joints))

        # TODO Send move command with coordinates and direct/indirect flag

        # TODO Check reason for error -> might need to write variables first
        positions = ','.join([start_pos.to_melfa_crcl(), target_pos.to_melfa_crcl(), center_pos.to_melfa_crcl()])
        self.tcp.send(MelfaCmd.CIRCULAR_INTRP + positions)
        self.tcp.receive()

        # Wait until position is reached
        cmp_response(MelfaCmd.CURRENT_XYZABC, target_pos.to_melfa_response(), self.tcp)

    def calibrate_origin(self, length, width, height, l0, l1, l2, l3, l4, l5):
        # Cartesian limits
        cl = xyz_borders(self.tcp)
        x_min, x_max, y_min, y_max, z_min, z_max = cl
        coordinate_avg = average(cl)

        # Joint limits
        joint_limits = joint_borders(self.tcp)
        j1_min, j1_max, j2_min, j2_max, j3_min, j3_max, _, _, j5_min, j5_max = joint_limits[0:10]

        # Saturate joint limits
        j1_min = max(-90, j1_min)  # Left
        j1_max = min(+90, j1_max)  # Right
        j2_min = max(-90, j2_min)  # Back flat
        j2_max = min(+90, j2_max)  # Back arm stretched
        j5_min = max(0, j5_min)  # Front arm stretched
        j5_max = min(90, j5_max)  # Front arm 90°
        j3_min = 180 - j2_min - j5_max
        j3_max = 180 - j2_max - j5_max

        # Borders by J1 (independent of J2 and J3)
        j1_x_max = min([abs(i) for i in range(j1_min, j1_max)])  # Straight line
        j1_x_min = min([90 - abs(i) for i in range(j1_min, j1_max)])  # Perpendicular line
        j1_y_max = j1_x_min
        j1_y_min = j1_x_max

        # Borders by J2 and J3
        r_min, r_max = 10000, 0
        z_min, z_max = 10000, -10000

        j2_r_min, j3_r_min = None, None
        j2_r_max, j3_r_max = None, None
        j2_z_min, j3_z_min = None, None
        j2_z_max, j3_z_max = None, None

        for j2 in range(j2_min, j2_max, 1):
            for j3 in range(j3_min, j3_max, 1):
                r_val = radius(l2, l3, l4, j2, j3)
                z_val = z(l0, l1, l2, l3, l4, l5, j2, j3)

                if r_val < r_min:
                    r_min = r_val
                    j2_r_min, j3_r_min = j2, j3
                if r_val > r_max:
                    r_max = r_val
                    j2_r_max, j3_r_max = j2, j3
                if z_val < z_min:
                    z_min = z_val
                    j2_z_min, j3_z_min = j2, j3
                if z_val > z_max:
                    z_max = z_val
                    j2_z_max, j3_z_max = j2, j3

        # Borders by all joints
        x_min_j = x(r_min, j1_x_min)
        x_max_j = x(r_max, j1_x_max)
        y_min_j = y(r_min, j1_y_min)
        y_max_j = y(r_max, j1_y_max)

        # Define origin
        origin = Coordinate(coordinate_avg, self.axes)


def radius(l2, l3, l4, j2, j3):
    return abs(l2 * sin(j2) + (l3 + l4) * sin(j2 + j3))


def x(r, j1):
    return r * cos(j1)


def y(r, j1):
    return r * sin(j1)


def z(l0, l1, l2, l3, l4, l5, j2, j3):
    return (l0 + l1 - l5) + l2 * cos(j2) + (l3 + l4) * cos(j2 + j3)


def average(x):
    return [0.5 * x[2 * i] + 0.5 * x[2 * i + 1] for i in range(len(x) // 2)]
