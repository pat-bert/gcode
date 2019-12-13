from math import sin, cos, pi
from time import sleep
from typing import *
from typing import Union, AnyStr

from printing import MelfaCmd, ApplicationExceptions
from printing.Coordinate import Coordinate
from printing.GCmd import GCmd
from printing.GRedirect import RedirectionTargets
from printing.PrinterComponent import PrinterComponent
from printing.TcpClientR3 import TcpClientR3
from printing.circle_util import get_angle
from printing.refactor import cmp_response, xyz_borders, joint_borders, reset_speeds, get_ovrd_speed


class MelfaRobot(PrinterComponent):
    """
    Class representing the physical robots with its unique routines, properties and actions.
    """

    redirector = [RedirectionTargets.MOVER, RedirectionTargets.BROADCAST]
    axes = 'XYZABC'

    def __init__(self, tcp_client, speed_threshold=10, number_axes: int = 6):
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
        self.joints: Sized[AnyStr] = list(['J' + str(i) for i in range(1, number_axes + 1)])
        self.servo: bool = False
        self.com_ctrl: bool = False
        self.speed_threshold = speed_threshold

    # Administration functions
    def boot(self, safe_return: bool = False, *args, **kwargs) -> None:
        """
        Starts the robot and initialises it.
        :param safe_return: Flag, whether the robot should return to its safe position
        :return: None
        """
        # Communication & Control on
        self.change_communication_state(True)
        # Check speed first
        self.check_speed_threshold(self.speed_threshold)
        # Servos on
        self.change_servo_state(True)
        # Safe position
        if safe_return:
            self.go_safe_pos()

        """
        Variables go into this program
        """
        # Program init
        # self.tcp.send('LOAD=1')
        # self.tcp.receive()
        """
        Global variables declare
        """
        try:
            self.tcp.send('EXECDEF POS P1')
            self.tcp.receive()
        except ApplicationExceptions.MelfaBaseException:
            self.tcp.send(MelfaCmd.ALARM_RESET_CMD)
            self.tcp.receive()
        try:
            self.tcp.send('EXECDEF POS P2')
            self.tcp.receive()
        except ApplicationExceptions.MelfaBaseException:
            self.tcp.send(MelfaCmd.ALARM_RESET_CMD)
            self.tcp.receive()
        try:
            self.tcp.send('EXECDEF POS P3')
            self.tcp.receive()
        except ApplicationExceptions.MelfaBaseException:
            self.tcp.send(MelfaCmd.ALARM_RESET_CMD)
            self.tcp.receive()

    def shutdown(self, safe_return: bool = False, *args, **kwargs) -> None:
        """
        Safely shuts down the robot.
        :param safe_return: Flag, whether the robot should return to its safe position
        :return: None
        """
        # TODO Fix deadlock when operation control is lost
        # Finish robot communication
        print("Finishing control...")
        # Safe position
        if safe_return:
            # Error reset to ensure safe return
            sleep(2)
            self.tcp.send(MelfaCmd.ALARM_RESET_CMD)
            self.tcp.receive(silence_errors=True)
            sleep(1)
            self.go_safe_pos()
        # Reset speed
        reset_speeds(self.tcp)
        # Servos off
        self.change_servo_state(False)
        # Communication & Control off
        self.change_communication_state(False)
        # Shutdown TCP
        self.tcp.close()

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
    def set_speed(self, speed: float, mode: str):
        """
        Set the speed modification factors for joint and interpolation movement.
        :param speed: Speed (for linear interpolation in mm/s, for joint interpolation in %)
        :param mode
        :return:
        """
        if speed <= 0:
            raise ValueError("Speed needs to be larger than zero.")
        elif mode == 'joint' and speed > 100:
            raise ValueError("Speed needs to be smaller than 100%.")

        ovrd_speed_factor = get_ovrd_speed(self.tcp) / 100
        speed_val = speed / ovrd_speed_factor
        if mode == 'linear':
            self.tcp.send(MelfaCmd.MVS_SPEED + '{:.{d}f}'.format(speed_val, d=2))
        elif mode == 'joint':
            self.tcp.send(MelfaCmd.MOV_SPEED + '{:.{d}f}'.format(speed_val, d=2))
        else:
            raise ValueError("Unknown speed type <" + str(mode) + ">")
        self.tcp.receive()

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
        # Set speed
        self.set_speed(speed, 'linear')

        # Send move command
        self.tcp.send(MelfaCmd.LINEAR_INTRP + target_pos.to_melfa_point())
        self.tcp.receive()

        # Wait until position is reached
        cmp_response(MelfaCmd.CURRENT_XYZABC, target_pos.to_melfa_response(), self.tcp)

    def circular_move_poll(self, target_pos: Coordinate, center_pos: Coordinate, is_clockwise: bool,
                           speed: float, start_pos=None) -> None:
        """
        Moves the robot on a (counter-)clockwise arc around a center position to a target position.
        :param start_pos:
        :param target_pos: Coordinate for the target position.
        :param center_pos: Coordinate for the center of the arc.
        :param is_clockwise: Flag to indicate clockwise|counter-clockwise direction.
        :param speed: Movement speed for tool.
        :return:
        """
        # Determine start position
        if start_pos is None:
            # Current position
            self.tcp.send(MelfaCmd.CURRENT_XYZABC)
            response = self.tcp.receive()

            # Reconstruct coordinate
            start_pos = Coordinate.from_melfa_response(response, len(self.joints))

        # Set speed
        self.set_speed(speed, 'linear')

        # Determine the angle
        angle = get_angle(start_pos, target_pos, center_pos)

        # Adjust the angle according to the direction
        if is_clockwise:
            # Angle needs to be positive
            if angle < 0:
                angle += 2 * pi
        else:
            # Angle needs to be negative
            if angle > 0:
                angle -= 2 * pi

        # Intermediate points for angles >= 180°
        if abs(angle) >= pi:
            # TODO Calculate intermediate point to move in two arcs
            pass

        # Save positions to program 1
        start_str = start_pos.to_melfa_point()
        target_str = target_pos.to_melfa_point()
        centre_str = center_pos.to_melfa_point()

        """
        Global (?) variables
        """
        sleep(0.01)
        self.tcp.send(MelfaCmd.DIRECT_CMD + ' P1=' + start_str)
        self.tcp.receive()
        sleep(0.01)
        self.tcp.send(MelfaCmd.DIRECT_CMD + ' P2=' + target_str)
        self.tcp.receive()
        sleep(0.01)
        self.tcp.send(MelfaCmd.DIRECT_CMD + ' P3=' + centre_str)
        self.tcp.receive()
        sleep(0.01)
        """
        Write variables in program 1 (needs to be loaded and saved)
        """
        # self.tcp.send('HOT1;P1=' + start_str)
        # self.tcp.receive()
        # self.tcp.send('HOT1;P2=' + target_str)
        # self.tcp.receive()
        # self.tcp.send('HOT1;P3=' + centre_str)
        # self.tcp.receive()
        # self.tcp.send('SAVE')
        # self.tcp.receive()

        # TODO Send move command with coordinates and direct/indirect flag

        # TODO Check reason for error -> might need to write variables first
        positions = ','.join([start_str, target_str, centre_str])
        self.tcp.send(MelfaCmd.CIRCULAR_INTRP + 'P1,P2,P3')
        # self.tcp.send(MelfaCmd.CIRCULAR_INTRP + positions)
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

    def check_speed_threshold(self, speed_threshold):
        reset_speeds(self.tcp)
        # Check for low speed
        speed = get_ovrd_speed(self.tcp)
        if speed > speed_threshold:
            try:
                self.tcp.send(MelfaCmd.OVERWRITE_CMD + '=' + str(speed_threshold))
                self.tcp.receive()
                print("Reduced speed to threshold value: " + str(speed_threshold))
            except ApplicationExceptions.MelfaBaseException:
                raise ApplicationExceptions.MelfaBaseException(
                    "Please ensure a speed lower or equal 10% in interactive mode!")
        else:
            print("Speed of " + str(speed) + "%. Okay!")

    def handle_gcode(self, gcode: GCmd, interactive=True, gcode_prev: Union[GCmd, None] = None, *args, **kwargs) -> \
            Union[AnyStr, None]:
        """
        Translates a G-Code to a Mitsubishi Melfa R3 command.
        :param gcode: G-Code object
        :param interactive: Flag indicating whether the command should be executed or stored
        :param gcode_prev: Optional object for previous G-Code to be considered for speed setting
        :return:
        """
        if interactive:
            # G-Code is executed directly
            if gcode.id in ['G00', 'G0']:
                self.linear_move_poll(gcode.cartesian_abs, gcode.speed)
            elif gcode.id in ['G01', 'G1']:
                self.linear_move_poll(gcode.cartesian_abs, gcode.speed)
            elif gcode.id in ['G02', 'G2']:
                self.circular_move_poll(gcode.cartesian_abs, gcode.cartesian_abs + gcode.cartesian_rel, True,
                                        gcode.speed)
            elif gcode.id in ['G03', 'G3']:
                self.circular_move_poll(gcode.cartesian_abs, gcode.cartesian_abs + gcode.cartesian_rel, False,
                                        gcode.speed)
            else:
                raise NotImplementedError
        else:
            # Melfa code is saved for later usage
            if gcode.id in ['G00', 'G0', 'G01', 'G1']:
                return MelfaCmd.LINEAR_INTRP + gcode.cartesian_abs.to_melfa_point()
            elif gcode.id in ['G02', 'G2']:
                raise NotImplementedError
            elif gcode.id in ['G03', 'G3']:
                raise NotImplementedError
            else:
                raise NotImplementedError


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
