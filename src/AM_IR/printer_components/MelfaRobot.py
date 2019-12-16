from math import pi
from time import sleep
from typing import *

from AM_IR import ApplicationExceptions
from AM_IR.Coordinate import Coordinate
from AM_IR.GRedirect import RedirectionTargets
from AM_IR.MelfaCoordinateService import MelfaCoordinateService, Plane
from AM_IR.circle_util import get_angle, get_intermediate_points
from AM_IR.gcode.GCmd import GCmd
from AM_IR.melfa import MelfaCmd
from AM_IR.melfa.TcpClientR3 import TcpClientR3
from AM_IR.printer_components.PrinterComponent import PrinterComponent
from AM_IR.refactor import cmp_response


class MelfaRobot(PrinterComponent):
    """
    Class representing the physical robots with its unique routines, properties and actions.
    """

    redirector = [RedirectionTargets.MOVER, RedirectionTargets.BROADCAST]
    AXES = 'XYZABC'
    INCH_IN_MM = 25.4

    def __init__(self, tcp_client, speed_threshold=10, number_axes: int = 6, safe_return=False):
        """
        Initialises the robot.
        :param tcp_client: Communication object for TCP/IP-protocol
        :param number_axes: Number of robot AXES, declared by 'J[n]', n>=1
        :param safe_return:
        """
        if not hasattr(tcp_client, 'send') or not hasattr(tcp_client, 'receive'):
            raise TypeError('TCP-client does not implement required methods.')
        if not number_axes > 0:
            raise TypeError('Illegal number of AXES.')

        self.tcp: TcpClientR3 = tcp_client
        self.joints: Sized[AnyStr] = list(['J' + str(i) for i in range(1, number_axes + 1)])
        self.servo: bool = False
        self.com_ctrl: bool = False
        self.speed_threshold = speed_threshold
        self.safe_return = safe_return
        self.active_plane = Plane.XY
        self.inch_active = False
        self.absolute_coordinates = True

    # Administration functions
    def boot(self, *args, **kwargs) -> None:
        """
        Starts the robot and initialises it.
        :return: None
        """
        # Communication & Control on
        self._change_communication_state(True)
        # Check speed first
        self._check_speed_threshold(self.speed_threshold)
        # Servos on
        self._change_servo_state(True)
        # Safe position
        if self.safe_return:
            self.go_safe_pos()
        # Variables allocated
        self._prepare_circle()

    def shutdown(self, safe_return: bool = False, *args, **kwargs) -> None:
        """
        Safely shuts down the robot.
        :param safe_return: Flag, whether the robot should return to its safe position
        :return: None
        """
        # Finish robot communication
        print("Finishing control...")
        try:
            # Safe position
            if self.safe_return:
                # Error reset to ensure safe return
                sleep(2)
                self.tcp.send(MelfaCmd.ALARM_RESET_CMD)
                self.tcp.receive(silence_errors=True)
                sleep(1)
                self.go_safe_pos()
                sleep(1)
            # Reset speed
            self.reset_speed_factors()
        finally:
            # Servos off
            self._change_servo_state(False)
            # Communication & Control off
            self._change_communication_state(False)
            # Shutdown TCP in ANY CASE
            self.tcp.close()

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

            # Movement G-code
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
            elif gcode.id in ['G04', 'G4']:
                self.wait(gcode.time_ms)
            # Plane selection
            elif gcode.id == 'G17':
                self.active_plane = Plane.XY
            elif gcode.id == 'G18':
                self.active_plane = Plane.XZ
            elif gcode.id == 'G19':
                self.active_plane = Plane.YZ

            # Units
            elif gcode.id == 'G20':
                self.inch_active = True
                raise NotImplementedError("Units are not yet fully supported.")
            elif gcode.id == 'G21':
                self.inch_active = False

            # Homing
            elif gcode.id == 'G28':
                raise NotImplementedError("Homing is not supported yet.")

            # Absolute/Relative mode
            elif gcode.id == 'G90':
                self.absolute_coordinates = True
            elif gcode.id == 'G91':
                self.absolute_coordinates = False
                raise NotImplementedError("Relative coordinates not implemented yet.")

            # Unsupported G-code
            else:
                raise NotImplementedError("Unsupported G-code: '{}'".format(str(gcode)))
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

    def _prepare_circle(self) -> None:
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

    def maintenance(self) -> None:
        # Communication & Control on
        self._change_communication_state(True)

    # Utility functions

    def _change_communication_state(self, activate: bool) -> None:
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

    def _change_servo_state(self, activate: bool) -> None:
        """
        Switch the servos on/off.
        :param activate: Boolean
        :return: None
        """
        if activate:
            self.tcp.send(MelfaCmd.SRV_ON)
            self.tcp.receive()
            cmp_response(MelfaCmd.VAR_READ + MelfaCmd.SRV_STATE_VAR, MelfaCmd.SRV_STATE_VAR + '=+1', self.tcp,
                         timeout_s=MelfaCmd.SERVO_INIT_SEC)
            sleep(1)
        else:
            self.tcp.send(MelfaCmd.SRV_OFF)
            self.tcp.receive()
            cmp_response(MelfaCmd.VAR_READ + MelfaCmd.SRV_STATE_VAR, MelfaCmd.SRV_STATE_VAR + '=+0', self.tcp,
                         timeout_s=MelfaCmd.SERVO_INIT_SEC)

        self.servo = activate

    def read_parameter(self, parameter: AnyStr) -> str:
        self.tcp.send(MelfaCmd.PARAMETER_READ + str(parameter))
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

        ovrd_speed_factor = self._get_ovrd_speed() / 100
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
        self.tcp.send(MelfaCmd.MVS_SPEED + MelfaCmd.MVS_MAX_SPEED)
        self.tcp.receive()
        # TODO Reset MOV Speed
        # tcp_client.send(MelfaCmd.MOV_SPEED + MelfaCmd.MOV_MAX_SPEED)
        # tcp_client.receive()

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

    def linear_move_poll(self, target_pos: Coordinate, speed: float = None, track_speed=False):
        """
        Moves the robot linearly to a coordinate.
        :param target_pos: Coordinate for the target position.
        :param speed: Movement speed for tool.
        :param track_speed:
        :return:
        """
        if speed is not None:
            # Set speed
            self.set_speed(speed, 'linear')

        # Only send command if any coordinates are passed, otherwise just set the speed
        if len(target_pos.coordinate.values()) > 0 and any(a is not None for a in target_pos.coordinate.values()):
            # Fill None values with current position to predict correct response
            current_pos = self.get_pos()
            target_pos.update_empty(current_pos)

            # Send move command
            self.tcp.send(MelfaCmd.LINEAR_INTRP + target_pos.to_melfa_point())
            self.tcp.receive()

            # Wait until position is reached
            t, v = cmp_response(MelfaCmd.CURRENT_XYZABC, target_pos.to_melfa_response(), self.tcp,
                                track_speed=track_speed)
            return t, v

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
            start_pos = self.get_pos()

        # Set speed
        self.set_speed(speed, 'linear')

        # Determine the angle
        angle = get_angle(start_pos, target_pos, center_pos, self.active_plane)

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
            intermediate_point = get_intermediate_points(angle, start_pos, target_pos, center_pos, self.active_plane)
            raise NotImplementedError("Angles >= 180 degrees are not yet supported.")

        # Global variables
        sleep(0.01)
        self.tcp.send(MelfaCmd.DIRECT_CMD + ' P1=' + start_pos.to_melfa_point())
        self.tcp.receive()
        sleep(0.01)
        self.tcp.send(MelfaCmd.DIRECT_CMD + ' P2=' + target_pos.to_melfa_point())
        self.tcp.receive()
        sleep(0.01)
        self.tcp.send(MelfaCmd.DIRECT_CMD + ' P3=' + center_pos.to_melfa_point())
        self.tcp.receive()
        sleep(0.01)

        # Wait until position is reached
        self.tcp.send(MelfaCmd.CIRCULAR_INTRP + 'P1,P2,P3')
        self.tcp.receive()
        cmp_response(MelfaCmd.CURRENT_XYZABC, target_pos.to_melfa_response(), self.tcp)

    def get_pos(self):
        # Current position
        self.tcp.send(MelfaCmd.CURRENT_XYZABC)
        response = self.tcp.receive()
        # Reconstruct coordinate
        pos = MelfaCoordinateService.from_melfa_response(response, len(self.joints))
        return pos

    def _check_speed_threshold(self, speed_threshold):
        self.reset_speed_factors()
        # Check for low speed
        speed = self._get_ovrd_speed()
        if speed > speed_threshold:
            try:
                self._set_ovrd(speed_threshold)
                print("Reduced speed to threshold value: " + str(speed_threshold))
            except ApplicationExceptions.MelfaBaseException:
                raise ApplicationExceptions.MelfaBaseException(
                    "Please ensure a speed lower or equal 10% in interactive mode!")
        else:
            print("Speed of " + str(speed) + "%. Okay!")

    # OVRD functions

    def _set_ovrd(self, factor):
        self.tcp.send(MelfaCmd.OVERWRITE_CMD + '=' + str(factor))
        self.tcp.receive()

    def _get_ovrd_speed(self):
        self.tcp.wait_send(MelfaCmd.OVERWRITE_CMD)
        speed = self.tcp.receive()
        return float(speed)

    def wait(self, time_ms):
        # self.tcp.wait_send('DLY')
        raise NotImplementedError
