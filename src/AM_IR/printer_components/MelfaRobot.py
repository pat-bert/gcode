from math import pi
from time import sleep
from typing import *

from AM_IR import ApplicationExceptions
from AM_IR.ApplicationExceptions import MelfaBaseException
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
        self.speed_threshold = speed_threshold

        # Operation Flags
        self.safe_return = safe_return
        self.servo: bool = False
        self.com_ctrl: bool = False
        self.work_coordinate_active = False

        # G-Code Flags
        self.inch_active = False
        self.absolute_coordinates = True
        self.active_plane = Plane.XY
        self.zero = Coordinate([0, 0, 0, None, None, None], 'XYZABC')

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

        # Deactivate work coordinates
        self.activate_work_coordinate(False)

        # Safe position
        if self.safe_return:
            self.go_safe_pos()

        # Variables allocated
        self._prepare_circle()

        # Activate work coordinates
        self.activate_work_coordinate(True)

    def shutdown(self, safe_return: bool = False, *args, **kwargs) -> None:
        """
        Safely shuts down the robot.
        :param safe_return: Flag, whether the robot should return to its safe position
        :return: None
        """
        # Finish robot communication
        print("Finishing control...")
        try:
            # Deactivate work coordinates
            self.activate_work_coordinate(False)

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
            self.reset_linear_speed_factor()
        finally:
            # Servos off
            self._change_servo_state(False)
            # Communication & Control off
            self._change_communication_state(False)
            # Shutdown TCP in ANY CASE
            self.tcp.close()

    def activate_work_coordinate(self, active: bool) -> None:
        # TODO Clean this up (is not considered in demo mode yet), implement homing
        if active:
            # Activate coordinate system
            self.tcp.send(MelfaCmd.DIRECT_CMD + 'BASE (-500,0,-200,0,0,0)')
            self.tcp.receive()
        else:
            self.tcp.send(MelfaCmd.DIRECT_CMD + 'BASE P_NBASE')
            self.tcp.receive()

        self.work_coordinate_active = active

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
            current_pos = self.get_pos()
            current_pos.reduce_to_axes('XYZ')

            # Inch conversion
            if self.inch_active:
                if gcode.cartesian_abs is not None and len(gcode.cartesian_abs.coordinate) > 0:
                    gcode.cartesian_abs *= self.INCH_IN_MM
                if gcode.cartesian_rel is not None and len(gcode.cartesian_rel.coordinate) > 0:
                    gcode.cartesian_rel *= self.INCH_IN_MM
                if gcode.speed is not None:
                    gcode.speed *= self.INCH_IN_MM

            # Speed conversion mm/min to mm/s
            if gcode.speed is not None:
                gcode.speed /= 60

            # Movement G-code
            if gcode.id in ['G00', 'G0', 'G01', 'G1']:
                if not self.absolute_coordinates:
                    self.linear_move_poll(gcode.cartesian_abs + current_pos, gcode.speed)
                else:
                    self.linear_move_poll(gcode.cartesian_abs, gcode.speed)
            elif gcode.id in ['G02', 'G2']:
                if not self.absolute_coordinates:
                    self.circular_move_poll(gcode.cartesian_abs + current_pos, current_pos + gcode.cartesian_rel, True,
                                            gcode.speed)
                else:
                    self.circular_move_poll(gcode.cartesian_abs, current_pos + gcode.cartesian_rel, True, gcode.speed)
            elif gcode.id in ['G03', 'G3']:
                if not self.absolute_coordinates:
                    self.circular_move_poll(gcode.cartesian_abs + current_pos, current_pos + gcode.cartesian_rel, False,
                                            gcode.speed)
                else:
                    self.circular_move_poll(gcode.cartesian_abs, current_pos + gcode.cartesian_rel, False, gcode.speed)
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
        """
        Attempts to read a given parameter from the robot.
        :param parameter:
        :return:
        """
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

    def reset_linear_speed_factor(self):
        """
        Reset the speed modification factors to maximum speed.
        :return:
        """
        self.tcp.send(MelfaCmd.MVS_SPEED + MelfaCmd.MVS_MAX_SPEED)
        self.tcp.receive()

    # Movement functions

    def go_home(self) -> None:
        """
        Moves the robot to its current home point (current work coordinate origin or global safe position respectively)
        :return:
        """
        if self.work_coordinate_active:
            current_position = self.get_pos()
            self.zero.update_empty(current_position)
            self.linear_move_poll(self.zero)
        else:
            self.go_safe_pos()

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
            self.tcp.send(MelfaCmd.LINEAR_INTRP + MelfaCoordinateService.to_melfa_point(target_pos, self.active_plane))
            self.tcp.receive()

            # Wait until position is reached
            t, v = cmp_response(MelfaCmd.CURRENT_XYZABC, target_pos.to_melfa_response(), self.tcp,
                                track_speed=track_speed)
            return t, v

    def circular_move_poll(self, target_pos: Coordinate, center_pos: Coordinate, is_clockwise: bool,
                           speed: float = None, start_pos=None) -> None:
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

        if speed is not None:
            # Set speed
            self.set_speed(speed, 'linear')

        # Only send command if any coordinates are passed, otherwise just set the speed
        if len(target_pos.coordinate.values()) > 0 and any(a is not None for a in target_pos.coordinate.values()):
            # Update positions to be complete
            target_pos.update_empty(start_pos)
            center_pos.update_empty(start_pos)

            # Determine the angle
            angle = get_angle(start_pos, target_pos, center_pos, self.active_plane)

            # Adjust the angle according to the direction
            if not is_clockwise:
                # Angle needs to be positive
                if angle < 0:
                    angle += 2 * pi
            else:
                # Angle needs to be negative
                if angle > 0:
                    angle -= 2 * pi

            # Intermediate points for angles >= 180°
            if abs(angle) >= pi:
                im_pos = get_intermediate_points(angle, start_pos, target_pos, center_pos, self.active_plane)

                # Global variables
                self.set_global_positions(['P1', 'P2', 'P3'], [start_pos, im_pos, target_pos])

                # Send move command
                self.tcp.send(MelfaCmd.CIRCULAR_INTERPOLATION_IM + 'P1,P2,P3')
                self.tcp.receive()
            else:
                # Global variables
                self.set_global_positions(['P1', 'P2', 'P3'], [start_pos, target_pos, center_pos])

                # Send move command
                self.tcp.send(MelfaCmd.CIRCULAR_INTERPOLATION_CENTRE + 'P1,P2,P3')
                self.tcp.receive()

            # Wait until position is reached
            cmp_response(MelfaCmd.CURRENT_XYZABC, target_pos.to_melfa_response(), self.tcp)

    def set_global_positions(self, var_names: List[AnyStr], coordinates: List[Coordinate]) -> None:
        """
        Write coordinates to a global variable name in the robot memory.
        :param var_names: List of the variable names
        :param coordinates:
        :return:
        """
        if len(var_names) == len(coordinates):
            for var, coordinate in zip(set(var_names), coordinates):
                self.tcp.send(MelfaCmd.DIRECT_CMD + str(var) + '=' + coordinate.to_melfa_point())
                self.tcp.receive()
                sleep(0.01)
        else:
            raise MelfaBaseException("Variable names and coordinates must be of same length.")

    def get_pos(self) -> Coordinate:
        """
        Get the current position.
        :return: Coordinate object containing the robot coordinates.
        """
        # Current position
        self.tcp.send(MelfaCmd.CURRENT_XYZABC)
        response = self.tcp.receive()

        # Convert response using coordinate factory
        pos = MelfaCoordinateService.from_melfa_response(response, len(self.joints))
        return pos

    def _check_speed_threshold(self, speed_threshold: float):
        """
        Verify that the speed setting meets the current threshold
        :param speed_threshold:
        :return:
        """
        # Reset the linear factor
        self.reset_linear_speed_factor()
        # Check for low speed
        speed = self._get_ovrd_speed()
        # Threshold violation
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

    def _set_ovrd(self, factor: float):
        """
        Sets the current override speed value.
        :param factor:
        :return:
        """
        if 1 <= float(factor) <= 100:
            self.tcp.send(MelfaCmd.OVERRIDE_CMD + '=' + str(factor))
            self.tcp.receive()
        else:
            raise ApplicationExceptions.MelfaBaseException("Override factor must be [1,100].")

    def _get_ovrd_speed(self) -> float:
        """
        Reads the current override speed value.
        :return:
        """
        self.tcp.wait_send(MelfaCmd.OVERRIDE_CMD)
        speed = self.tcp.receive()
        return float(speed)

    def wait(self, time_ms):
        """
        Waits for a specified time.
        :param time_ms:
        :return:
        """
        # TODO Implement waiting (G04)
        # self.tcp.wait_send('DLY')
        raise NotImplementedError
