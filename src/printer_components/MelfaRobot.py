from math import pi
from time import sleep
from typing import AnyStr, Union, List, Optional

import numpy as np

import src.protocols.R3Protocol as R3Protocol_Cmd
from src import ApplicationExceptions
from src.ApplicationExceptions import MelfaBaseException, ApiException
from src.Coordinate import Coordinate
from src.GRedirect import RedirectionTargets
from src.MelfaCoordinateService import MelfaCoordinateService, Plane
from src.circle_util import get_angle, get_intermediate_point
from src.clients.TcpClientR3 import TcpClientR3
from src.gcode.GCmd import GCmd
from src.printer_components.PrinterComponent import PrinterComponent
from src.protocols.R3Protocol import R3Protocol
from src.refactor import cmp_response


class IllegalAxesCount(ApiException):
    pass


class SpeedBelowMinimum(ApiException):
    pass


class MelfaRobot(PrinterComponent):
    """
    Class representing the physical robots with its unique routines, properties and actions.
    """

    redirector = [RedirectionTargets.MOVER, RedirectionTargets.BROADCAST]
    AXES = "XYZABC"
    INCH_IN_MM = 25.4

    def __init__(
            self, io_client, speed_threshold=10, number_axes: int = 6, safe_return=False
    ):
        """
        Initialises the robot.
        :param io_client: Communication object for TCP/IP-protocol
        :param number_axes: Number of robot AXES, declared by 'J[n]', n>=1
        :param safe_return:
        """
        if not hasattr(io_client, "send") or not hasattr(io_client, "receive"):
            raise TypeError("Client does not implement required methods.")
        if number_axes <= 0:
            raise IllegalAxesCount

        self.client: TcpClientR3 = io_client
        self.work_coordinate_offset = "(-500,0,-250,0,0,0)"
        self.joints = ["J{}".format(i) for i in range(1, number_axes + 1)]
        self.speed_threshold = speed_threshold
        self.protocol = R3Protocol(io_client, MelfaCoordinateService(), self.joints)

        # Operation Flags
        self.safe_return = safe_return
        self.servo: bool = False
        self.com_ctrl: bool = False
        self.work_coordinate_active = False

        # G-Code Flags
        self.inch_active = False
        self.absolute_coordinates = True
        self.active_plane = Plane.XY
        self.zero = Coordinate([0, 0, 0, None, None, None], "XYZABC")

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

        # Variables allocated
        self._prepare_circle()

        # Safe position
        if self.safe_return:
            self.go_safe_pos()

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
                self.protocol.reset_alarm()
                sleep(1)
                self.go_safe_pos()
                sleep(1)

            # Reset all speed factors
            self.protocol.reset_all_speeds()
        finally:
            # Servos off
            self._change_servo_state(False)
            # Communication & Control off
            self._change_communication_state(False)
            # Shutdown TCP in ANY CASE
            self.client.close()

    def activate_work_coordinate(self, active: bool) -> None:
        """
        Toggle the work coordinate system on/off.
        :param active: New state
        :return: None
        """
        if active:
            # Set coordinate system
            self.protocol.set_work_coordinate(self.work_coordinate_offset)
        else:
            # Reset coordinate system
            self.protocol.reset_base_coordinate_system()

        self.work_coordinate_active = active

    def handle_gcode(self, gcode: GCmd, gcode_prev: Union[GCmd, None] = None, *args, **kwargs) -> None:
        """
        Translates a G-Code to a Mitsubishi Melfa R3 command.
        :param gcode: G-Code object
        :param gcode_prev: Optional object for previous G-Code to be considered for speed setting
        :return:
        """
        # G-Code is executed directly
        current_pos = self.protocol.get_current_xyzabc()
        current_pos = current_pos.reduce_to_axes("XYZ")

        # Inch conversion
        if self.inch_active:
            self.adjust_units(gcode)

        # Speed conversion mm/min to mm/s
        if gcode.speed is not None:
            gcode.speed /= 60

        # Movement G-code
        if gcode.id in ["G00", "G0", "G01", "G1"]:
            if not self.absolute_coordinates:
                self.linear_move_poll(
                    gcode.cartesian_abs + current_pos,
                    gcode.speed,
                    current_pos=current_pos,
                )
            else:
                self.linear_move_poll(
                    gcode.cartesian_abs, gcode.speed, current_pos=current_pos
                )
        elif gcode.id in ["G02", "G2"]:
            if not self.absolute_coordinates:
                self.circular_move_poll(
                    gcode.cartesian_abs + current_pos,
                    current_pos + gcode.cartesian_rel,
                    True,
                    gcode.speed,
                    start_pos=current_pos,
                )
            else:
                self.circular_move_poll(
                    gcode.cartesian_abs,
                    current_pos + gcode.cartesian_rel,
                    True,
                    gcode.speed,
                    start_pos=current_pos,
                )
        elif gcode.id in ["G03", "G3"]:
            if not self.absolute_coordinates:
                self.circular_move_poll(
                    gcode.cartesian_abs + current_pos,
                    current_pos + gcode.cartesian_rel,
                    False,
                    gcode.speed,
                )
            else:
                self.circular_move_poll(
                    gcode.cartesian_abs,
                    current_pos + gcode.cartesian_rel,
                    False,
                    gcode.speed,
                )
        elif gcode.id in ["G04", "G4"]:
            self.wait(gcode.time_ms)

        elif gcode.id in ["G04", "G4"]:
            # Adjust the offsets for the current tool
            # TODO self.protocol.set_current_tool_data(gcode.cartesian_abs) (requires Hardware)
            pass

        # Plane selection
        elif gcode.id == "G17":
            self.active_plane = Plane.XY
        elif gcode.id == "G18":
            self.active_plane = Plane.XZ
        elif gcode.id == "G19":
            self.active_plane = Plane.YZ

        # Units
        elif gcode.id == "G20":
            self.inch_active = True
        elif gcode.id == "G21":
            self.inch_active = False

        # Homing
        elif gcode.id == "G28":
            self.go_home(option=gcode.home_opt)

        # Absolute/Relative mode
        elif gcode.id == "G90":
            self.absolute_coordinates = True
        elif gcode.id == "G91":
            self.absolute_coordinates = False

        # Tools
        elif gcode.id.startswith('T'):
            # Tool commands start with T followed by the tool number.
            # G-Code starts counting at zero, Mitsubishi starts at one
            self.protocol.set_current_tool(int(gcode.id[1:]) + 1)

        elif gcode.id == 'G200':
            self.move_joint(gcode.joints)

        # Unsupported G-code
        else:
            raise NotImplementedError("Unsupported G-code: '{}'".format(str(gcode)))

    def adjust_units(self, gcode: GCmd):
        if (
                gcode.cartesian_abs is not None
                and len(gcode.cartesian_abs.coordinate) > 0
        ):
            gcode.cartesian_abs *= self.INCH_IN_MM
        if (
                gcode.cartesian_rel is not None
                and len(gcode.cartesian_rel.coordinate) > 0
        ):
            gcode.cartesian_rel *= self.INCH_IN_MM
        if gcode.speed is not None:
            gcode.speed *= self.INCH_IN_MM
        if gcode.extrude_len is not None:
            gcode.extrude_len *= self.INCH_IN_MM

    def _prepare_circle(self) -> None:
        for i in range(1, 4):
            self.declare_position("P{}".format(i))

    def declare_position(self, var_name: str) -> None:
        try:
            self.protocol.define_variable(var_name, var_type="position")
        except ApplicationExceptions.MelfaBaseException as e:
            if str(e.status).startswith(
                    ApplicationExceptions.DuplicateVariableDeclaration
            ):
                self.protocol.reset_alarm()
            else:
                raise

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
            self.protocol.open_communication()
            self.protocol.obtain_control()
        else:
            # Open communication and obtain control
            self.protocol.release_control()
            self.protocol.close_communication()

        self.com_ctrl = activate

    def _change_servo_state(self, activate: bool) -> None:
        """
        Switch the servos on/off.
        :param activate: Boolean
        :return: None
        """
        if activate:
            self.protocol.activate_servo()

            # Poll for active state
            cmp_response(
                R3Protocol_Cmd.VAR_READ + R3Protocol_Cmd.SRV_STATE_VAR,
                R3Protocol_Cmd.SRV_STATE_VAR + "=+1",
                self.protocol.reader,
                timeout_s=R3Protocol_Cmd.SERVO_INIT_SEC,
            )
            sleep(1)
        else:
            self.protocol.deactivate_servo()

            # Poll for inactive state
            cmp_response(
                R3Protocol_Cmd.VAR_READ + R3Protocol_Cmd.SRV_STATE_VAR,
                R3Protocol_Cmd.SRV_STATE_VAR + "=+0",
                self.protocol.reader,
                timeout_s=R3Protocol_Cmd.SERVO_INIT_SEC,
            )

        self.servo = activate

    # Speed functions

    def set_speed(self, speed: float, mode: str) -> None:
        """
        Set the speed modification factors for joint and interpolation movement.
        :param speed: Speed (for linear interpolation in mm/s, for joint interpolation in %)
        :param mode: Type of speed setting to be changed (linear, joint).
        :return: None
        :raises: ValueError if mode is unknown.
        """
        ovrd_speed_factor = self.protocol.get_override()
        speed_val = 100 * speed / ovrd_speed_factor

        try:
            if mode == "linear":
                self.protocol.set_linear_speed(speed_val)
            elif mode == "joint":
                self.protocol.set_joint_speed(speed_val)
            else:
                raise ValueError("Unknown speed type <{}>".format(mode))
        except ValueError:
            # Indicate spare speed reserve
            if ovrd_speed_factor < 100 and speed >= 1.0:
                raise ValueError(
                    "Could not increase the speed setting above the maximum. Please increase override."
                )
            raise

    # Movement functions

    def go_home(self, option="") -> None:
        """
        Moves the robot to its current home point (current work coordinate origin or global safe position respectively)
        :return:
        """
        if self.work_coordinate_active:
            # Acquire new zero coordinate
            zero = self._zero()

            if option != "":
                zero = zero.reduce_to_axes(option, make_none=True)

            # Acquire current position to determine robot orientation
            current_position = self.protocol.reader.get_current_xyzabc()
            zero.update_empty(current_position)

            # Move to zero
            self.linear_move_poll(self.zero)
        else:
            self.go_safe_pos()

    def go_safe_pos(self) -> None:
        """
        Moves the robot to its safe position.
        :return:
        """
        # Read safe position
        safe_pos = self.protocol.get_safe_pos()

        # Go to safe position
        self.protocol.go_safe_pos()

        # Wait until position is reached
        cmp_response(R3Protocol_Cmd.CURRENT_JOINT, safe_pos.to_melfa_response(), self.protocol.reader)

    def linear_move_poll(self, target_pos: Coordinate, speed: float = None, track_speed=False, current_pos=None):
        """
        Moves the robot linearly to a coordinate.
        :param target_pos: Coordinate for the target position.
        :param speed: Movement speed for tool.
        :param track_speed:
        :param current_pos:
        :return:
        """
        if speed is not None:
            self.set_speed(speed, "linear")

        # Only send command if any coordinates are passed, otherwise just set the speed
        if len(target_pos.values) > 0 and any(a is not None for a in target_pos.values):
            # Fill None values with current position to predict correct response
            if current_pos is None:
                # No need to do this twice
                current_pos = self.protocol.get_current_xyzabc()
            target_pos.update_empty(current_pos)

            # Send move command
            self.protocol.linear_move(target_pos)

            # Wait until position is reached
            t, v = cmp_response(
                R3Protocol_Cmd.CURRENT_XYZABC,
                target_pos.to_melfa_response(),
                self.protocol.reader,
                track_speed=track_speed,
            )
            return t, v
        return None, None

    def circular_move_poll(self, target_pos: Coordinate, center_pos: Coordinate, is_clockwise: bool,
                           speed: Optional[float] = None, start_pos: Optional[Coordinate] = None) -> None:
        """
        Moves the robot on a (counter-)clockwise arc around a center position to a target position.
        :param start_pos: Coordinate for the start position, defaults to current position if None.
        The robot first performs a linear movement to the start position if it is not equal to the current position.
        :param target_pos: Coordinate for the target position.
        :param center_pos: Coordinate for the center of the arc.
        :param is_clockwise: Flag to indicate clockwise|counter-clockwise direction.
        :param speed: Movement speed for tool.
        """
        # Determine start position
        if start_pos is None:
            start_pos = self.protocol.get_current_xyzabc()

        # Set speed
        if speed is not None:
            self.set_speed(speed, "linear")

        # Only send command if any coordinates are passed, otherwise just set the speed
        if len(target_pos.values) > 0 and any(a is not None for a in target_pos.values):
            # Update positions to be complete
            target_pos.update_empty(start_pos)
            center_pos.update_empty(start_pos)

            angle = self.get_directed_angle(start_pos, target_pos, center_pos, is_clockwise)

            if abs(angle) >= pi:
                # Intermediate points for angles >= 180°
                start_pos_np = np.array(start_pos.values)
                target_pos_np = np.array(target_pos.values)
                center_pos_np = np.array(center_pos.values)

                im_pos_np = get_intermediate_point(angle, start_pos_np, target_pos_np, center_pos_np, self.active_plane)

                im_pos = Coordinate(list(im_pos_np), 'XYZ')
                im_pos.update_empty(start_pos)

                # Position assignments
                if abs(angle) == 2 * pi:
                    # Calculate additional intermediate point
                    angle = self.get_directed_angle(start_pos, im_pos, center_pos, is_clockwise)
                    im_pos2_np = get_intermediate_point(angle, start_pos_np, im_pos_np, center_pos_np,
                                                        self.active_plane)

                    im_pos2 = Coordinate(list(im_pos2_np), 'XYZ')
                    im_pos2.update_empty(start_pos)

                    # Global variables
                    self.set_global_positions(["P1", "P2", "P3"], [start_pos, im_pos2, im_pos])

                    # Send move command
                    self.protocol.circular_move_full("P1", "P2", "P3")
                else:
                    # Global variables
                    self.set_global_positions(["P1", "P2", "P3"], [start_pos, im_pos, target_pos])

                    # Send move command
                    self.protocol.circular_move_intermediate("P1", "P2", "P3")
            else:
                # Global variables
                self.set_global_positions(["P1", "P2", "P3"], [start_pos, target_pos, center_pos])

                # Send move command
                self.protocol.circular_move_centre("P1", "P2", "P3")

            # Wait until position is reached
            cmp_response(R3Protocol_Cmd.CURRENT_XYZABC, target_pos.to_melfa_response(), self.protocol.reader)

    def get_directed_angle(self, start_pos: Coordinate, target_pos: Coordinate, center_pos: Coordinate,
                           is_clockwise: bool):
        # Determine the angle
        start_pos = np.array(start_pos.values)
        target_pos = np.array(target_pos.values)
        center_pos = np.array(center_pos.values)

        angle = get_angle(start_pos, target_pos, center_pos, self.active_plane)
        # Adjust the angle according to the direction
        if not is_clockwise:
            # Angle needs to be positive
            if angle <= 0:
                angle += 2 * pi
        else:
            # Angle needs to be negative
            if angle >= 0:
                angle -= 2 * pi
        return angle

    def set_global_positions(self, var_names: List[AnyStr], coordinates: List[Coordinate]) -> None:
        """
        Write coordinates to a global variable name in the robot memory.
        :param var_names: List of the variable names
        :param coordinates:
        """
        if len(var_names) == len(coordinates):
            for var_name, coordinate in zip(var_names, coordinates):
                self.protocol.set_position(var_name, coordinate)
        else:
            raise MelfaBaseException(
                "Variable names and coordinates must be of same length."
            )

    def _check_speed_threshold(self, speed_threshold: float) -> None:
        """
        Verify that the speed setting meets the current threshold
        :param speed_threshold:
        """
        # Reset all speed factors for clean initial state
        self.protocol.reset_all_speeds()
        # Check the override
        speed = self.protocol.get_override()
        # Threshold violation
        if speed > speed_threshold:
            try:
                self.protocol.set_override(speed_threshold)
                print("Reduced speed to threshold value: {}".format(speed_threshold))
            except ApplicationExceptions.MelfaBaseException:
                raise ApplicationExceptions.MelfaBaseException(
                    "Please ensure a speed lower or equal 10% in interactive mode!"
                )
        else:
            print("Speed of {}%. Okay!".format(speed))

    @staticmethod
    def wait(time_ms) -> None:
        """
        Waits for a specified time.
        :param time_ms: Time to wait in ms.
        """
        sleep(1000 * time_ms)

    def _zero(self) -> Coordinate:
        return Coordinate(self.zero.values, self.zero.axes)

    def move_joint(self, joints: Coordinate) -> None:
        """
        Move to a position in joint coordinates.
        :param joints: Coordinate
        """
        if len(joints.values) != len(self.joints):
            raise ValueError('Joint movements need to specify all axes.')
        self.protocol.joint_move(joints)
