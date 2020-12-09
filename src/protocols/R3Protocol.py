from time import sleep
from typing import Tuple, Optional, Callable, List

from src.Coordinate import Coordinate
from src.MelfaCoordinateService import MelfaCoordinateService
from src.clients.IClient import IClient, ClientError

# General commands
DELIMITER = ";"
DIRECT_CMD = "EXEC"
VAR_READ = "VAL"
SRV_ON = "SRVON"
SRV_OFF = "SRVOFF"
SRV_STATE_VAR = "M_SVO"
SERVO_INIT_SEC = 5

# Speed commands
OVERRIDE_CMD = "OVRD"
JOINT_SPEED = "JOVRD"
MOV_SPEED = DIRECT_CMD + "JOVRD "
LINEAR_SPEED = "SPD"
MVS_SPEED = DIRECT_CMD + "SPD "

# Coordinate commands
BASE_COORDINATE_CMD = "BASE"
CURRENT_XYZABC = "PPOSF"
CURRENT_JOINT = "JPOSF"

# Move commands
LINEAR_INTRP = DIRECT_CMD + "MVS "

# Tool commands
STANDARD_TOOL_COORD = "MEXTL"
CURRENT_TOOL_NO = "M_TOOL"
CURRENT_BASE = "MEXBS"
TOOL_OFFSET_CMD = "TOOL"

# Parameters for R3 protocol
ROBOT_NO = 1
PROGRAM_NO = 1


class R3SubApi:
    """
    All R3 APIs need to inherit from this to ensure that the root (object) is initialized correctly.
    """

    def __init__(self, *, client, **kwargs):
        # Discard any further optional parameters
        super().__init__()
        self.client = client

    def protocol_send(self, msg: str, silent_send=False, silent_recv=False):
        msg = f'{ROBOT_NO}{DELIMITER}{PROGRAM_NO}{DELIMITER}{msg}'

        if silent_send is False and silent_recv is False:
            self.client.send(msg)
        else:
            self.client.send(msg, silent_send=silent_send, silent_recv=silent_recv)


class R3Utility(R3SubApi):
    """
    API-functions related to general utility.
    """

    def __init__(self, client: IClient, **kwargs):
        super().__init__(client=client, **kwargs)

    def reset_alarm(self) -> None:
        """
        Reset the alarm bell on the rboot control.
        :return: None
        """
        self.protocol_send("RSTALRM")
        self.client.receive(silence_errors=True)

    def activate_servo(self) -> None:
        """
        Activates the servos.
        :return: None
        """
        self.protocol_send("SRVON")
        self.client.receive()

    def deactivate_servo(self) -> None:
        """
        Deactivate the servos.
        :return: None
        """
        self.protocol_send("SRVOFF")
        self.client.receive()

    def obtain_control(self) -> None:
        """
        Sends the cmd to obtain control.
        :return: None
        """
        self.protocol_send("CNTLON")
        self.client.receive()

    def release_control(self) -> None:
        """
        Sends the cmd to release control.
        :return: None
        """
        self.protocol_send("CNTLOFF")
        self.client.receive()

    def open_communication(self) -> None:
        """
        Sends the cmd to open the communication.
        :return: None
        """
        self.protocol_send("OPEN=NARCUSER")
        self.client.receive()

    def close_communication(self) -> None:
        """
        Sends the cmd to close the communication.
        :return: None
        """
        self.protocol_send("CLOSE")
        self.client.receive()


class R3Positions(R3SubApi):
    """
    API functions related to moving to cetain positions and declaring them.
    """

    def __init__(self, client: IClient, *, coordinate2cmd: Callable, digits: int, **kwargs):
        super().__init__(client=client, digits=digits, **kwargs)
        self.from_coord_to_cmd = coordinate2cmd
        self.digits = digits

    def set_position(self, name: str, pos: Coordinate) -> None:
        """
        Assign a value to an existing variable
        :param name: Name to be assigned to the variable
        :param pos: Coordinate to be set
        :return: None
        """
        self.protocol_send(f"{DIRECT_CMD}{name}={pos.to_melfa_point()}")
        self.client.receive()
        sleep(0.01)

    def define_variable(self, name: str, *, var_type: str) -> None:
        """

        :param name: Name to be assigned to the variable
        :param var_type:
        :return: None
        """
        if var_type == "position":
            var_cmd = "POS"
        elif var_type == "joint":
            var_cmd = "JNT"
        else:
            raise ValueError("Unknown variable type.")

        self.protocol_send(f"{DIRECT_CMD}DEF {var_cmd} {name}")
        self.client.receive()
        sleep(0.01)

    def linear_move(self, target: Coordinate) -> None:
        """
        Move to a position using linear interpolation.
        :param target: End position of the movement
        :return: None
        """
        coord_str = self.from_coord_to_cmd(target)
        self.protocol_send(f"{DIRECT_CMD}MVS {coord_str}")
        self.client.receive()

    def joint_move(self, values: List[float]) -> None:
        """
        Move to a position using joint interpolation.
        :param values: List of joint values
        :return: None
        """
        raise NotImplementedError
        # coord_str = ''.join(f"{identifier}{val}" for identifier, val in zip(identifiers, values))
        # self._protocol_send(f"{DIRECT_CMD}MOV ({coord_str})")
        # self.client.receive()

    def circular_move_centre(self, start: str, target: str, center: str) -> None:
        """
        Move to a position using circular interpolation.
        :param start: Start position of the arc (variable name)
        :param target: End position of the arc (variable name)
        :param center: Center position of the arc (variable name)
        :return: None
        """
        self.protocol_send(f"{DIRECT_CMD}MVR3 {start},{target},{center}")
        self.client.receive()

    def circular_move_intermediate(self, start: str, intermediate: str, target: str) -> None:
        """
        Move to a position using circular interpolation.
        :param start: Start position of the arc (variable name)
        :param intermediate: Intermediate position of the arc (variable name)
        :param target: End position of the arc (variable name)
        :return: None
        """
        self.protocol_send(f"{DIRECT_CMD}MVR {start},{intermediate},{target}")
        self.client.receive()

    def circular_move_full(self, start: str, intermediate1: str, intermediate2: str) -> None:
        """
        Move to a position using circular interpolation.
        :param start: Start position of the arc (variable name)
        :param intermediate1: First intermediate position of the arc (variable name)
        :param intermediate2: Second intermediate position of the arc (variable name)
        :return: None
        """
        self.protocol_send(f"{DIRECT_CMD}MVC {start},{intermediate1},{intermediate2}")
        self.client.receive()

    def go_safe_pos(self) -> None:
        """
        Moves the robot to its safe position.
        :return: None
        """
        self.protocol_send("MOVSP")
        self.client.receive()


class R3Reader(R3SubApi):
    """
    API functions related to reading values, parameters, ...
    """

    def __init__(self, client: IClient, joints: List[str], *, r2c: Callable, digits: int, **kwargs):
        """
        Create an interface object for reading functions.
        :param client: Communication client
        :param joints:
        :param r2c: Callable to convert a response to a target object.
        :param digits: Number of digits to be used for float to string conversions.
        """
        super().__init__(client=client, digits=digits, **kwargs)
        self.joints = joints
        self.from_response_to_coordinate = r2c
        self.digits = digits

    def get_current_tool_number(self) -> int:
        """
        Get the current tool number
        :return: Current tool number as integer
        """
        tool_no = self.read_variable(CURRENT_TOOL_NO)
        return int(tool_no)

    def get_current_tool_data(self) -> List[float]:
        # TODO Verify response with actual hardware
        tool_response = self.read_variable('P_TOOL')
        tool_data = tool_response
        tool_offsets = self.parse_comma_string(tool_data)
        return [float(i) for i in tool_offsets]

    def get_override(self) -> float:
        """
        Get the current override factor.
        :return: Current override factor, float.
        """
        return self._get_float_cmd(OVERRIDE_CMD, direct=False)

    def get_current_linear_speed(self) -> float:
        """
        Get the current linear speed in mm/s.
        :return: Current linear speed factor, float.
        """
        val = self.read_variable("M_RSPD")
        return float(val)

    def get_current_joint_speed(self) -> float:
        """
        Get the current joint speed in percent.
        :return: Current joint override, float.
        """
        val = self.read_variable("M_JOVRD")
        return float(val)

    def get_joint_borders(self) -> Tuple[float, ...]:
        """
        Get the limits for the joints in degrees.
        :return: Tuple of the joint borders: -J1, +J1, -J2, +J2, ...
        """
        borders = self._read_parameter("MEJAR")
        coordinates = self.parse_comma_string(borders)
        return tuple(float(i) for i in coordinates[0:2 * len(self.joints)])

    def get_xyz_borders(self) -> Tuple[float, ...]:
        """
        Get the limits for XYZ in mm.
        :return: Tuple of the cartesian borders: -X, +X, -Y, +X, -Z, +Z
        """
        borders = self._read_parameter("MEPAR")
        coordinates = self.parse_comma_string(borders)
        return tuple(float(i) for i in coordinates[0: 2 * 3])

    @staticmethod
    def parse_comma_string(response: str) -> List[str]:
        relevant_str = response.split(DELIMITER)[1]
        return relevant_str.split(", ")

    def get_current_xyzabc(self) -> Coordinate:
        """
        Get the current positions for XYZABC.
        :return: Coordinate object
        """
        self.protocol_send("PPOSF")
        coord_str = self.client.receive()
        return self.from_response_to_coordinate(melfa_str=coord_str, number_axes=len(self.joints))

    def get_current_joint(self) -> Coordinate:
        """
        Get the current joint angles.
        :return: Coordinate object
        """
        self.protocol_send("JPOSF")
        coord_str = self.client.receive()
        return self.from_response_to_coordinate(melfa_str=coord_str, number_axes=len(self.joints))

    def get_safe_pos(self) -> Coordinate:
        """
        Get the current value of the safe position.
        :return: Coordinate object
        """
        answer_str = self._read_parameter("JSAFE")
        safe_pos_values = [float(i) for i in answer_str.split(DELIMITER)[1].split(", ")]
        return Coordinate(safe_pos_values, self.joints)

    def get_servo_state(self) -> int:
        servo_state = self.read_variable(SRV_STATE_VAR)
        return int(servo_state)

    @staticmethod
    def poll(func: Callable, val, poll_rate_ms: int = 5, timeout_ms: int = 60000):
        """
        Poll a reading method until a given value is received or a timeout occurs.
        :param func: Method of R3Reader to be called
        :param val:
        :param poll_rate_ms:
        :param timeout_ms:
        :return:
        :raises: ValueError, if the method is not listed in the pollable methods.
        """
        # Iteratively call the method
        total_time_waited = 0
        response = ''
        while total_time_waited < timeout_ms:
            response = func()
            if response == val:
                return
            sleep(poll_rate_ms / 1000)
            total_time_waited += poll_rate_ms
        raise ClientError(f"Timeout after {timeout_ms} ms. Expected: '{val}' but got '{response}'")

    def _read_parameter(self, parameter: str) -> str:
        """
        Auxilary function to read any parameter
        :param parameter: String representation of the paramter
        :return: Client response excluding status
        """
        self.protocol_send(f"PNR{parameter}")
        return self.client.receive()

    def read_variable(self, variable: str) -> str:
        """
        Auxilary function to read number variables.
        :param variable: String representation of the variable
        :return: Client response excluding status
        """
        self.protocol_send(f"VAL{variable}")
        response = self.client.receive()
        return response.split(f"{variable}=")[-1]

    def _get_float_cmd(self, cmd: str, direct=True) -> float:
        """
        Use an arbitrary command getting a float value
        :param cmd: Command to be used to get the value
        :param direct: Flag to specify whether direct execution is required
        :return: Value converted to float
        """
        # Send cmd
        if direct:
            self.protocol_send(f"{DIRECT_CMD}{cmd}")
        else:
            self.protocol_send(cmd)
        return_val = self.client.receive()

        # Convert value to float
        return_val = float(return_val)
        return return_val

    def _get_coordinate_cmd(self, cmd: str, direct=True) -> float:
        if direct:
            self.protocol_send(f"{DIRECT_CMD}{cmd}")
        else:
            self.protocol_send(cmd)
        coord_str = self.client.receive()
        return self.from_response_to_coordinate(melfa_str=coord_str, number_axes=len(self.joints))


class R3Setter(R3SubApi):
    """
    API functions related to setting.
    """

    def __init__(self, client: IClient, digits: int, **kwargs):
        """
        Create an interface object for reading functions.
        :param client: Communication client
        :param digits: Number of digits to be used for float to string conversions.
        """
        super().__init__(client=client, digits=digits, **kwargs)
        self.digits = digits

    def set_current_tool(self, tool_number: int) -> None:
        """
        Select the current tool
        :param tool_number: Number of the tool to be selected
        :return: None
        :raises: ValueError if tool number is outside of specified bounds
        """
        tool_number = int(tool_number)
        self._check_bounds(tool_number, lbound=1, ubound=4)
        self.protocol_send(f"{DIRECT_CMD}{CURRENT_TOOL_NO}={int(tool_number)}")
        self.client.receive()

    def set_current_tool_data(self, tool_offset: str) -> None:
        """
        Set the coordinate transformation for the currently selected tool.
        :param tool_offset:
        :return: None
        """
        # TODO Check conversion from coordinate to tool offset
        self.protocol_send(f"{DIRECT_CMD}{TOOL_OFFSET_CMD} {tool_offset}")
        self.client.receive()

    def set_work_coordinate(self, xyz: Tuple[Optional[float], Optional[float], Optional[float]]) -> None:
        """
        Sets the current coordinate system to the origin specified by the offset
        :param xyz:
        :return: None
        """
        # Conversion from coordinate to offset
        xyz_offsets = [f'{var:.2f}' if var is not None else '' for var in xyz]
        offset = f'({",".join(xyz_offsets)},,,)'
        self.protocol_send(f"{DIRECT_CMD}{BASE_COORDINATE_CMD} {offset}")
        self.client.receive()
        sleep(5)

    def set_override(self, factor: float) -> None:
        """
        Sends the cmd to set the override value to the specified factor.
        :param factor: Override factor to set in percent, [1,100] %
        :return: None
        :raises: ValueError, if the value is outside of the bounds
        """
        self._check_bounds(factor, lbound=1.0, ubound=100.0)
        self.protocol_send("{}={:.{d}f}".format(OVERRIDE_CMD, factor, d=self.digits))
        self.client.receive()

    def set_linear_speed(self, speed: float) -> None:
        """
        Set the linear speed to a given value.
        :param speed: Desired speed in mm/s, [1,1000]
        :return: None
        :raises: ValueError, if the value is outside of the bounds
        """
        self._set_float_cmd(LINEAR_SPEED, speed, lbound=1.0)

    def set_joint_speed(self, speed: float) -> None:
        """
        Set the joint speed to a given value.
        :param speed: Desired speed in percent, [1,100] %
        :return: None
        :raises: ValueError, if the value is outside of the bounds
        """
        self._set_float_cmd(JOINT_SPEED, speed, lbound=1.0, ubound=100.0)

    def _set_float_cmd(self, cmd: str, val: float, direct=True, lbound: Optional[float] = None,
                       ubound: Optional[float] = None, ):
        """
        Sets a float value to a variable.
        :param cmd: Command used to set the variable
        :param val: Value to be set, will be converted to float
        :param direct: Flag to specify whether a direct execution using EXEC is required
        :param lbound: Lower bound as float, defaults to None
        :param ubound: Upper bound as float, defaults to None
        :return: None
        :raises: ValueError, if the value is outside of the bounds specified by lbound and ubound
        """

        # Defensive programming
        value = float(val)

        # Check range
        self._check_bounds(value, lbound, ubound)

        # Send cmd
        if direct:
            self.protocol_send(
                "{}{} {:.{d}f}".format(DIRECT_CMD, cmd, float(value), d=self.digits)
            )
        else:
            self.protocol_send("{} {:.{d}f}".format(cmd, float(value), d=self.digits))
        self.client.receive()

    def _check_bounds(self, value: float, lbound: Optional[float], ubound: Optional[float]) -> None:
        """
        Check whether a value is within specified bounds.
        :param value: Value to be checked
        :param lbound: Lower bound, None if not relevant
        :param ubound: Upper bound, None if not relevant
        :return: None
        :raises: ValueError, if the value is outside of the bounds specified
        """
        # Check lower range range
        if lbound is not None and value < lbound:
            raise ValueError("Value must be >= {:.{d}f}.".format(lbound, d=self.digits))

        # Check upper range
        if ubound is not None and value > ubound:
            raise ValueError("Value must be <= {:.{d}f}.".format(ubound, d=self.digits))


class R3Resetter(R3SubApi):
    """
    API functions related to resets.
    """

    def __init__(self, client: IClient, **kwargs):
        super().__init__(client=client, **kwargs)

    def reset_base_coordinate_system(self) -> None:
        """
        Reset the base coordinate system to its default.
        :return: None
        """
        self._reset_cmd(BASE_COORDINATE_CMD, var_type="point")
        sleep(5)

    def reset_tool_data(self) -> None:
        """
        Reset the current tool to the default tool offset
        :return: None
        """
        self._reset_cmd(TOOL_OFFSET_CMD, var_type="point")

    def reset_override(self) -> None:
        """
        Reset the override factor to its maximum..
        :return: None
        """
        self._reset_cmd(OVERRIDE_CMD, var_type="number", direct=False)

    def reset_linear_speed(self) -> None:
        """
        Reset the linear speed to its maximum.
        :return: None
        """
        self._reset_cmd(LINEAR_SPEED, var_type="number")

    def reset_joint_speed(self) -> None:
        """
        Reset the joint speed factor to its maximum.
        :return: None
        """
        self._reset_cmd(JOINT_SPEED, var_type="number")

    def reset_all_speeds(self) -> None:
        """
        Reset the speed modification factors to maximum speed.
        :return: None
        """
        self.reset_linear_speed()
        self.reset_joint_speed()

    def _reset_cmd(self, cmd: str, *, var_type: str, default_name: Optional[str] = None, direct: bool = True) -> None:
        """
        Resets a variable to its default.
        :param cmd: Command used to manipulate the variable
        :param var_type: Variable type: point, number
        :param default_name: Name of the parameter with the default value, defaults to M_N{command}, e.g. M_NOVRD
        :param direct: Flag to specify whether a direct execution using EXEC is required
        :return: None
        """
        if default_name is None:
            if var_type == "point":
                default_name = f"P_N{cmd}"
            elif var_type == "number":
                default_name = f"M_N{cmd}"
            else:
                raise ValueError(f"Unknown variable type: {var_type}")
        if direct:
            self.protocol_send(f"{DIRECT_CMD}{cmd} {default_name}")
        else:
            self.protocol_send(f"{cmd} {default_name}")
        self.client.receive()
        sleep(0.1)


class R3Protocol(R3Resetter, R3Setter, R3Reader, R3Positions, R3Utility):
    """
    Mitsubishi robots do not support G-Code natively, hence the protocol for the commands is implemented.
    Inherits all individual sub-APIs.
    """

    DIGITS = 2

    def __init__(self, client: IClient, coordinate_adapter: MelfaCoordinateService, joints: int = 6,
                 digits: int = DIGITS):
        """
        Create a protocol object.
        :param client: Client to be used for the communication.
        :param coordinate_adapter:
        :param joints: Number of joints
        :param digits: Number of digits to be used for string to float conversions, defaults to 2
        """
        # Initialize the individual parts of the API
        super().__init__(
            client=client,
            digits=digits,
            joints=["J{}".format(i) for i in range(1, joints + 1)],
            coordinate2cmd=coordinate_adapter.to_cmd,
            r2c=coordinate_adapter.from_response
        )

    @property
    def reader(self) -> R3Reader:
        """
        Extract the Sub-API for reading
        :return: Sub-API element
        """
        return R3Reader(self.client, self.joints, r2c=self.from_response_to_coordinate, digits=self.digits)

    @property
    def setter(self) -> R3Setter:
        """
        Extract the Sub-API for setting
        :return: Sub-API element
        """
        return R3Setter(self.client, digits=self.digits)

    @property
    def resetter(self) -> R3Resetter:
        """
        Extract the Sub-API for resetting
        :return: Sub-API element
        """
        return R3Resetter(self.client, digits=self.digits)

    @property
    def util(self) -> R3Utility:
        """
        Extract the Sub-API for utilities
        :return: Sub-API element
        """
        return R3Utility(self.client)

    @property
    def pos(self) -> R3Positions:
        """
        Extract the Sub-API for positions
        :return: Sub-API element
        """
        return R3Positions(self.client, coordinate2cmd=self.from_coord_to_cmd, digits=self.digits)
