from typing import Tuple, Optional, Callable

from src.Coordinate import Coordinate
from src.protocols.IProtocol import CoordinateAdapter

DELIMITER = ';'
DIRECT_CMD = 'EXEC'
OVERRIDE_CMD = 'OVRD'
JOINT_SPEED = 'JOVRD'
LINEAR_SPEED = 'SPD'

COM_OPEN = "OPEN=NARCUSER"
COM_CLOSE = "CLOSE"
SRV_ON = "SRVON"
SRV_OFF = "SRVOFF"
CNTL_ON = "CNTLON"
CNTL_OFF = "CNTLOFF"
MOVE_SAFE_POSITION = "MOVSP"
PARAMETER_SAFE_POSITION = "JSAFE"
LINEAR_INTRP = DIRECT_CMD + "MVS "
JOINT_INTRP = DIRECT_CMD + "MOV "
CIRCULAR_INTERPOLATION_CENTRE = DIRECT_CMD + "MVR3 "
CIRCULAR_INTERPOLATION_IM = DIRECT_CMD + "MVR "
CIRCULAR_INTERPOLATION_FULL = DIRECT_CMD + "MVC "
ALARM_RESET_CMD = "RSTALRM"
VAR_READ = "VAL"
SRV_STATE_VAR = "M_SVO"
CURRENT_SPEED_VAR = "M_RSPD"
CURRENT_TOOL_NO = "M_TOOL"
CURRENT_TOOL_DATA = "MEXTL"
CURRENT_BASE = "MEXBS"
PARAMETER_READ = "PNR"
JOINT_BORDERS = "MEJAR"
XYZ_BORDERS = "MEPAR"
CURRENT_XYZABC = "PPOSF"
CURRENT_JOINT = "JPOSF"
RESET_BASE_COORDINATES = DIRECT_CMD + "BASE P_NBASE"
SET_BASE_COORDINATES = DIRECT_CMD + "BASE "
MVS_SPEED = DIRECT_CMD + "SPD "
MVS_MAX_SPEED = "M_NSPD"
MOV_SPEED = DIRECT_CMD + "JOVRD "
MOV_MAX_SPEED = "M_NJOVRD"
DEF_POS = DIRECT_CMD + "DEF POS "
SERVO_INIT_SEC = 5


class R3Protocol:
    """
    Mitsubishi robots do not support G-Code natively, hence the protocol for the commands is implemented.
    """
    DIGITS = 2

    def __init__(self, client, coordinate_adapter: CoordinateAdapter, digits: int = DIGITS):
        """
        Create a protocol object.
        :param client: Client to be used for the communication.
        :param coordinate_adapter:
        :param digits:
        """
        self.client = client
        self.coord_adapt = coordinate_adapter
        self.digits = digits

    def open_communication(self) -> None:
        """
        Sends the cmd to open the communication.
        :return: None
        """
        self.client.send('OPEN=NARCUSER')

    def close_communication(self) -> None:
        """
        Sends the cmd to close the communication.
        :return: None
        """
        self.client.send('CLOSE')

    def obtain_control(self) -> None:
        """
        Sends the cmd to obtain control.
        :return: None
        """
        self.client.send('CNTLON')

    def release_control(self) -> None:
        """
        Sends the cmd to release control.
        :return: None
        """
        self.client.send('CNTLOFF')

    def activate_servo(self) -> None:
        """
        Activates the servos.
        :return: None
        """
        self.client.send('SRVON')

    def deactivate_servo(self) -> None:
        """
        Deactivate the servos.
        :return: None
        """
        self.client.send('SRVOFF')

    def go_safe(self) -> None:
        self.client.send('MOVSP')
        self.client.receive()

    def reset_alarm(self) -> None:
        self.client.send('ALARM_RESET_CMD')
        self.client.receive()


class R3Positions:
    def __init__(self, client, coordinate2cmd: Callable, digits: int):
        self.client = client
        self.from_cmd_to_coord = coordinate2cmd
        self.digits = digits

    def linear_move(self, target: Coordinate) -> None:
        """
        Move to a position using linear interpolation.
        :param target: End position of the movement
        :return: None
        """
        coord_str = self.from_cmd_to_coord(target)
        self.client.send('{}MVS{}'.format(DIRECT_CMD, coord_str))

    def joint_move(self, target: Coordinate) -> None:
        """
        Move to a position using joint interpolation.
        :param target: End position of the movement
        :return:
        """
        coord_str = self.from_cmd_to_coord(target)
        self.client.send('{}MOV{}'.format(DIRECT_CMD, coord_str))

    def circular_move_centre(self, start: str, target: str, center: str) -> None:
        """
        Move to a position using circular interpolation.
        :param start: Start position of the arc
        :param target: End position of the arc
        :param center: Center position of the arc
        :return:
        """
        self.client.send('{}MVR3 {},{},{}'.format(DIRECT_CMD, start, target, center))

    def circular_move_intermediate(self, start: str, intermediate: str, target: str) -> None:
        """
        Move to a position using circular interpolation.
        :param start: Start position of the arc
        :param intermediate: Intermediate position of the arc
        :param target: End position of the arc
        :return:
        """
        self.client.send('{}MVR {},{},{}'.format(DIRECT_CMD, start, intermediate, target))

    def circular_move_full(self, start: str, intermediate1: str, intermediate2: str) -> None:
        """
        Move to a position using circular interpolation.
        :param start: Start position of the arc
        :param intermediate1: First intermediate position of the arc
        :param intermediate2: Second intermediate position of the arc
        :return:
        """
        self.client.send('{}MVC {},{},{}'.format(DIRECT_CMD, start, intermediate1, intermediate2))


class R3Reader:
    def __init__(self, client, response2coordinate: Callable, digits: int):
        """

        :param client:
        :param response2coordinate: Callable to convert a response to a target object.
        :param digits:
        """
        self.client = client
        self.from_response_to_coordinate = response2coordinate
        self.digits = digits

    def get_override(self) -> float:
        """
        Get the current override factor.
        :return: Current override factor, float.
        """
        self.client.send(OVERRIDE_CMD)
        override = self.client.receive()
        return round(override, ndigits=self.digits)

    def get_current_linear_speed(self) -> float:
        pass

    def get_joint_speed(self) -> float:
        pass

    def get_joint_borders(self) -> Tuple:
        """
        Get the limits for the joints.
        :return:
        """
        borders = self._parameter_read('MEJAR')
        coordinate_str = borders.split(DELIMITER)[1]
        coordinates = coordinate_str.split(", ")
        return tuple(float(i) for i in coordinates)

    def get_xyz_borders(self) -> Tuple:
        """
        Get the limits for XYZ.
        :return:
        """
        borders = self._parameter_read('MEPAR')
        coordinate_str = borders.split(DELIMITER)[1]
        coordinates = coordinate_str.split(", ")
        return tuple(float(i) for i in coordinates)

    def get_current_xyz(self) -> Coordinate:
        """
        Get the current position.
        :return: None
        """
        self.client.send('PPOSF')
        coord_str = self.client.receive()
        return self.from_response_to_coordinate(coord_str)

    def get_current_joint(self) -> Coordinate:
        """
        Get the current joint angles.
        :return: None
        """
        self.client.send('JPOSF')
        coord_str = self.client.receive()
        return self.from_response_to_coordinate(coord_str)

    def _parameter_read(self, parameter: str) -> str:
        self.client.send('PNR{}'.format(parameter))
        return self.client.receive()

    def _variable_read(self, variable: str) -> str:
        self.client.send('VAL{}'.format(variable))
        return self.client.receive()

    def _get_float_cmd(self, cmd: str, direct=True) -> float:
        self.client.send()
        return_val = self.client.receive()

        # Send cmd
        if direct:
            self.client.send('{}{}'.format(DIRECT_CMD, cmd))
        else:
            self.client.send('{}{}'.format(DIRECT_CMD, cmd))

        # Convert value to float
        return_val = round(return_val, ndigits=self.digits)
        return return_val

    def _get_coordinate_cmd(self, cmd: str, direct=True) -> float:
        if direct:
            self.client.send('{}{}'.format(DIRECT_CMD, cmd))
        else:
            self.client.send(cmd)
        coord_str = self.client.receive()
        return self.from_response_to_coordinate(coord_str)


class R3Setter:
    def __init__(self, client, digits):
        self.client = client
        self.digits = digits

    def set_override(self, factor: float) -> None:
        """
        Sends the cmd to set the override value to the specified factor.
        :param factor: Override factor to set in percent
        :return: None
        """
        self._set_float_cmd(OVERRIDE_CMD, factor, direct=False, lbound=1.0, ubound=100.0)

    def set_linear_speed(self, speed: float) -> None:
        """
        Set the linear speed to a given value.
        :param speed: Desired speed in mm/s
        :return:
        """
        self._set_float_cmd(LINEAR_SPEED, speed)

    def set_joint_speed(self, speed: float) -> None:
        """
        Set the joint speed to a given value.
        :param speed: Desired speed in percent
        :return:
        """
        self._set_float_cmd(JOINT_SPEED, speed, lbound=1.0, ubound=100.0)

    def _set_float_cmd(self, cmd: str, val: float, direct=True, lbound: Optional[float] = None,
                       ubound: Optional[float] = None):
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
        value = float(val)

        # Check range
        if lbound is not None:
            if value < lbound:
                if ubound is not None:
                    raise ValueError('Value out of range [{:.{d}f};{:.{d}f}].'.format(lbound, ubound, d=self.digits))
                else:
                    raise ValueError('Value must be >= {:.{d}f}.'.format(lbound, d=self.digits))
        if ubound is not None:
            if value > ubound:
                if lbound is not None:
                    raise ValueError('Value out of range [{:.{d}f};{:.{d}f}].'.format(lbound, ubound, d=self.digits))
                else:
                    raise ValueError('Value must be <= {:.{d}f}.'.format(ubound, d=self.digits))

        # Send cmd
        if direct:
            self.client.send('{}{} {:.{d}f}'.format(DIRECT_CMD, cmd, float(value), d=self.digits))
        else:
            self.client.send('{}{} {:.{d}f}'.format(DIRECT_CMD, cmd, float(value), d=self.digits))
        self.client.receive()


class R3Resetter:
    def __init__(self, client, digits):
        self.client = client
        self.digits = digits

    def reset_override(self) -> None:
        """
        Reset the override factor to its maximum..
        :return:
        """
        self._reset_cmd(OVERRIDE_CMD, direct=False)

    def reset_linear_speed(self) -> None:
        """
        Reset the linear speed to its maximum.
        :return:
        """
        self._reset_cmd(LINEAR_SPEED)

    def reset_joint_speed(self) -> None:
        """
        Reset the joint speed factor to its maximum.
        :return:
        """
        self._reset_cmd(JOINT_SPEED)

    def _reset_cmd(self, command, default=None, direct=True) -> None:
        """
        Resets a variable to its default.
        :param command: Command used to manipulate the variable
        :param default: Name of the parameter with the default value, defaults to M_N{command}, e.g. M_NOVRD
        :param direct: Flag to specify whether a direct execution using EXEC is required
        :return: None
        """
        if default is None:
            default = 'M_N{}'.format(command)
        if direct:
            self.client.send('{}{} {}'.format(DIRECT_CMD, command, default))
        else:
            self.client.send('{} {}'.format(command, default))
        self.client.receive()
