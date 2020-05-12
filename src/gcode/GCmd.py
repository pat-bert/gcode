from typing import Union, Tuple, List
from src.BaseCmd import BaseCmd
from src.Coordinate import Coordinate


class GCmd(BaseCmd):
    """
    This class implements a base G-code command.
    """

    # Define number of relevant post-comma digits
    DIGITS = 2
    # Code standard
    CMD_REMOVE_LEAD_ZERO = True
    # Define axis descriptors for absolute coordinates
    ABS_AXES = "XYZ"
    # Define axis descriptors for relative coordinates
    REL_AXES = "IJK"
    # Define speed descriptor
    SPEED_DESCRIPTOR = "F"
    # Define feeder descriptor
    EXTRUDE_DESCRIPTOR = "E"
    # Time descriptors
    TIME_MS_DESCRIPTOR = "P"
    TIME_S_DESCRIPTOR = "S"
    # M-command descriptor
    M_DESCRIPTOR = "S"
    # Homing command
    HOME_CMD = "G28"
    # Command types that allow misc arguments
    MISC_CMD_IDS = "M"
    # Comment descriptor
    COMMENT = ";"
    # Supported commands
    SUPPORTED_G_CODES = {
        "G": [0, 1, 2, 3, 4, 17, 18, 19, 20, 21, 28, 90, 91],
        "M": [104, 106, 109, 140],
        "T": [0, 1, 2, 3]
    }

    def __init__(
            self,
            code_id: str,
            abs_cr: Tuple[float, ...] = None,
            rel_cr: Tuple[float, ...] = None,
            speed: float = None,
            e_length: float = None,
            time_ms: int = None,
            misc_cmd: Union[float, str] = None,
            home: str = "",
            line_number: int = None,
    ) -> None:
        """
        Initialise an object.
        :param code_id: G-Code identifier
        :param abs_cr: Optional tuple of absolute cartesian coordinates
        :param rel_cr: Optional tuple of relative cartesian coordinates
        :param speed: Optional number for speed of printer head
        :param e_length: Optional number for extrude length
        :param time_ms: Optional number for time in ms
        :param misc_cmd: Optional argument for M-commands
        :param home: Optional string for homing
        :param line_number: Optional number for resend identification
        """
        self.id = code_id
        self.cartesian_abs: Coordinate = Coordinate(abs_cr, self.ABS_AXES, self.DIGITS)
        self.cartesian_rel: Coordinate = Coordinate(
            rel_cr, self.ABS_AXES, self.DIGITS, print_axes=self.REL_AXES
        )
        self.speed = speed
        self.extrude_len = e_length
        self.time_ms = time_ms
        self.machine_option = misc_cmd
        self.home_opt = home
        self.line_number = line_number

        if not self._is_valid():
            raise ValueError("Unsupported or unknown command passed: " + self.id)

    def _is_valid(self) -> bool:
        """
        Validate input.
        :return:
        """
        cmd_char = self.id[0]
        cmd_cnt = int(self.id[1:])
        return (
                cmd_char in self.SUPPORTED_G_CODES.keys()
                and cmd_cnt in self.SUPPORTED_G_CODES[cmd_char]
        )

    def __str__(self):
        """
        Create string representation of command.
        """
        if self.cartesian_abs is not None:
            abs_str = str(self.cartesian_abs)
            if len(abs_str) > 0:
                abs_str += " "
        else:
            abs_str = ""

        if self.cartesian_rel is not None:
            rel_str = str(self.cartesian_rel)
            for abs_axis, rel_axis in zip(self.ABS_AXES, self.REL_AXES):
                rel_str = rel_str.replace(abs_axis, rel_axis)
            if len(rel_str) > 0:
                rel_str += " "
        else:
            rel_str = ""

        speed_str = self.combine(self.SPEED_DESCRIPTOR, self.speed)
        extruder_len_str = self.combine(self.EXTRUDE_DESCRIPTOR, self.extrude_len)
        time_str = self.combine(self.TIME_MS_DESCRIPTOR, self.time_ms)
        m_str = self.combine(self.M_DESCRIPTOR, self.machine_option)
        home_str = " ".join(self.home_opt)

        total_str = (
                str(self.id)
                + " "
                + abs_str
                + rel_str
                + speed_str
                + extruder_len_str
                + time_str
                + m_str
                + home_str
        )
        return total_str.strip()

    @classmethod
    def read_cmd_str(cls, command_str: str) -> Union["GCmd", None]:
        """
        Converts a command string into an object.
        :param command_str: Input string
        :return:
        """
        # Split space-separated parts of the command
        segments = command_str.split(" ")

        if command_str.startswith(cls.COMMENT):
            # Passed string is a comment so you cannot return a command/maybe an empty one in the future
            return None
        else:
            # Command identifier is required first
            if cls.CMD_REMOVE_LEAD_ZERO:
                try:
                    cmd_id = segments[0][0] + str(int(segments[0][1:]))
                except ValueError:
                    # TODO Do something else
                    raise
            else:
                cmd_id = segments[0]

            # Split remaining parts into their identifiers and recognise each of them
            args = {}
            for arg in segments[1:]:
                if len(arg) > 1:
                    # Arguments consisting of descriptor and value
                    try:
                        val = float(arg[1:])
                    except ValueError:
                        val = arg[1:]
                    args[arg[0]] = val
                else:
                    # Arguments consisting only of a descriptor
                    args[arg] = None

            # Get speed arguments
            speed = args.get(cls.SPEED_DESCRIPTOR, None)
            e_length = args.get(cls.EXTRUDE_DESCRIPTOR, None)

            # Get time argument, preferring milliseconds
            time_ms = args.get(cls.TIME_MS_DESCRIPTOR, None)
            if time_ms is None and cmd_id[0] not in cls.MISC_CMD_IDS:
                time_ms = args.get(cls.TIME_S_DESCRIPTOR, None)
                if time_ms is not None:
                    time_ms *= 1000

            # Get miscellaneous argument
            if cmd_id[0] in cls.MISC_CMD_IDS:
                misc_cmd = args.get(cls.M_DESCRIPTOR, None)
            else:
                misc_cmd = None

            # Get relative arguments
            rel_cr = [args.get(axis, None) for axis in cls.REL_AXES]
            rel_cr = cls.expand_coordinates(rel_cr)

            # Get absolute coordinates or home axis respectively
            if cmd_id == cls.HOME_CMD:
                abs_cr = None
                home = "".join((axis for axis in cls.ABS_AXES if axis in args))
            else:
                home = ""
                abs_cr = [args.get(axis, None) for axis in cls.ABS_AXES]
                abs_cr = cls.expand_coordinates(abs_cr)

            # Initialise command
            return cls(
                cmd_id,
                abs_cr=abs_cr,
                rel_cr=rel_cr,
                speed=speed,
                e_length=e_length,
                time_ms=time_ms,
                misc_cmd=misc_cmd,
                home=home,
            )

    @classmethod
    def expand_coordinates(
            cls, coordinates: List[Union[str, None]]
    ) -> Union[Tuple[float, ...], None]:
        """

        :param coordinates:
        :return:
        """
        if coordinates.count(None) == len(coordinates):
            coordinates = None
        else:
            try:
                coordinates = tuple((float(i) for i in coordinates))
            except TypeError:
                pass
        return coordinates
