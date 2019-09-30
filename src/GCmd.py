from BaseCmd import BaseCmd
from Coordinate import Coordinate

supported_g_codes = {
    'G': [0, 1, 2, 3, 4, 17, 18, 19, 20, 21, 28, 90, 91],
    'M': [104, 106, 109, 140],
}


class GCmd(BaseCmd):
    """
    This class implements a base G-code command.
    """
    # Define number of relevant post-comma digits
    DIGITS = 3
    # Code standard
    CMD_REMOVE_LEAD_ZERO = True
    # Define axis descriptors for absolute coordinates
    ABS_COORDINATES = 'XYZ'
    # Define axis descriptors for relative coordinates
    REL_COORDINATES = 'IJK'
    # Define speed descriptor
    SPEED_DESCRIPTOR = 'F'
    # Define feeder descriptor
    FEED_DESCRIPTOR = 'E'
    # Time descriptors
    TIME_MS_DESCRIPTOR = 'P'
    TIME_S_DESCRIPTOR = 'S'
    # M-command descriptor
    M_DESCRIPTOR = 'S'
    # Homing command
    HOME_CMD = 'G28'
    # Command types that allow misc arguments
    MISC_CMD_IDS = 'M'
    # Comment descriptor
    COMMENT = ';'

    def __init__(self, code_id, abs_cr=None, rel_cr=None, speed=None, f_speed=None, time_ms=None, misc_cmd=None,
                 home='', line_number=None):
        """
        Initialise an object.
        :param code_id: G-Code identifier
        :param abs_cr: Optional tuple of absolute cartesian coordinates
        :param rel_cr: Optional tuple of relative cartesian coordinates
        :param speed: Optional number for speed
        :param f_speed: Optional number for feeding rate
        :param time_ms: Optional number for time in ms
        :param misc_cmd: Optional argument for M-commands
        :param home: Optional string for homing
        :param line_number: Optional number for resend identification
        """
        self.id = code_id
        self.cartesian_abs = Coordinate(abs_cr, self.ABS_COORDINATES, self.DIGITS)
        self.cartesian_rel = Coordinate(rel_cr, self.REL_COORDINATES, self.DIGITS)
        self.speed = speed
        self.feeder_speed = f_speed
        self.time_ms = time_ms
        self.machine_option = misc_cmd
        self.home_opt = home
        self.line_number = line_number

        if not self._is_valid():
            raise ValueError('Unsupported or unknown command passed: ' + self.id)

    def _is_valid(self):
        """
        Validate input.
        :return:
        """
        cmd_char = self.id[0]
        cmd_cnt = int(self.id[1:])
        return cmd_char in supported_g_codes.keys() and cmd_cnt in supported_g_codes[cmd_char]

    def __str__(self):
        """
        Create string representation of command.
        """
        if self.cartesian_abs is not None:
            abs_str = str(self.cartesian_abs)
            if len(abs_str) > 0:
                abs_str += ' '
        else:
            abs_str = ''

        if self.cartesian_rel is not None:
            rel_str = str(self.cartesian_rel)
            if len(rel_str) > 0:
                rel_str += ' '
        else:
            rel_str = ''

        speed_str = self.combine(self.SPEED_DESCRIPTOR, self.speed)
        feeder_speed_str = self.combine(self.FEED_DESCRIPTOR, self.feeder_speed)
        time_str = self.combine(self.TIME_MS_DESCRIPTOR, self.time_ms)
        m_str = self.combine(self.M_DESCRIPTOR, self.machine_option)
        home_str = ' '.join(self.home_opt)

        total_str = str(self.id) + ' ' + abs_str + rel_str + speed_str + feeder_speed_str + time_str + m_str + home_str
        return total_str.strip()

    @classmethod
    def read_cmd_str(cls, command_str):
        """
        Converts a command string into an object.
        :param command_str: Input string
        :return:
        """
        # Split space-separated parts of the command
        segments = command_str.split(' ')

        if command_str.startswith(cls.COMMENT):
            # Passed string is a comment so you cannot return a command/maybe an empty one in the future
            return None
        else:
            # Command identifier is required first
            if cls.CMD_REMOVE_LEAD_ZERO:
                try:
                    cmd_id = segments[0][0] + str(int(segments[0][1:]))
                except ValueError:
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
            else:
                # Get speed arguments
                speed = args.get(cls.SPEED_DESCRIPTOR, None)
                f_speed = args.get(cls.FEED_DESCRIPTOR, None)

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
                rel_cr = list(args.get(axis, None) for axis in cls.REL_COORDINATES)
                if rel_cr.count(None) == len(rel_cr):
                    rel_cr = None
                else:
                    for val in rel_cr:
                        try:
                            val = float(val)
                        except TypeError:
                            pass

                # Get absolute coordinates or home axis respectively
                if cmd_id == cls.HOME_CMD:
                    abs_cr = None
                    home = ''.join([axis for axis in cls.ABS_COORDINATES if axis in args])
                else:
                    home = ''
                    abs_cr = list(args.get(axis, None) for axis in cls.ABS_COORDINATES)
                    if abs_cr.count(None) == len(abs_cr):
                        abs_cr = None
                    else:
                        for val in abs_cr:
                            try:
                                val = float(val)
                            except TypeError:
                                pass

                # Initialise command
                return cls(cmd_id, abs_cr=abs_cr, rel_cr=rel_cr, speed=speed, f_speed=f_speed, time_ms=time_ms,
                           misc_cmd=misc_cmd, home=home)
