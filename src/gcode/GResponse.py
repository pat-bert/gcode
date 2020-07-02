from src.BaseCmd import BaseCmd


class GResponse(BaseCmd):
    """
    Implements possible return messages from a 3D-printer in response to a G-code.
    """

    STATE_OK = "ok"
    STATE_RESEND = "rs"
    STATE_HW_FAILURE = "!!"
    DELIMITER = ":"
    COORDINATE_ID = "C:"
    EXTRUDER_TEMP = "T"
    BED_TEMP = "B"
    DEFAULT_T = -273
    AXIS_LIST = "XYZ"

    def __init__(self, state, line_number=None, **kwargs):
        self.state = state
        self.resend_line = line_number
        self.extr_t = kwargs.pop(self.EXTRUDER_TEMP, self.DEFAULT_T)
        self.bed_t = kwargs.pop(self.BED_TEMP, self.DEFAULT_T)

        if self.COORDINATE_ID in kwargs:
            self.coordinate = tuple(kwargs.pop(axis, None) for axis in self.AXIS_LIST)
        else:
            self.coordinate = None

        # Assign remaining arguments to debug
        self.debug = kwargs

        # Check the consistency of the arguments passed
        self._validate()

    def _validate(self):
        if self.state == self.STATE_RESEND and self.resend_line is None:
            raise ValueError(
                "For state resend command a corresponding command line must be provided."
            )

    @classmethod
    def read_response_str(cls, response_str):
        """
        Read a possible response string and parse the arguments to initialise an object.
        :param response_str:
        :return:
        """
        segments = response_str.split(" ")
        state = segments[0]

        if state == cls.STATE_RESEND:
            # If the printer requests a resend there will be a line number next
            line = segments.pop(1)
        else:
            line = None

        # Parse remaining arguments
        kwargs = {}
        for arg in segments[1:]:
            key, val = arg.split(cls.DELIMITER)
            kwargs[key] = val

        return cls(state, kwargs, line_number=line)

    def __str__(self):
        """
        Return the message representation of an response object.
        :return:
        """
        if self.state == self.STATE_RESEND:
            line_str = str(self.resend_line)
        else:
            line_str = ""

        bed_t_str = self.combine(self.BED_TEMP, self.bed_t, delimiter=self.DELIMITER)
        extr_t_str = self.combine(
            self.EXTRUDER_TEMP, self.extr_t, delimiter=self.DELIMITER
        )

        return self.state + line_str + bed_t_str + extr_t_str
