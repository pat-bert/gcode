from typing import *

from AM_IR.BaseCmd import BaseCmd

# Basic commands for R3 protocol
DELIMITER = ';'
COM_OPEN = 'OPEN=NARCUSER'
COM_CLOSE = 'CLOSE'
SRV_ON = 'SRVON'
SRV_OFF = 'SRVOFF'
CNTL_ON = 'CNTLON'
CNTL_OFF = 'CNTLOFF'
DIRECT_CMD = 'EXEC'
MOVE_SAFE_POSITION = 'MOVSP'
PARAMETER_SAFE_POSITION = 'JSAFE'
LINEAR_INTRP = DIRECT_CMD + 'MVS '
JOINT_INTRP = DIRECT_CMD + 'MOV '
CIRCULAR_INTERPOLATION_CENTRE = DIRECT_CMD + 'MVR3 '
CIRCULAR_INTERPOLATION_IM = DIRECT_CMD + 'MVR '
ALARM_RESET_CMD = 'RSTALRM'
VAR_READ = 'VAL'
SRV_STATE_VAR = 'M_SVO'
CURRENT_SPEED_VAR = 'M_RSPD'

# Parameter modifications
PARAMETER_READ = 'PNR'

# Reachable area
JOINT_BORDERS = 'MEJAR'
XYZ_BORDERS = 'MEPAR'

# Current positions
CURRENT_XYZABC = 'PPOSF'
CURRENT_JOINT = 'JPOSF'

# Speed manipulations
OVERWRITE_CMD = 'OVRD'
MVS_SPEED = DIRECT_CMD + 'SPD '
MVS_MAX_SPEED = 'M_NSPD'
MOV_SPEED = DIRECT_CMD + 'JOVRD '
MOV_MAX_SPEED = ''

# Parameters
SERVO_INIT_SEC = 5


class MelfaCmd(BaseCmd):
    """
    This class implements a command for the Mitsubishi Melfa series.
    """
    COMMENT = ''
    SUPPORTED_CMDS = [
        'MVS', 'MOV'
    ]

    def __init__(self, code_id: str):
        self.id = code_id

        # Validate input
        if not self._is_valid():
            raise ValueError('Unsupported or unknown command passed: ' + self.id)

    def _is_valid(self) -> bool:
        return self.id in self.SUPPORTED_CMDS

    def __str__(self):
        pass

    @classmethod
    def read_cmd_str(cls, command_str) -> Union['MelfaCmd', None]:
        return cls('0')
