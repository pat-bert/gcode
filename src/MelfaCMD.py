from BaseCmd import BaseCmd
from typing import *


class MelfaCMD(BaseCmd):
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
    def read_cmd_str(cls, command_str) -> Union['MelfaCMD', None]:
        return cls('0')
