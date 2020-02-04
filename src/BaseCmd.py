"""
File:       BaseCmd.py
Author:     Patrick Bertsch
Content:    Add common properties and methods for any machine commands here for inheriting.
"""
from typing import *


class BaseCmd:
    """
    Base class for machine commands.
    """

    id = ""

    @staticmethod
    def combine(descriptor: Any, value: Any, delimiter: str = "") -> str:
        if value is not None:
            return "{}{}{} ".format(descriptor, delimiter, value)
        else:
            return ""

    def _is_valid(self) -> bool:
        return True

    @classmethod
    def read_cmd_str(cls, command_str):
        raise NotImplementedError

    def __str__(self):
        raise NotImplementedError

    def validate(self) -> bool:
        return self._is_valid()
