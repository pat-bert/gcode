"""
File:       BaseCmd.py
Author:     Patrick Bertsch
Content:    Add common properties and methods for any machine commands here for inheriting.
"""


class BaseCmd:
    """
    Base class for machine commands.
    """

    @staticmethod
    def combine(descriptor, value, delimiter=''):
        if value is not None:
            return '{}{}{} '.format(descriptor, delimiter, value)
        else:
            return ''

    def _is_valid(self):
        return True
