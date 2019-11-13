"""
File:       MelfaError.py
Author:     Patrick Bertsch
Content:    Exceptions to signal events in communication with robot.
"""


class MelfaBaseException(Exception):
    def __init__(self, status_code):
        self.status = status_code


class MelfaMinorIssue(MelfaBaseException):
    def __str__(self):
        return "Minor issue executing command (" + self.status + ")."


class MelfaInvalidCommand(MelfaBaseException):
    def __str__(self):
        return "Command understood but not valid (" + self.status + ")."


class MelfaUnknownCommand(MelfaBaseException):
    def __str__(self):
        return "Unknown command (" + self.status + ")."


# Status codes from robot controller
COMMAND_OK_EXECUTED = "QoK"
COMMAND_OK_MINOR_ISSUE = "Qok"
COMMAND_OK_INVALID = "QeR"
COMMAND_NOK = "Qer"

# Mapping
ErrorDispatch = {
    COMMAND_OK_EXECUTED: None,
    COMMAND_OK_MINOR_ISSUE: MelfaMinorIssue,
    COMMAND_OK_INVALID: MelfaInvalidCommand,
    COMMAND_NOK: MelfaUnknownCommand
}
