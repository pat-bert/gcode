from threading import RLock
from time import sleep
from typing import Optional

from src.clients.ComClient import ComClient


class SerialEcho:
    """
    Test class to fake responses from a serial device
    """

    def __init__(self):
        # Create a COM-Client for mutual testing
        self.client = ComClient((2, 3))

    def run(self):
        with self.client:
            incoming_msg = self.client.receive()
            outgoing_msg = self.resolve_msg(incoming_msg)
            self.client.send(outgoing_msg)

    @staticmethod
    def resolve_msg(msg):
        return msg


class ConfigurableEcho(SerialEcho):
    def __init__(self):
        super().__init__()
        self._prefix = None
        self._postfix = None
        self._replace_msg = None
        self._delay = 0
        self.lock = RLock()

    def reconfigure(self, pre: Optional[str] = None, post: Optional[str] = None, msg: Optional[str] = None,
                    dly: Optional[float] = None) -> None:
        """
        Adjust the calculation of the server response.
        :param pre: String to be inserted before each actual message
        :param post: String to be inserted before each actual message
        :param msg: String to replace the actual message with
        :param dly: Float representing the time to wait before responding in seconds
        :return: None
        """
        # Ensure that the parameters are not read while setting new values
        with self.lock:
            # Set the new parameters
            self._prefix = pre or None
            self._postfix = post or None
            self._replace_msg = msg or None
            self._delay = dly or 0

    def resolve_msg(self, msg: str) -> str:
        # Ensure that the parameters are not changed in the mean time
        with self.lock:
            # Do the manipulations
            if self._replace_msg is not None:
                msg = self._replace_msg
            if self._prefix is not None:
                msg = self._prefix + msg
            if self._postfix is not None:
                msg = msg + self._postfix
            # Delay the return of the message to simulate processing time at the responding end
            sleep(self._delay)
        return msg
