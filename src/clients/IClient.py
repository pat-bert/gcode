import abc


class Msg:
    """
    Message object to be used internally in the clients.
    """

    def __init__(self, msg, silent_send, silent_recv):
        self.msg = msg
        self.ss = silent_send
        self.sr = silent_recv

    def unpack(self):
        return self.msg, self.ss, self.sr


class Response:
    """
    Response object to be used internally in the clients.
    """
    ERROR, WARNING, SUCCESS = range(3)

    def __init__(self, kind: int, data: str):
        pass


class ClientError(IOError):
    pass


class ClientOpenError(ClientError):
    """
    Exception to be raised if a connection to the client cannot be opened.
    """
    pass


class ClientNotAvailableError(ClientOpenError):
    """
    Exception to be raised if the client is known but not available on the I/O.
    """

    def __init__(self, vid, pid):
        super().__init__('Client with VID:PID = {}:{} not available.'.format(vid, pid))


class IClient(metaclass=abc.ABCMeta):
    """
    Interface to the communication client layer of the application.
    """

    @abc.abstractmethod
    def connect(self) -> None:
        """ Connect to the hardware component via the client.
        All parameters necessary should be specified when creating the client.
        This method is used to decouple the I/O access from collecting the information to do so.
        :return: None
        :raises: ClientError if client could not be opened.
        """
        pass

    @abc.abstractmethod
    def close(self) -> None:
        """
        Close the connection to the hardware component.
        :return: None
        :raises: ClientError if client could not be closed.
        """
        pass

    @abc.abstractmethod
    def send(self, msg: str, silent_send: bool = False, silent_recv: bool = False):
        pass

    @abc.abstractmethod
    def receive(self, silence_errors=False):
        pass
