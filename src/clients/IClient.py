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


class ServerClosedConnectionError(IOError):
    """
    Exception to be raised if a connection closed unexpectedly.
    """
    pass


class ClientCloseError(ClientError):
    """
    Exception to be raised if a connection to the client cannot be closed.
    """
    pass


class ClientOpenError(ClientError):
    """
    Exception to be raised if a connection to the client cannot be opened.
    """
    pass


class AmbiguousHardwareError(ClientOpenError):
    """
    Exception to be raised if multiple clients match the search criteria.
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
    Abstract base class to the communication client layer of the application.

    Inheriting classes need to implement:
    - connecting and closing
    - sending and receiving
    - connected status

    These features are provided if all methods are implemented:
    - Usage as context manager via connect and close
    - Availability check
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

    @abc.abstractmethod
    def is_connected(self) -> bool:
        pass

    def is_available(self) -> bool:
        """
        Check whether the client can currently be opened.
        :return: Boolean to indicate whether it can be opened.
        """
        # Check whether it is open currently
        if self.is_connected:
            return True
        # Otherwise check whether it can be opened
        try:
            self.connect()
        except ClientError:
            return False
        else:
            return True
        finally:
            # Closing should always be done
            self.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
