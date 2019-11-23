import abc


class PrinterComponent(object, metaclass=abc.ABCMeta):
    """
    Defines the interface that all printer component need to provide.
    """

    @abc.abstractmethod
    def boot(self):
        raise NotImplementedError

    @abc.abstractmethod
    def shutdown(self):
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def redirector(self):
        raise NotImplementedError
