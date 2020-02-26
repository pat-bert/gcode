import abc
from src.GRedirect import RedirectionTargets
from typing import Iterable


class PrinterComponent(metaclass=abc.ABCMeta):
    """
    Defines the interface that all printer component need to provide.
    """

    @abc.abstractmethod
    def boot(self, *args, **kwargs):
        raise NotImplementedError

    @abc.abstractmethod
    def shutdown(self, *args, **kwargs):
        raise NotImplementedError

    @property
    @abc.abstractmethod
    def redirector(self) -> Iterable[RedirectionTargets]:
        raise NotImplementedError

    @abc.abstractmethod
    def handle_gcode(self, *args, **kwargs):
        raise NotImplementedError
