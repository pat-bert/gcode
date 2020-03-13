import abc

from src.Coordinate import Coordinate


class CoordinateAdapter(metaclass=abc.ABCMeta):
    @abc.abstractmethod
    def to_cmd(self, coordinate: Coordinate) -> str:
        pass

    @abc.abstractmethod
    def from_response(self, response: str) -> Coordinate:
        pass
