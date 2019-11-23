from math import sqrt, acos
from typing import *


class Coordinate:
    DIGITS = 2

    def __init__(self, values, axes, digits=DIGITS):
        """
        :param values: List of coordinate values
        :param axes: List of axis descriptors
        """
        self.digits = digits
        # Create dictionary of coordinates
        try:
            self.coordinate = {i[0]: i[1] for i in zip(axes, values)}
        except TypeError:
            self.coordinate = {}

    @classmethod
    def from_melfa_response(cls, melfa_str: str, number_axes: int):
        segments = melfa_str.split(';')
        values = [float(i) for i in segments[1:2 * number_axes:2]]
        axes = segments[0:2 * number_axes:2]
        return cls(values, axes)

    def to_melfa_response(self):
        txt = ['{};{:.{d}f}'.format(key, v, d=self.digits) for (key, v) in self.coordinate.items()]
        return ';'.join(txt)

    def to_melfa_point(self):
        txt = ['{:.{d}f}'.format(i, d=self.digits) for i in self.coordinate.values()]
        return '(' + ','.join(txt) + ')' + '(7,0)'

    def to_melfa_crcl(self):
        txt = ['{:.{d}f}'.format(i, d=self.digits) for i in self.coordinate.values()]
        return '(' + ','.join(txt) + ')' + '(7,00100000)'

    def __str__(self):
        """
        Converts coordinates into space-separated string if coordinate value exists.
        :return:
        """
        txt = ['{}{:.{d}f}'.format(key, v, d=self.digits) for (key, v) in self.coordinate.items() if v is not None]
        return ' '.join(txt)

    def __add__(self, other: 'Coordinate') -> 'Coordinate':
        """
        Adds the coordinates for the individual axes.
        :param other: Set of coordinates to be added
        :return: New set of coordinates as sum of both inputs
        """
        axis_list = self.coordinate.keys()
        if axis_list == other.coordinate.keys():
            values = (self.coordinate[axis] + other.coordinate[axis] for axis in axis_list)
            digits = min(self.digits, other.digits)
            return Coordinate(values, axis_list, digits)
        else:
            raise TypeError('Incompatible axis.')

    def __radd__(self, other: Union['Coordinate', int]) -> 'Coordinate':
        """
        Allows adding a zero and right hand adding to a coordinate.
        :param other:
        :return: New set of coordinates as sum of both inputs
        """
        if other == 0:
            return self
        else:
            return self.__add__(other)

    def __sub__(self, other: 'Coordinate') -> 'Coordinate':
        axis_list = self.coordinate.keys()
        if axis_list == other.coordinate.keys():
            values = (self.coordinate[axis] - other.coordinate[axis] for axis in axis_list)
            digits = min(self.digits, other.digits)
            return Coordinate(values, axis_list, digits)
        else:
            raise TypeError('Incompatible axis.')

    def __rsub__(self, other: 'Coordinate') -> 'Coordinate':
        """
        Allows right hand subtraction
        :param other: Second set of coordinate
        :return: Difference of coordinates
        """
        return self.__sub__(other)

    def __mul__(self, other: Union[float, int]) -> 'Coordinate':
        """
        Multiplication of a coordinate and a constant
        :param other: Constant, float or int
        :return: New set of coordinates
        """
        values = (self.coordinate[axis] * other for axis in self.coordinate.keys())
        return Coordinate(values, self.coordinate.keys(), self.digits)

    def __rmul__(self, other: Union[float, int]) -> 'Coordinate':
        """
        Allows right hand multiplication.
        :param other: Constant, float or int
        :return: New set of coordinates
        """
        return self.__mul__(other)

    def __truediv__(self, other: float) -> 'Coordinate':
        """
        Float division for coordinates and a constant.
        :param other: Constant, float or int
        :return: New set of coordinates divided by constant
        """
        values = (self.coordinate[axis] / other for axis in self.coordinate.keys())
        return Coordinate(values, self.coordinate.keys(), self.digits)

    def __floordiv__(self, other: float) -> 'Coordinate':
        values = (self.coordinate[axis] // other for axis in self.coordinate.keys())
        return Coordinate(values, self.coordinate.keys(), self.digits)

    def scalar_multiply(self, other: 'Coordinate') -> float:
        """
        Scalar product of two coordinates that are handled as vectors
        :param other:
        :return:
        """
        axis_list = self.coordinate.keys()
        if axis_list == other.coordinate.keys():
            values = (self.coordinate[axis] * other.coordinate[axis] for axis in axis_list)
            return sum(values)
        else:
            raise TypeError('Incompatible axis.')

    def vector_len(self):
        root_sum = 0
        for axis in self.coordinate.keys():
            root_sum += self.coordinate[axis] ** 2
        return sqrt(root_sum)

    def vector_angle_rad(self, other: 'Coordinate') -> float:
        phi = acos(self.scalar_multiply(other) / (self.vector_len() * other.vector_len()))
        return phi
