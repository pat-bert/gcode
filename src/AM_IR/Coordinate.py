from math import sqrt, acos
from typing import *


class Coordinate:
    DIGITS = 2

    def __init__(self, values, axes, digits=DIGITS, print_axes=None):
        """

        :param values: List of coordinate values
        :param axes: List of axis descriptors
        :param digits: Represented number of digits
        :param print_axes: Allow renaming for string representation
        """
        self.digits = digits
        # Create dictionary of coordinates
        try:
            self.coordinate = {i[0]: i[1] for i in zip(axes, values)}
        except TypeError:
            self.coordinate = {}
        if print_axes is not None and len(print_axes) != len(axes):
            raise TypeError('Axes and print representation need to be of same length.')
        else:
            self.print_axes = print_axes

    def to_melfa_response(self):
        txt = ['{};{:.{d}f}'.format(key, v, d=self.digits) for (key, v) in self.coordinate.items()]
        return ';'.join(txt)

    def to_melfa_point(self):
        angles = {'A': -180, 'B': 0, 'C': 0}
        for angle, val in angles.items():
            if angle not in self.coordinate.keys():
                self.coordinate[angle] = val

        txt = ['{:.{d}f}'.format(i, d=self.digits) if i is not None else '' for i in self.coordinate.values()]
        return '(' + ','.join(txt) + ')' + '(7,0)'

    def update_empty(self, other: 'Coordinate'):
        for key, val in self.coordinate.items():
            if val is None:
                try:
                    self.coordinate[key] = other.coordinate[key]
                except KeyError:
                    raise TypeError('Incompatible axis.')

    def reduce_to_axes(self, axes_to_keep):
        self.coordinate = {key: val for key, val in self.coordinate.items() if key in axes_to_keep}

    def __str__(self):
        """
        Converts coordinates into space-separated string if coordinate value exists.
        :return:
        """
        if self.print_axes is not None:
            txt = ['{}{:.{d}f}'.format(key, v, d=self.digits) for (key, v) in
                   zip(self.print_axes, self.coordinate.values())
                   if v is not None]
        else:
            txt = ['{}{:.{d}f}'.format(key, v, d=self.digits) for (key, v) in self.coordinate.items() if v is not None]
        return ' '.join(txt)

    def __add__(self, other: 'Coordinate') -> 'Coordinate':
        """
        Adds the coordinates for the individual AXES.
        :param other: Set of coordinates to be added
        :return: New set of coordinates as sum of both inputs
        """
        axis_list = self.coordinate.keys()
        if axis_list == other.coordinate.keys():
            values = []
            for axis in axis_list:
                try:
                    val = self.coordinate[axis] + other.coordinate[axis]
                except TypeError:
                    val = None
                    values.append(val)
                else:
                    values.append(val)
            values = tuple(values)
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

    def dot(self, other: 'Coordinate') -> float:
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

    def cross(self, other: 'Coordinate') -> 'Coordinate':
        axis_list = self.coordinate.keys()
        if axis_list == other.coordinate.keys():
            digits = min(self.digits, other.digits)
            values = []

            axis_list = list(axis_list)

            indices_a = [i for i in range(1, len(axis_list))] + [0]
            indices_b = [len(axis_list) - 1] + [i for i in range(0, len(axis_list) - 1)]

            for idx_a, idx_b in zip(indices_a, indices_b):
                cross_1 = self.coordinate[axis_list[idx_a]] * other.coordinate[axis_list[idx_b]]
                cross_2 = self.coordinate[axis_list[idx_b]] * other.coordinate[axis_list[idx_a]]
                values.append(cross_1 - cross_2)
            return Coordinate(values, axis_list, digits)
        else:
            raise TypeError('Incompatible axis.')

    def vector_len(self):
        root_sum = 0
        for axis in self.coordinate.keys():
            root_sum += self.coordinate[axis] ** 2
        return sqrt(root_sum)

    def vector_angle_rad(self, other: 'Coordinate') -> float:
        phi = acos(self.dot(other) / (self.vector_len() * other.vector_len()))
        return phi


if __name__ == '__main__':
    axes = 'XYZ'
    a = Coordinate([1, 0, 0], axes)
    b = Coordinate([0, 1, 0], axes)
    print(a.cross(b))

    a = Coordinate([0, 1, 0], axes)
    b = Coordinate([0, 0, 1], axes)
    print(a.cross(b))

    a = Coordinate([0, 0, 1], axes)
    b = Coordinate([1, 0, 0], axes)
    print(a.cross(b))

    a = Coordinate([0, 0, 1], axes)
    b = Coordinate([1, 0, 0], axes)
    print(b.cross(a))
