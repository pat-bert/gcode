from math import sqrt
from typing import Union, List


class Coordinate:
    DIGITS = 2

    def __init__(self, values, axes, digits=DIGITS, print_axes=None):
        """

        :param values: List of coordinate values
        :param axes: List of axis descriptors
        :param digits: Represented number of digits
        :param print_axes: Allow renaming for string representation
        """
        if digits < 0:
            raise ValueError("Digits must be positive.")
        self.digits = digits

        # Create dictionary of coordinates
        try:
            self.coordinate = {i[0]: i[1] for i in zip(axes, values)}
        except TypeError:
            self.coordinate = {}

        # Print representation
        if print_axes is not None and len(print_axes) != len(axes):
            raise ValueError("Axes and print representation need to be of same length.")
        self.print_axes = print_axes

    def to_melfa_response(self) -> str:
        txt = [
            "{};{:.{d}f}".format(key, v, d=self.digits)
            for (key, v) in self.coordinate.items()
        ]
        return ";".join(txt)

    def to_melfa_point(self) -> str:
        angles = {"A": -180, "B": 0, "C": 0}
        for angle, val in angles.items():
            if angle not in self.coordinate.keys():
                self.coordinate[angle] = val

        txt = [
            "{:.{d}f}".format(i, d=self.digits) if i is not None else ""
            for i in self.coordinate.values()
        ]
        return "(" + ",".join(txt) + ")" + "(7,0)"

    def update_empty(self, other: "Coordinate"):
        for key, val in self.coordinate.items():
            if val is None:
                try:
                    self.coordinate[key] = other.coordinate[key]
                except KeyError:
                    raise TypeError("Incompatible axis.")

    def add_axis(self, other: "Coordinate"):
        for axis in other.axes:
            if axis not in self.coordinate.keys():
                self.coordinate[axis] = None

    def reduce_to_axes(self, axes_to_keep, make_none=False) -> "Coordinate":
        if make_none:
            coordinate = {
                key: None if key not in axes_to_keep else val
                for key, val in self.coordinate.items()
            }
            return self.__class__(
                coordinate.values(), coordinate.keys(), digits=self.digits
            )
        else:
            coordinate = {
                key: val for key, val in self.coordinate.items() if key in axes_to_keep
            }
            return self.__class__(
                coordinate.values(), coordinate.keys(), digits=self.digits
            )

    def __str__(self):
        """
        Converts coordinates into space-separated string if coordinate value exists.
        :return:
        """
        if self.print_axes is not None:
            txt = [
                "{}{:.{d}f}".format(key, v, d=self.digits)
                for (key, v) in zip(self.print_axes, self.values)
                if v is not None
            ]
        else:
            txt = [
                "{}{:.{d}f}".format(key, v, d=self.digits)
                for (key, v) in self.coordinate.items()
                if v is not None
            ]
        return " ".join(txt)

    def __add__(self, other: Union["Coordinate", int]) -> "Coordinate":
        """
        Adds the coordinates for the individual AXES.
        :param other: Set of coordinates to be added
        :return: New set of coordinates as sum of both inputs
        """
        if other == 0:
            return self
        if self._are_axes_compatible(other):
            axis_list = self.axes
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
            raise TypeError("Incompatible axis.")

    def __radd__(self, other: Union["Coordinate", int]) -> "Coordinate":
        """
        Allows adding a zero and right hand adding to a coordinate.
        :param other:
        :return: New set of coordinates as sum of both inputs
        """
        if other == 0:
            return self
        return self.__add__(other)

    def __sub__(self, other: "Coordinate") -> "Coordinate":
        if self._are_axes_compatible(other):
            values = [
                self.coordinate[axis] - other.coordinate[axis] for axis in self.axes
            ]
            digits = min(self.digits, other.digits)
            return Coordinate(values, self.axes, digits)
        raise TypeError("Incompatible axis.")

    def __rsub__(self, other: "Coordinate") -> "Coordinate":
        """
        Allows right hand subtraction
        :param other: Second set of coordinate
        :return: Difference of coordinates
        """
        return self.__sub__(other)

    def __mul__(self, other: Union[float, int]) -> "Coordinate":
        """
        Multiplication of a coordinate and a constant
        :param other: Constant, float or int
        :return: New set of coordinates
        """
        values = [
            val * other if val is not None else None
            for axis, val in self.coordinate.items()
        ]
        return Coordinate(values, self.axes, self.digits)

    def __rmul__(self, other: Union[float, int]) -> "Coordinate":
        return self.__mul__(other)

    def __truediv__(self, other: float) -> "Coordinate":
        """
        Float division for coordinates and a constant.
        :param other: Constant, float or int
        :return: New set of coordinates divided by constant
        """
        values = (self.coordinate[axis] / other for axis in self.coordinate.keys())
        return Coordinate(values, self.coordinate.keys(), self.digits)

    def __floordiv__(self, other: float) -> "Coordinate":
        values = (
            self.coordinate[axis] // other
            if self.coordinate[axis] is not None
            else None
            for axis in self.axes
        )
        return Coordinate(values, self.axes, self.digits)

    def dot(self, other: "Coordinate") -> float:
        """
        Scalar product of two coordinates that are handled as vectors
        :param other:
        :return:
        """
        if self._are_axes_compatible(other):
            return sum(
                (self.coordinate[axis] * other.coordinate[axis] for axis in self.axes)
            )
        raise TypeError("Incompatible axis.")

    def cross(self, other: "Coordinate") -> "Coordinate":
        if self._are_axes_compatible(other):
            axis_list = self.axes
            digits = min(self.digits, other.digits)

            indices_a = [i for i in range(1, len(axis_list))] + [0]
            indices_b = [len(axis_list) - 1] + [i for i in range(0, len(axis_list) - 1)]

            def cross_row_formula(a, b):
                add1 = self.coordinate[axis_list[a]] * other.coordinate[axis_list[b]]
                add2 = self.coordinate[axis_list[b]] * other.coordinate[axis_list[a]]
                return add1 - add2

            values = [
                cross_row_formula(idx_a, idx_b)
                for idx_a, idx_b in zip(indices_a, indices_b)
            ]

            return Coordinate(values, axis_list, digits)
        raise TypeError("Incompatible axis.")

    def vector_len(self):
        return sqrt(sum((i ** 2 for i in self.values)))

    @property
    def axes(self) -> List:
        return list(self.coordinate.keys())

    @property
    def values(self) -> List:
        return list(self.coordinate.values())

    def _are_axes_compatible(self, other: "Coordinate") -> bool:
        return sorted(self.axes) == sorted(other.axes)
