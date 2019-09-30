class Coordinate:
    def __init__(self, values, axes, digits=3):
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

    def __str__(self):
        """
        Converts coordinates into space-separated string if coordinate value exists.
        :return:
        """
        txt = ['{}{:.{d}f}'.format(key, v, d=self.digits) for (key, v) in self.coordinate.items() if v is not None]
        return ' '.join(txt)

    def __add__(self, other):
        axis_list = self.coordinate.keys()
        if axis_list == other.coordinate.keys():
            values = (self.coordinate[axis] + other.coordinate[axis] for axis in axis_list)
            digits = min(self.digits, other.digits)
            return Coordinate(values, axis_list, digits)
        else:
            raise TypeError('Incompatible axis.')

    def __radd__(self, other):
        if other == 0:
            return self
        else:
            return self.__add__(other)
