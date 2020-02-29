from Coordinate import Coordinate


class Arc:
    def __init__(self, start: Coordinate, finish: Coordinate, third_point: Coordinate, is_centre: bool, is_cw: bool):
        self.start = start
        self.finish = finish
        self.is_clockwise = is_cw

        if is_centre:
            self.centre = third_point
            self.intermediate = None
        else:
            self.centre = None
            self.intermediate = third_point

    @property
    def centri_angle(self) -> float:
        return 0

    def get_centre(self) -> Coordinate:
        if self.centre is not None:
            return self.centre
        else:
            pass

    def get_intermediate(self) -> Coordinate:
        if self.intermediate is not None:
            return self.intermediate
        else:
            pass

    @property
    def first_segment(self) -> 'Arc':
        """ Get the first segment of the arc going from start to intermediate.
        If the latter is missing it is calculated.
        :return:
        """
        if self.intermediate is None:
            self.intermediate = self.get_intermediate()
        if self.centre is None:
            self.centre = self.get_centre()
        return self.__class__(self.start, self.intermediate, self.centre, 1, self.is_clockwise)

    @property
    def second_segment(self) -> 'Arc':
        """ Get the second segment of the arc going from intermediate to finish.
        If the latter is missing it is calculated.
        :return:
        """
        if self.intermediate is None:
            self.intermediate = self.get_intermediate()
        if self.centre is None:
            self.centre = self.get_centre()
        return self.__class__(self.intermediate, self.finish, self.centre, 1, self.is_clockwise)
