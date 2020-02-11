import pytest

from src.Coordinate import Coordinate


class TestCoordinate:
    """
    Bundle tests for the Coordinate class.
    """

    @pytest.mark.parametrize(
        "values,axes,digits,print_axes,ex, ex_info",
        [
            # Normal
            (3, "X", 2, None, None, None),
            # Too many print axes
            (3, "X", 2, "XY", ValueError, "Axes and print representation need to be of same length.",),
            # Too few print axes
            ([3, 4], "XY", 2, "Y", ValueError, "Axes and print representation need to be of same length.",),
            # Too few digits
            (3, "X", -1, None, ValueError, "Digits must be positive."),
            # Not enough values
            ([3], "XY", 2, "ZY", None, None),
        ],
    )
    def test_init_exception(self, values, axes, digits, print_axes, ex, ex_info):
        if ex is not None:
            with pytest.raises(ex) as excinfo:
                Coordinate(values, axes, digits, print_axes)
            assert str(excinfo.value.args[0]) == ex_info
        else:
            Coordinate(values, axes, digits, print_axes)
            assert True

    @pytest.mark.parametrize(
        "values,axes,digits,print_axes,expected",
        [
            ([3.141, 0.02, -4.3], "XYZ", 2, None, "X3.14 Y0.02 Z-4.30"),
            ([3.141, 0.02, -4.3], "XYZ", None, None, "X3.14 Y0.02 Z-4.30"),
            ([3.141, 0.02, None], "CBA", 2, "YZA", "Y3.14 Z0.02"),
            ([3.141, 0.02, -4.3], "XYZ", 0, None, "X3 Y0 Z-4"),
        ],
    )
    def test_str(self, values, axes, digits, print_axes, expected):
        if digits is not None:
            a = Coordinate(values, axes, digits=digits, print_axes=print_axes)
        else:
            a = Coordinate(values, axes, print_axes=print_axes)

        assert str(a) == expected

    @pytest.mark.parametrize(
        "axes1,axes2,result",
        [("XYZ", "XYZ", True), ("ABC", "CAB", True), ("AC", "AY", False)],
    )
    def test_are_axes_compatible(self, axes1, axes2, result):
        a = Coordinate((None, None, None), axes1)
        b = Coordinate((None, None, None), axes2)

        actual_result = a._are_axes_compatible(b)
        assert actual_result == result

    @pytest.mark.parametrize(
        "values,factor,expected",
        [
            ([1, -3, 0], 3, [3, -9, 0]),
            ([1, None, 0], 0, [0, None, 0]),
            ([1, None, 0], -2, [-2, None, 0]),
        ],
    )
    def test_mul(self, values, factor, expected):
        a = Coordinate(values, "XYZ")
        right_result = a * factor
        left_result = factor * a

        assert list(right_result.values) == list(left_result.values) == expected

    @pytest.mark.parametrize(
        "values,factor,expected",
        [([1, -4, 0], 3, [0, -2, 0]), ([1, None, 0], -2, [-1, None, 0])],
    )
    def test_floordiv(self, values, factor, expected):
        a = Coordinate(values, "XYZ")
        right_result = a // factor

        assert list(right_result.values) == expected

    @pytest.mark.parametrize(
        "a,b,expected",
        [
            ([1, -3, 0], [-4, None, 2], [-3, None, 2]),
            ([1, None, 0], [-4, None, 2], [-3, None, 2]),
        ],
    )
    def test_add(self, a, b, expected):
        """
        Individual coordinates of correct axes should be added. Result of None + x is defined as None.
        :param a:
        :param b:
        :param expected:
        :return:
        """
        axes = "XYZ"
        add1 = Coordinate(a, axes)
        add2 = Coordinate(b, axes)
        left_sum_coordinate = add1 + add2
        right_sum_coordinate = add2 + add1

        assert list(left_sum_coordinate.values) == list(right_sum_coordinate.values) == expected

    def test_add_zero(self):
        a = Coordinate([1, None, 0], "XYZ")
        assert list((a + 0).values) == [1, None, 0]

    def test_add_incompatible(self):
        a = Coordinate((0, 0, None), "XYZ")
        b = Coordinate((1, 2, 3), "XYA")
        with pytest.raises(TypeError) as excinfo:
            a + b
        assert str(excinfo.value.args[0]) == "Incompatible axis."

    def test_sub_incompatible(self):
        a = Coordinate((0, 0, None), "XYZ")
        b = Coordinate((1, 2, 3), "XYA")
        with pytest.raises(TypeError) as excinfo:
            a - b
        assert str(excinfo.value.args[0]) == "Incompatible axis."

    def test_update_empty_successful(self):
        """
        Test successful update.
        :return:
        """
        axes = "XYZ"
        a = Coordinate((0, 0, None), axes)
        b = Coordinate((1, 2, 3), axes)
        a.update_empty(b)
        assert list(a.values) == [0, 0, 3] and list(a.axes) == ["X", "Y", "Z"]

    def test_update_empty_exception(self):
        """
        Test exception raising for not-present axis.
        :return:
        """
        a = Coordinate((0, 0, None), "XYZ")
        b = Coordinate((1, 2, 3), "XYA")
        with pytest.raises(TypeError) as excinfo:
            a.update_empty(b)
        assert str(excinfo.value.args[0]) == "Incompatible axis."

    @pytest.mark.parametrize(
        "test_input,remaining,expected", [("XYZ", "Y", "Y"), ("XYZ", "AZ", "Z")]
    )
    def test_reduce_to_axes(self, test_input, remaining, expected):
        """
        Test that only remaining axes are kept
        :param test_input: Input axes
        :param remaining: Function input
        :param expected: Expected remaining axes
        :return:
        """
        a = Coordinate((1, -3, None), test_input)
        a.reduce_to_axes(remaining)
        assert "".join(list(a.axes)) == expected

    @pytest.mark.parametrize(
        "values,axes,reduced,expected_values",
        [
            ([1, -3, None], "XYZ", "Y", [None, -3, None]),
            ([1, -3, 2], "XYZ", "A", [None, None, None]),
            ([1, -3, 2], "XYZ", "XYZ", [1, -3, 2]),
            ([1, -3, 0], "XYZ", "XYZB", [1, -3, 0]),
        ],
    )
    def test_reduce_to_axes_make_none(self, values, axes, reduced, expected_values):
        a = Coordinate(values, axes)
        a.reduce_to_axes(reduced, make_none=True)

        assert list(a.axes) == list(axes) and list(a.values) == expected_values

    @pytest.mark.parametrize(
        "values_first,values_second,expected",
        [([10, 5], [-3, 4], -10), ([10, 0], [-3, 4], -30)],
    )
    def test_dot(self, values_first, values_second, expected):
        """
        Test that the scalar product is calculated correctly
        :param values_first:
        :param values_second:
        :param expected:
        :return:
        """
        a = Coordinate(values_first, "XY")
        b = Coordinate(values_second, "XY")
        assert a.dot(b) == expected

    def test_dot_incompatible_axes(self):
        """
        Test that for incompatible axes the correct exception is raised
        :return:
        """
        a = Coordinate((0, 0, None), "XZ")
        b = Coordinate((1, 2, 3), "XYZ")

        with pytest.raises(TypeError) as excinfo:
            a.dot(b)

        assert str(excinfo.value.args[0]) == "Incompatible axis."

    def test_dot_none_values(self):
        """
        Test that an exception is raised if one value is None
        :return:
        """
        a = Coordinate((0, 0, None), "XYZ")
        b = Coordinate((1, 2, 3), "XYZ")

        with pytest.raises(TypeError):
            a.dot(b)

    @pytest.mark.parametrize(
        "values_first,values_second,values_result", [([1, 0, 0], [0, 1, 0], (0, 0, 1))]
    )
    def test_cross(self, values_first, values_second, values_result):
        a = Coordinate(values_first, "XYZ")
        b = Coordinate(values_second, "XYZ")
        result = a.cross(b)
        assert list(result.values) == list(values_result)

    def test_cross_incompatible_axes(self):
        a = Coordinate((0, 0, None), "XYZ")
        b = Coordinate((1, 2, 3), "XZ")

        with pytest.raises(TypeError) as excinfo:
            a.cross(b)

        assert str(excinfo.value.args[0]) == "Incompatible axis."

    def test_cross_none(self):
        a = Coordinate((0, 0, None), "XYZ")
        b = Coordinate((1, 2, 3), "XYZ")

        with pytest.raises(TypeError):
            a.cross(b)

    @pytest.mark.parametrize(
        "axes,values,expected",
        [("XYZ", [2, 5, 8], 9.64), ("XY", [3, 4], 5), ("XY", [0, -3], 3)],
    )
    def test_vector_len(self, axes, values, expected):
        a = Coordinate(values, axes)
        assert a.vector_len() == pytest.approx(expected, abs=0.01)

    def test_vector_len_none(self):
        a = Coordinate([3, 2, None], "XYZ")
        with pytest.raises(TypeError):
            a.vector_len()
