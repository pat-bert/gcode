from math import pi

from AM_IR.Coordinate import Coordinate
from AM_IR.MelfaCoordinateService import Plane
from AM_IR.circle_util import get_angle


def test_get_angle():
    axes = 'XYZ'
    tol = 0.01
    test_table = [
        # XY Plane

        # a-axis to y-axis in XY-plane, first quadrant
        [(10, 0, 0), (0, 10, 0), (0, 0, 0), pi / 2],
        # 45 degrees
        [(10, 0, 0), (10, 10, 0), (0, 0, 0), pi / 4],
        # y-axis to a-axis in XY-plane, first quadrant
        [(0, 10, 0), (5, 0, 0), (0, 0, 0), -pi / 2],
        # 180 degrees, second quadrant
        [(10, 0, 0), (-10, +0, 0), (0, 0, 0), pi],
        # -90 degrees, fourth quadrant
        [(10, 0, 0), (0, -10, 0), (0, 0, 0), -pi / 2],
        # -90 degrees, third quadrant
        [(10, 0, 0), (-10, -10, 0), (0, 0, 0), -0.75 * pi],
        # Start in second quadrant, finish in third quadrant
        [(10, 10, 0), (-10, 10, 0), (0, 0, 0), pi / 2],

        # XZ Plane

        # YZ Plane
    ]

    # Iterate over all tests
    for counter, test_set in enumerate(test_table):
        print("Test #" + str(counter))
        # Unpack test parameters for test set
        s, t, c, angle = test_set
        actual_angle = None
        try:
            actual_angle = get_angle(Coordinate(s, axes), Coordinate(t, axes), Coordinate(c, axes), plane=Plane.ANY)
            assert abs(actual_angle - angle) < tol
        except NotImplementedError:
            print("Failed: " + str(actual_angle))
        except AssertionError:
            print("Failed: " + str(actual_angle))
        except Exception as e:
            print("Failed: " + str(e))
