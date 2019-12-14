from math import atan2, pi

import numpy

from AM_IR.Coordinate import Coordinate


def get_angle(start: Coordinate, target: Coordinate, center: Coordinate, preferred_plane='XY') -> float:
    # Get vectors from center
    cs = start - center
    ct = target - center

    # Convert to numpy arrays
    veca = numpy.array([cs.coordinate['X'], cs.coordinate['Y'], cs.coordinate['Z']])
    vecb = numpy.array([ct.coordinate['X'], ct.coordinate['Y'], ct.coordinate['Z']])

    # Get z-axis perpendicular to plane of circle (can be any)
    z_axis = numpy.cross(veca, vecb)
    z_axis[2] = abs(z_axis[2])

    z_len = numpy.sqrt(numpy.sum(z_axis ** 2))

    # Normalise z-axis, fallback to preferred plane if angle is 180°
    if z_len != 0:
        z_axis = z_axis / z_len
    else:
        if 'X' in preferred_plane:
            if 'Y' in preferred_plane:
                z_axis = numpy.array([0, 0, 1])
            elif 'Z' in preferred_plane:
                z_axis = numpy.array([0, -1, 0])
            else:
                raise ValueError('Illegal plane.')
        elif 'Y' in preferred_plane and 'Z' in preferred_plane:
            z_axis = numpy.array([1, 0, 0])
        else:
            raise ValueError('Illegal plane.')

    # Put a-axis in direction of vector to start
    y_axis_r = numpy.array([0, 1, 0])
    x_axis = numpy.cross(y_axis_r, z_axis)
    # Get corresponding y-axis for right-hand system
    y_axis = numpy.cross(z_axis, x_axis)
    y_axis = y_axis / numpy.sqrt(numpy.sum(y_axis ** 2))

    # Projections onto a and y axis
    x_b = numpy.dot(x_axis, vecb)
    y_b = numpy.dot(y_axis, vecb)
    x_a = numpy.dot(x_axis, veca)
    y_a = numpy.dot(y_axis, veca)

    # Get the angle
    return atan2(y_b, x_b) - atan2(y_a, x_a)


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
            actual_angle = get_angle(Coordinate(s, axes), Coordinate(t, axes), Coordinate(c, axes))
            assert abs(actual_angle - angle) < tol
        except AssertionError:
            print("Failed: " + str(actual_angle))


if __name__ == '__main__':
    test_get_angle()
