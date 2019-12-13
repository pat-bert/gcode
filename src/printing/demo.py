import time

from printing import ApplicationExceptions
from printing.Coordinate import Coordinate
from printing.MelfaRobot import MelfaRobot
from printing.TcpClientR3 import TcpClientR3


def cube(robot: MelfaRobot, speed):
    """
    Demo Example 1 - Cube
    :param robot: Instance of an active robot
    :param speed:
    :return:
    """
    # Base coordinates
    z_vector = Coordinate([0, 0, 5, 0, 0, 0], robot.axes)
    square_corners = [
        Coordinate([500, 50, 200, 180, 0, 0], robot.axes),
        Coordinate([500, -50, 200, 180, 0, 0], robot.axes),
        Coordinate([600, -50, 200, 180, 0, 0], robot.axes),
        Coordinate([600, 50, 200, 180, 0, 0], robot.axes)
    ]

    # Go to points
    for _ in range(10):
        # Square
        for point in square_corners:
            robot.linear_move_poll(point, speed)
        # Back to first point
        robot.linear_move_poll(square_corners[0], speed)
        # Increment z
        square_corners = [point + z_vector for point in square_corners]


def cylinder(robot: MelfaRobot, speed):
    """
    Demo Example 2 - Cylinder
    :param robot: Instance of an active robot
    :param speed:
    :return:
    """
    # Base coordinates
    z_vector = Coordinate([0, 0, 15, 0, 0, 0], robot.axes)
    start = Coordinate([500, 0, 200, 180, 0, 0], robot.axes)
    target = Coordinate([550, 50, 200, 180, 0, 0], robot.axes)
    center = Coordinate([550, 0, 200, 180, 0, 0], robot.axes)

    for _ in range(10):
        # Move circle segment
        robot.circular_move_poll(target, center, True, speed, start_pos=start)

        # Increase height and swap start and target
        start, target = target + z_vector, start + z_vector
        center += z_vector


def speed_test(robot, speed):
    start = Coordinate([350, -200, 600, 180, 0, 0], robot.axes)
    vector = Coordinate([200, 400, -300, 0, 0, 0], robot.axes)
    finish = start + vector

    robot.linear_move_poll(start, speed)
    start_time = time.clock()
    robot.linear_move_poll(finish, speed)
    finish_time = time.clock()
    velocity = vector.vector_len() / (finish_time - start_time)
    print("Velocity is: " + str(velocity))


def demo_mode(ip=None, port=None, safe_return=False):
    # Create TCP client
    if ip is not None and port is not None:
        tcp_client = TcpClientR3(host=ip, port=port)
    else:
        tcp_client = TcpClientR3()
    tcp_client.connect()

    # Executing communication
    robot = MelfaRobot(tcp_client, number_axes=6, speed_threshold=10, safe_return=safe_return)
    robot.boot()
    try:
        while True:
            selection = input("Please choose a mode (1=cube, 2=cylinder, 3=speed test): ")
            try:
                speed = float(input("Please enter the speed (linear: mm/s): "))
            except ValueError:
                break
            if selection == '1':
                cube(robot, speed)
            elif selection == '2':
                cylinder(robot, speed)
            elif selection == '3':
                speed_test(robot, speed)
            else:
                break
    except KeyboardInterrupt:
        pass
    except NotImplementedError:
        pass
    except ApplicationExceptions.MelfaBaseException as e:
        print(str(e))
    finally:
        # Cleaning up
        robot.shutdown()
