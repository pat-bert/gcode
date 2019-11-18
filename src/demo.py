import ApplicationExceptions
from Coordinate import Coordinate
from MelfaRobot import MelfaRobot
from TcpClientR3 import TcpClientR3


def cube(robot: MelfaRobot):
    """
    Demo Example 1 - Cube
    :param robot: Instance of an active robot
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
            robot.linear_move_poll(point, 0)
        # Back to first point
        robot.linear_move_poll(square_corners[0], 0)
        # Increment z
        square_corners = [point + z_vector for point in square_corners]


def cylinder(robot: MelfaRobot):
    """
    Demo Example 2 - Cylinder
    :param robot: Instance of an active robot
    :return:
    """
    # Base coordinates
    z_vector = Coordinate([0, 0, 5, 0, 0, 0], robot.axes)
    center = Coordinate([550, 0, 200, 180, 0, 0], robot.axes)
    start = Coordinate([500, 50, 200, 180, 0, 0], robot.axes)

    # Go to start
    robot.linear_move_poll(start, 0)

    is_clockwise = True
    # Go to points
    for _ in range(10):
        # Circle
        robot.circular_move_poll(start, center, is_clockwise, 0)
        # Increment z
        center += z_vector
        start += z_vector
        # Toggle direction
        is_clockwise = not is_clockwise


def demo_mode(ip=None, port=None):
    # Create TCP client
    if ip is not None and port is not None:
        tcp_client = TcpClientR3(host=ip, port=port)
    else:
        tcp_client = TcpClientR3()
    tcp_client.connect()

    # Executing communication
    try:
        tcp_client.start(speed_threshold=10)
        robot = MelfaRobot(tcp_client, number_axes=6)
        while True:
            selection = input("Please choose a mode (1=cube, 2=cylinder): ")
            if selection == '1':
                cube(robot)
            elif selection == '2':
                # cylinder(robot)
                pass
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
        tcp_client.close()
