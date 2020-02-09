import time

from src import ApplicationExceptions
from src.Coordinate import Coordinate
from src.gcode.GCmd import GCmd
from src.melfa.TcpClientR3 import TcpClientR3
from src.printer_components.MelfaRobot import MelfaRobot
from src.speed_profile import draw_speed


def cube(robot: MelfaRobot, speed: float) -> None:
    """
    Demo Example 1 - Cube
    :param robot: Instance of an active robot
    :param speed:
    :return:
    """
    # Base coordinates
    start = Coordinate([0, 0, 0, 180, 0, 0], robot.AXES)  # pragma: no mutate

    x_vec = Coordinate([100, 0, 0, 0, 0, 0], robot.AXES)  # pragma: no mutate
    y_vec = Coordinate([0, -100, 0, 0, 0, 0], robot.AXES)  # pragma: no mutate
    z_vector = Coordinate([0, 0, 5, 0, 0, 0], robot.AXES)  # pragma: no mutate

    square_corners = [
        start,
        start + y_vec,
        start + x_vec + y_vec,
        start + x_vec,
    ]  # pragma: no mutate

    # Go to points
    for _ in range(10):
        # Square
        for point in square_corners:
            robot.linear_move_poll(point, speed)
        # Back to first point
        robot.linear_move_poll(square_corners[0], speed)
        # Increment z
        square_corners = [point + z_vector for point in square_corners]


def cylinder(robot: MelfaRobot, speed: float) -> None:
    """
    Demo Example 2 - Cylinder
    :param robot: Instance of an active robot
    :param speed:
    :return:
    """
    # Base coordinates
    start = Coordinate([0, 0, 0, 180, 0, 0], robot.AXES)  # pragma: no mutate
    z_vector = Coordinate([0, 0, 15, 0, 0, 0], robot.AXES)  # pragma: no mutate
    target_vec = Coordinate([50, 50, 0, 0, 0, 0], robot.AXES)  # pragma: no mutate
    target = start + target_vec  # pragma: no mutate
    center_vec = Coordinate([50, 0, 0, 0, 0, 0], robot.AXES)  # pragma: no mutate
    center = start + center_vec  # pragma: no mutate
    clockwise = False

    for _ in range(10):
        # Move circle segment
        robot.circular_move_poll(target, center, clockwise, speed, start_pos=start)

        # Increase height and swap start and target
        start, target = target + z_vector, start + z_vector
        center += z_vector
        clockwise = not clockwise


def speed_test(robot: MelfaRobot, speed: float) -> None:
    start = Coordinate([-150, -200, 400, 180, 0, 0], robot.AXES)  # pragma: no mutate
    vector = Coordinate([200, 400, -300, 0, 0, 0], robot.AXES)  # pragma: no mutate
    finish = start + vector  # pragma: no mutate

    # Back to start
    robot.reset_linear_speed_factor()
    robot.linear_move_poll(start)

    # Test distance
    start_time = time.clock()
    t, v = robot.linear_move_poll(finish, speed, track_speed=True)
    finish_time = time.clock()

    # Average velocity
    velocity = vector.vector_len() / (finish_time - start_time)

    # Draw speed
    draw_speed(speed, t, v)

    print("Velocity is: " + str(velocity))


def gcode_santa(robot: MelfaRobot):
    gcode = """G1 X30 F100
G1 Y30
G1 X0
G1 Y0
G1 X30 Y30
G1 X15 Y45
G1 X0 Y30
G1 X30 Y0
G1 X0 Y0
G02 X0 Y30 J15
G02 X30 Y30 I15 J15
G91 
G02 Y-30 J-15
G90"""

    for command in gcode.split("\n"):
        cmd = GCmd.read_cmd_str(command)
        robot.handle_gcode(cmd)


def demo_mode(ip=None, port=None, safe_return=False) -> None:
    # Create TCP client
    if ip is not None and port is not None:
        tcp_client = TcpClientR3(host=ip, port=port)
    else:
        tcp_client = TcpClientR3()
    tcp_client.connect()

    # Executing communication
    robot = MelfaRobot(
        tcp_client, number_axes=6, speed_threshold=10, safe_return=safe_return
    )
    robot.boot()

    try:
        while True:
            selection = input(
                "Please choose a mode (1=cube, 2=cylinder, 3=speed test, 4=G-Code Santa): "
            )
            try:
                if selection in ["1", "2", "3"]:
                    speed = float(input("Please enter the speed (linear: mm/s): "))
                    if selection == "1":
                        cube(robot, speed)
                    elif selection == "2":
                        cylinder(robot, speed)
                    elif selection == "3":
                        speed_test(robot, speed)
                elif selection == "4":
                    gcode_santa(robot)
                else:
                    break
            except ValueError:
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
