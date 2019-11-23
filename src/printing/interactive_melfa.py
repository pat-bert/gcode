from time import sleep

from printing import ApplicationExceptions, MelfaCmd
from printing.MelfaRobot import MelfaRobot
from printing.TcpClientR3 import TcpClientR3


def interactive_melfa(ip, port, log_file=None) -> None:
    print("Launching interactive R3 protocol shell...")

    if log_file is not None:
        # TODO Implement logging
        pass

    # Create robot object
    robot = MelfaRobot(TcpClientR3(host=ip, port=port), number_axes=6)
    robot.tcp.connect()
    # Executing communication
    try:
        robot.tcp.start(speed_threshold=10)
        while True:
            usr_msg = input("Melfa>")
            if usr_msg.lower() in ['quit']:
                raise KeyboardInterrupt
            elif len(usr_msg) > 0:
                robot.tcp.wait_send(usr_msg.upper())
                try:
                    robot.tcp.receive()
                except ApplicationExceptions.MelfaBaseException as ex:
                    # Print error message
                    if len(ex.status) > 0:
                        print(str(ex))
                    else:
                        # Resolve empty status codes
                        print("Empty status code. Trying to resolve.")
                        robot.tcp.wait_send("ERROR")
                        try:
                            robot.tcp.receive()
                        except ApplicationExceptions.MelfaBaseException as ex_res:
                            print(str(ex_res))
                    # Reset alarm
                    sleep(1)
                    print("Error Reset")
                    robot.tcp.send(MelfaCmd.ALARM_RESET_CMD)
                    robot.tcp.receive(silence_errors=True)
    except KeyboardInterrupt:
        pass
    except ApplicationExceptions.MelfaBaseException as e:
        print(str(e))
    finally:
        # Cleaning up
        robot.tcp.close()
