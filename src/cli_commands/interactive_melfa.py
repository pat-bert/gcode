from time import sleep

from src import ApplicationExceptions
from src.clients.TcpClientR3 import TcpClientR3
from src.printer_components.MelfaRobot import MelfaRobot


def interactive_melfa(ip, port, log_file=None, safe_return=False) -> None:
    print("Launching interactive R3 protocol shell...")

    if log_file is not None:
        # TODO Implement logging
        pass

    # Create robot object
    tcp = TcpClientR3(host=ip, port=port)
    tcp.connect()
    robot = MelfaRobot(tcp, number_axes=6, speed_threshold=10, safe_return=safe_return)

    # Executing communication
    try:
        robot.boot()
        while True:
            usr_msg = input("Melfa>")
            if usr_msg.lower() in ["quit"]:
                raise KeyboardInterrupt
            elif len(usr_msg) > 0:
                robot.client.wait_send(usr_msg.upper())
                try:
                    robot.client.receive()
                except ApplicationExceptions.MelfaBaseException as ex:
                    # Print error message
                    if len(ex.status) > 0:
                        print(str(ex))
                    else:
                        # Resolve empty status codes
                        print("Empty status code. Trying to resolve.")
                        robot.client.wait_send("ERROR")
                        try:
                            robot.client.receive()
                        except ApplicationExceptions.MelfaBaseException as ex_res:
                            print(str(ex_res))
                    # Reset alarm
                    sleep(1)
                    print("Error Reset")
                    robot.protocol.reset_alarm()
    except KeyboardInterrupt:
        pass
    except ApplicationExceptions.MelfaBaseException as e:
        print(str(e))
    finally:
        # Cleaning up
        robot.shutdown()
