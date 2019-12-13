from printing import ApplicationExceptions
from printing.GCmd import GCmd
from printing.GPrinter import GPrinter


def interactive_gcode(ip, port, log_file=None, safe_return=False) -> None:
    """
    Launches an interactive shell accepting G-code.
    :param ip: IPv4 network address of the robot
    :param port: Port of the robot
    :param log_file: Log file handle
    :param safe_return:
    :return:
    """
    print("Launching interactive G-code shell...")

    if log_file is not None:
        # TODO Implement logging
        pass

    # Create printer object
    printer = GPrinter.default_init(ip, port, safe_return)

    # Executing communication
    try:
        while True:
            usr_msg = input("G-Code>")
            if usr_msg.lower() in ['quit']:
                raise KeyboardInterrupt
            elif len(usr_msg) > 0:
                # Parse G-code
                gcode = GCmd.read_cmd_str(usr_msg)
                print(str(gcode))
                printer.execute(gcode)
    except KeyboardInterrupt:
        pass
    except ApplicationExceptions.MelfaBaseException as e:
        print(str(e))
    finally:
        printer.shutdown()
