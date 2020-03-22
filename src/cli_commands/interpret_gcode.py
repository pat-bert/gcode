from src.ApplicationExceptions import GCmdError, MelfaBaseException
from src.gcode.GCmd import GCmd
from src.printer_components.MelfaRobot import MelfaRobot
from clients.IClient import IClient


def interpret_gcode(f_input: str, f_output: str = "out.txt") -> None:
    """
    Interpret a G-code file and translate it to R3 protocol commands.
    :param f_input: Input file path
    :param f_output: Output file path
    :return: None
    :raises: OSError
    """
    # Read input file and translate commands
    print("Parsing G-Code...")
    try:
        with open(f_input, "r") as f:
            gcode_list = [GCmd.read_cmd_str(line) for line in f.readlines()]
    except OSError:
        print("Error reading file.")
        raise
    except GCmdError:
        print("Error reading G-code.")
        raise
    else:
        print("Done.")

    # Start translation to MELFA commands
    tcp = IClient()
    robot = MelfaRobot(io_client=IClient)
    print("Translating commands to R3 protocol commands...")
    try:
        r3_code_list = [
            robot.handle_gcode(gcode, interactive=False) for gcode in gcode_list
        ]
    except MelfaBaseException:
        print("Error translating G-code.")
        raise
    else:
        print("Done.")

    # Write R3 commands to output file
    print("Writing R3 commands...")
    try:
        with open(f_output, "w") as f:
            f.writelines([str(r3_code) for r3_code in r3_code_list])
    except OSError:
        print("Error writing file.")
        raise
    else:
        print("Done.")
