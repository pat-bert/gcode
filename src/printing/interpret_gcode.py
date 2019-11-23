from printing.ApplicationExceptions import GCmdError, MelfaBaseException
from printing.GCmd import GCmd
from printing.gcode2melfa import gcode2melfa


def interpret_gcode(f_input: str, f_output: str = 'out.txt') -> None:
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
        with open(f_input, 'r') as f:
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
    print("Translating commands to R3 protocol commands...")
    try:
        r3_code_list = [gcode2melfa(gcode, 0) for gcode in gcode_list]
    except MelfaBaseException:
        print("Error translating G-code.")
        raise
    else:
        print("Done.")

    # Write R3 commands to output file
    print("Writing R3 commands...")
    try:
        with open(f_output, 'w') as f:
            f.writelines([str(r3_code) for r3_code in r3_code_list])
    except OSError:
        print("Error writing file.")
        raise
    else:
        print("Done.")
