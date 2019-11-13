import sys
from GCmd import GCmd
from MelfaCmd import translate_cmd
from ApplicationExceptions import *
from TCPClient import *


def interpreter_gcode(f_input: str, f_output: str = 'out.txt') -> None:
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
        r3_code_list = [translate_cmd(gcode) for gcode in gcode_list]
    except MelfaError:
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


def execute_r3(f_input: str, f_log: str = 'run.log') -> None:
    """
    Execute a R3 protocol command text file.
    :param f_input: Input file path
    :param f_log: Log file path
    :return: None
    """
    # Read commands from file
    print("Reading command file.")
    try:
        with open(f_input, 'r') as f:
            r3_commands = f.readlines()
    except OSError:
        print("Error reading command file.")
        raise
    else:
        print("Done.")

    # Pre-checks
    print("Running compliance checks...")
    try:
        # TODO Add start up checks and communication
        tcp = TCPClient()
        # tcp.start(speed_threshold=10)
    except TcpError:
        raise
    except PreCheckError:
        raise
    else:
        print("Done.")

    # Execute commands
    print("Executing commands...")
    try:
        with open(f_log, 'w') as fid_log:
            for r3_cmd in r3_commands:
                # TODO Send command
                pass
    except OSError as file_exception:
        print(file_exception)
        raise
    except ApiException:
        # TODO Replace by specific exception (communication, command, ...) and print something
        raise
    else:
        print("Done.")


if __name__ == '__main__':
    # Parsing command line parameters
    if len(sys.argv) == 1:
        # Development mode
        mode = '-i'
        sys.argv[1:] = [mode, 'test.gcode', 'melfa.txt']
    else:
        # Get mode from parameters
        mode = sys.argv[1].lower()

    try:
        # Call respective sub-functions
        if mode in ['-i', '--interpret']:
            input_file, output_file = sys.argv[2:]
            interpreter_gcode(input_file, output_file)
        elif mode in ['-e', '--execute']:
            command_file, log_file = sys.argv[2:]
            execute_r3(command_file, log_file)
        elif mode in ['--help']:
            # TODO Add this
            print("more info")
        else:
            raise ApiException("Unknown option passed. Type --help for more info.")
    except ValueError:
        print("False number of arguments passed.")
        sys.exit(-1)
    except ApiException as e:
        print(e)
        sys.exit(-2)
    except Exception as e:
        print("External or unexpected exception!")
        print(e)
        sys.exit(-3)
    else:
        sys.exit(0)
