#! /usr/bin/env python3
"""
RobotPrint3D:
Control your robot remotely using the Mitsubishi R3 protocol.
Translate G-Code to Mitsubishi commands!

Usage:
    main.py (-I | --interpret) IN_FILE [-o OUTPUT_FILE] [-l LOG_FILE] [--quiet | --verbose]
    main.py (-E | --execute) IN_FILE [-l LOG_FILE] [--ip=<ip>] [--port=<port>] [--quiet | --verbose]
    main.py --gi [--ip=<ip>] [--port=<port>]  [-l LOG_FILE]  [--quiet | --verbose]
    main.py --mi [--ip=<ip>] [--port=<port>]  [-l LOG_FILE]  [--quiet | --verbose]
    main.py (-h | --help)
    main.py --version

Options:
    -h --help       Show this screen.
    --version       Show version.
    -I --interpret  Interpret a G-code file and create an R3-protocol (Mitsubishi robots) file.
    -E --execute    Execute a file containing R3-compatible (Mitsubishi robots) commands.
    -o OUTPUT_FILE  Specify output file [default: ./r3_cmd.txt].
    -l LOG_FILE     Specify log file.
    --gi             Launch interactive G-code shell.
    --mi             Launch interactive R3 shell (Mitsubishi robots).
    --ip=<ip>       Specify IPv4-address of the robot [default: 192.168.0.1].
    --port=<port>   Specify port of the robot [default: 10002].
    --quiet         Print less text.
    --verbose       Print more text.

"""

__version__ = '1.0'

import sys
import os

try:
    from docopt import docopt
    from schema import Schema, And, Or, Use, SchemaError
    from schema import Optional as Opt
except ImportError:
    print('This application requires that "schema" data-validation library and "docopt" is installed.')
    sys.exit(-10)

from GCmd import GCmd
from MelfaCmd import translate_cmd
from ApplicationExceptions import *
from TCPClient import TCPClient


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
    # Gather command line arguments
    args = docopt(__doc__, argv=None, help=True, version=__version__, options_first=False)

    # Create input schemata
    input_schema = Schema({'IN_FILE': And(os.path.exists, error='IN_FILE should exist')}, ignore_extra_keys=True)
    log_schema = Schema({Opt('-l'): Or(os.path.exists, None, error='Log file should exist')}, ignore_extra_keys=True)
    output_schema = Schema({'-o': And(os.path.exists, error='Output file should exist')}, ignore_extra_keys=True)
    connection_schema = Schema({
        '--ip': And(lambda n: (all([(int(i) in range(0, 256)) for i in n.split('.')]) and len(n.split('.')) == 4),
                    error='IPv4 needs to be in format "[0-255].[0-255].[0-255].[0-255]"'),
        '--port': And(Use(int), lambda u: u in range(0, 65536),
                      error='Port needs to be unsigned short (16 bits): 0..65535)')
    }, ignore_extra_keys=True)

    # Dispatch sub-functions
    try:
        if args['--interpret']:
            input_schema.validate(args)
            output_schema.validate(args)
            log_schema.validate(args)
            interpreter_gcode(args['IN_FILE'], args['-o'])
        elif args['--execute']:
            input_schema.validate(args)
            log_schema.validate(args)
            connection_schema.validate(args)
            execute_r3(args['IN_FILE'], args['-l'])
        elif args['--gi']:
            log_schema.validate(args)
            args.update(connection_schema.validate(args))
            # TODO G-Code interactive mode
            pass
        elif args['--mi']:
            log_schema.validate(args)
            connection_schema.validate(args)
            # TODO Melfa interactive mode
        else:
            raise ApiException("Unknown option passed. Type --help for more info.")
    except SchemaError as e:
        exit(e)
    except ApiException as e:
        print(e)
        sys.exit(-2)
    except KeyError as e:
        print(e)
        print("This might have happened due to different versions of CLI documentation and parsing.")
    except Exception as e:
        print("External or unexpected exception!")
        print(e)
        sys.exit(-3)
    else:
        sys.exit(0)
