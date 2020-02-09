#! /usr/bin/env python3
"""
RobotPrint3D:
Control your robot remotely using the Mitsubishi R3 protocol.
Translate G-Code to Mitsubishi commands.

Usage:
    main.py (-I | --interpret) IN_FILE [-o OUTPUT_FILE] [-l LOG_FILE] [--quiet | --verbose]
    main.py (-E | --execute) IN_FILE [-l LOG_FILE] [--ip=<ip>] [--port=<port>] [--quiet | --verbose] [--safe]
    main.py --gi [--ip=<ip>] [--port=<port>]  [-l LOG_FILE]  [--quiet | --verbose] [--safe]
    main.py --mi [--ip=<ip>] [--port=<port>]  [-l LOG_FILE]  [--quiet | --verbose] [--safe]
    main.py --demo [--ip=<ip>] [--port=<port>] [-l LOG_FILE] [--safe]
    main.py --ghelp
    main.py (-h | --help)
    main.py --version

Options:
    -h --help       Show this screen.
    --version       Show version.
    -I --interpret  Interpret a G-code file and create an R3-protocol (Mitsubishi robots) file.
    -E --execute    Execute a file containing R3-compatible (Mitsubishi robots) commands.
    -o OUTPUT_FILE  Specify output file [default: ./r3_cmd.txt].
    -l LOG_FILE     Specify log file.
    --gi            Launch interactive G-code shell.
    --mi            Launch interactive R3 shell (Mitsubishi robots).
    --demo          Launch shell with demonstration examples.
    --ip=<ip>       Specify IPv4-address of the robot [default: 192.168.0.1].
    --port=<port>   Specify port of the robot [default: 10002].
    --quiet         Print less text.
    --verbose       Print more text.
    --safe          Start and finish at safe position.
    --ghelp         List supported G-code commands.

"""
__version__ = "0.1.0"  # pragma: no mutate

# Built-in libraries
import logging
import os
import sys

# Own libraries
from src.ApplicationExceptions import ApiException
from src.GRedirect import GRedirect
from src.cli_commands.demo import demo_mode
from src.cli_commands.execute_r3 import execute_r3
from src.cli_commands.interactive_gcode import interactive_gcode
from src.cli_commands.interactive_melfa import interactive_melfa
from src.cli_commands.interpret_gcode import interpret_gcode
from src.exit_codes import *
from src.melfa.TcpClientR3 import validate_ip, validate_port

# Third-party libraries
try:
    from docopt import docopt
    from schema import Schema, And, Or, Use, SchemaError
    from schema import Optional as Opt
except ImportError:
    print(
        "This application requires some modules that you can install using the requirements.txt file."
    )
    sys.exit(EXIT_PACKAGE_ERROR)


def main(*argv):
    # Gather command line arguments
    argv = list(*argv) if len(argv) == 1 else list(i for i in argv)
    args = docopt(
        __doc__, argv=argv, help=True, version=__version__, options_first=False
    )

    """
    Create input schemata - Options accepting user input as value are checked for plausibility
    """
    log_schema = Schema(
        {Opt("-l"): Or(str, None, error="Log file should be possible to open")},
        ignore_extra_keys=True,
    )
    input_schema = Schema(
        {"IN_FILE": And(os.path.exists, error="IN_FILE should exist")},
        ignore_extra_keys=True,
    )
    output_schema = Schema(
        {"-o": And(os.path.exists, error="Output file should exist")},
        ignore_extra_keys=True,
    )
    connection_schema = Schema(
        {
            "--ip": And(
                validate_ip,
                error='IPv4 needs to be in format "[0-255].[0-255].[0-255].[0-255]"',
            ),
            "--port": And(
                Use(int),
                validate_port,
                error="Port needs to be unsigned short (16 bits): 0..65535)",
            ),
        },
        ignore_extra_keys=True,
    )

    """
    Dispatch sub-functions - new sub-commands are called here
    """
    # noinspection PyBroadException
    try:
        if args["--ghelp"]:
            print("Supported G-Codes:")
            print(GRedirect.supported_gcodes())
        else:
            # Options usable in all commands
            log_schema.validate(args)
            if args["--interpret"]:
                # Functions without TCP/IP-connection
                input_schema.validate(args)
                output_schema.validate(args)
                interpret_gcode(args["IN_FILE"], args["-o"])
            else:
                # Functions using TCP/IP-connection
                args.update(connection_schema.validate(args))
                ip, port, log, safe = (
                    args["--ip"],
                    args["--port"],
                    args["-l"],
                    args["--safe"],
                )

                if args["--execute"]:
                    input_schema.validate(args)
                    execute_r3(args["IN_FILE"], ip, port, f_log=log)
                elif args["--gi"]:
                    interactive_gcode(ip, port, log_file=log, safe_return=safe)
                elif args["--mi"]:
                    interactive_melfa(ip, port, log_file=log, safe_return=safe)
                elif args["--demo"]:
                    demo_mode(ip, port, safe_return=safe)
                else:
                    raise ApiException(
                        "Unknown option passed. Type --help for more info."
                    )
    except SchemaError as e:
        # Input validation error
        sys.exit(EXIT_BAD_INPUT)
    except ApiException as e:
        # Intentionally thrown exception by functions of this module
        print("Application crashed due to '{}'".format(e))
        sys.exit(EXIT_INTERNAL_ERROR)
    except KeyError as e:
        # Accessing the arg dictionary with different keys as specified in docstring
        print(e)
        print(
            "This might have happened due to different versions of CLI documentation and parsing."
        )
        sys.exit(EXIT_UNEXPECTED_ERROR)
    except NotImplementedError:
        # This might be used in some functions
        logging.exception("Encountered not implemented feature.")
        sys.exit(EXIT_UNEXPECTED_ERROR)
    except Exception:
        # Exception that has not been caught and rethrown as a proper ApiException (= Bug)
        logging.exception("External or unexpected exception!")
        sys.exit(EXIT_UNEXPECTED_ERROR)
    else:
        # Everything okay, no exception occurred
        sys.exit(EXIT_SUCCESS)


if __name__ == "__main__":
    main(sys.argv[1:])
