#! /usr/bin/env python3
"""
RobotPrint3D:
Control your robot remotely using the Mitsubishi R3 protocol.
Translate G-Code to Mitsubishi commands.

Usage:
    main.py (-V | --validate) IN_FILE CONFIG_FILE [-o OUTPUT_FILE] [--quiet | --verbose]
    main.py --gi --ip=<ip> --port=<port> --vid=<vid> --pid=<pid> [--quiet | --verbose] [--safe]
    main.py --gi --ip=<ip> --port=<port> [--quiet | --verbose] [--safe]
    main.py --gi --vid=<vid> --pid=<pid> [--quiet | --verbose] [--safe]
    main.py --mi --ip=<ip> --port=<port> [--quiet | --verbose] [--safe]
    main.py --demo --ip=<ip> --port=<port> [--safe]
    main.py (-h | --help)
    main.py --version

Options:
    -h --help               Show this screen.
    --version               Show version.
    -V --validate           Validate a G-code file and create an extended G-code file on success.
    -o OUTPUT_FILE          Specify output file [default: ./r3_cmd.txt].
    --gi                    Launch interactive G-code shell.
    --mi                    Launch interactive R3 shell (Mitsubishi robots).
    --demo                  Launch shell with demonstration examples.
    --ip=<ip>               Specify IPv4-address of the robot.
    --port=<port>           Specify port of the robot.
    --vid=<vid>             Specify the USB vendor id
    --pid=<pid>             Specify the USB product id
    --quiet                 Print less text.
    --verbose               Print more text.
    --safe                  Start and finish at safe position.

"""
__version__ = "0.1.0"  # pragma: no mutate

# Built-in libraries
import logging
import os
import sys

from docopt import docopt
from schema import Schema, And, Use, SchemaError

from src.ApplicationExceptions import ApiException
from src.cli_commands.check_trajectory import check_trajectory
from src.cli_commands.demo import demo_mode
from src.cli_commands.interactive_gcode import interactive_gcode
from src.cli_commands.interactive_gcode_printer_only import interactive_gcode_printer_only
from src.cli_commands.interactive_gcode_robot_only import interactive_gcode_robot_only
from src.cli_commands.interactive_melfa import interactive_melfa
from src.clients.ComClient import validate_id
from src.clients.TcpClientR3 import validate_ip, validate_port
from src.kinematics.inverse_kinematics import OutOfReachError
from src.kinematics.joints import Singularity
from src.prechecks.exceptions import TrajectoryError

EXIT_SUCCESS = 0
EXIT_UNEXPECTED_ERROR = -1
EXIT_INTERNAL_ERROR = -2
EXIT_BAD_INPUT = -3


def main(*argv):
    logging.basicConfig(level=logging.DEBUG, format='%(levelname)s:%(asctime)s %(message)s',
                        datefmt='%d/%m/%Y %H:%M:%S')
    # Gather command line arguments
    argv = list(*argv) if len(argv) == 1 else [i for i in argv]
    args = docopt(__doc__, argv=argv, help=True, version=__version__, options_first=False)

    """
    Create input schemata - Options accepting user input as value are checked for plausibility
    """
    input_schema = Schema(
        {"IN_FILE": And(os.path.exists, error="IN_FILE should exist")},
        ignore_extra_keys=True,
    )
    config_schema = Schema(
        {"CONFIG_FILE": And(os.path.exists, error="CONFIG_FILE should exist")},
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
    usb_schema = Schema(
        {
            "--vid": And(
                Use(lambda x: int(x, 16)),
                validate_id,
                error="Vendor ID needs to be unsigned short (16 bits): 0..65535)"
            ),
            "--pid": And(
                Use(lambda x: int(x, 16)),
                validate_id,
                error="Product ID needs to be unsigned short (16 bits): 0..65535)"
            )
        },
        ignore_extra_keys=True,
    )

    """
    Dispatch sub-functions - new sub-commands are called here
    """
    # noinspection PyBroadException
    try:
        # Functions using TCP/IP-connection
        if args["--gi"]:
            usb_present = args["--vid"] is not None and args["--pid"] is not None
            tcp_present = args["--ip"] is not None and args["--port"] is not None
            if tcp_present:
                args.update(connection_schema.validate(args))
                ip, port, safe = args["--ip"], args["--port"], args["--safe"]

                if usb_present:
                    # Robot and PCB
                    args.update(usb_schema.validate(args))
                    interactive_gcode(ip, port, (args["--vid"], args["--pid"]))
                else:
                    # Robot only
                    interactive_gcode_robot_only(ip, port, safe_return=safe)
            elif usb_present:
                # PCB only
                args.update(usb_schema.validate(args))
                interactive_gcode_printer_only((args["--vid"], args["--pid"]))
            else:
                # None present
                raise ValueError
        elif args["--validate"]:
            input_schema.validate(args)
            config_schema.validate(args)
            check_trajectory(config_f=args["CONFIG_FILE"], gcode_f=args["IN_FILE"])
        else:
            args.update(connection_schema.validate(args))
            ip, port, safe = (args["--ip"], args["--port"], args["--safe"],)

            if args["--mi"]:
                interactive_melfa(ip, port, safe_return=safe)
            elif args["--demo"]:
                demo_mode(ip, port, safe_return=safe)
            else:
                raise ApiException("Unknown option passed. Type --help for more info.")
    except SchemaError:
        # Input validation error
        logging.exception("Input data invalid.")
        sys.exit(EXIT_BAD_INPUT)
    except (ApiException, Singularity, OutOfReachError, TrajectoryError):
        # Intentionally thrown exception by functions of this module
        logging.exception("Internal error.")
        sys.exit(EXIT_INTERNAL_ERROR)
    except KeyError:
        # Accessing the arg dictionary with different keys as specified in docstring
        logging.exception("This might have happened due to different versions of CLI documentation and parsing.")
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
