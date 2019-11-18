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
    main.py --demo [--ip=<ip>] [--port=<port>] [-l LOG_FILE]
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

"""

__version__ = '1.0'

# Built-in libraries
import sys
import os

# Third-party libraries
try:
    from docopt import docopt
    from schema import Schema, And, Or, Use, SchemaError
    from schema import Optional as Opt
except ImportError:
    print('This application requires that "schema" data-validation library and "docopt" is installed.')
    sys.exit(-10)

# Own libraries
from ApplicationExceptions import ApiException
from commands import demo_mode, interpret_gcode, execute_r3, interactive_gcode, interactive_melfa

if __name__ == '__main__':
    # Gather command line arguments
    args = docopt(__doc__, argv=None, help=True, version=__version__, options_first=False)

    # Create input schemata
    log_schema = Schema({Opt('-l'): Or(os.path.exists, None, error='Log file should exist')}, ignore_extra_keys=True)
    input_schema = Schema({'IN_FILE': And(os.path.exists, error='IN_FILE should exist')}, ignore_extra_keys=True)
    output_schema = Schema({'-o': And(os.path.exists, error='Output file should exist')}, ignore_extra_keys=True)
    connection_schema = Schema({
        '--ip': And(lambda n: (all([(int(i) in range(0, 256)) for i in n.split('.')]) and len(n.split('.')) == 4),
                    error='IPv4 needs to be in format "[0-255].[0-255].[0-255].[0-255]"'),
        '--port': And(Use(int), lambda u: u in range(0, 65536),
                      error='Port needs to be unsigned short (16 bits): 0..65535)')
    }, ignore_extra_keys=True)

    # Dispatch sub-functions
    try:
        # Options usable in all commands
        log_schema.validate(args)
        if args['--interpret']:
            # Functions without TCP/IP-connection
            input_schema.validate(args)
            output_schema.validate(args)
            interpret_gcode(args['IN_FILE'], args['-o'])
        else:
            # Functions using TCP/IP-connection
            args.update(connection_schema.validate(args))
            ip, port, log = args['--ip'], args['--port'], args['-l']

            if args['--execute']:
                input_schema.validate(args)
                execute_r3(args['IN_FILE'], ip, port)
            elif args['--gi']:
                interactive_gcode(ip, port, log_file=log)
            elif args['--mi']:
                interactive_melfa(ip, port, log_file=log)
            elif args['--demo']:
                demo_mode(ip, port)
            else:
                raise ApiException("Unknown option passed. Type --help for more info.")
    except SchemaError as e:
        # Input validation error
        exit(e)
    except ApiException as e:
        # Intentionally thrown exception by functions of this module
        print(e)
        sys.exit(-2)
    except KeyError as e:
        # Accessing the arg dictionary with different keys as specified in docstring
        print(e)
        print("This might have happened due to different versions of CLI documentation and parsing.")
    except NotImplementedError:
        # This might be used in some functions.
        print("Encountered not implemented feature.")
    except Exception as e:
        print("External or unexpected exception!")
        raise e
    else:
        sys.exit(0)
