from GCmd import GCmd
import MelfaCmd
from MelfaRobot import MelfaRobot
from typing import *


def translate_gcode(gcode: GCmd, interactive, gcode_prev: Union[GCmd, None] = None, robot: MelfaRobot = None) \
        -> Union[AnyStr, None]:
    """
    Translates a G-Code to a Mitsubishi Melfa R3 command.
    :param gcode: G-Code object
    :param interactive: Flag indicating whether the command should be executed or stored
    :param gcode_prev: Optional object for previous G-Code to be considered for speed setting
    :param robot: Optional robot object, required for interactive = True
    :return:
    """
    if interactive:
        # G-Code is executed directly
        if gcode.id in ['G00', 'G0']:
            robot.linear_move_poll(gcode.cartesian_abs, gcode.speed)
        elif gcode.id in ['G01', 'G1']:
            robot.linear_move_poll(gcode.cartesian_abs, gcode.speed)
        elif gcode.id in ['G02', 'G2']:
            robot.circular_move_poll(gcode.cartesian_abs, gcode.cartesian_abs + gcode.cartesian_rel, True, gcode.speed)
        elif gcode.id in ['G03', 'G3']:
            robot.circular_move_poll(gcode.cartesian_abs, gcode.cartesian_abs + gcode.cartesian_rel, False, gcode.speed)
        else:
            raise NotImplementedError
    else:
        # Melfa code is saved for later usage
        if gcode.id in ['G00', 'G0', 'G01', 'G1']:
            return MelfaCmd.LINEAR_INTRP + gcode.cartesian_abs.to_melfa_point()
        elif gcode.id in ['G02', 'G2']:
            raise NotImplementedError
        elif gcode.id in ['G03', 'G3']:
            raise NotImplementedError
        else:
            raise NotImplementedError
