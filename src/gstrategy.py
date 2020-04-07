import abc

from MelfaCoordinateService import Plane
from gcode.GCmd import GCmd
from printer_components.MelfaRobot import MelfaRobot


class GCodeImplementation(metaclass=abc.ABCMeta):
    @abc.abstractmethod
    def execute(self, executor: MelfaRobot, cmd: GCmd):
        pass


class G01(GCodeImplementation):
    """
    Linear interpolation
    """

    def execute(self, executor: MelfaRobot, cmd: GCmd):
        current_pos = executor.protocol.reader.get_current_xyzabc()
        current_pos.reduce_to_axes('XYZ')

        # Use the basic functionality of the executor
        if not executor.absolute_coordinates:
            executor.linear_move_poll(cmd.cartesian_abs + current_pos, cmd.speed, current_pos=current_pos)
        else:
            executor.linear_move_poll(cmd.cartesian_abs, cmd.speed, current_pos=current_pos)


class G02(GCodeImplementation):
    """
    Circular interpolation clockwise
    """

    def execute(self, executor: MelfaRobot, cmd: GCmd):
        current_pos = executor.protocol.reader.get_current_xyzabc()
        current_pos.reduce_to_axes('XYZ')

        if not executor.absolute_coordinates:
            executor.circular_move_poll(
                cmd.cartesian_abs + current_pos,
                current_pos + cmd.cartesian_rel,
                True,
                cmd.speed,
                start_pos=current_pos,
            )
        else:
            executor.circular_move_poll(
                cmd.cartesian_abs,
                current_pos + cmd.cartesian_rel,
                True,
                cmd.speed,
                start_pos=current_pos,
            )


class G03(GCodeImplementation):
    """
    Circular interpolation counter-clockwise
    """

    def execute(self, executor: MelfaRobot, cmd: GCmd):
        current_pos = executor.protocol.reader.get_current_xyzabc()
        current_pos.reduce_to_axes('XYZ')

        if not executor.absolute_coordinates:
            executor.circular_move_poll(
                cmd.cartesian_abs + current_pos,
                current_pos + cmd.cartesian_rel,
                False,
                cmd.speed,
                start_pos=current_pos,
            )
        else:
            executor.circular_move_poll(
                cmd.cartesian_abs,
                current_pos + cmd.cartesian_rel,
                False,
                cmd.speed,
                start_pos=current_pos,
            )


class G17G18G19(GCodeImplementation):
    """
    Setting the active plane to XY/XZ/YZ
    """

    def execute(self, executor: MelfaRobot, cmd: GCmd):
        if cmd.id == 'G17':
            executor.active_plane = Plane.XY
        elif cmd.id == 'G18':
            executor.active_plane = Plane.XZ
        else:
            executor.active_plane = Plane.YZ


class G20G21(GCodeImplementation):
    """
    Units in inch/mm
    """

    def execute(self, executor: MelfaRobot, cmd: GCmd):
        if cmd.id == 'G20':
            executor.inch_active = True
        else:
            executor.inch_active = False


class G9091(GCodeImplementation):
    """
    Absolute/relative coordinates
    """

    def execute(self, executor: MelfaRobot, cmd: GCmd):
        if cmd.id == 'G90':
            executor.absolute_coordinates = True
        elif cmd.id == 'G91':
            executor.absolute_coordinates = False
