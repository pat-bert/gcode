from enum import unique, Enum

from printing.GCmd import GCmd


@unique
class RedirectionTargets(Enum):
    BROADCAST = 1
    MOVER = 2
    EXTRUDER = 3
    HEATERS = 4


class GRedirect(object):
    br = []
    robot = []
    extruder = []
    heaters = []

    @classmethod
    def redirect_cmd(cls, gcode: GCmd):
        g_id = gcode.id

        if g_id in cls.br:
            return RedirectionTargets.BROADCAST
        elif g_id in cls.robot:
            return RedirectionTargets.MOVER
        elif g_id in cls.extruder:
            return RedirectionTargets.EXTRUDER
        elif g_id in cls.heaters:
            return RedirectionTargets.HEATERS
        else:
            raise ValueError('Unregistered G-code: {}'.format(g_id))
