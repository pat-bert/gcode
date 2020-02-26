from enum import unique, Enum

from src.gcode.GCmd import GCmd


@unique
class RedirectionTargets(Enum):
    BROADCAST = 1
    MOVER = 2
    EXTRUDER = 3
    HEATERS = 4
    UX = 5


class GRedirect:
    broadcast = [
        "G0",  # Rapid movement
        "G1",  # Move
        "G2",  # Clockwise arc
        "G3",  # Counter-clockwise arc
        "G4",  # Dwell
        "G20",  # Set units to inch
        "G21",  # Set units to mm
        "G90",  # Absolute positioning
        "G91",  # Relative positioning
        "G92",  # Set position
        "M0",  # Unconditional stop
        "M1",  # Conditional stop
        "M112",  # Emergency stop
        "M114",  # Get current position
    ]
    ux = [
        "M117",  # Display message
        "M300",  # Play beep
    ]
    mover = [
        "G17",  # Set plane to XY
        "G18",  # Set plane to XZ
        "G19",  # Set plane to YZ
        "G28",  # Move to origin
        "M220",  # Set speed factor override percentage
    ]
    extruder = [
        "M82",  # Set extruder to absolute mode
        "M83",  # Set extruder to relative mode
        "M104",  # Set extruder temperature
        "M105",  # Get extruder temperature
        "M109",  # Set extruder temperature and wait
        "M221",  # Set extruder speed factor override percentage
    ]
    heaters = [
        "M106",  # Fan on (individual component?)
        "M107",  # Fan off
        "M140",  # Set Bed Temperature fast
        "M190",  # Wait for bed temperature
    ]

    @classmethod
    def redirect_cmd(cls, gcode: GCmd):
        g_id = gcode.id

        if g_id in cls.broadcast:
            return RedirectionTargets.BROADCAST
        elif g_id in cls.mover:
            return RedirectionTargets.MOVER
        elif g_id in cls.extruder:
            return RedirectionTargets.EXTRUDER
        elif g_id in cls.heaters:
            return RedirectionTargets.HEATERS
        elif g_id in cls.ux:
            return RedirectionTargets.UX
        else:
            raise ValueError("Unregistered G-code: {}".format(g_id))

    @classmethod
    def supported_gcodes(cls):
        """
        Summarizes all G-codes that are supported by any component defined in the redirection
        """
        return sorted(
            cls.broadcast + cls.extruder + cls.mover + cls.heaters + cls.ux,
            key=lambda x: (x[0], int(x[1:])),
        )


if __name__ == "__main__":
    print(GRedirect.supported_gcodes())
