from AM_IR.GRedirect import RedirectionTargets
from AM_IR.gcode.GCmd import GCmd
from AM_IR.printer_components.PrinterComponent import PrinterComponent


class Extruder(PrinterComponent):
    redirector = [RedirectionTargets.BROADCAST, RedirectionTargets.EXTRUDER]
    INCH_IN_MM = 25.4

    def __init__(self):
        self.inch_active = False

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        pass

    def handle_gcode(self, gcode: GCmd, *args, **kwargs):
        # TODO Implement G Codes
        if gcode.id == 'G20':
            self.inch_active = True
            raise NotImplementedError
        elif gcode.id == 'G21':
            self.inch_active = False
            raise NotImplementedError
