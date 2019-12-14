from AM_IR.gcode.GCmd import GCmd
from AM_IR.GRedirect import RedirectionTargets
from AM_IR.printer_components.PrinterComponent import PrinterComponent


class Extruder(PrinterComponent):
    redirector = [RedirectionTargets.BROADCAST, RedirectionTargets.EXTRUDER]

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        pass

    def handle_gcode(self, gcode: GCmd, *args, **kwargs):
        # TODO Implement G Codes
        pass
