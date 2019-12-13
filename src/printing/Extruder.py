from printing.GCmd import GCmd
from printing.GRedirect import RedirectionTargets
from printing.PrinterComponent import PrinterComponent


class Extruder(PrinterComponent):
    redirector = [RedirectionTargets.BROADCAST, RedirectionTargets.EXTRUDER]

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        pass

    def handle_gcode(self, gcode: GCmd, *args, **kwargs):
        # TODO Implement G Codes
        pass
