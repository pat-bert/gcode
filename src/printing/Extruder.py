from printing.GCmd import GCmd
from printing.GRedirect import RedirectionTargets
from printing.PrinterComponent import PrinterComponent


class Extruder(PrinterComponent):
    redirector = RedirectionTargets.EXTRUDER

    def boot(self):
        pass

    def shutdown(self):
        pass

    def handle_gcode(self, gcode: GCmd):
        # TODO Implement G Codes
        raise NotImplementedError
