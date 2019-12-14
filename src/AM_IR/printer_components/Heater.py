from AM_IR.GRedirect import RedirectionTargets
from AM_IR.printer_components.PrinterComponent import PrinterComponent


class Heater(PrinterComponent):
    redirector = [RedirectionTargets.HEATERS]

    def handle_gcode(self, *args, **kwargs):
        pass

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        pass
