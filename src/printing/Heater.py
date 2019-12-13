from printing.GRedirect import RedirectionTargets
from printing.PrinterComponent import PrinterComponent


class Heater(PrinterComponent):
    redirector = [RedirectionTargets.HEATERS]

    def handle_gcode(self, *args, **kwargs):
        pass

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        pass
