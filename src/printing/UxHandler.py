from printing.GCmd import GCmd
from printing.GRedirect import RedirectionTargets
from printing.PrinterComponent import PrinterComponent


class UxHandler(PrinterComponent):
    redirector = [RedirectionTargets.UX]

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        pass

    def handle_gcode(self, gcode: GCmd, *args, **kwargs):
        pass
