from printing.GCmd import GCmd
from printing.GRedirect import RedirectionTargets
from printing.PrinterComponent import PrinterComponent


class UxHandler(PrinterComponent):
    redirector = RedirectionTargets.UX

    def boot(self):
        pass

    def shutdown(self):
        pass

    def handle_gcode(self, gcode: GCmd):
        pass
