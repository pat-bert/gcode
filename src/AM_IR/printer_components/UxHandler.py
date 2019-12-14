from AM_IR.gcode.GCmd import GCmd
from AM_IR.GRedirect import RedirectionTargets
from AM_IR.printer_components.PrinterComponent import PrinterComponent


class UxHandler(PrinterComponent):
    redirector = [RedirectionTargets.UX]

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        pass

    def handle_gcode(self, gcode: GCmd, *args, **kwargs):
        pass
