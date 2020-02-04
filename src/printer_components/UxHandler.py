from src.GRedirect import RedirectionTargets
from src.gcode.GCmd import GCmd
from src.printer_components.PrinterComponent import PrinterComponent


class UxHandler(PrinterComponent):
    redirector = [RedirectionTargets.UX]

    def boot(self, *args, **kwargs):
        pass

    def shutdown(self, *args, **kwargs):
        pass

    def handle_gcode(self, gcode: GCmd, *args, **kwargs):
        if gcode.id == "M117":
            self._display_msg()
        elif gcode.id == "M300":
            self._beep()

    def _display_msg(self):
        raise NotImplementedError

    def _beep(self):
        raise NotImplementedError
