from printing.GRedirect import RedirectionTargets
from printing.PrinterComponent import PrinterComponent


class Heater(PrinterComponent):
    redirector = RedirectionTargets.HEATERS

    def boot(self):
        pass

    def shutdown(self):
        pass
