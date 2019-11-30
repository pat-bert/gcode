from printing.Extruder import Extruder
from printing.GCmd import GCmd
from printing.GRedirect import RedirectionTargets, GRedirect
from printing.Heater import Heater
from printing.MelfaRobot import MelfaRobot
from printing.PrinterComponent import PrinterComponent
from printing.TcpClientR3 import TcpClientR3
from printing.UxHandler import UxHandler
from printing.gcode2melfa import gcode2melfa


class GPrinter(object):
    """
    Bundles absolutely all components necessary to control the 3D printer.
    """

    def __init__(self, mover: MelfaRobot, extruder: PrinterComponent, heater: PrinterComponent):
        self._mover = mover
        self._mover.boot()

        self._extruder = extruder
        self._extruder.boot()

        self._heater = heater
        self._heater.boot()

    def execute(self, gcode: GCmd):
        try:
            target = GRedirect.redirect_cmd(gcode)
        except ValueError as e:
            print(e)
        else:
            if target is RedirectionTargets.MOVER or RedirectionTargets.BROADCAST:
                # Redirect command to Melfa
                gcode2melfa(gcode, 1, robot=self._mover)
            if target is RedirectionTargets.EXTRUDER or RedirectionTargets.BROADCAST:
                self._extruder.handle_gcode(gcode)
            elif target is RedirectionTargets.HEATERS:
                self._heater.handle_gcode(gcode)
            elif target is RedirectionTargets.UX:
                # TODO Redirect command to UX
                pass
            else:
                raise ValueError("Unsupported redirection target: {}".format(target))

    def shutdown(self):
        self._mover.shutdown()
        self._extruder.shutdown()
        self._heater.shutdown()
        self._mover.shutdown(safe_return=True)

    @classmethod
    def default_init(cls, ip, port):
        # Create TCP client
        tcp_client = TcpClientR3(host=ip, port=port)
        tcp_client.connect()

        # Create mover object
        mover = MelfaRobot(tcp_client, number_axes=6, speed_threshold=10)

        # Create extruder object
        extruder = Extruder()

        # Create heater object
        heater = Heater()

        # Create UX object
        user_output = UxHandler()

        # Create printer object
        return cls(mover, extruder, heater)
