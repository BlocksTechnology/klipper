import logging
import typing


class Heatsoak:
    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.state = "idle"

        self.heater = config.get("heater", "heater_bed")
        if config.get("temperature_sensor", None):
            self.temperature_sensor = config.get("temperature_sensor")

        self.gcode.register_command(
            "HEATSOAK", self.cmd_HEATSOAK, "Heat and stabilize chamber temperature"
        )
        self.stable_time: int = config.getint(
            "default_stable_time", default=20, minval=15, maxval=120
        ) # In minutes
        self.stabilize_timer = self.reactor.register_timer(
            self.stabilize_check, self.reactor.NEVER
        )

    def wait_heat(self):
        eventtime = self.reactor.monotonic()
        
    def stabilize_check(self): ...

    def cmd_HEATSOAK(self, gcmd): ...

    def get_status(self):
        return {"state": self._state}
