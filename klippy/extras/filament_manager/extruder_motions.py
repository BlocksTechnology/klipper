import typing
import logging


class ExtruderMotion:
    """Class that handles extruder motions for filament loading and unloading"""

    def __init__(self, printer, extruder, name) -> None:
        self.printer = printer
        self.name = name
        self.reactor = self.printer.get_reactor()
        self.gcode = None
        self.pheaters = self.extruder_heater = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.extruder = extruder
        self.old_extruder = None

    def handle_ready(self) -> None:
        """Handle `klippy:ready` events"""
        self.old_extruder = self.printer.lookup_object("toolhead").get_extruder()
        self.gcode = self.printer.lookup_object("gcode")

    def _enable_extruder_stepper(self) -> bool:
        """Enable extruder motor

        Returns
            bool : True the stepper was enabled, False otherwise
        """
        stepper_enable = self.printer.lookup_object("stepper_enable")
        return stepper_enable.set_motors_enable([self.name], True)

    def _force_activate(self) -> None:
        """Activate the current extruder if it's not active yet"""
        if self.extruder != self.old_extruder:
            self.old_extruder = self.printer.lookup_object("toolhead").get_extruder()
            toolhead = self.printer.lookup_object("toolhead")
            toolhead.flush_step_generation()
            last_position = self.extruder.extruder_stepper.find_past_position(
                self.reactor.monotonic()
            )
            toolhead.set_extruder(self.extruder, last_position)
            self.printer.send_event("extruder:activate_extruder")
            logging.info("Activated extruder %s", str(self.extruder.get_name()))

    def heat(self, temp: int, threshold: float = 0.1, wait: bool = False) -> None:
        """Heat the heater associated with the extruder"""
        eventtime = self.reactor.monotonic()
        heater = self.extruder.get_heater()
        if (
            (temp * (1 - threshold))
            <= heater.get_temp(eventtime)
            <= (temp * (1 + threshold))
        ):
            heater.set_temp(temp)
        while not self.printer.is_shutdown() and wait:
            heater_temp, target_temp = heater.get_temp(eventtime)
            if (
                (target_temp * (1 - threshold))
                <= heater_temp
                <= (target_temp * (1 + threshold))
            ):
                return
            eventtime = self.reactor.pause(eventtime + 1.0)

    def sync_aux(self) -> None:
        pass

    def unsync_aux(self) -> None:
        pass

    def move(
        self,
        distance: float = 10.0,
        speed: float = 10.0,
        acceleration: float = 50.0,
        wait=True,
    ) -> None:
        """Move the extruder

        Extrude factor is always 1

        Args:
            distance (float): Move distance
            speed    (float): Speed of the movement
            wait     (bool) : Wait for movements to finish. Defaults to True
        """
        extruder_heater = self.extruder.get_heater()
        if not extruder_heater.can_extrude:
            raise self.printer.command_error("Extruder below minimum temperature")
        self._force_activate()
        eventtime = self.reactor.monotonic()
        force_move = self.printer.lookup_object("force_move")
        mcu = self.printer.lookup_object("mcu")
        est_print_time = mcu.estimated_print_time(eventtime)
        prev_position = self.extruder.find_past_position(est_print_time)
        # Ignore extrude factor always 1 in this case.
        npos = prev_position + distance
        force_move.manual_move(
            self.extruder.extruder_stepper.stepper,
            npos,
            speed,
            acceleration,
        )
