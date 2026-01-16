import math, chelper
import logging


PLA_TEMPERATURE = 230
PETG_TEMPERATURE = 240
ABS_TEMPERATURE = 250
NYLON_TEMPERATURE = 270
DEFAULT_TEMPERATURE = 250


class SensorChecker:
    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.is_enabled = False
        self.sensor = None
        self.callback = None
        self.trigger_state: bool = False
        self.current_state = False
        self.check_interval = 1.5
        self.last_check_time = 0
        self.min_event_systime = self.reactor.NEVER
        self.event_delay = 0.5
        self.check_timer = self.reactor.register_timer(
            self.verify_sensor, self.reactor.NEVER
        )
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self) -> None:
        """Handle `klippy:ready` event"""
        self.min_event_systime = self.reactor.monotonic() + 2.0

    def toggle_check(self) -> None:
        """Enable/Disable sensor state checking"""
        if not self.sensor:
            return
        self.is_enabled = not self.is_enabled
        if self.is_enabled and self.sensor:
            self.reactor.update_timer(self.check_timer, self.reactor.NOW)
            return
        self.reactor.update_timer(self.check_timer, self.reactor.NEVER)

    def register_sensor(
        self,
        sensor_type,
        sensor_name,
    ):
        """Register a sensor for checking"""
        self.sensor = self.printer.lookup_object(f"{sensor_type} {sensor_name}", None)

    def set_check_interval(self, interval: float) -> None:
        """Set the check interval in seconds"""
        self.check_interval = max(0.1, interval)

    def register_callback(self, callback, trigger: bool) -> None:
        """Register a callback to be triggered when the sensor matches the `trigger`"""
        if callable(callback):
            self.trigger_state = trigger
            self.callback = callback

    def verify_sensor(self, eventtime):
        """Periodic check of switch sensor state"""
        if not self.is_enabled:
            return self.reactor.NEVER
        status = self.sensor.get_status(eventtime)
        filament_present = status.get("filament_detected", self.trigger_state)
        if (filament_present != self.current_state) and (
            filament_present == self.trigger_state
        ):
            if eventtime >= self.min_event_systime:
                self.current_state = filament_present
                self.min_event_systime = self.reactor.NEVER
                self.reactor.register_callback(self._handle_trigger)
        self.last_check_time = eventtime
        return eventtime + self.check_interval

    def _handle_trigger(self):
        completion = self.reactor.register_callback(self.callback)
        self.min_event_systime = self.reactor.monotonic() + self.event_delay
        return completion.wait()


class ExtruderMotions:
    def __init__(self, config, extruder):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.pheaters = self.extruder_heater = None
        self.travel_speed = config.getfloat(
            "mov_speed", default=100.0, minval=50.0, maxval=500.0
        )
        self.extruder_speed = config.getfloat(
            "extruder_speed", default=10.0, minval=2.0, maxval=50.0
        )

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.extruder = extruder
        self.old_extruder = None

    def handle_ready(self) -> None:
        """Handle `klippy:ready` events"""
        logging.info(type(self.extruder))

    def _force_activate(self) -> None:
        self.old_extruder = self.printer.lookup_object("toolhead").get_extruder()
        toolhead = self.printer.lookup_object("toolhead")
        if self.extruder != self.old_extruder:
            toolhead.flush_step_generation()
            last_position = self.extruder.extruder_setpper.find_past_position(
                self.reactor.monotonic()
            )
            toolhead.set_extruder(self.extruder, last_position)
            self.printer.send_event("extruder:activate_extruder")
            logging.info("Activated extruder %s", str(self.extruder.get_name()))

    def heat_extruder(
        self, temp: int, threshold: float = 0.1, wait: bool = False
    ) -> None:
        """Heats the extruder to a specified temperature"""
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
        return

    def _move_extruder(
        self, distance: float = 10.0, speed: float = 10.0, wait=True
    ) -> None:
        extruder_heater = self.extruder.get_heater()
        if not extruder_heater.can_extrude:
            raise self.printer.command_error("Extruder below minimum temperature")
        self._force_activate()
        eventtime = self.reactor.monotonic()
        toolhead = self.printer.lookup_object("toolhead")
        mcu = self.printer.lookup_object("mcu")
        est_print_time = mcu.estimated_print_time(eventtime)
        prev_position = self.extruder.find_past_position(est_print_time)
        # Ignore extrude factor always 1 in this case.
        npos = prev_position + distance
        self.gcode.respond_info(f"old position: {prev_position} | new position {npos}")
        toolhead.manual_move(
            [None, None, None, npos],
            speed,
        )
        self.gcode.respond_info(f"After movement position {toolhead.get_position()}")
        # oldpos = toolhead.get_position()
        # oldpos[-1] = prev_position
        # toolhead.commanded_pos[-1] = prev_position
        # toolhead.set_position(oldpos)
        # self.gcode.respond_info(
        #     f"After resetting old positions {toolhead.get_position()}"
        # )
        # toolhead.set_position([toolhead.get_position()[:3], prev_position])
        gmove = self.printer.lookup_object("gcode_move")
        gmove.reset_last_position
        if wait:
            toolhead.wait_moves()


class FilamentMotions:
    def __init__(self, config, name, extruder):
        self.printer = config.get_printer()
        self.name = name
        self.reactor = self.printer.get_reactor()
        self.toolhead = None
        self.gcode = self.printer.lookup_object("gcode")
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.extruder_motion = ExtruderMotions(config, extruder)

    def handle_ready(self) -> None:
        """Handle `klippy:ready` event"""
        self.toolhead = self.printer.lookup_object("toolhead")

    def do_extrude(self) -> None:
        self.extruder_motion._move_extruder(10, 60, False)

    def jitter(self) -> None:
        """Makes the extruder stepper buz for identification"""
        self.gcode.respond_info(f"Jittering {self.name}... Check for sound")
        self.gcode.run_script_from_command(f"STEPPER_BUZZ STEPPER={self.name}\n")


class FilamentManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.toolhead = self.extruder_objects = None
        self.config = config
        self.controllable_motions: dict = {}
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.gcode.register_command(
            "QUERY_CONTROLLABLES",
            self.cmd_QUERY_CONTROLLABLES,
            "Returns the controllable filament toolheads",
        )
        self.gcode.register_command(
            "IDENTIFY", self.cmd_IDENTIFY, "Identify controllable filament extruders"
        )
        self.gcode.register_command("TEST_EXTRUDER", self.cmd_TEST_EXTRUDER, "hel")

    def handle_ready(self) -> None:
        """Handle `klippy:ready` events"""
        self.toolhead = self.printer.lookup_object("toolhead")
        self.extruder_objects = self.printer.lookup_objects("extruder")
        for name, extruder in self.extruder_objects:
            self.controllable_motions.update(
                {f"{name}": FilamentMotions(self.config, name, extruder)}
            )
            self.gcode.respond_info(
                f"Registered controllable filament extruder : {name}, {str(extruder)}"
            )

    def conditional_homing(self) -> None:
        """Performs Homing operation, only if axes are not homed"""
        eventtime = self.reactor.monotonic()
        kin = self.toolhead.get_kinematics()
        homed_axes = kin.get_status(eventtime)["homed_axes"]
        if "xyz" in homed_axes.lower():
            return
        self.gcode.run_script_from_command("G28")

    def cmd_IDENTIFY(self, gcmd) -> None:
        for _, controllable in self.controllable_motions.items():
            controllable.jitter()

    def cmd_QUERY_CONTROLLABLES(self, gcmd) -> None:
        self.extruder_objects = self.printer.lookup_objects("extruder")
        for name, extruder in self.extruder_objects:
            self.controllable_motions.update(
                {f"{name}": FilamentMotions(self.config, name, extruder)}
            )
        self.gcode.respond_info(
            f"Available controllables: \n {self.controllable_motions}"
        )

    def cmd_TEST_EXTRUDER(self, gcmd) -> None:
        self.controllable_motions.get("extruder").do_extrude()

    def cmd_LOAD_FILAMENT(self, gcmd) -> None:
        raise NotImplementedError()

    def cmd_UNLOAD_FILAMENT(self, gcmd) -> None:
        raise NotImplementedError()

    def cmd_CHANGE_FILAMENT(self, gcmd) -> None:
        raise NotImplementedError()

    # def get_status(self) -> dict:
    #     return {motion.get_status for motion in self.controllable_motions}


def load_config(config):
    return FilamentManager(config)
