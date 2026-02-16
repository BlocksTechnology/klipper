import enum
import logging
import typing

PLA_TEMPERATURE = 230
PETG_TEMPERATURE = 240
ABS_TEMPERATURE = 250
NYLON_TEMPERATURE = 270
DEFAULT_TEMPERATURE = 250


# _K = typing.TypeVar("_K", object, typing.Callable[[typing.Any], typing.Any])
KObject: typing.TypeAlias = object | typing.Callable[[typing.Any], typing.Any]


class FilamentMotionsError(Exception):
    """Class for filament load and unload errors"""

    def __init__(self, message, errors: list[str] | None = None) -> None:
        super().__init__(message)
        self.errors: list[str] | None = errors


class FilamentStates(enum.Enum):
    LOADING = "loading"
    LOADED = "loaded"
    UNLOADED = "unloaded"
    UNLOADING = "unloading"
    UNKNOWN = "unknown"


class SensorChecker:
    def __init__(self, config: KObject) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.is_enabled: bool = False
        self.sensor = None
        self.callback: typing.Callable[..., typing.Any] | None = None
        self.trigger_state: bool = False
        self.current_state = False
        self.check_interval: float = 1.5
        self.last_check_time = 0
        self.min_event_systime = self.reactor.NEVER
        self.event_delay: float = 0.5
        self.check_timer = self.reactor.register_timer(
            self.verify_sensor, self.reactor.NEVER
        )
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self) -> None:
        """Handle `klippy:ready` event"""
        self.min_event_systime: float = self.reactor.monotonic() + 2.0

    def toggle_check(self) -> None:
        """Toggle sensor state checking

        If `sensor` has not yet been set then the method will
        return without toggling the sensor check
        """
        if not self.sensor:
            return
        self.is_enabled = not self.is_enabled
        if self.is_enabled and self.sensor:
            self.reactor.update_timer(self.check_timer, self.reactor.NOW)
            return
        self.reactor.update_timer(self.check_timer, self.reactor.NEVER)

    def register_sensor(
        self,
        sensor_type: str,
        sensor_name: str,
    ):
        """Register sensor for verification"""
        self.sensor = self.printer.lookup_object(f"{sensor_type} {sensor_name}", None)
        if not self.sensor:  # pyright: ignore[reportUnknownMemberType]
            raise self.printer.config_error(  # pyright: ignore[reportUnknownMemberType]
                f"Unknown Sensor {sensor_type} {sensor_name}"
            )

    def set_check_interval(self, interval: float) -> None:
        """Set the sensor verification interval in seconds"""
        self.check_interval = max(0.1, interval)

    def register_callback(
        self, callback: typing.Callable[..., object], trigger: bool
    ) -> None:
        """Register a callback to be triggered when the sensor
        matches the set `trigger`
        """
        self.trigger_state = trigger
        self.callback: typing.Callable[..., object] = callback

    def verify_sensor(self, eventtime: float) -> float:
        """Periodic check of switch sensor state"""
        if not self.is_enabled:
            return self.reactor.NEVER
        if not self.sensor:
            self.printer.command_error(f"Sensor check failed")
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
        self.last_check_time: float = eventtime
        return eventtime + self.check_interval

    def _handle_trigger(self):
        completion = self.reactor.register_callback(self.callback)
        self.min_event_systime = self.reactor.monotonic() + self.event_delay
        return completion.wait()

    def active(self) -> bool:
        """Check if this SensorChecker is currently active"""
        return self.is_enabled


class ExtruderMotions:
    """Class that handles extruder motions for filament loading and unloading"""

    def __init__(self, config, extruder, name) -> None:
        self.printer = config.get_printer()
        self.name = name
        self.reactor = self.printer.get_reactor()
        self.gcode = None
        self.pheaters = self.extruder_heater = None
        self.travel_speed: float = config.getfloat(
            "mov_speed", default=100.0, minval=50.0, maxval=500.0
        )
        self.extruder_speed: float = config.getfloat(
            "extruder_speed", default=10.0, minval=2.0, maxval=50.0
        )
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

    def move(self, distance: float = 10.0, speed: float = 10.0, wait=True) -> None:
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
            distance,
            speed,
        )


class FilamentMotions:
    def __init__(self, config, name, extruder):
        self.config = config
        self.printer = config.get_printer()
        self.name = name
        self.reactor = self.printer.get_reactor()
        self.toolhead = None
        self.gcode = self.printer.lookup_object("gcode")
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.extruder_motion = ExtruderMotions(config, extruder, name)
        self.sensor_checkers = {}
        self.state: FilamentStates = FilamentStates.UNKNOWN
        self.timeout = config.getint("timeout", default=None, minval=10, maxval=1000)

        self.unextrude_timer = self.reactor.register_timer(
            self._unextrude, self.reactor.NEVER
        )
        self.extrude_timer = self.reactor.register_timer(
            self._extrude, self.reactor.NEVER
        )

    def handle_ready(self) -> None:
        """Handle `klippy:ready` event"""
        pass

    def _register_sensor_callback(
        self, sensor_type, name, trigger, interval, callback
    ) -> None:
        """Register a sensor with a specified trigger to run a callback"""
        sensor_check = SensorChecker(self.config)
        sensor_check.register_sensor(sensor_type, name)
        sensor_check.set_check_interval(interval)
        sensor_check.register_callback(callback, trigger)
        self.sensor_checkers.update({name: sensor_check})

    def _deregister_sensor(self, name) -> None:
        """Deletes a sensor checker object by name"""
        sensor_checker = self.sensor_checkers.pop(name)
        del sensor_checker

    def _unextrude(self, eventtime) -> None:
        if self.timeout:
            if self.extrude_count >= self.timeout:
                self.sensor_checkers.get(
                    self.unextrude_control_sensor_name
                ).trigger_check()
                completion = self.reactor.register_callback(self.unextrude_end)
                return completion.wait()
            self.extrude_count += 1
        self.extruder_motion.move(-1, self.speed)
        return float(eventtime + float(1 / self.speed))

    def _purge(self, eventtime) -> None: ...

    def _extrude(self, eventtime) -> None:
        if self.timeout:
            if self.extrude_count >= self.timeout:
                self.sensors_check.get(self.extrude_control_sensor_name).trigger_check()
                completion = self.reactor.register_callback(self.extrude_end)
                return completion.wait()
            self.extrude_count += 1
        self.extruder_motion.move(1, self.speed)
        return float(eventtime + float(1 / self.speed))

    def _handle_unload_sensor_trigger(self, eventtime) -> None:
        if not self.sensors_check.get("unload-helper").active():
            return
        if self.state == typing.Literal["unloading"]:
            self.sensors_check.get("unload-helper").trigger_check()

    def _handle_load_sensor_trigger(self, eventtime) -> None:
        if not self.sensors_check.get("load-helper").active():
            return
        if self.state == typing.Literal["loading"]:
            self.sensors_check.get("load-helper").trigger_check()

    def unload(self) -> None:
        if self.state != typing.Literal["loaded"]:
            raise self.gcode.error(
                f"Cannot unload \n Extruder {self.name} is currently loading or unloaded."
            )
        self.state = "unloading"
        self.extrude_count = 0
        if self.cutter:
            completion = self.reactor.register_callback(self.cutter.cut())
            completion.wait()
        self.unextrude_timeout.update(self.reactor.NOW)
        self.sensors_check.get("unload-helper").trigger_check()

        # cut
        # register unextrude start when cut signals that is actually cut
        # start helper sensor verification
        # stop unextrude on timeout or when sensor is triggered
        # clean or end with error

    def load(self) -> None:
        if self.state != typing.Literal["unloaded"]:
            raise self.gcode.error(
                f"Cannot load \n Extruder {self.name} is currently loaded or unloading"
            )
        self.state = "loading"
        self.extrude_count = 0
        # start sensor verification
        # start extrude
        # stop extrude on cutter sensor or when timeout is reached
        # purge or end with error


class FilamentManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = None
        self.toolhead = self.extruder_objects = self.bucket = None
        self.custom_boundary = None
        self.config = config
        self.controllable_motions: dict = {}
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.gcode.register_command(
            "QUERY_CONTROLLABLES",
            self.cmd_QUERY_CONTROLLABLES,
            "Returns the controllable filament toolheads",
        )
        self.config_bucket = config.getboolean("bucket", False)
        self.config_custom_boundary = config.getboolean("custom_boundary", False)

    def handle_ready(self) -> None:
        """Handle `klippy:ready` events"""
        self.extruder_objects = self.printer.lookup_objects("extruder")
        self.gcode = self.printer.lookup_object("gcode")
        for name, extruder in self.extruder_objects:
            self.controllable_motions.update(
                {f"{name}": FilamentMotions(self.config, name, extruder)}
            )
            self.gcode.respond_info(
                f"Registered controllable filament extruder : {name}, {str(extruder)}"
            )
        if self.config_bucket:
            self.bucket = self.printer.lookup_object("bucket")
        if self.config_custom_boundary:
            self.custom_boundary = self.printer.lookup_object("bed_custom_bound")

    def conditional_homing(self) -> None:
        """Performs Homing operation, only if axes are not homed"""
        eventtime = self.reactor.monotonic()
        kin = self.toolhead.get_kinematics()
        homed_axes = kin.get_status(eventtime)["homed_axes"]
        if "xyz" in homed_axes.lower():
            return
        self.gcode.run_script_from_command("G28")

    def save_idex_state(self) -> None:
        """Save Idex carriage states"""
        self.gcode.respond_info("Save carriage state")
        toolhead = self.printer.lookup_object("toolhead")
        carriages = self.printer.lookup_object("dual_carriages")
        carriages.cmd_SAVE_DUAL_CARRIAGE_STATE({"FILAMENT_MANAGER_CARRIAGE_STATE"})
        toolhead.dwell(1.0)

    def restore_idex_state(self) -> None:
        """Restore Idex carriage states"""
        self.gcode.respond_info("Restoring carriage state")
        toolhead = self.printer.lookup_object("toolhead")
        carriages = self.printer.lookup_object("dual_carriages")
        carriages.cmd_RESTORE_DUAL_CARRIAGE_STATE({"FILAMENT_MANAGER_CARRIAGE_STATE"})
        toolhead.dwell(1.0)

    def save_printer_state(self) -> None:
        """Saves printer gcode state"""
        self.gcode.respond_info("Saving printer state")
        toolhead = self.printer.lookup_object("toolhead")
        gcode_move = self.printer.lookup_object("gcode_move")
        gcode_move.cmd_SAVE_GCODE_STATE({"FILAMENT_MANAGER_STATE"})
        toolhead.dwell(1.0)

    def restore_printer_state(self) -> None:
        """Restores printer gcode state"""
        self.gcode.respond_info("Restoring printer state")
        toolhead = self.printer.lookup_object("toolhead")
        gcode_move = self.printer.lookup_object("gcode_move")
        gcode_move.cmd_RESTORE_GCODE_STATE({"FILAMENT_MANAGER_STATE", "MOVE=0"})
        toolhead.dwell(1.0)

    def cmd_QUERY_CONTROLLABLES(self, gcmd) -> None:
        self.extruder_objects = self.printer.lookup_objects("extruder")
        for name, extruder in self.extruder_objects:
            self.controllable_motions.update(
                {f"{name}": FilamentMotions(self.config, name, extruder)}
            )
        self.gcode.respond_info(
            f"Available controllables: \n {self.controllable_motions}"
        )

    def cmd_UNLOAD(self, gcmd) -> None:
        # home if needed, not during a print
        # TODO: Check if it is printing or not
        toolhead = self.printer.lookup_object("toolhead")
        self.conditional_homing()
        if self.custom_boundary:
            self.reactor.register_callback(
                self.custom_boundary.restore_default_boundary
            )
        if self.bucket:
            self.bucket.move_to_bucket()

        toolhead.dwell(1.0)
        gcmd.error("fucked up no toolhead")


def load_config(config):
    return FilamentManager(config)
