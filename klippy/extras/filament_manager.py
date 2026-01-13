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
        self.is_heated = False
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.toolhead = self.pheaters = self.extruder_heater = None
        self.travel_speed = config.getfloat(
            "mov_speed", default=100.0, minval=50.0, maxval=500.0
        )
        self.extruder_speed = config.getfloat(
            "extruder_speed", default=10.0, minval=2.0, maxval=50.0
        )
        self.toolhead = self.printer.lookup_object("toolhead")
        self.current_extruder = self.toolhead.get_extruder()
        self.pheaters = self.printer.lookup_object("heaters")
        self.extruder_heater = extruder.get_heater()
        self.extruder = extruder

    def heat_extruder(
        self, temp: int, threshold: float = 0.1, wait: bool = False
    ) -> None:
        """Heats the extruder to a specified temperature"""
        eventtime = self.reactor.monotonic()
        self.pheaters.set_temperature(self.extruder_heater, temp, False)
        while not self.printer.is_shutdown() and wait:
            heater_temp, _ = self.extruder_heater.get_temp(eventtime)
            if (temp * (1 - threshold)) <= heater_temp <= (temp * (1 + threshold)):
                self.is_heated = True
                return
            eventtime = self.reactor.pause(eventtime + 1.0)
        return

    def conditional_homing(self) -> None:
        """Performs Homing operation, only if axes are not homed"""
        eventtime = self.reactor.monotonic()
        kin = self.toolhead.get_kinematics()
        homed_axes = kin.get_status(eventtime)["homed_axes"]
        if "xyz" in homed_axes.lower():
            return
        self.gcode.run_script_from_command("G28")

    def _move_extruder(
        self, distance: float = 10.0, speed: float = 20.0, wait=True
    ) -> None:
        self.current_extruder = self.toolhead.get_extruder()
        if not self.is_heated and self.current_extruder != self.extruder:
            return
        eventtime = self.reactor.monotonic()
        gcode_move = self.printer.lookup_object("gcode_move")
        prev_position = self.toolhead.get_position()
        gcode_move.absolute_coord = False  # G91
        v = distance * gcode_move.get_status(eventtime)["extrude_factor"]
        new_distance = v + prev_position[3]
        self.toolhead.manual_move(
            [prev_position[0], prev_position[1], prev_position[2], new_distance]
        )
        if wait:
            self.toolhead.wait_moves()

    def purge_movement(self) -> None:
        """Perform purging for a toolhead"""
        self.current_extruder = self.toolhead.get_extruder()
        if not self.is_heated and self.current_extruder != self.extruder:
            return
        pass


class FilamentMotions:
    # TODO: Received filament sensors, checks filament presences
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.cutter_name = config.get("cutter_sensor_name", None)
        self.filament_switch_sensor_name = config.get(
            "filament_switch_sensor_name", None
        )
        self.verify_switch_sensor_timer = self.reactor.register_timer(
            self.verify_switch_state, self.reactor.NEVER
        )
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.cutter_object = self.filament_switch_sensor_object = None

    def handle_ready(self) -> None:
        pass

    def _unload_end(self, eventtime=None):
        if not self._unload_started:
            return self.reactor.NEVER

        self._unload_started = False
        self._unextrude_count = 0

    def do_unextrude(self, eventtime) -> None:
        if not self._unload_started and not self._load_started:
            return self.reactor.NEVER

        if self.unload_timeout:
            if self._unextrude_count >= self.unload_timeout:
                self.reactor.update_timer(
                    self.sensors.verify_switch_sensor_timer, self.reactor.NEVER
                )
                completion = self.reactor.register_callback(self._unload_end)
                return completion.wait()
            self._unextrude_count += 1
            self._move_extruder(distance=-10.0, speed=self.extruder_speed, wait=False)
            return float(eventtime + float(10 / self.extruder_speed))

    def do_extrude(self, eventtime) -> None:
        pass

    def do_change_filament(self, eventtime) -> None:
        pass


class FilamentManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_gcode("gcode")
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.motions = ExtruderMotions(config)
        self.fil_motions = FilamentMotions(config)

    def handle_connect(self) -> None:
        self.toolhead = self.printer.lookup_object("toolhead")

    def handle_ready(self) -> None:
        # TODO: Maybe check if filament is actually present and then update local variables
        ...

    def cmd_LOAD_FILAMENT(self, gcmd) -> None:
        tool = gcmd.get("TOOLHEAD")
        self.fil_motions.load_filament()
        pass

    def cmd_UNLOAD_FILAMENT(self, gcmd) -> None:
        pass

    def cmd_CHANGE_FILAMENT(self, gcmd) -> None:
        pass

    def get_status(self, eventtime) -> None:
        return {"state": self.state}


def load_config_prefix(config):
    return FilamentManager(config)
