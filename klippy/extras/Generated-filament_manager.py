import enum
import logging
import typing
from functools import partial

PLA_TEMPERATURE = 230
PETG_TEMPERATURE = 240
ABS_TEMPERATURE = 250
NYLON_TEMPERATURE = 270
DEFAULT_TEMPERATURE = 250

FILAMENT_TEMPERATURES = {
    "PLA": PLA_TEMPERATURE,
    "PETG": PETG_TEMPERATURE,
    "ABS": ABS_TEMPERATURE,
    "NYLON": NYLON_TEMPERATURE,
}


class FilamentStates(enum.Enum):
    LOADING = "loading"
    LOADED = "loaded"
    UNLOADED = "unloaded"
    UNLOADING = "unloading"
    UNKNOWN = "unknown"


class SensorChecker:
    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.is_enabled = False
        self.sensor = None
        self.callback = None
        self.trigger_state = False
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
        self.min_event_systime = self.reactor.monotonic() + 2.0

    def toggle_check(self) -> None:
        if not self.sensor:
            return
        self.is_enabled = not self.is_enabled
        if self.is_enabled and self.sensor:
            self.reactor.update_timer(self.check_timer, self.reactor.NOW)
            return
        self.reactor.update_timer(self.check_timer, self.reactor.NEVER)

    def trigger_check(self) -> None:
        if self.sensor and self.callback:
            self.reactor.register_callback(self.callback)

    def register_sensor(self, sensor_type: str, sensor_name: str):
        self.sensor = self.printer.lookup_object(f"{sensor_type} {sensor_name}", None)
        if not self.sensor:
            raise self.printer.config_error(
                f"Unknown Sensor {sensor_type} {sensor_name}"
            )

    def set_check_interval(self, interval: float) -> None:
        self.check_interval = max(0.1, interval)

    def register_callback(
        self, callback: typing.Callable[..., None], trigger: bool
    ) -> None:
        self.trigger_state = trigger
        self.callback = callback

    def verify_sensor(self, eventtime: float) -> float:
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
        self.last_check_time = eventtime
        return eventtime + self.check_interval

    def _handle_trigger(self):
        completion = self.reactor.register_callback(self.callback)
        self.min_event_systime = self.reactor.monotonic() + self.event_delay
        return completion.wait()

    def active(self) -> bool:
        return self.is_enabled


class ExtruderMotions:
    def __init__(self, config, extruder, name) -> None:
        self.printer = config.get_printer()
        self.name = name
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.pheaters = self.extruder_heater = None
        self.travel_speed = config.getfloat(
            "travel_speed", default=100.0, minval=50.0, maxval=500.0
        )
        self.extruder_speed = config.getfloat(
            "extruder_speed", default=10.0, minval=2.0, maxval=50.0
        )
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.extruder = extruder
        self.old_extruder = None
        self.toolhead = None

    def handle_connect(self) -> None:
        self.toolhead = self.printer.lookup_object("toolhead")

    def handle_ready(self) -> None:
        if self.toolhead:
            self.old_extruder = self.toolhead.get_extruder()

    def _force_activate(self) -> None:
        if not self.toolhead or not self.extruder:
            return
        if self.extruder != self.old_extruder:
            self.old_extruder = self.toolhead.get_extruder()
            self.toolhead.flush_step_generation()
            last_position = self.extruder.extruder_stepper.find_past_position(
                self.reactor.monotonic()
            )
            self.toolhead.set_extruder(self.extruder, last_position)
            self.printer.send_event("extruder:activate_extruder")
            logging.info("Activated extruder %s", str(self.extruder.get_name()))

    def heat(self, temp: int, threshold: float = 0.1, wait: bool = False) -> None:
        if not self.extruder:
            return
        eventtime = self.reactor.monotonic()
        heater = self.extruder.get_heater()
        if (
            (temp * (1 - threshold))
            <= heater.get_temp(eventtime)[0]
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

    def move(
        self, distance: float = 10.0, speed: float = 10.0, wait: bool = True
    ) -> None:
        if not self.extruder or not self.toolhead:
            return
        extruder_heater = self.extruder.get_heater()
        if not extruder_heater.can_extrude:
            raise self.printer.command_error("Extruder below minimum temperature")
        self._force_activate()
        eventtime = self.reactor.monotonic()
        force_move = self.printer.lookup_object("force_move")
        mcu = self.printer.lookup_object("mcu")
        est_print_time = mcu.estimated_print_time(eventtime)
        prev_position = self.extruder.find_past_position(est_print_time)
        npos = prev_position + distance
        force_move.manual_move(
            self.extruder.extruder_stepper.stepper,
            npos,
            distance,
            speed,
        )
        if wait:
            self.toolhead.wait_moves()


class FilamentMotions:
    def __init__(self, config, name, extruder, manager):
        self.config = config
        self.printer = config.get_printer()
        self.name = name
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.manager = manager
        self.toolhead = None
        self.bucket = None
        self.extruder_motion = ExtruderMotions(config, extruder, name)
        self.sensor_checkers: dict[str, SensorChecker] = {}
        self.state: FilamentStates = FilamentStates.UNKNOWN
        self.timeout = config.getint("timeout", default=30, minval=10, maxval=1000)
        self.load_timeout = config.getint(
            "load_timeout", default=self.timeout, minval=10, maxval=1000
        )
        self.unload_timeout = config.getint(
            "unload_timeout", default=self.timeout, minval=10, maxval=1000
        )
        self.sensors = config.getlist("sensors", None)
        self.load_speed = config.getfloat(
            "load_speed", default=10.0, minval=1.0, maxval=50.0
        )
        self.unload_speed = config.getfloat(
            "unload_speed", default=10.0, minval=1.0, maxval=50.0
        )
        self.purge_count = config.getint("purge_count", default=3, minval=0, maxval=20)
        self.purge_length = config.getfloat(
            "purge_length", default=5.0, minval=0.5, maxval=50.0
        )
        self.purge_speed = config.getfloat(
            "purge_speed", default=5.0, minval=1.0, maxval=50.0
        )
        self.purge_interval = config.getfloat(
            "purge_interval", default=2.0, minval=0.5, maxval=10.0
        )
        self.extrude_count = 0
        self.current_purge_index = 0
        self.operation_temp = DEFAULT_TEMPERATURE
        self.save_variables = config.getboolean("save_variables", True)

        self.unextrude_timer = self.reactor.register_timer(
            self._unextrude, self.reactor.NEVER
        )
        self.extrude_timer = self.reactor.register_timer(
            self._extrude, self.reactor.NEVER
        )
        self.purge_timer = self.reactor.register_timer(self._purge, self.reactor.NEVER)

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self) -> None:
        self.toolhead = self.printer.lookup_object("toolhead")
        self.bucket = self.printer.lookup_object("bucket", None)
        self._load_saved_state()
        self._register_sensors()

    def handle_ready(self) -> None:
        pass

    def _load_saved_state(self) -> None:
        if not self.save_variables:
            return
        try:
            save_vars = self.printer.lookup_object("save_variables")
            if save_vars:
                variables = save_vars.get_status(0).get("variables", {})
                saved_state = variables.get(f"filament_state_{self.name}")
                if saved_state:
                    try:
                        self.state = FilamentStates(saved_state)
                        logging.info(
                            f"[FilamentManager] Loaded saved state for {self.name}: {self.state.value}"
                        )
                    except ValueError:
                        logging.warning(
                            f"[FilamentManager] Invalid saved state for {self.name}: {saved_state}"
                        )
        except Exception as e:
            logging.debug(f"[FilamentManager] Could not load saved state: {e}")

    def _save_state(self) -> None:
        if not self.save_variables:
            return
        try:
            self.gcode.run_script_from_command(
                f'SAVE_VARIABLE VARIABLE=filament_state_{self.name} VALUE="{self.state.value}"'
            )
        except Exception as e:
            logging.warning(f"[FilamentManager] Could not save state: {e}")

    def _register_sensors(self) -> None:
        if not self.sensors:
            return
        for sensor_name in self.sensors:
            sensor_name = sensor_name.strip()
            if not sensor_name:
                continue
            for sensor_type in [
                "filament_switch_sensor",
                "filament_motion_sensor",
                "cutter_sensor",
            ]:
                sensor = self.printer.lookup_object(
                    f"{sensor_type} {sensor_name}", None
                )
                if sensor:
                    logging.info(
                        f"[FilamentManager] Registered {sensor_type} '{sensor_name}' for {self.name}"
                    )
                    break

    def _get_temperature(self, filament_type: str) -> int:
        return FILAMENT_TEMPERATURES.get(filament_type.upper(), DEFAULT_TEMPERATURE)

    def _enable_sensors(self) -> None:
        for checker in self.sensor_checkers.values():
            checker.toggle_check()

    def _disable_sensors(self) -> None:
        for checker in self.sensor_checkers.values():
            if checker.active():
                checker.toggle_check()

    def _unextrude(self, eventtime: float) -> float:
        if self.extrude_count >= self.unload_timeout:
            self._cleanup_unload(error=False)
            return self.reactor.NEVER
        self.extrude_count += 1
        self.extruder_motion.move(-10, self.unload_speed, wait=False)
        return eventtime + float(10 / self.unload_speed)

    def _extrude(self, eventtime: float) -> float:
        if self.extrude_count >= self.load_timeout:
            self.reactor.update_timer(self.extrude_timer, self.reactor.NEVER)
            self._start_purge()
            return self.reactor.NEVER
        self.extrude_count += 1
        self.extruder_motion.move(10, self.load_speed, wait=False)
        return eventtime + float(10 / self.load_speed)

    def _purge(self, eventtime: float) -> float:
        if self.current_purge_index >= self.purge_count:
            self._cleanup_load(error=False)
            return self.reactor.NEVER
        self.current_purge_index += 1
        self.extruder_motion.move(self.purge_length, self.purge_speed, wait=True)
        self.gcode.respond_info(
            f"[FilamentManager] Purge {self.current_purge_index}/{self.purge_count}"
        )
        return eventtime + self.purge_interval

    def _start_purge(self) -> None:
        if self.purge_count <= 0:
            self._cleanup_load(error=False)
            return
        if self.bucket:
            self.bucket.move_to_bucket()
            if self.toolhead:
                self.toolhead.wait_moves()
        self.current_purge_index = 0
        self.reactor.update_timer(self.purge_timer, self.reactor.NOW)

    def _cleanup_load(self, error: bool = False) -> None:
        self.reactor.update_timer(self.extrude_timer, self.reactor.NEVER)
        self.reactor.update_timer(self.purge_timer, self.reactor.NEVER)
        self._disable_sensors()
        if error:
            self.state = FilamentStates.UNKNOWN
            self.printer.send_event("filament_manager:error")
        else:
            self.state = FilamentStates.LOADED
            self.printer.send_event("filament_manager:loaded")
        self._save_state()
        self.gcode.respond_info(
            f"[FilamentManager] Load {'failed' if error else 'complete'} for {self.name}"
        )

    def _cleanup_unload(self, error: bool = False) -> None:
        self.reactor.update_timer(self.unextrude_timer, self.reactor.NEVER)
        self._disable_sensors()
        if error:
            self.state = FilamentStates.UNKNOWN
            self.printer.send_event("filament_manager:error")
        else:
            self.state = FilamentStates.UNLOADED
            self.printer.send_event("filament_manager:unloaded")
        self._save_state()
        self.gcode.respond_info(
            f"[FilamentManager] Unload {'failed' if error else 'complete'} for {self.name}"
        )

    def _handle_load_sensor_trigger(self) -> None:
        if self.state != FilamentStates.LOADING:
            return
        self.reactor.update_timer(self.extrude_timer, self.reactor.NEVER)
        self._start_purge()

    def _handle_unload_sensor_trigger(self) -> None:
        if self.state != FilamentStates.UNLOADING:
            return
        self.reactor.update_timer(self.unextrude_timer, self.reactor.NEVER)
        self._cleanup_unload(error=False)

    def load(self, filament_type: str = "PLA") -> None:
        if self.state not in [FilamentStates.UNLOADED, FilamentStates.UNKNOWN]:
            raise self.printer.command_error(
                f"Cannot load filament on {self.name}: current state is {self.state.value}"
            )
        if not self.toolhead:
            raise self.printer.command_error(f"Toolhead not available for {self.name}")

        self.state = FilamentStates.LOADING
        self.printer.send_event("filament_manager:loading")
        self.extrude_count = 0
        self.operation_temp = self._get_temperature(filament_type)

        self._disable_sensors()
        self.sensor_checkers.clear()

        for sensor_name in self.sensors or []:
            checker = SensorChecker(self.config)
            try:
                checker.register_sensor("filament_switch_sensor", sensor_name)
                checker.set_check_interval(1.0)
                checker.register_callback(self._handle_load_sensor_trigger, True)
                self.sensor_checkers[f"load_{sensor_name}"] = checker
            except Exception:
                try:
                    checker.register_sensor("filament_motion_sensor", sensor_name)
                    checker.set_check_interval(1.0)
                    checker.register_callback(self._handle_load_sensor_trigger, True)
                    self.sensor_checkers[f"load_{sensor_name}"] = checker
                except Exception:
                    try:
                        checker.register_sensor("cutter_sensor", sensor_name)
                        checker.set_check_interval(1.0)
                        checker.register_callback(
                            self._handle_load_sensor_trigger, True
                        )
                        self.sensor_checkers[f"load_{sensor_name}"] = checker
                    except Exception as e:
                        logging.warning(
                            f"[FilamentManager] Could not register sensor {sensor_name}: {e}"
                        )

        self.extruder_motion.heat(self.operation_temp, wait=True)

        self._enable_sensors()
        self.reactor.update_timer(self.extrude_timer, self.reactor.NOW)
        self.gcode.respond_info(
            f"[FilamentManager] Loading {filament_type} at {self.operation_temp}C on {self.name}"
        )

    def unload(self) -> None:
        if self.state not in [FilamentStates.LOADED, FilamentStates.UNKNOWN]:
            raise self.printer.command_error(
                f"Cannot unload filament on {self.name}: current state is {self.state.value}"
            )
        if not self.toolhead:
            raise self.printer.command_error(f"Toolhead not available for {self.name}")

        self.state = FilamentStates.UNLOADING
        self.printer.send_event("filament_manager:unloading")
        self.extrude_count = 0

        self._disable_sensors()
        self.sensor_checkers.clear()

        for sensor_name in self.sensors or []:
            checker = SensorChecker(self.config)
            try:
                checker.register_sensor("filament_switch_sensor", sensor_name)
                checker.set_check_interval(1.0)
                checker.register_callback(self._handle_unload_sensor_trigger, False)
                self.sensor_checkers[f"unload_{sensor_name}"] = checker
            except Exception:
                try:
                    checker.register_sensor("filament_motion_sensor", sensor_name)
                    checker.set_check_interval(1.0)
                    checker.register_callback(self._handle_unload_sensor_trigger, False)
                    self.sensor_checkers[f"unload_{sensor_name}"] = checker
                except Exception:
                    try:
                        checker.register_sensor("cutter_sensor", sensor_name)
                        checker.set_check_interval(1.0)
                        checker.register_callback(
                            self._handle_unload_sensor_trigger, False
                        )
                        self.sensor_checkers[f"unload_{sensor_name}"] = checker
                    except Exception as e:
                        logging.warning(
                            f"[FilamentManager] Could not register sensor {sensor_name}: {e}"
                        )

        self.extruder_motion.heat(self.operation_temp, wait=True)

        self._enable_sensors()
        self.reactor.update_timer(self.unextrude_timer, self.reactor.NEVER)
        self.gcode.respond_info(f"[FilamentManager] Unloading {self.name}")

    def get_status(self, eventtime: float) -> dict:
        return {
            "state": self.state.value,
            "extruder": self.name,
            "loading": self.state == FilamentStates.LOADING,
            "unloading": self.state == FilamentStates.UNLOADING,
        }


class FilamentManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = None
        self.toolhead = self.extruder_objects = self.bucket = None
        self.custom_boundary = None
        self.config = config
        self.motions: dict[str, FilamentMotions] = {}
        self.default_timeout = config.getint(
            "default_timeout", default=30, minval=10, maxval=1000
        )
        self.purge_count = config.getint("purge_count", default=3, minval=0, maxval=20)
        self.purge_length = config.getfloat(
            "purge_length", default=5.0, minval=0.5, maxval=50.0
        )
        self.purge_speed = config.getfloat(
            "purge_speed", default=5.0, minval=1.0, maxval=50.0
        )
        self.load_speed = config.getfloat(
            "load_speed", default=10.0, minval=1.0, maxval=50.0
        )
        self.unload_speed = config.getfloat(
            "unload_speed", default=10.0, minval=1.0, maxval=50.0
        )
        self.travel_speed = config.getfloat(
            "travel_speed", default=100.0, minval=20.0, maxval=1000.0
        )

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self) -> None:
        self.toolhead = self.printer.lookup_object("toolhead")

    def handle_ready(self) -> None:
        self.gcode = self.printer.lookup_object("gcode")
        self.extruder_objects = self.printer.lookup_objects("extruder")

        for name, extruder in self.extruder_objects:
            try:
                extruder_config = self.config.getsection(f"filament_manager {name}")
            except Exception:
                extruder_config = self.config
            self.motions[name] = FilamentMotions(extruder_config, name, extruder, self)
            self.gcode.respond_info(f"[FilamentManager] Registered extruder: {name}")

        self.gcode.register_mux_command(
            "LOAD_FILAMENT",
            "EXTRUDER",
            None,
            self.cmd_LOAD_FILAMENT,
            desc="Load filament into extruder",
        )
        self.gcode.register_mux_command(
            "UNLOAD_FILAMENT",
            "EXTRUDER",
            None,
            self.cmd_UNLOAD_FILAMENT,
            desc="Unload filament from extruder",
        )
        self.gcode.register_command(
            "QUERY_FILAMENT",
            self.cmd_QUERY_FILAMENT,
            desc="Query filament manager status",
        )

    def _get_motion(self, extruder_name: str) -> FilamentMotions:
        if extruder_name not in self.motions:
            raise self.printer.command_error(f"Unknown extruder: {extruder_name}")
        return self.motions[extruder_name]

    def cmd_LOAD_FILAMENT(self, gcmd) -> None:
        extruder_name = gcmd.get("EXTRUDER")
        filament_type = gcmd.get("FILAMENT", "PLA")

        if not extruder_name and self.toolhead:
            extruder_name = self.toolhead.get_extruder().get_name()

        if not extruder_name:
            raise self.printer.command_error("No extruder specified")

        motion = self._get_motion(extruder_name)

        self.reactor.register_callback(partial(motion.load, filament_type))

    def cmd_UNLOAD_FILAMENT(self, gcmd) -> None:
        extruder_name = gcmd.get("EXTRUDER")

        if not extruder_name and self.toolhead:
            extruder_name = self.toolhead.get_extruder().get_name()

        if not extruder_name:
            raise self.printer.command_error("No extruder specified")

        motion = self._get_motion(extruder_name)

        self.reactor.register_callback(motion.unload)

    def cmd_QUERY_FILAMENT(self, gcmd) -> None:
        extruder_name = gcmd.get("EXTRUDER")

        if extruder_name:
            motion = self._get_motion(extruder_name)
            status = motion.get_status(0)
            self.gcode.respond_info(f"Filament {status['extruder']}: {status['state']}")
        else:
            for name, motion in self.motions.items():
                status = motion.get_status(0)
                self.gcode.respond_info(
                    f"Filament {status['extruder']}: {status['state']}"
                )

    def get_status(self, eventtime: float) -> dict:
        return {
            "extruders": {
                name: m.get_status(eventtime) for name, m in self.motions.items()
            }
        }


def load_config(config):
    return FilamentManager(config)
