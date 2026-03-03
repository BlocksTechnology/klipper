import logging
import enum
import typing


SAVE_VARS_REQUIREMENT_MSG = """
    Filament motions requires [save_variables] object 
    declaration. Please create the variables file and
    configure the corresponding object
"""


class SensorRole:
    PRE_GATE = "pre_gate"
    POST_GEAR = "post_gear"
    POST_GATE = "post_gate"
    GATE = "gate"
    SYNC_FEEDBACK = "sync_feedback"
    TOOLHEAD = "toolhead"
    EXTRUDER = "extruder"

    @classmethod
    def exists(cls, value) -> bool:
        return hasattr(cls, value.strip().upper())


class FilamentMotionsError(Exception):
    """Class for filament load and unload errors"""

    def __init__(self, message, errors: list[str] | None = None) -> None:
        super().__init__(message)
        self.errors: list[str] | None = errors


class Motion(enum.Enum):
    LOAD = "load"
    UNLOAD = "unload"

    @classmethod
    def __missing__(cls, value: str):
        value: str = value.strip().lower()
        for member in cls:
            if member.value == value:
                return member
        return None

    @classmethod
    def exists(cls, value: str) -> bool:
        """Simple checker for values"""
        return value.strip().lower() in [m.value for m in cls]


MotionType: typing.TypeAlias = typing.Annotated[
    Motion, "Defines the available type of movements"
]

KlippyObj: typing.TypeVar = typing.TypeVar("KlippyObj")


class FilamentStates(enum.Enum):
    LOADING = "loading"
    LOADED = "loaded"
    UNLOADED = "unloaded"
    UNLOADING = "unloading"
    UNKNOWN = "unknown"

    @classmethod
    def __missing__(cls, value):
        value = value.strip().lower()
        for member in cls:
            if member.value == value:
                return member
        return None

    @classmethod
    def exists(cls, value: str) -> bool:
        """Simple checker for values"""
        return value.strip().lower() in [m.value for m in cls]


_Klippyobj = typing.TypeVar("_Klippyobj")


class FilamentMotion:
    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.name: str = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.min_event_systime = self.reactor.NEVER
        self.id: str = f"FM-{self.name}"
        self.state: FilamentStates = FilamentStates.UNKNOWN
        self.debug: int = config.getint("debug", default=0)
        self.bucket = None
        self.bucket_name: str = config.get("bucket", None)
        self.extruder_name: str = config.get("extruder", None)
        self.extruder = None
        self.emotion = None
        self.aux_extruder_name: str = config.get("aux_extruder", None)
        self.aux_extruder = None
        self.aux_emotion = None
        self.require_heating: bool = config.getboolean("require_heating", default=False)
        self.timeout: float = config.getfloat("timeout", default=40.0, minval=5.0)
        self.timeout_type: str = config.getchoice(
            "timeout_type",
            default=None,
            choices=["time", "distance"],
            note_valid=True,
        )
        self.directions: str = config.getchoice(
            "direction",
            default=None,
            choices=["positive", "negative"],
            note_valid=True,
        )
        self.extruder_speed: float = config.getfloat(
            "extruder_speed", default=10.0, minval=2.0, maxval=30.0
        )
        self.travel_speed: float = config.getfloat(
            "travel_speed", default=50.0, minval=30.0, maxval=500.0
        )
        # Available sensor configurations
        self.configured_sensors = {}
        self._used_sensors = set()
        prefix = "sensor_"
        for option in config.get_prefix_options(prefix):
            norm_option = option.strip().lower()
            norm_opt_name = norm_option.removeprefix(prefix)
            if not SensorRole.exists(norm_opt_name):
                raise config.error(f"Unknown config option {option}")
            try:
                opt_value: str = config.get(option)
                sensor_role: str = option[len(prefix) :]
                splitted_norm: list[str] = opt_value.strip().split(" ")
                if len(splitted_norm) <= 1:
                    raise config.error(
                        f"Specified sensor {sensor_role} requires definition"
                        ' of "<sensor_object> <sensor_name>"'
                    )
                sensor_type: str = splitted_norm[0]
                sensor_name: str = splitted_norm[1]
                sensor_key = (sensor_type, sensor_name)
                if sensor_key in self._used_sensors:
                    raise config.error(
                        "Duplicate sensor declaration: %s, sensor already used"
                        % (str(sensor_key),)
                    )
                if sensor_role in self.configured_sensors.keys():
                    raise config.error(
                        "Duplicate sensor role declaration: %s" % (str(sensor_role),)
                    )
                self._used_sensors.add(sensor_key)
                sc = SensorChecker(
                    printer=self.printer,
                    name=sensor_name,
                    sensor_role=sensor_role,
                    sensor_type=sensor_type,
                )
                self.configured_sensors[sensor_role] = sc
            except (SyntaxError, TypeError, ValueError) as e:
                raise config.error(
                    "Option '%s' in section '%s' is not valid literal: %s"
                    % (option, config.get_name(), e)
                )

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        ## Register GCODE commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "MOVE",
            self.cmd_MOVE,
            "move the extruder with acceleration",
        )
        gcode.register_mux_command(
            "RUN_MOTION",
            "NAME",
            self.name,
            self.cmd_RUN_MOTION,
            "Gcode for running individual motions",
        )

    def cmd_MOVE(self, gcmd) -> None:
        if not self.emotion:
            raise gcmd.error("No extruder associated unable to move")
        speed = gcmd.get("SPEED", parser=int)
        distance = gcmd.get("DIST", parser=int)
        accel = gcmd.get("ACCEL", parser=int)
        gcmd.respond_info(f"Moving extruder {speed}, {accel} and {distance}")
        self.emotion.move(distance, speed, accel)

    def handle_connect(self) -> None:
        """Handle klippy connect event
        Fetch necessary objects declared at configuration
        """
        if not self.directions:
            raise self.printer.config_error(
                "Option 'direction' is mandatory. Configure to 'positive' or 'negative'"
            )
        if len(self.configured_sensors.keys()) <= 0 and not self.timeout_type:
            raise self.printer.config_error(
                "No gate sensors are configured. Either specify gate sensors or timeouts"
            )
        if not self.extruder_name:
            raise self.printer.config_error(
                "Filament motion requires extruder specification."
            )
        if self.bucket_name:
            self.bucket: KlippyObj = self.printer.lookup_object(  # pyright: ignore[reportInvalidTypeForm]
                f"bucket {self.bucket_name}", None
            )
            if not self.bucket:
                raise self.printer.config_error(
                    f"Configured Bucket {self.name} does not exist."
                )

        self.extruder = self.printer.lookup_object(self.extruder_name)
        if not self.extruder:
            raise self.printer.config_error(
                "Invalid extruder specification. Unable to load."
            )
        self.emotion = ExtruderMotion(self.printer, self.extruder, "main extruder")

        if self.aux_extruder_name:
            self.aux_extruder = self.printer.lookup_object(
                f"extruder_stepper {self.aux_extruder_name}", None
            )
            if not self.aux_extruder:
                raise self.printer.config_error("Invalid auxiliar extruder specified.")
            self.aux_emotion = ExtruderMotion(
                self.printer, self.aux_extruder, "aux extruder"
            )

    def handle_ready(self) -> None:
        """Handle klippy ready event"""
        self.min_event_systime = self.reactor.monotonic() + 2.0

    def run_motion(self) -> None:
        self._init_motion()

        self._end_motion()

    def _init_motion(self) -> None:
        self.printer.send_event(f"motion-{self.name}:start")

    def _end_motion(self) -> None:
        self.printer.send_event(f"motion-{self.name}:end")

    def cmd_RUN_MOTION(self, gcmd) -> None:
        self.run_motion()


class SensorChecker:
    def __init__(self, printer, name, sensor_role, sensor_type) -> None:
        self.printer = printer
        self.sensor_type = sensor_type
        self._name = name
        self.role = sensor_role
        self.reactor = self.printer.get_reactor()
        self.is_enabled: bool = False
        self.sensor = None
        self.callback: typing.Callable[[typing.Any], typing.Any] | None = None
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
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self) -> None:
        """Handle connect event"""
        self.register_sensor(self.sensor_type, self.name)

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
        if not self.sensor:
            raise self.printer.config_error(
                f"Unknown Sensor {sensor_type} {sensor_name}"
            )

    @property
    def name(self) -> str:
        return self._name

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
        """Periodically check of sensor state"""
        # TEST:* Use the "pins" module to configure a pin on a micro-controller. This
        # is typically done with something similar to
        # `printer.lookup_object("pins").setup_pin("pwm",
        # config.get("my_pin"))`. The returned object can then be commanded at
        # run-time.
        if not self.is_enabled:
            return self.reactor.NEVER
        if not self.sensor:
            self.printer.command_error(f"Sensor check failed")
            return self.reactor.NEVER
        status = self.sensor.get_status(eventtime)
        filament_present = status.get("filament_detected")
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


def load_config_prefix(config):
    return FilamentMotion(config)
