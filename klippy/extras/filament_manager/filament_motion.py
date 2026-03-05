import enum
import logging
import typing

from .sensor_checker import SensorChecker, SensorRole, PRIORITY
from .extruder_motions import ExtruderMotion

SAVE_VARS_REQUIREMENT_MSG = """
    Filament motions requires [save_variables] object 
    declaration. Please create the variables file and
    configure the corresponding object
"""


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


class MotionState(enum.Enum):
    IDLE = "idle"
    ARMED = "armed"
    MOVING = "moving"
    SENSOR_CONTROL = "sensor_control"
    FILAMENT_ENDED = "filament_ended"
    ERROR = "error"


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
    # _mutex = threading.Lock()  # Only one motion can run at a time
    _mutex: typing.Any

    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.name: str = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self._mutex = self.reactor.mutex()  # Get the mutex from the reactor
        self.min_event_systime = self.reactor.NEVER

        self.state: FilamentStates = FilamentStates.UNKNOWN
        self.motion_state: MotionState = MotionState.IDLE
        self.motion_mutex = self.reactor.mutex()
        self.active_sensor: str = ""
        self.start_sensor: str = ""
        self.configured_sensors: dict[str, SensorChecker] = {}
        self._used_sensors: set[tuple[str, str]] = set()
        self.ordered_sensor_paths: list[SensorChecker] = []
        self.can_move = False
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
        self.require_purge: bool = config.getboolean("require_purge", default=False)
        self.timeout: float = config.getfloat("timeout", default=40.0, minval=5.0)
        self.timeout_type: str = config.getchoice(
            "timeout_type",
            default=None,
            choices=["time", "distance"],
            note_valid=True,
        )
        self.direction: str = config.getchoice(
            "direction",
            default=None,
            choices=["positive", "negative"],
            note_valid=True,
        )
        self.extruder_speed: float = config.getfloat(
            "extruder_speed", default=100.0, minval=2.0, maxval=300.0
        )
        self.extruder_accel: float = config.getfloat(
            "extruder_accel", default=50.0, minval=2.0
        )
        self.travel_speed: float = config.getfloat(
            "travel_speed", default=50.0, minval=30.0, maxval=500.0
        )
        # Configure available sensor configurations
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
        self.move_timer = self.reactor.register_timer(
            self.timed_move, self.reactor.NEVER
        )
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        ## Register GCODE commands
        gcode = self.printer.lookup_object("gcode")

    def handle_connect(self) -> None:
        """Handle klippy connect event
        Fetch necessary objects declared at configuration
        """
        if not self.direction:
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
        # Configure callbacks for each sensor role
        for role, checker in self.configured_sensors.items():
            _mname = f"handle_{role}_checker"
            if hasattr(self, _mname):
                _callback = getattr(self, _mname)
                if callable(_callback):
                    checker.register_callback(
                        _callback, True if self.direction == "positive" else False
                    )
                    checker.toggle_check()
        # Order sensors
        self.ordered_sensor_paths = self._build_sensor_path_ord()

    def handle_ready(self) -> None:
        """Handle klippy ready event"""
        self.min_event_systime = self.reactor.monotonic() + 2.0

    def _build_sensor_path_ord(self) -> list[SensorChecker]:
        ordered: list[tuple[int, SensorChecker]] = []
        for role, checker in self.configured_sensors.items():
            priority = PRIORITY.get(role, 99)
            ordered.append((priority, checker))
        ordered.sort(key=lambda x: x[0])
        result: list[SensorChecker] = [checker for _, checker in ordered]
        if self.direction == "negative":
            result.reverse()
        return result

    def timed_move(self, eventtime):
        """Executes timed movements,

        Can be stopped by stopping calling `self.stop_motion`, or
        explicitly stopping the associated timer and locking `self.motion_mutex`
        """
        with self.motion_mutex:
            if not self.emotion:
                return self.reactor.NEVER
            direction_multiplier = 1 if self.direction == "positive" else -1
            self.emotion.move(
                distance=5 * direction_multiplier,
                speed=self.extruder_speed,
            )
            return float(eventtime + float(5.0 / self.extruder_speed))

    def _stop_timed_move(self) -> None:
        self.motion_state = MotionState.IDLE
        self.motion_mutex.lock()  # Prevents triggering motions
        self.reactor.update_timer(self.move_timer, self.reactor.NEVER)
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()

    def handle_sensor_trigger(self, trigger, eventtime) -> None:
        """Handles generic sensor triggers"""
        pass

    def handle_pre_gate_checker(self, trigger, eventtime) -> None:
        """Handles `pre gate` sensor triggers"""
        pass

    def handle_post_gear_checker(self, trigger, eventtime) -> None:
        """Handles `post_gear` sensor triggers"""
        self.reactor.update_timer(self.move_timer, self.reactor.NOW)
        self.start_sensor = "post_gear"

    def handle_toolhead_checker(self, trigger, eventtime) -> None:
        """Handles `toolhead` sensor triggers"""
        pass

    def handle_extruder_checker(self, trigger, eventtime) -> None:
        """Handles `extruder` sensor triggers"""
        if self.active_sensor != "extruder":
            self.active_sensor = "extruder"
        self._stop_timed_move()

    def start_motion(self) -> None:
        """Start filament motion"""
        self.motion_state = MotionState.ARMED

    def stop_motion(self) -> None:
        """Stop active filament motion"""
        self.motion_state = MotionState.IDLE
        self._stop_timed_move()

    def calculate_sensor_dists(self) -> None: ...


def load_config(config):
    return FilamentMotion(config)
