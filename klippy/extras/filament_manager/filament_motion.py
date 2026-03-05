import enum
import logging
import typing

from .sensor_checker import SensorChecker, SensorRole
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
        self.id: str = f"FM-{self.name}"
        self.state: FilamentStates = FilamentStates.UNKNOWN
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
            "extruder_speed", default=10.0, minval=2.0, maxval=30.0
        )
        self.extruder_accel: float = config.getfloat(
            "extruder_accel", default=50.0, minval=2.0
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
        self.move_timer = self.reactor.register_timer(
            self.timed_move, self.reactor.NEVER
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

    def handle_ready(self) -> None:
        """Handle klippy ready event"""
        self.min_event_systime = self.reactor.monotonic() + 2.0

    def cmd_MOVE(self, gcmd) -> None:
        if not self.emotion:
            raise gcmd.error("No extruder associated unable to move")
        return

    def _build_sensor_waiting_list(self) -> list[SensorChecker]:
        ordered_sensors: list[SensorChecker] = []
        for key in self.configured_sensors.keys():
            # TODO: build here a list that containes the order of
            # activating and deactivating sensors
            # pre_gate -> <extruder_movemnt> -> post_gear -> gate -> encoder if needed -> sync_feedback -> toolhead -> <extruder> -> post-e-gears extruder
            pass
        return ordered_sensors

    def handle_pre_gate_checker(self, trigger, eventtime) -> None:
        self.can_move = False
        if trigger:
            return

        pass

    def handle_post_gear_checker(self, trigger, eventtime) -> None:
        self.can_move = False
        # if trigger:
        #     # Then start movement until the next sensor triggers
        #     # Stop movement there and let the other sensor handle it
        #     return
        # toolhead = self.printer.lookup_object("toolhead")
        # toolhead.flush_step_generation()
        # toolhead.dwell(1)

        self.emotion.move(500, 100, 15, self.aux_extruder)

    def handle_toolhead_checker(self, trigger, eventtime) -> None:
        self.can_move = False
        if trigger:
            return
        pass

    def handle_extruder_checker(self, trigger, eventtime) -> None:
        self.can_move = False
        if trigger:
            if "toolhead" in self.configured_sensors.keys():
                # Now make the extruder move in a positive direction
                # until the next sensor is hit
                pass
            return
        pass
        # Make the same negative movements as the positive ones

    def timed_move(self, eventtime):
        if self.can_move:
            direction_multiplier = 1 if self.direction == "positive" else -1
            # TODO: Add verification for timeout before the emotion movements
            self.emotion.move(
                distance=5 * direction_multiplier,
                speed=100,
                acceleration=50,
            )
            return eventtime + float(5 / speed)
        return self.reactor.NEVER
        pass

    def _stop_timed_move(self) -> None:
        self.reactor.update_timer(self.move_timer, self.reactor.NEVER)
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.flush_step_generation()

    def run_motion(self) -> None:
        with self._mutex:
            if not self.emotion:
                raise gcmd.error("No extruder associated unable to move")
            self._init_motion()
            self._end_motion()

    def _init_motion(self) -> None:
        self.printer.send_event(f"motion-{self.name}:start")

    def _end_motion(self) -> None:
        self.printer.send_event(f"motion-{self.name}:end")

    def cmd_RUN_MOTION(self, gcmd) -> None:
        self.run_motion()


def load_config(config):
    return FilamentMotion(config)
