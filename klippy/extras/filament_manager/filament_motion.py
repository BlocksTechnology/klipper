import json
import logging
import ast
import enum
import typing

# from .extruder_motions import ExtruderMotion
# from .sensor_checker import SensorChecker, SensorRole

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
        self.aux_extruder_name: str = config.get("aux_extruder", None)
        self.aux_extruder = None

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
        self.sensors = {}
        prefix = "sensor_"
        for option in config.get_prefix_options(prefix):
            norm_option = option.strip().lower()
            logging.info(norm_option)
            if SensorRole.exists(norm_option):
                raise config.error(f"Unknown config option {option}")
            try:
                logging.info(option)
                opt = config.get(option)
                logging.info("Fetched option %s" % (opt,))
                # literal = ast.literal_eval(config.get(option))

                json.dumps(literal, separators=(",", ":"))
                if literal in self.configured_sensors.keys():
                    raise config.error(
                        "Option %s defined multiple times." % (literal.split(":")[0])
                    )
                logging.info(literal)
                logging.info(option)
                print(literal)
                print(option)
                self.configured_sensors[option[len(prefix) :]] = literal
            except (SyntaxError, TypeError, ValueError) as e:
                raise config.error(
                    "Option '%s' in section '%s' is not valid literal: %s"
                    % (option, config.get_name(), e)
                )

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self) -> None:
        """Handle klippy connect event

        Fetch necessary objects declared at configuration
        """
        if not self.directions:
            raise self.printer.config_error(
                "Option 'direction' is mandatory. Configure to 'positive' or 'negative'"
            )
        if not self.gate_sensor_name and not self.timeout_type:
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

        # Fetch sensors

    def handle_ready(self) -> None:
        self.min_event_systime = self.reactor.monotonic() + 2.0


def load_config_prefix(config):
    return FilamentMotion(config)
