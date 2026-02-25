import logging
import enum
import typing

from .extruder_motions import ExtruderMotion
from .sensor_checker import SensorChecker

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
    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.name: str = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.id: str = f"FM-{self.name}"
        self.state: FilamentStates = FilamentStates.UNKNOWN
        self.debug: int = config.getint("debug", default=0)
        self.bucket = None
        self.bucket_name: str = config.get("bucket", None)
        self.aux_extruder_name: str = config.get("aux_extruder", None)
        self.aux_extruder = None
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
        self.pre_gate_sensor = None
        self.pre_gate_sensor_name: str = config.get("pre_gate_sensor", default=None)

        self.post_gear_sensor = None
        self.post_gear_sensor_name: str = config.get("post_gear_sensor", default=None)

        self.post_gate_sensor = None
        self.post_gate_sensor_name: str = config.get("post_gate_sensor", default=None)

        self.end_gate_sensor = None
        self.end_gate_sensor_name: str = config.get("end_gate_sensor", default=None)

        self.sync_feedback_sensor = None
        self.sync_feedback_sensor_name: str = config.get(
            "sync_feedback_sensor", default=None
        )

        self.gate_sensor = None
        self.gate_sensor_name: str = config.get("gate_sensor", default=None)

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
                "No gate sensors are configured. Either configure gate sensors or timeouts"
            )
        if self.bucket_name:
            self.bucket: KlippyObj = self.printer.lookup_object(
                f"bucket {self.bucket_name}", None
            )
            if not self.bucket:
                raise self.printer.config_error(
                    f"Configured Bucket {self.name} does not exist."
                )
        if self.aux_extruder_name:
            # TODO: aux extruder should also accept mmu gates
            self.aux_extruder: KlippyObj = self.printer.lookup_object(
                f"extruder_stepper {self.aux_extruder_name}", None
            )
            if not self.aux_extruder:
                raise self.printer.config_error(
                    f"Configured Auxiliary Extruder {self.aux_extruder_name} does not exist."
                )

        if self.pre_gate_sensor_name:
            self.pre_gate_sensor: KlippyObj = self.printer.lookup_object(
                self.pre_gate_sensor_name, default=None
            )
            if not self.pre_gate_sensor:
                raise self.printer.config_error(
                    f"Configured pre gate sensor {self.pre_gate_sensor_name} does not exist."
                )

        if self.post_gate_sensor_name:
            # NOTE: Can be a filament_switch_sensor, filament_motion_sensor or a belay
            self.post_gate_sensor: KlippyObj = self.printer.lookup_object(
                self.post_gate_sensor_name, default=None
            )
            if not self.post_gate_sensor:
                raise self.printer.config_error(
                    f"Configured post gate sensor {self.post_gate_sensor_name} does not exist."
                )

        if self.end_gate_sensor_name:
            self.end_gate_sensor: KlippyObj = self.printer.lookup_object(
                self.end_gate_sensor_name, default=None
            )
            if not self.end_gate_sensor:
                raise self.printer.config_error(
                    f"Configured end gate sensor {self.end_gate_sensor_name} does not exist"
                )

        if self.gate_sensor_name:
            # NOTE: Can be a filament_switch_sensor, filament_motion_sensor or a cutter_sensor
            self.gate_sensor: KlippyObj = self.printer.lookup_object(
                self.gate_sensor_name, default=None
            )
            if not self.gate_sensor:
                raise self.printer.config_error(
                    f"Configured gate sensor {self.end_gate_sensor_name} does not exist."
                )

    def handle_ready(self) -> None:
        """Handle klippy ready event"""
        pass

    def register_sensor_callback(
        self,
        sensor_type: str,
        sensor_name: str,
        callback: typing.Callable[[float], None],
        interval: float = 1.0,
        trigger: bool = False,
    ) -> SensorChecker:
        sensor_checker: SensorChecker = SensorChecker(self.printer)
        sensor_checker.register_sensor(sensor_type, sensor_name)
        sensor_checker.set_check_interval(interval)
        sensor_checker.register_callback(callback, trigger)
        return sensor_checker

    def note_filament_np(self, eventtime) -> None:
        pass

    def note_filament_p(self, eventtime) -> None:
        pass

    def motion_end(self) -> None:
        # Send event that the motino finished
        pass

    # def __call__(self, *args: Any, **kwds: Any) -> Any:
    #     pass
