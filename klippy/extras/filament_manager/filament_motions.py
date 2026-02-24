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


class FilamentMotion:
    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.control_sensors: dict[str, str] = {}
        self.sensor_checkers: list[SensorChecker] = []
        self.state: FilamentStates = FilamentStates.UNKNOWN
        self.cutter = config.g
        self.timeout = config.getfloat("timeout", default=300.0)
        self.timeout_type = config.getchoice(
            "timeout_type",
            choices=["time", "distance"],
            default="time",
            note_valid=True,
        )
        self.directions = config.getchoice(
            "direction",
            choices=["positive", "negative"],
            default="positive",
            note_valid=True,
        )
        self.extruder_speed = config.getfloat(
            "extruder_speed", default=10.0, minval=2.0, maxval=30.0
        )

        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_ready(self) -> None:
        """Handle klippy ready event"""
        pass

    def handle_connect(self) -> None:
        """Handle klippy connect event

        Fetch necessary objects declared at configuration
        """
        save_vars = self.printer.lookup_object("save_variables", None)
        if not save_vars:
            self.state = FilamentStates.UNKNOWN
            raise self.printer.config_error(SAVE_VARS_REQUIREMENT_MSG)
        sv_dict = save_vars.get_status(self.reactor.monotonic())
        variables: dict[str, typing.Any] = sv_dict.get("variables", None)
        if not variables:
            self.state = FilamentStates.UNKNOWN
        for sw_sensor in self.filament_sensors:
            self.sensor_checkers.append(
                self.register_sensor_callback(
                    sensor_type="filament_switch_sensor",
                    sensor_name=sw_sensor,
                    callback=self.note_filament_np,
                    interval=3,
                    trigger=True,
                )
            )

    def register_sensor_callback(
        self,
        sensor_type: str,
        sensor_name: str,
        callback: typing.Callable[[float], None],
        interval: float = 1.0,
        trigger: bool = False,
    ) -> SensorChecker:
        sensor_checker = SensorChecker(self.printer)
        sensor_checker.register_sensor(sensor_type, sensor_name)
        sensor_checker.set_check_interval(interval)
        sensor_checker.register_callback(callback, trigger)
        return sensor_checker

    def note_filament_np(self, eventtime) -> None: ...

    def note_filament_p(self, eventtime) -> None: ...

    def motion_end(self) -> None:
        # Send event that the motino finished
        pass

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        pass


class FilamentMotions:
    def __init__(self, config, name, extruder):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        self.identifier: str = "FT"
        self.debug: int = config.getint("debug", 0)
        self.bucket = config.get()
        self.cutter = None
        self.cutter_name: str = config.get("cutter", None)
        self.aux_extruder = None
        self.aux_extruder_name: str = config.get("aux_extruder", None)
        self.extrude_speed: float = config.getfloat(
            "extruder_speed", 10.0, minval=2.0, maxval=50.0
        )
        self.travel_speed: float = config.getfloat(
            "travel_speed", 50.0, minval=20.0, maxval=500.0
        )
        self.goto_bucket: bool = config.getboolean("goto_bucket", False)
        self.extrude_count: int = 0
        self.emotion: ExtruderMotion = ExtruderMotion(config, extruder, name)
        self.sensor_notes: dict[str, SensorChecker] = {}
        self.state: FilamentStates = FilamentStates.UNKNOWN
        self.timeout = config.getint("timeout", default=None, minval=10, maxval=1000)
        # self.unextrude_timer = self.reactor.register_timer(
        #     self._unextrude, self.reactor.NEVER
        # )
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        # Register motions sensors
        # sensor_check = SensorChecker(printer)
        # sensor_check.register_sensor(sensor_type, name)
        # sensor_check.set_check_interval(interval)
        # sensor_check.register_callback(callback, trigger)
        # self.sensor_notes.update({name: sensor_check})

    def handle_connect(self) -> None:
        # TODO : fetch here the last filament state saved on variables file
        save_vars = self.printer.lookup_object("save_variables", None)
        if not save_vars:
            self.state = FilamentStates.UNKNOWN
        # Normal toolhead filament states will be FT == FT0, FT1, ..., FTX
        sv_dict = save_vars.get_status(self.reactor.monotonic())
        variables: dict[str, typing.Any] = sv_dict.get("variables", None)
        if not variables:
            self.state = FilamentStates.UNKNOWN

        # TODO :  Make this fetch last staste logic here
        # variables.get()
        # index_c = 0
        # dict.get()
        #
        # for key, item in variables.items():
        #     if key ==
        #
        gcode = self.printer.lookup_object("gcode")
        self.bucket = self.printer.lookup_object("bucket", None)
        if self.cutter_name:
            if self.printer.lookup_object(f"cutter_sensor {self.cutter_name}", None):
                self.cutter = self.printer.lookup_object(
                    f"cutter_sensor {self.cutter_name}"
                )
            else:
                raise self.printer.config_error(
                    f"{self.cutter_name} is undefined, expected cutter_sensor object"
                )
        elif self.debug > 0:
            gcode.respond_info(
                f"Not using cutter_sensor for filament manager on extruder: {self.emotion.name}"
            )
        logging.info(
            f"Not using cutter_sensor for filament manager on extruder: {self.emotion.name}"
        )

        if self.aux_extruder_name:
            if self.printer.lookup_object(
                f"extruder_stepper {self.aux_extruder_name}", None
            ):
                self.aux_extruder = self.printer.lookup_object(
                    f"extruder_stepper {self.aux_extruder_name}"
                )
            else:
                raise self.printer.config_error(
                    f"{self.aux_extruder_name} is undefined, expected extruder_stepper object"
                )
        elif self.debug > 0:
            gcode.respond_info(f"Not using auxiliar extruder on filament manager")
            logging.info(f"Not using auxiliar extruder on filament manager")

    def handle_ready(self) -> None:
        """Handle `klippy:ready` event"""
        # XXX: Klipper recomends not raising errors here
        pass

    def _initialize_motion(
        self,
        eventtime: float | None,
        motion_type: MotionType,
        clean: bool = True,
    ) -> None:
        """Initialize filament motion motion"""
        # if motion_type.lower() not in Motion:
        #     raise self.printer.command_error("Motion type expected either load or unload")
        #
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info(f"initializing filament motion: {motion_type}")
        gcode_macro = self.printer.lookup_object("gcode_macro")
        toolhead = self.printer.lookup_object("toolhead")
        if clean:
            gcode.run_script_from_command("CLEAN_NOZZLE")

        if self.bucket and self.goto_bucket:
            gcode.respond_info(f"Moving toolhead FT{self.identifier}")
            self.bucket.move_to_bucket(split=False)

    def _deregister_sensor(self, name) -> None:
        """Deletes a sensor checker object by name"""
        sensor_checker: SensorChecker = self.sensor_notes.pop(name)
        del sensor_checker

    # def _unextrude(self, eventtime) -> float:
    #     if self.timeout:
    #         if self.extrude_count >= self.timeout:
    #             self.sensor_notes.get(
    #                 self.
    #             ).toggle_check()  # TEST: Stop the sensor checking
    #             completion = self.reactor.register_callback(
    #                 self._handle_unload_finish()
    #             )
    #             _ = completion.wait()
    #             return self.reactor.NEVER
    #         self.extrude_count += 1
    #     self.emotion.move(distance=-1.0, speed=self.speed)
    #     return float(eventtime + float(1.0 / self.speed))
    #
    # def _handle_unload_sensor_trigger(self, eventtime) -> None:
    #     if not self.sensors_check.get("unload-helper").active():
    #         return
    #     if self.state == FilamentStates.UNLOADING:
    #         self.sensors_check.get("unload-helper").trigger_check()

    def unload(self) -> None:
        gcode = self.printer.lookup_object("gcode")
        if self.state != FilamentStates.LOADED:
            raise gcode.error(
                f"Cannot unload \n Extruder {self.name} is currently loading or unloaded."
            )
        self.state = FilamentStates.UNLOADING
        self.extrude_count = 0
        if self.cutter:
            completion = self.reactor.register_callback(self.cutter.cut())
            completion.wait()
        # self.unextrude_timer.update(self.reactor.NOW)
        # self.sensors_check.get("unload-helper").trigger_check()
        #

    def _handle_unload_finish(self, eventtime=None) -> None:
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info("Unload end method")

    # def _extrude(self, eventtime) -> None:
    #     if self.timeout:
    #         if self.extrude_count >= self.timeout:
    #             self.sensors_check.get(self.extrude_control_sensor_name).trigger_check()
    #             completion = self.reactor.register_callback(self.extrude_end)
    #             return completion.wait()
    #         self.extrude_count += 1
    #     self.emotion.move(1, self.speed)
    #     return float(eventtime + float(1 / self.speed))
    #
    # def _handle_load_sensor_trigger(self, eventtime) -> None:
    #     if not self.sensors_check.get("load-helper").active():
    #         return
    #     if self.state == typing.Literal["loading"]:
    #         self.sensors_check.get("load-helper").trigger_check()
    #
    # cut
    # register unextrude start when cut signals that is actually cut
    # start helper sensor verification
    # stop unextrude on timeout or when sensor is triggered
    # clean or end with error

    # def load(self) -> None:
    #     gcode = self.printer.lookup_object("gcode")
    #     if self.state != FilamentStates.UNLOADED:
    #         raise gcode.error(
    #             f"Cannot load \n Extruder {self.name} is currently loaded or unloading"
    #         )
    #     self.state = FilamentStates.LOADING
    #     self.extrude_count = 0
    #     # start sensor verification
    #     # start extrude
    #     # stop extrude on cutter sensor or when timeout is reached
    #     # purge or end with error
    #
