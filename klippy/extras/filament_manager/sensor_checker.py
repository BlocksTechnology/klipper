from functools import partial
import typing
import logging


class SensorRole:
    PRE_GATE = "pre_gate"
    POST_GEAR = "post_gear"
    GATE = "gate"
    SYNC_FEEDBACK = "sync_feedback"
    POST_GATE = "post_gate"
    EXTRUDER = "extruder"
    TOOLHEAD = "toolhead"

    @classmethod
    def exists(cls, value) -> bool:
        return hasattr(cls, value.strip().upper())


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
        self.state: bool = False
        self.check_interval: float = 0.5
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
        if self.state != filament_present:
            logging.info(f"Filament state changed {self.state}")
            self.state = filament_present
            # if filament_present == self.trigger_state and :
            if filament_present == self.trigger_state:
                if eventtime >= self.min_event_systime:
                    self.min_event_systime = self.reactor.NEVER
                    self.reactor.register_callback(self._handle_trigger)
        self.last_check_time: float = eventtime
        return eventtime + self.check_interval

    def _handle_trigger(self, eventtime):
        logging.info(
            f"EXTRUDER CHECKER -> Sensor triggered state: {self.trigger_state}"
        )
        self.min_event_systime = self.reactor.monotonic() + self.event_delay
        completion = self.reactor.register_async_callback(
            partial(self.callback, self.trigger_state)
        )

    def active(self) -> bool:
        """Check if this SensorChecker is currently active"""
        return self.is_enabled
