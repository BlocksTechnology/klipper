import typing


class SensorChecker:
    def __init__(self, printer) -> None:
        self.printer = printer
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
