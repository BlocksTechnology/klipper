import logging
import typing


class Heatsoak:
    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.state = "idle"

        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.heater = None
        self.pheaters = None
        self.temperature_sensor = None
        self.heater_name = config.get("heater", "heater_bed")
        self.stabilization_type = config.getchoice(
            "stabilization_mode", choices=["time", "goal"], default=""
        )
        if config.get("temperature_sensor", None):
            self.temperature_sensor_name = config.get("temperature_sensor")
        if config.get("chamber_max_temp", None):
            self.chamber_max_temp = config.get("chamber_max_temp")
        if config.get("chamber_min_hum", None):
            self.chamber_min_hum = config.get("chamber_min_hum")

        self.gcode.register_command(
            "HEATSOAK", self.cmd_HEATSOAK, "Heat and stabilize chamber temperature"
        )
        # self.stabilize_timer = self.reactor.register_timer(
        #     self.stabilize_check, self.reactor.NEVER
        # )

    def handle_ready(self):
        self.pheaters = self.printer.lookup_object("heaters")
        self.heater = self.pheaters.lookup_heater(
            self.heater_name
        )  # This already handles the case where it does not exist
        pheaters_status = self.pheaters.get_status(self.reactor.monotonic())

        if self.temperature_sensor_name in pheaters_status.get("available_sensors", {}):
            self.temperature_sensor = self.pheaters[self.temperature_sensor_name]
        else:
            self.temperature_sensor = self.printer.lookup_object(
                self.temperature_sensor_name
            )

    def stabilize_heat(
        self, temperature: int, limit_time: int = 20, threshold: int = 5
    ):
        """Heats the configured object heater and stabilizes
        the temperature for the specified amount of time

        Args:
            temperature (int): Desired temperature
            limit_time (int, optional): Desired stabilization time. Defaults to 20.

        Raises:
            self.printer.config_error: Raised when temperature cannot be retrieved by the
                specified sensor
        """
        self.state = "initializing"
        self.pheaters.set_temperature(self.heater, temperature, True)
        target_time = self.reactor.monotonic() + (limit_time * 60)  # convert to seconds
        reactor = self.printer.get_reactor()
        toolhead = self.printer.lookup_object("toolhead")
        eventtime = reactor.monotonic()
        # while not self.printer.is_shutdown() and self.heater.check_busy(eventtime):
        while not self.printer.is_shutdown():
            curr_temp, _ = self.heater.get_temp(eventtime)
            # if self.stabilization_type == "goal":
            reported_sensor_temp = self.temperature_sensor.get_status(eventtime).get(
                "temperature", None
            )
            reported_sensor_hum = self.temperature_sensor.get_status(eventtime).get(
                "humidity", None
            )

            if not reported_sensor_temp:
                raise self.printer.config_error(
                    "Cannot retrieve temperature from provided sensor %s"
                    % (self.temperature_sensor_name)
                )
            min_temp = temperature + (temperature * threshold)
            max_temp = temperature - (temperature * threshold)
            if (
                not (min_temp <= curr_temp <= max_temp)
                and self.stabilization_type == "goal"
            ):
                target_time += 300  # Increase goal time by 5 minutes

            if target_time != self.reactor.monotonic():
                self.state = "finished"
                return
            print_time = toolhead.get_last_move_time()
            self.gcode.respond_raw("Stabilizing")
            self.gcode.respond_raw(
                "Stabilizing Chamber | Temp: %.1f | hum: %.1f"
                % (
                    reported_sensor_temp,
                    reported_sensor_hum if reported_sensor_hum else "Not Reported",
                )
            )
            self.state = "stabilizing"
            eventtime = reactor.pause(eventtime + 1.0)

    def cmd_HEATSOAK(self, gcmd):
        temp = gcmd.get("TEMPERATURE", 50, minval=15, maxval=100)
        limit_time = gcmd.get("TIME", 20, minval=15, maxval=120)  # In minutes
        self.stabilize_heat(temp, limit_time)

    def get_status(self):
        return {"state": self.state}
