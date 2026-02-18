import logging 
import typing 

class FilamentMotions:
    def __init__(self, config, name, extruder):
        self.config = config
        self.printer = config.get_printer()
        self.name = name
        self.reactor = self.printer.get_reactor()

        self.debug: int = config.getint("debug", 0)
        self.bucket = config.get()

        self.cutter = None
        self.cutter_name: str = config.get("cutter", None)

        self.aux_extruder = None
        self.aux_extruder_name: str = config.get("aux_extruder", None)

        self.extrude_speed: float = config.getfloat("extruder_speed", 10., minval=2., maxval =50. )
        self.travel_speed: float = config.getfloat("travel_speed", 50., minval=20., maxval=500.)

        self.extruder_motion: ExtruderMotions = ExtruderMotions(config, extruder, name)
        # TODO: self.printer.load_object("ExtruderMotion", ..., ..) maybe i can do this 
        # to load extruder motions and other obejcts that 
        # i need here, just like filament_switch_sensor does
        # for creating the on_runout macros etc
        # Can also do this for the sensor_checkers
 
        self.sensor_checkers : dict[str, SensorChecker]= {}
        self.state: FilamentStates = FilamentStates.UNKNOWN

        self.timeout = config.getint("timeout", default=None, minval=10, maxval=1000)

        self.unextrude_timer = self.reactor.register_timer(
            self._unextrude, self.reactor.NEVER
        )
        # self.extrude_timer = self.reactor.register_timer(
        #     self._extrude, self.reactor.NEVER
        # )

        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self) -> None:
        """Handle `klippy:ready` event"""
        self.gcode = self.printer.lookup_object("gcode")
        self.bucket = self.printer.lookup_object("bucket", None)

        if self.cutter_name:
            if self.printer.lookup_object(f"cutter_sensor {self.cutter_name}", None):
                self.cutter = self.printer.lookup_object(
                    f"cutter_sensor {self.cutter_name}"
                )
            else:
                self.printer.config_error(
                    f"{self.cutter_name} is undefined, expected cutter_sensor object"
                )
        elif self.debug > 0:
            self.gcode.respond_info(
                f"Not using cutter_sensor for filament manager on extruder: {self.extruder_motion.name}"
            )
            logging.info(
                f"Not using cutter_sensor for filament manager on extruder: {self.extruder_motion.name}"
            )

        if self.aux_extruder_name:
            if self.printer.lookup_object(
                f"extruder_stepper {self.aux_extruder_name}", None
            ):
                self.aux_extruder = self.printer.lookup_object(
                    f"extruder_stepper {self.aux_extruder_name}"
                )
            else:
                self.printer.config_error(
                    f"{self.aux_extruder_name} is undefined, expected extruder_stepper object"
                )
        elif self.debug > 0:
            self.gcode.respond_info(f"Not using auxiliar extruder on filament manager")
            logging.info(f"Not using auxiliar extruder on filament manager")
    
    def _initialize_motion(self, eventtime = None, clean: bool = True, motion_type: [FilamentStates.LOADING, FilamentStates.UNLOADING]) -> None: 
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info(f"Initializing filament motion: {motion_type.name}")
        gcode_macro = self.printer.lookup_object("gcode_macro")
        toolhead = self.printer.lookup_object("toolhead") 
        if clean: 
            gcode.run_script_from_command("CLEAN_NOZZLE")

        if self.bucket: 
            gcode.respond_info(f"Moving extruder {self.extruder_motion.name} ")
            self.bucket.move_to_bucket(split=False)
              
    def _register_sensor_callback(
        self, sensor_type, name, trigger, interval, callback
    ) -> None:
        """Register a sensor with a specified trigger to run a callback"""
        sensor_check = SensorChecker(self.config)
        sensor_check.register_sensor(sensor_type, name)
        sensor_check.set_check_interval(interval)
        sensor_check.register_callback(callback, trigger)
        self.sensor_checkers.update({name: sensor_check})

    def _deregister_sensor(self, name) -> None:
        """Deletes a sensor checker object by name"""
        sensor_checker = self.sensor_checkers.pop(name)
        del sensor_checker

    def _unextrude(self, eventtime) -> None:
        if self.timeout:
            if self.extrude_count >= self.timeout:
                self.sensor_checkers.get(
                    # TODO: Get the sensor check here
                    self.unextrude_control_sensor_name
                ).trigger_check()
                completion = self.reactor.register_callback(self._unload_end)
                return completion.wait()
            self.extrude_count += 1
        self.extruder_motion.move(distance=-1., speed=self.speed)
        return float(eventtime + float(1. / self.speed))

    def _handle_unload_sensor_trigger(self, eventtime) -> None:
        if not self.sensors_check.get("unload-helper").active():
            return
        if self.state == FilamentStates.UNLOADING:
            self.sensors_check.get("unload-helper").trigger_check()

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
        self.unextrude_timer.update(self.reactor.NOW)
        self.sensors_check.get("unload-helper").trigger_check()
    
    def _unload_end(self, eventtime = None) -> None: 
        gcode = self.printer.lookup_object("gcode")
        gcode.respond_info("Unload end method")


    # def _extrude(self, eventtime) -> None:
    #     if self.timeout:
    #         if self.extrude_count >= self.timeout:
    #             self.sensors_check.get(self.extrude_control_sensor_name).trigger_check()
    #             completion = self.reactor.register_callback(self.extrude_end)
    #             return completion.wait()
    #         self.extrude_count += 1
    #     self.extruder_motion.move(1, self.speed)
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

