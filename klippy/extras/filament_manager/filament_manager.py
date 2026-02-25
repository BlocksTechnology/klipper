import logging
import typing

PLA_TEMPERATURE = 230
PETG_TEMPERATURE = 240
ABS_TEMPERATURE = 250
NYLON_TEMPERATURE = 270
DEFAULT_TEMPERATURE = 250


class FilamentManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = None
        self.toolhead = self.extruder_objects = self.bucket = None
        self.custom_boundary = None
        self.config = config
        self.motions: dict[str, FilamentMotions] = {}
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.gcode.register_command(
            "QUERY_CONTROLLABLES",
            self.cmd_QUERY_CONTROLLABLES,
            "Returns the controllable filament toolheads",
        )

        # TODO: There can be more than one bucket, or no buckets at all
        self.config_bucket = config.getboolean("bucket", False)
        self.config_custom_boundary = config.getboolean("custom_boundary", False)

    def handle_ready(self) -> None:
        """Handle `klippy:ready` events"""
        self.extruder_objects = self.printer.lookup_objects("extruder")
        self.gcode = self.printer.lookup_object("gcode")
        for name, extruder in self.extruder_objects:
            self.motions.update(
                {f"{name}": FilamentMotions(self.config, name, extruder)}
            self.gcode.respond_info(
                f"Registered controllable filament extruder : {name}, {str(extruder)}"
            )
        if self.config_custom_boundary:
            self.custom_boundary = self.printer.lookup_object("bed_custom_bound")
        

        # TODO: Grab here the toolhead state by identifier on the variables.cfg file 
        # We are looking for load, unload states, if loading or unloading states 
        # are present on the variables file then we are obligated to wait untilt
        # the state is actually load or unload to execute the new movement
        save_vars = self.printer.lookup_object("save_variables", None)
        if not save_vars:
            self.state = FilamentStates.UNKNOWN
            raise self.printer.config_error(SAVE_VARS_REQUIREMENT_MSG)
        sv_dict = save_vars.get_status(self.reactor.monotonic())
        variables: dict[str, typing.Any] = sv_dict.get("variables", None)
        if not variables:
            self.state = FilamentStates.UNKNOWN



    def conditional_homing(self) -> None:
        """Performs Homing operation, only if axes are not homed"""
        eventtime = self.reactor.monotonic()
        kin = self.toolhead.get_kinematics()
        homed_axes = kin.get_status(eventtime)["homed_axes"]
        if "xyz" in homed_axes.lower():
            return
        self.gcode.run_script_from_command("G28")

    def save_idex_state(self) -> None:
        """Save Idex carriage states"""
        self.gcode.respond_info("Save carriage state")
        toolhead = self.printer.lookup_object("toolhead")
        carriages = self.printer.lookup_object("dual_carriages")
        carriages.cmd_SAVE_DUAL_CARRIAGE_STATE({"FILAMENT_MANAGER_CARRIAGE_STATE"})
        toolhead.dwell(1.0)

    def restore_idex_state(self) -> None:
        """Restore Idex carriage states"""
        self.gcode.respond_info("Restoring carriage state")
        toolhead = self.printer.lookup_object("toolhead")
        carriages = self.printer.lookup_object("dual_carriages")
        carriages.cmd_RESTORE_DUAL_CARRIAGE_STATE({"FILAMENT_MANAGER_CARRIAGE_STATE"})
        toolhead.dwell(1.0)

    def save_printer_state(self) -> None:
        """Saves printer gcode state"""
        self.gcode.respond_info("Saving printer state")
        toolhead = self.printer.lookup_object("toolhead")
        gcode_move = self.printer.lookup_object("gcode_move")
        gcode_move.cmd_SAVE_GCODE_STATE({"FILAMENT_MANAGER_STATE"})
        toolhead.dwell(1.0)

    def restore_printer_state(self) -> None:
        """Restores printer gcode state"""
        self.gcode.respond_info("Restoring printer state")
        toolhead = self.printer.lookup_object("toolhead")
        gcode_move = self.printer.lookup_object("gcode_move")
        gcode_move.cmd_RESTORE_GCODE_STATE({"FILAMENT_MANAGER_STATE", "MOVE=0"})
        toolhead.dwell(1.0)

    def cmd_QUERY_CONTROLLABLES(self, gcmd) -> None:
        self.extruder_objects = self.printer.lookup_objects("extruder")
        for name, extruder in self.extruder_objects:
            self.motions.update(
                {f"{name}": FilamentMotions(self.config, name, extruder)}
            )
        self.gcode.respond_info(
            f"Available controllables: \n {self.controllable_motions}"
        )

    def cmd_UNLOAD(self, gcmd) -> None:
        # home if needed, not during a print
        # TODO: Check if it is printing or not
        # gcmd.get("TOOLHEAD")
        toolhead = self.printer.lookup_object("toolhead")
        self.conditional_homing()
        if self.custom_boundary:
            self.custom_boundary.restore_default_boundary()
        if self.bucket:
            self.bucket.move_to_bucket()
        toolhead.dwell(1.0)
        gcmd.error("fucked up no toolhead")

    def get_status(self, eventtime: float) -> dict[str, typing.Any]:
        return {
            "extruders": {
                name: m.get_status(eventtime) for name, m in self.motions.items()
            }
        }


def load_config(config):
    return FilamentManager(config)
