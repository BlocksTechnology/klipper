import logging
import typing


class PrintCore:
    def __init__(self, config) -> None:
        super().__init__()
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.toolhead = None
        self.saved_variables = None
        self.state = "idle"
        self.filament_var = config.get("filament_var")
        self.park_pos = config.getlist("park_pos", sep=",", count=2)
        self.travel_speed = config.getint("travel_speed", default=150)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.printer.register_event_handler(
            "unload_filament:end", self.handle_unload_end
        )
        self.gcode.register_command(
            "CHANGE_PRINTCORE",
            self.cmd_CHANGE_PRINTCORE,
            "Preparation for changing the print core",
        )

    def handle_ready(self):
        """Handle klippy ready event"""
        self.saved_variables = self.printer.lookup_object("save_variables")
        self.toolhead = self.printer.lookup_object("toolhead")
        # self.unload_obj = self.printer.lookup_object("unload_filament")s

    def handle_unload_end(self):
        self.state = "unloaded"
        self.toolhead.manual_move(self.park_pos, self.travel_speed)
        self.toolhead.wait_moves()
        self.state = "in_pos"

    def change_printcore(self):
        """Preparation for changing the print core"""

        loaded = self.saved_variables.get(self.filament_var)
        if loaded:
            self.state = "unloading"
            self.gcode.run_script_from_command("UNLOAD_FILAMENT TEMPERATURE=250")

    def cmd_CHANGE_PRINTCORE(self, gcmd):
        """Gcode for running the preparation of the change print core"""
        self.change_printcore()
        self.gcode.respond_info("Finished")

    def get_status(self, eventtime):
        """Gets the status of the PrintCore object"""
        return {"state": self.state}


def load_config(config):
    return PrintCore
