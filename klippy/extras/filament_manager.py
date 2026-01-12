import logging



PLA_TEMPERATURE = 
PETG_TEMPERATURE = 
ABS_TEMPERATURE
NYLON_TEMPERATURE

DEFAULT_TEMPERATURE


class FilamentMotions: 
    def __init__(self, config): 
        pass 

class FilamentCheck: 
    # TODO: Received filament sensors, checks filament presences 
    def __init__(self, config):
        pass
    
    
class FilamentManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_gcode("gcode")
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        
        self.filament_motion = FilamentMotions()

    def handle_connect(self) -> None:
        self.toolhead = self.printer.lookup_object("toolhead")

    def handle_ready(self) -> None:
        # TODO: Maybe check if filament is actually present and then update local variables
        ...





def load_config(config): 
    return FilamentManager(config)