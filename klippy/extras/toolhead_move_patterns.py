import logging
import math


class ToolheadMovePatterns:
    """Nozzle cleaning implementation"""

    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.toolhead = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.gcode.register_command(
            "CIRCUMFERENCE", self.cmd_CIRCUMFERENCE, self.cmd_CIRCUMFERENCE_helper
        )

        self.gcode.register_command(
            "ELLIPSE", self.cmd_ELLIPSE, self.cmd_ELLIPSE_helper
        )

    def handle_ready(self) -> None:
        """Handle klippy ready event"""
        self.toolhead = self.printer.lookup_object("toolhead")

    def move_toolhead(self, x, y, speed) -> None:
        self.toolhead.manual_move([x, y], speed)
        self.toolhead.wait_moves()

    cmd_CIRCUMFERENCE_helper = "Moves the toolhead in a circle"

    def cmd_CIRCUMFERENCE(self, gcmd) -> None:
        center_x = gcmd.get("CENTER_X", 0, parser=float)
        center_y = gcmd.get("CENTER_Y", 0, parser=float)
        radius = gcmd.get("RADIUS", 0, parser=float, minval=0)
        vel = gcmd.get("VELOCITY", 50, parser=int, minval=50)
        iterations = gcmd.get("ITERATIONS", 5, parser=int, minval=1)
        for _ in range(0, iterations):
            for angle in range(0, 360, 20):
                x_position = center_x + radius * math.cos(math.radians(angle))
                y_position = center_y + radius * math.sin(math.radians(angle))
                self.move_toolhead(x_position, y_position, vel)

    cmd_ELLIPSE_helper = "Moves the toolhead in a ellipse"

    def cmd_ELLIPSE(self, gcmd) -> None:
        """Move the toolhead in a ellipse

        h = center x
        k = center y
        """
        center_h = gcmd.get("H", 0, parser=float)
        center_k = gcmd.get("K", 0, parser=float)
        x_width = gcmd.get("X_WIDTH", 0, parser=float)
        y_width = gcmd.get("Y_WIDTH", 0, parser=float)
        vel = gcmd.get("VELOCITY", 50, parser=int)
        iterations = gcmd.get("ITERATIONS", 5, parser=int, minval=1)
        for _ in range(0, iterations):
            for angle in range(0, 360, 10):
                x_position = center_h + x_width * math.cos(math.radians(angle))
                y_position = center_k + y_width * math.sin(math.radians(angle))
                self.move_toolhead(x_position, y_position, vel)


def load_config(config):
    return ToolheadMovePatterns(config)
