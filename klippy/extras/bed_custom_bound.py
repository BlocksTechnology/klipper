import typing


###############################
# Example configuration
#
# [bed_custom_bound]
# custom_boundary_x: 0.0, 500.0
# custom_boundary_y: 0.0, 500.0
# travel_speed: 50.0
# park_xy: 0., 500
###############################


class BedCustomBound:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.custom_boundary_x = None
        if config.getfloatlist("custom_boundary_x", None, count=2) is not None:
            self.custom_boundary_x = config.getfloatlist(
                "custom_boundary_x", count=2
            )
        self.custom_boundary_y = None
        if config.getfloatlist("custom_boundary_y", None, count=2) is not None:
            self.custom_boundary_y = config.getfloatlist(
                "custom_boundary_y", count=2
            )
        self.park = None
        if config.getfloatlist("park_xy", None, count=2) is not None:
            self.park = config.getfloatlist("park_xy", count=2)

        self.travel_speed = config.getfloat(
            "travel_speed", 100.0, above=1.0, minval=1.0, maxval=300.0
        )
        self.default_limits_x = self.default_limits_y = None
        self.gcode.register_command(
            "SET_CUSTOM_BOUNDARY",
            self.cmd_SET_CUSTOM_BOUNDARY,
            "Sets a custom boundary for the bed",
        )
        self.gcode.register_command(
            "RESTORE_DEFAULT_BOUNDARY",
            self.cmd_RESTORE_DEFAULT_BOUNDARY,
            "Restores the axis boundaries to the ones found when homing.",
        )

        self.current_boundary: str = "default"

    def handle_ready(self):
        self.toolhead = self.printer.lookup_object("toolhead")

    def cmd_SET_CUSTOM_BOUNDARY(self, gcmd):
        if not self.toolhead:
            return
        if not self.custom_boundary_x or not self.custom_boundary_y:
            return
        move_to_custom_pos = gcmd.get("MOVE_TO_PARK", False, parser=bool)
        self.set_custom_boundary()

        if move_to_custom_pos and self.park:
            self.toolhead.manual_move(
                [float(self.park[0]), float(self.park[1])],
                self.travel_speed,
            )

    def cmd_RESTORE_DEFAULT_BOUNDARY(self, gcmd):
        self.restore_default_boundary()

    def restore_default_boundary(self, eventtime=None):
        if not self.toolhead:
            return
        if not self.default_limits_x or not self.default_limits_y:
            return

        self.gcode.respond_info(
            f"[CUSTOM BED BOUNDARY] Restoring printer boundary limits"
        )

        kin = self.toolhead.get_kinematics()
        kin.limits[0] = (
            float(self.default_limits_x[0]),
            float(self.default_limits_x[1]),
        )  # X min, X max
        kin.limits[1] = (
            float(self.default_limits_y[0]),
            float(self.default_limits_y[1]),
        )  # Y min , Y max
        self.current_boundary = "default"
        return

    def set_custom_boundary(self, eventtime=None):
        if not self.toolhead:
            return
        if not self.custom_boundary_x or not self.custom_boundary_y:
            return
        self.gcode.respond_info(
            "[CUSTOM BED BOUNDARY] Setting specified custom boundary"
        )
        kin = self.toolhead.get_kinematics()

        self.default_limits_x, self.default_limits_y = (
            kin.limits[0],
            kin.limits[1],
        )

        kin.limits[0] = (
            float(self.custom_boundary_x[0]),
            float(self.custom_boundary_x[1]),
        )  # X min, X max
        kin.limits[1] = (
            float(self.custom_boundary_y[0]),
            float(self.custom_boundary_y[1]),
        )  # Y min , Y max

        self.current_boundary = "custom"

    def move_to_park(self):
        if self.park:
            self.toolhead.manual_move(
                [self.park[0], self.park[1]], self.travel_speed
            )

    def check_boundary_limits(
        self, position: typing.Tuple[float, float], bound_type: str = "default"
    ):
        if not self.toolhead or not position:
            return

        min_limit_x = max_limit_x = min_limit_y = max_limit_y = None

        if (
            bound_type == "default"
            and self.default_limits_x
            and self.default_limits_y
        ):
            min_limit_x = float(self.default_limits_x[0])
            max_limit_x = float(self.default_limits_x[1])
            min_limit_y = float(self.default_limits_y[0])
            max_limit_y = float(self.default_limits_y[1])
        elif bound_type == "current":
            kin = self.toolhead.get_kinematics()
            min_limit_x = float(kin.limits[0][0])
            max_limit_x = float(kin.limits[0][1])
            min_limit_y = float(kin.limits[1][0])
            max_limit_y = float(kin.limits[1][1])
        elif (
            bound_type == "custom"
            and self.custom_boundary_x
            and self.custom_boundary_y
        ):
            min_limit_x = float(self.custom_boundary_x[0])
            max_limit_x = float(self.custom_boundary_x[1])
            min_limit_y = float(self.custom_boundary_y[0])
            max_limit_y = float(self.custom_boundary_y[1])

        if None in (min_limit_x, max_limit_x, min_limit_y, max_limit_y):
            return None

        va = all([min_limit_x, max_limit_x, min_limit_y, max_limit_y])
        if va :
            _limits = {
                "x": bool(min_limit_x < position[0] < max_limit_x),
                "y": bool(min_limit_y < position[1] < max_limit_y),
            }
            return _limits
        return None

        # _limits =  {}
        # if min_limit_x < position[0] or max_limit_x < position[0]:
        #     _limits.update({"x": False})

        # if min_limit_y < position[1] or max_limit_y < position[1]:
        #     _limits.update({"y": False})


    def get_status(self, eventtime=None):
        """Get the status of the current boundary"""
        return {"status": self.current_boundary}


def load_config(config):
    return BedCustomBound(config)
