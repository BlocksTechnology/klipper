"""
Example configuration

[sync_feedback my_feedback]
compression_pin: XX
tension_pin: XX
extruder_stepper: my extruder stepper
"""

import logging
import typing


class TriggerHelper:
    def __init__(self, config) -> None:
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.min_event_systime = self.reactor.NEVER
        self.is_compressed: bool = False
        self.is_tensioned: bool = False
        self.is_relaxed: bool = False
        self.sensor_enabled: bool = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self) -> None:
        self.min_event_systime = self.reactor.monotonic() + 2.0

    def note_compression(self, eventtime, is_triggered) -> None:
        """Handles compression triggering"""
        if is_triggered == self.is_compressed:
            return
        self.is_compressed = is_triggered
        if eventtime < self.min_event_systime or not self.sensor_enabled:
            return

    def note_tension(self, eventtime, is_triggered) -> None:
        """Handles tension switch triggering"""
        if is_triggered == self.is_tensioned:
            return
        self.is_tensioned = is_triggered
        if eventtime < self.min_event_systime or not self.sensor_enabled:
            return

    def get_status(self, eventtime) -> dict[str, typing.Any]:
        """Get the state of Sync Feedback sensor on eventtime"""
        return {
            "is_compressed": bool(self.is_compressed),
            "is_tensioned": bool(self.is_tensioned),
            "enabled": bool(self.sensor_enabled),
        }


class SyncFeedback:
    def __init__(self, config) -> None:
        printer = config.get_printer()
        buttons = printer.load_object(config, "buttons")
        compression_pin = config.get("compression_pin")
        tension_pin = config.get("tension_pin")
        extruder_stepper = config.get("extruder_stepper")
        buttons.register_debounce_button(compression_pin, self._handle_compression)
        buttons.register_debounce_button(tension_pin, self._handle_tension)
        self.trigger_helper = TriggerHelper(config)
        self.get_status = self.trigger_helper.get_status

    def _handle_compression(self, eventtime, state) -> None:
        self.trigger_helper.note_compression(eventtime, state)

    def _handle_tension(self, eventtime, state) -> None:
        self.trigger_helper.note_tension(eventtime, state)


def load_config_prefix(config):
    return SyncFeedback(config)
