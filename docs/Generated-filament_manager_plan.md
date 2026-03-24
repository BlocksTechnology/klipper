# Filament Manager Implementation Plan

## Discussion Summary

### 1. Enum vs Literal for States
- Current code uses `typing.Literal["loading", "loaded", "unloaded", "unloading", "unknown"]`
- **Decision**: Use `enum.Enum` (already implemented as `FilamentStates`)
- Benefits: Proper values for comparison, IDE autocomplete, no typo bugs

### 2. Temperature Constants
- Current code has: `PLA_TEMPERATURE`, `PETG_TEMPERATURE`, `ABS_TEMPERATURE`, `NYLON_TEMPERATURE`, `DEFAULT_TEMPERATURE`
- **Decision**: Keep as constants (current approach is fine)
- Reasoning: They're truly constants, no behavior, Python idiom

---

## Requirements

1. Multiple extruders support - load/unload each head independently
2. Sensors (switch sensors, filament_motion_sensor, cutter_sensor) for detecting filament state
3. Configuration per extruder with optional sensors
4. Status via `get_status(eventtime)`: loading/unloading/loaded/unloaded/unknown
5. Generic sensor support - any switch sensor can be used
6. Purge at end of load (move to bucket position)
7. Timeout fallback if no sensors
8. State persistence via variables.py

---

## Key Design Decisions

### Temperature
- **Option B**: G-code parameter `LOAD_FILAMENT EXTRUDER=extruder FILAMENT=PLA`
- Use `FILAMENT_TEMPERATURES` dict lookup

### Bucket Integration
- Use existing `bucket.py` functionality
- Call `self.bucket.move_to_bucket()` during purge

### Sensors Configuration
```ini
[filament_manager extruder]
sensors: my_switch_sensor, my_motion_sensor, cutter
```

### Initial State
- Default to `unknown`
- Load from `save_variables` on startup
- Save state after each load/unload operation

### Error Handling
- Use `self.printer.command_error("<explanation>")`
- Stop procedure, don't continue

---

## Class Structure

```
FilamentManager (main config + G-code commands)
    |
    +-- FilamentMotions (per-extruder controller)
            |
            +-- SensorChecker (generic sensor wrapper)
            +-- ExtruderMotions (heating, movement)
```

---

## Events Emitted

- `filament_manager:loading`
- `filament_manager:loaded`
- `filament_manager:unloading`
- `filament_manager:unloaded`
- `filament_manager:error`

---

## G-code Commands

```bash
LOAD_FILAMENT EXTRUDER=extruder FILAMENT=PLA
UNLOAD_FILAMENT EXTRUDER=extruder
QUERY_FILAMENT [EXTRUDER=extruder]
```

---

## Configuration Example

```ini
[filament_manager]
default_timeout: 30
purge_count: 3
purge_length: 5
purge_speed: 5
load_speed: 10
unload_speed: 10
travel_speed: 100
save_variables: True

[filament_manager extruder]
sensors: my_switch_sensor, my_motion_sensor
load_timeout: 20
unload_timeout: 15
```

---

## Implementation Issues Fixed

| Issue | Fix |
|-------|-----|
| `typing.Literal` used as values | Use `FilamentStates` enum properly |
| Incomplete load/unload methods | Fully implemented |
| No state persistence | Added `_save_state()` / `_load_saved_state()` |
| No bucket integration | Added `self.bucket.move_to_bucket()` on purge |
| No filament type/temperature | Added `FILAMENT_TEMPERATURES` dict |
| No purge logic | Added `_purge()` timer and purge_count |
| Broken sensor callbacks | Fixed trigger handlers |
| Incomplete status | `get_status()` returns proper dict |
