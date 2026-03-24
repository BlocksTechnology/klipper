# AGENTS.md - Klipper Development Guide

This file provides guidance for AI agents working on the Klipper 3D printer firmware.

## Project Overview

Klipper is a 3D printer firmware combining a Raspberry Pi (host) with microcontrollers. This is a Blocks fork with additional modules (belay, filament cutter, load/unload, bucket, bed boundaries).

## Directory Structure

| Directory | Purpose |
|-----------|---------|
| `klippy/` | Host-side Python code (runs on Raspberry Pi) |
| `klippy/extras/` | Optional modules (~100+ plugins) |
| `klippy/kinematics/` | Printer motion models (cartesian, delta, corexy, etc.) |
| `src/` | Microcontroller firmware in C |
| `config/` | Printer board configurations (150+ boards) |
| `scripts/` | Build/flash/utilities scripts |
| `test/klippy/` | Test configuration files |
| `docs/` | Documentation |
| `lib/` | External dependencies (CMSIS, HAL, SDKs) |

## Build Commands

### Firmware Build
```bash
# Configure (select board via menu)
make menuconfig

# Build firmware
make

# Clean build artifacts
make clean

# Full clean (removes config too)
make distclean

# Verbose build (see actual commands)
make V=1
```

### Running Tests

```bash
# Run all tests
python3 scripts/test_klippy.py test/klippy/*.test

# Run a single test (verbose)
python3 scripts/test_klippy.py -v test/klippy/commands.test

# Run single test, keep temp files
python3 scripts/test_klippy.py -k test/klippy/commands.test
```

Test files are `.test` files in `test/klippy/` that reference:
- `DICTIONARY` - MCU dictionary file
- `CONFIG` - Printer config file
- G-code commands to execute

## Code Style - Python (klippy/)

### General Guidelines
- **Python 2/3 compatible** (shebang uses `python2`, runs on Python 3)
- No type annotations (keep compatibility)
- 4-space indentation
- Maximum line length: ~80-100 characters (flexible)
- Use `logging` module for logging, not print statements

### Imports
```python
# Standard library first, then third-party, then local
import os
import re
import logging
import collections

import util, reactor, queuelogger
import gcode, configfile, pins
```

### Naming Conventions
- **Classes**: CamelCase (e.g., `class Printer:`, `class GCodeDispatch:`)
- **Functions/methods**: snake_case (e.g., `def get_position():`, `def setup_registers():`)
- **Constants**: UPPER_CASE (e.g., `MAX_SPEED = 1000`)
- **Private methods**: prefix with underscore (e.g., `def _connect(self):`)

### Error Handling
```python
# Configuration errors
raise self.config_error("Error message: %s" % (value,))

# G-code command errors
raise gcode.CommandError("Error processing command")

# Use logging for errors
logging.error("Error message: %s" % (detail,))
logging.info("Informational message")
logging.debug("Debug details")
```

### Class Structure Pattern
```python
class Printer:
    config_error = configfile.error
    command_error = gcode.CommandError

    def __init__(self, main_reactor, bglogger, start_args):
        self.bglogger = bglogger
        self.start_args = start_args
        self.reactor = main_reactor

    def get_reactor(self):
        return self.reactor
```

### Config File Pattern
```python
class MyModule:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        # Register handlers
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
```

## Code Style - C (src/)

### General Guidelines
- **C11 standard** with GNU extensions (`-std=gnu11`)
- 4-space indentation
- K&R brace style
- Comment style: `// Single line` or `/* Multi-line */`

### Includes (order matters)
```c
#include "autoconf.h"      // CONFIG_* - generated
#include "basecmd.h"        // oid_alloc
#include "board/gpio.h"     // gpio_out_write
#include "command.h"        // DECL_COMMAND
#include "sched.h"          // struct timer
```

### Naming Conventions
- **Structs**: lowercase with underscores (e.g., `struct stepper_move`)
- **Enums**: UPPER_CASE with prefix (e.g., `enum { MF_DIR=1<<0 }`)
- **Functions**: snake_case (e.g., `stepper_load_next()`)
- **Macros**: UPPER_CASE (e.g., `#define HAVE_EDGE_OPTIMIZATION 1`)

### Code Patterns
```c
// Struct definition
struct stepper {
    struct timer time;
    uint32_t interval;
    struct gpio_out step_pin;
};

// Function with error handling
static uint_fast8_t
stepper_load_next(struct stepper *s)
{
    if (move_queue_empty(&s->mq)) {
        s->count = 0;
        return SF_DONE;
    }
    // ... implementation
}
```

## Configuration System

Uses Kconfig (Linux kernel style):
- `make menuconfig` opens interactive configurator
- Board configs in `config/` directory
- Creates `.config` file and `out/autoconf.h`

## VSCode Settings

Project has `.vscode/settings.json` with pylint:
```json
{
    "pylint.args": ["--disable=C0115", "--disable=R0902"]
}
```
- C0115: Missing class docstring
- R0902: Too many instance attributes

## Key Files to Know

- `klippy/klippy.py` - Main entry point
- `klippy/gcode.py` - G-code parser
- `klippy/toolhead.py` - Motion planning
- `klippy/configfile.py` - Config parsing
- `klippy/mcu.py` - MCU communication
- `src/sched.c` - Task scheduler
- `src/command.c` - Command protocol
- `src/stepper.c` - Stepper motor control
