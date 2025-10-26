# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

motorFader is a bidirectional motor fader control system with integrated capacitive touch sensing and I2C communication. The hardware consists of a PCB that mounts directly onto Soundwell 60mm motorized faders, with an ATtiny1616 microcontroller managing motor control, touch detection, and I2C communication. Multiple faders can be chained together and controlled by an ESP32 host (designed for ESPHome/Home Assistant integration).

## Project Structure

- **electronics/** - KiCad PCB design files (schematic and board layout)
- **firmware/** - ATtiny1616 firmware (PlatformIO project, Arduino framework)
  - `src/main.cpp` - Main firmware logic with motor control loop and I2C peripheral
  - `src/shared/i2c_data.h` - I2C protocol definitions (shared with production tools)
- **production_tools/programAndTest/** - ESP32-based production test fixture
- **ci/** - Python scripts for electronics export (JLCPCB files, PDFs, renders)

## Build Commands

### Firmware (ATtiny1616)

**IMPORTANT**: All PlatformIO commands require activating the pipenv environment first:
```bash
source ~/.platformio/penv/bin/activate
```

Then you can use PlatformIO commands - make sure to cd to the repo root before running these (there are multiple platformio setups within the repo).
```bash
# Build firmware
pio run

# Upload firmware via UPDI (default port: /dev/ttyUSB0)
pio run --target upload

# Monitor serial output (default port: /dev/ttyUSB1)
pio run --target monitor

# Clean build
pio run --target clean
```

The firmware uses UPDI programming via a USB-to-serial adapter. Upload port and monitor port can be configured in `platformio.ini`.

Generally we don't have the serial RX/TX lines hooked up, so prefer to debug firmware on the ATtiny1616 through other means than serial when possible.


### Firmware (production programAndTest ESP32 jig)

**IMPORTANT**: All PlatformIO commands require activating the pipenv environment first:
```bash
source ~/.platformio/penv/bin/activate
```

You can use PlatformIO commands to build, upload, and view the serial monitor; just cd into the production_tools/programAndTest directory before running these commands.

e.g. `source ~/.platformio/penv/bin/activate && cd production_tools/programAndTest && pio run --target upload`


### Electronics Export

Electronics artifacts (JLCPCB files, schematics, PDFs, 3D renders) are generated via Python scripts in `ci/electronics/`:

```bash
# Install dependencies (KiCad, KiBot, etc) -- only to be used within CI by github runners!
./ci/electronics/dependencies.sh

# Generate JLCPCB fabrication files (gerbers, BOM, CPL)
./ci/electronics/export_jlcpcb.py --release-prefix releases/electronics/ \
  --assembly-schematic electronics/motor_fader_main.kicad_sch \
  electronics/motor_fader_main.kicad_pcb

# Generate PCB overview PDF
./ci/electronics/generate_pdf.py --release-prefix releases/electronics/ \
  electronics/motor_fader_main.kicad_pcb
```

CI automatically exports all electronics artifacts on push (see `.github/workflows/electronics.yml`).

## Architecture

### I2C Protocol

The motor fader acts as an I2C peripheral with a configurable address (base 0x20 + 3-bit hardware address from jumpers). The protocol is defined in `firmware/src/shared/i2c_data.h`.

**Key Registers:**
Refer to i2c_data.h for register definitions.

### State Machine

The firmware implements a state machine to arbitrate between remote control and local user input:

1. **MODE_REMOTE_MOVEMENT_IN_PROGRESS** (0)
   - Motor actively moving to commanded target position
   - Haptics disabled, simple PID movement
   - Transitions to INPUT_ACTIVE if touch detected for >50ms
   - Transitions to INPUT_IDLE when target reached and stable for >300ms
   - Transitions to ERROR if movement timeout (8 seconds)

2. **MODE_INPUT_ACTIVE** (1)
   - User is touching/moving the fader
   - Haptic engine active (future: detents, magnetic endpoints)
   - Remote position commands ignored (local input has priority)
   - Transitions to INPUT_IDLE after 1 second of no touch/movement

3. **MODE_INPUT_IDLE** (2)
   - Fader at rest, motor off
   - Waiting for either remote commands or user input
   - Transitions to REMOTE_MOVEMENT on new target position
   - Transitions to INPUT_ACTIVE on touch or movement

4. **MODE_ERROR** (3)
   - Motor failed to reach target within timeout
   - All control disabled until error cleared (REG_CLEAR_ERROR) or haptic settings change

5. **MODE_SELF_CALIBRATION** (4)
   - Automatic calibration of potentiometer range
   - Motor drives to both endpoints to measure ADC values

### Position Nonces

To handle bidirectional control with latency, the protocol uses "nonces" (2-bit counters):
- **Position nonce**: Incremented by the fader when user changes position (differentiates new local input from remote echo)
- **Settings nonce**: Changed when haptic settings are updated (reserved for future use)

This allows the host controller to distinguish between position updates caused by local user input vs. remote commands it sent earlier.

### Shared Code

The file `firmware/src/shared/i2c_data.h` contains the I2C protocol definitions and is shared between:
- The ATtiny1616 firmware (I2C peripheral implementation)
- The production test tool (ESP32 I2C controller)
- Future ESPHome component (host controller)

When modifying the I2C protocol, ensure changes are compatible across all consumers.

## Hardware Details

- **MCU**: ATtiny1616 @ 20MHz
- **Programming**: UPDI interface (uses pyupdi tool, auto-installed via PlatformIO extra_scripts)
- **Motor Driver**: H-bridge with PWM control, configurable frequency via TCA0 timer
- **Position Feedback**: Potentiometer (ADC input with EWMA filtering)
- **Touch Sensing**: Capacitive touch via ptc_touch library
- **I2C Address**: Base 0x20, configurable via 3 address pins (A0-A2)

## PCB Fabrication

The PCB is designed for JLCPCB assembly with 1.6mm thickness and HASL lead-free finish. All fabrication files (gerbers, BOM, CPL) are auto-generated by CI and available as artifacts (see README for links).