# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FaderBuddy is a bidirectional motor fader control system with integrated capacitive touch sensing and I2C communication. The hardware consists of a PCB that mounts directly onto Soundwell 60mm motorized faders, with an ATtiny1616 microcontroller managing motor control, touch detection, and I2C communication. Multiple faders can be chained together and controlled by an ESP32 host (designed for ESPHome/Home Assistant integration).

## Project Structure

- **electronics/** - KiCad PCB design files (schematic and board layout)
- **firmware/** - ATtiny1616 firmware (PlatformIO project, Arduino framework)
  - `src/main.cpp` - Main firmware logic with motor control loop and I2C peripheral
  - `src/shared/i2c_data.h` - I2C protocol v5 definitions (shared across all components)
- **esphome/** - ESPHome custom component for Home Assistant integration
  - `components/fader_buddy/` - Layer-aware motor fader component
  - `examples/multi-fader-display.yaml` - ESP32-S3 example with LVGL display
- **software/** - Software tools and demos
  - `mcp2221-webhid/` - WebHID-based browser demo tool for I2C control via MCP2221 USB-to-I2C bridge
- **production_tools/programAndTest/** - ESP32-based production test fixture
  - ESP32 PlatformIO project with display and current monitoring
  - Python test scripts for automated hardware validation
- **ci/** - Python scripts for electronics export (JLCPCB files, PDFs, renders)
  - Automated workflow for PCB fabrication files, S3 upload

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

### ESPHome Component

The ESPHome component provides Home Assistant integration with full layer and haptic support:

```bash
# Deploy to ESP32 device
cd esphome
esphome run examples/multi-fader-display.yaml
```

**Key Features:**
- Layer-aware automation triggers: `manual_move`, `touch_change`, `double_tap`
- Per-layer haptic configuration (detent count, strength, mode)
- Per-layer position restore on layer changes
- Rate limiting per sensor (value_change_min_interval)
- Protocol v5 compatible with layer-addressed registers

**Example Configuration:**
See `esphome/examples/multi-fader-display.yaml` for ESP32-S3 with LVGL display integration.

**Important:** Copy `esphome/examples/secrets.yaml.example` to `secrets.yaml` and fill in your WiFi credentials.

### WebHID Demo Tool

Browser-based control interface for testing and diagnostics:

```bash
cd software/mcp2221-webhid
npm install
npm run build
# Open index.html in Chrome/Edge browser
```

**Requirements:**
- Chromium-based browser (Chrome 89+, Edge 89+) with WebHID support
- MCP2221 USB-to-I2C bridge device
- Linux: Install udev rules (`99-mcp2221.rules`) for non-root access

**Features:**
- Real-time position monitoring (10Hz polling)
- Touch sensor diagnostics (raw/delta/reference values)
- Self-calibration trigger
- Layer switching and per-layer position control
- Direct I2C register read/write

## Architecture

### I2C Protocol

The motor fader acts as an I2C peripheral with a configurable address (base 0x20 + 3-bit hardware address from jumpers). The protocol is defined in `firmware/src/shared/i2c_data.h`.

**Current Protocol Version:** v5

**Key Concepts:**
- **Layer-addressed registers** (0x0E, 0x0F): Require layer index (0-7) in the access sequence
- **8 layers per fader**: Each with independent target position and haptic configuration
- **State register** (0x01): 32-bit packed register containing mode, layer, nonces, and touch state
- **Position/haptic nonces**: Used to detect user input vs. remote command echo

Refer to `i2c_data.h` for complete register map and bit field definitions.

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

### Layer Architecture

The firmware implements firmware-managed layers (0-7), allowing each fader to maintain independent configurations and positions across multiple contexts:

- **Per-layer state**: Each layer has its own target position and haptic configuration
- **Position restoration**: When switching layers, the fader automatically moves to the stored position for that layer
- **Deferred layer changes**: Layer switches are deferred during INPUT_ACTIVE mode to prevent interrupting user input
- **Active layer tracking**: Current layer exposed in STATE register bits 4-6

This enables use cases like multiple mixing scenes, parameter pages, or control contexts on a single physical fader.

### Position Nonces

To handle bidirectional control with latency, the protocol uses "nonces" (2-bit counters):
- **Position nonce**: Incremented by the fader when user changes position (differentiates new local input from remote echo)
- **Double tap nonce**: Incremented when double-tap gesture is detected

This allows the host controller to distinguish between position updates caused by local user input vs. remote commands it sent earlier.

### Shared Code

The file `firmware/src/shared/i2c_data.h` contains the I2C protocol definitions and is shared between:
- The ATtiny1616 firmware (I2C peripheral implementation)
- The ESPHome component (`esphome/components/fader_buddy/i2c_data.h`)
- The production test tool (ESP32 I2C controller)
- The WebHID demo tool (JavaScript implementation)

When modifying the I2C protocol, ensure changes are compatible across all consumers and update all copies of i2c_data.h.

## Hardware Details

- **MCU**: ATtiny1616 @ 20MHz
- **Programming**: UPDI interface (uses pyupdi tool, auto-installed via PlatformIO extra_scripts)
- **Motor Driver**: H-bridge with PWM control, configurable frequency via TCA0 timer
- **Position Feedback**: Potentiometer (ADC input with EWMA filtering)
- **Touch Sensing**: Capacitive touch via ptc_touch library
- **I2C Address**: Base 0x20, configurable via 3 address pins (A0-A2)

## PCB Fabrication

The PCB is designed for JLCPCB assembly with 1.6mm thickness and HASL lead-free finish. All fabrication files (gerbers, BOM, CPL) are auto-generated by CI and available as artifacts (see README for links).