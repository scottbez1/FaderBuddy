# motorFader

The motorFader is a modular control board for 60mm motorized linear potentiometers, making it dead simple to add
one (or many) to project with just 2 I/O pins (I2C, which can be shared with your other I2C peripherals)!

![motor_fader_demo_tiny](https://github.com/user-attachments/assets/fc8dd191-fca0-4ac6-80d8-bb88dc9d0a7a)

The 0.1" pitch headers make them easily chainable with 18mm or 19mm spacing between modules with minimal wiring,
and STEMMA QT/QWIIC-compatible connectors make it easy to hook motorFaders to the rest of your design (note: a
separate 5v supply wire to power the motor is needed when using 3.3v STEMMA QT/QWIIC; 5v STEMMA QT can power the
motor directly without an additional power wire).

TODO: photo of chaining

The onboard ATtiny1616 microcontroller handles all the complex real-time logic (PID motor control and capacitive
touch handling) and provides a simple interface for controlling bidirectional input/output and haptic feedback
from your main microcontroller.

When combined with an ESP32 main controller, the provided [ESPHome component](ABOUT_ESPHOME_INTEGRATION.md) allows
faders to be seamlessly integrated into Home Assistant. Or, use a simple I2C<>USB adapter like an MCP2221A to
connect faders to your computer.

<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_perspective.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_perspective.png" width="300" />
</a>
<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_top.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_top.png" width="300" />
</a>
<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_bottom.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_bottom.png" width="300" />
</a>

## Features

**Hardware:**
- ATtiny1616 microcontroller with UPDI programming interface
- Supports 3.3v or 5v logic (with 3.3v logic, 5v is still required for powering the motor; with 5v logic Vio and Vmot can be combined for only 4 wires)
- Capacitive touch sensing for fader touch detection
- Motor driver with position feedback (potentiometer)
- I2C communication interface for host integration
- Power and debug status LEDs
- Compact PCB design optimized for JLCPCB assembly

**Firmware:**
- Programmable haptic-feedback modes:
  - Smooth operation (no detents)
  - Magnetic endpoints (snap to 0% and 100%)
  - Configurable detents (1-15 positions)
- **8 independent layers** per fader - each with its own position and haptic configuration
- Touch-aware motor control prevents unnecessary power consumption
- High level interface ("move to position X") with arbitration allows for stable bidirectional control even with tens of milliseconds of latency between remote and local systems
- Double-tap gesture detection

## PCB Fabrication

Latest auto-generated (untested and likely broken!) artifacts⚠️:
- Review
  - [Schematic](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-schematic.pdf)
  - [Interactive BOM](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-ibom.html)
  - [PCB Packet](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-pcb-packet.pdf)
- Ordering (Configured for JLCPCB)
  - 1.6mm, any color, HASL lead-free
  - [Gerbers](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-jlc/gerbers.zip)
  - [BOM csv](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-jlc/bom.csv)
  - [CPL (POS) csv](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-jlc/pos.csv)

## ESPHome Integration

The motor fader integrates seamlessly with [ESPHome](https://esphome.io/) for Home Assistant automation. The custom component provides:
- Bidirectional (input & output) position sync with Home Assistant entities
- Layer-aware automation triggers (manual_move, touch_change, double_tap)
- Per-layer haptic configuration
- Multiple faders on a single ESP32 via I2C

See [ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md) for setup instructions and examples.

## Project Structure

```
motorFader/
├── electronics/          # KiCad PCB design files
├── firmware/            # ATtiny1616 firmware source (PlatformIO)
├── esphome/             # ESPHome custom component
│   ├── components/      # Component source code
│   └── examples/        # Example configurations
├── software/            # Software tools and demos
│   └── mcp2221-webhid/  # WebHID browser demo tool
├── production_tools/    # ESP32-based production test fixture
└── ci/                  # CI scripts for PCB export
```

## Documentation

- **[ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md)** - ESPHome setup and API reference
- **[ABOUT_LAYERS.md](ABOUT_LAYERS.md)** - Layer architecture and implementation details

## Production Testing

The `production_tools/programAndTest` directory contains unsupported (source-provided) code for programming and quality control of motor fader boards during production.
