# motorFader

The motorFader is a physical, bidirectional control interface for things like lighting, volume, and more -- "birdirectionality"
means that you can move the fader to control your lights by hand, but the fader will also physically move to reflect any 
external changes (like from another switch, dashboard, or automation)!

The motorFader is a fully-integrated control board that fits directly onto Soundwell 60mm motorized faders for a compact,
modular system. Many faders can be chained together via the I2C bus, and 0.1" headers allow for easy, wire-free connections
between adjacent modules.

Each module is independent, with an ATTiny1616 microcontroller powering the motor control loop, capacitive touch input, and
I2C communications.

When combined with an ESP32 main controller, the associated ESPHome component allows faders to be seamlessly integrated
into Home Assistant. 

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
- Capacitive touch sensing for fader touch detection
- Motor driver with position feedback (potentiometer)
- I2C communication interface for host integration
- Power and debug status LEDs
- Compact PCB design optimized for JLCPCB assembly

**Firmware:**
- Programmable haptic modes:
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
- Bidirectional position sync with Home Assistant entities
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
├── software/            # WebHID browser demo tool
├── production_tools/    # ESP32-based production test fixture
└── ci/                  # CI scripts for PCB export
```

## Documentation

- **[ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md)** - ESPHome setup and API reference
- **[ABOUT_LAYERS.md](ABOUT_LAYERS.md)** - Layer architecture and implementation details

## Production Testing

The `production_tools/programAndTest` directory contains unsupported (source-provided) code for programming and quality control of motor fader boards during production.
