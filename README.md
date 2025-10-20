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
- Programmable haptic modes (TODO):
  - Smooth operation (no detents)
  - Magnetic endpoints (snap to 0% and 100%)
  - Configurable detents (2-10 positions)
- Touch-aware motor control prevents unnecessary power consumption
- High level interface ("move to position X") with arbitration allows for stable bidirectional control even with tens of milliseconds of latency between remote and local systems

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

## Project Structure

```
motorFader/
├── electronics/          # KiCad PCB design files
├── firmware/            # ATtiny1616 firmware source (PlatformIO)
└── production_tools/
    └── programAndTest/  # ESP32-based production test fixture
```

## Production Testing

The `production_tools/programAndTest` directory contains unsupported (source-provided) code for programming and quality control of motor fader boards during production.
