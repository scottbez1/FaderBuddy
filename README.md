# motorFader

The motorFader is a modular control board for 60mm motorized linear potentiometers, making it dead simple to add
one (or many) to project with just 2 I/O pins (I2C, which can be shared with your other I2C peripherals)!

The 0.1" pitch headers make them easily chainable with 18mm or 19mm spacing between modules with minimal wiring,
and STEMMA QT/QWIIC-compatible connectors make it easy to hook motorFaders to the rest of your design (note: a
separate 5v supply wire to power the motor is needed when using 3.3v STEMMA QT/QWIIC; 5v STEMMA QT can power the
motor directly without an additional power wire).

TODO: photo of chaining

### Wiring Overview

TODO: Add wiring diagram showing:
- Motor fader board pinout (Vio, Vmot, SDA, SCL, GND, UPDI)
- Connection from fader board to ESP32
- Daisy-chaining multiple fader boards via headers
- Power supply connections (3.3V logic + 5V motor)

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

## What You'll Need

To build a complete motor fader setup, you'll need the following. This list covers a single-fader setup; add one fader + one PCB per additional channel.

**Motor Fader Hardware:**
| Item | Qty | Notes |
|------|-----|-------|
| 60mm motorized fader | 1+ | Designed for Soundwell 60mm travel faders. Available retail as the Behringer MF60T (sold in 5-packs as replacement parts) from music/AV retailers like [Sweetwater](https://www.sweetwater.com/store/detail/MOTORFADER--behringer-mf60t-motorized-faders-set-of-5-for-motor-controllers), [B&H](https://www.bhphotovideo.com/c/product/1821670-REG/behringer_motor_fader_mf60t_high_performance_60mm_motor_faders.html), and [Amazon](https://www.amazon.com/Behringer-MOTOR-High-Performance-Faders-Keyboards/dp/B01DT827IC) |
| motorFader PCB (assembled) | 1+ | Order from JLCPCB using fabrication files below |

**Host Controller:**
| Item | Qty | Notes |
|------|-----|-------|
| ESP32 dev board | 1 | TODO: recommended board(s)? ESP32-S3-DevKitC-1 used in example |
| USB cable for ESP32 | 1 | For programming and power (USB-C or micro-B depending on board) |

**Firmware Programming:**
| Item | Qty | Notes |
|------|-----|-------|
| [Adafruit UPDI Friend](https://www.adafruit.com/product/5879) | 1 | USB-to-UPDI programmer for flashing ATtiny1616 firmware. See [Firmware Flashing](#firmware-flashing) below |
| TODO: jumper wires / pogo pins? | | TODO: what's needed to connect the UPDI Friend to the board? |

**Power Supply:**
| Item | Qty | Notes |
|------|-----|-------|
| 5V power supply | 1 | TODO: current requirements per fader? Recommended supply? |
| TODO: wiring/connectors? | | TODO: how does 5V get to the fader boards? |

**Optional:**
| Item | Qty | Notes |
|------|-----|-------|
| MCP2221A USB-to-I2C bridge | 1 | Only needed for WebHID debug tool (not required for normal use) |
| STEMMA QT / QWIIC cable | 1+ | Alternative to header wiring for I2C connection |

> **Note:** The motorFader PCB is designed for JLCPCB SMT assembly -- all surface-mount components are placed by the factory. See [PCB Fabrication](#pcb-fabrication) below for ordering instructions.

### Assembly

The PCB comes fully assembled from JLCPCB. The only soldering required is attaching the PCB to the fader itself -- a handful of through-hole solder joints, nothing too small:

- **2 connections** for the motor
- **4 connections** for the fader potentiometer
- 2 optional mechanical-only connections (recommend skipping these)

No fine-pitch or SMD soldering is required. If you're comfortable with basic through-hole soldering, you can do this.

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

## Firmware Flashing

The motorFader PCBs ship from JLCPCB with a blank ATtiny1616 microcontroller -- you'll need to flash the firmware yourself using a UPDI programmer. This only needs to be done once per board (unless you want to update the firmware later).

### What You'll Need

- [Adafruit UPDI Friend](https://www.adafruit.com/product/5879) -- a compact USB-to-UPDI programmer
- [PlatformIO](https://platformio.org/) -- the build system used for compiling and uploading firmware
- TODO: connection details -- jumper wires, pogo pins, or header connection to the board's UPDI pad/header

### Connecting the UPDI Friend

TODO: fill in specific connection details:
- Which pad/pin on the motorFader board is UPDI?
- Which pads/pins are GND?
- Photo or diagram of the UPDI Friend connected to the board
- Are pogo pins needed, or is there a header?

### Flashing Steps

1. **Install PlatformIO:**

   If you don't already have PlatformIO installed, the easiest way is via VS Code: install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode). Alternatively, install the CLI:
   ```bash
   pip install platformio
   ```

2. **Connect the UPDI Friend** to the motorFader board's UPDI and GND pads, and plug it into your computer via USB.

3. **Build and upload the firmware:**
   ```bash
   cd firmware
   pio run --target upload
   ```

   PlatformIO will automatically download the required toolchain, compile the firmware, and upload it to the ATtiny1616 via the UPDI Friend.

4. **Verify** the board is working by connecting it to an I2C bus -- it should appear at its configured address (default 0x20). You can verify with the ESPHome I2C scan (`scan: true`) or the WebHID debug tool.

### Troubleshooting Firmware Upload

- **"No device found" or upload fails:** Check your UPDI and GND connections. UPDI requires only a single data line plus ground.
- **Wrong serial port:** PlatformIO should auto-detect, but you can set the upload port explicitly in `platformio.ini` or via command line: `pio run --target upload --upload-port /dev/ttyUSB0`
- **Permission denied (Linux):** You may need to add your user to the `dialout` group: `sudo usermod -a -G dialout $USER` (then log out and back in).

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
