# motorFader

The motorFader is a modular control board for 60mm motorized linear potentiometers, making it dead simple to add
one (or many) to project with just 2 I/O pins for I2C!

TODO: photo of assembled fader

The control board allows you to **read** the fader position, **move** the fader to a specified position, and it can even provide **haptic feedback** and virtual detents (kind of like a linear version of my [SmartKnob](https://github.com/scottbez1/smartknob) project).

<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_perspective.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_perspective.png" width="300" />
</a>
<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_top.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_top.png" width="300" />
</a>
<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_bottom.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_bottom.png" width="300" />
</a>

The onboard ATtiny1616 microcontroller handles all the real-time logic (PID motor control and capacitive
touch handling) and provides a simple I2C interface for controlling bidirectional input/output and haptic feedback
from your ESP32 or other main microcontroller.

An [ESPHome component](ABOUT_ESPHOME_INTEGRATION.md) allows
motorFader assemblies to be seamlessly integrated into Home Assistant with just a few lines of yaml! 

The 0.1" pitch headers make them easily chainable with 18mm or 19mm spacing between modules with minimal wiring,
and STEMMA QT/QWIIC-compatible connectors make it easy to hook motorFaders to the rest of your design.

TODO: photo of chaining



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
- Touch-aware motor control avoids fighting against user input and supports touch and double-tap detection
- High level interface ("move to position X") with arbitration allows for stable bidirectional control even with tens of milliseconds of latency between remote and local systems

## What You'll Need

To build a complete motor fader setup, you'll need the following. This list covers a single-fader setup; add one fader + one PCB per additional channel.

### Motor Fader Hardware:
| Item | Qty | Notes |
|------|-----|-------|
| 60mm motor fader | 1 each | Designed for Soundwell 60mm travel faders. Available retail as the Behringer MF60T (sold in 5-packs as replacement parts) from music/AV retailers like [Sweetwater](https://www.sweetwater.com/store/detail/MOTORFADER--behringer-mf60t-motorized-faders-set-of-5-for-motor-controllers) or [Amazon](https://www.amazon.com/Behringer-MOTOR-High-Performance-Faders-Keyboards/dp/B01DT827IC) |
| motorFader PCB (assembled) | 1 each | Order from Bezek Labs LLC or from JLCPCB using provided fabrication files and instructions |


### Host Controller:
| Item | Qty | Notes |
|------|-----|-------|
| ESP32 dev board | 1 total | Any ESPHome-compatible ESP32 board. Requires 2 free GPIO pins for I2C. |

### Firmware Programming (only needed when ordering from JLCPCB)

motorFader boards from Bezek Labs come pre-flashed with stable firmware, but boards ordered directly from JLCPCB will need firmware flashed before they will work. You might also want an UPDI programmer to update the motorFader firmware to the latest.

| Item | Qty | Notes |
|------|-----|-------|
| UPDI programmer | 1 total | [Adafruit UPDI Friend](https://www.adafruit.com/product/5879) is recommended, but you can also build an UPDI programmer with a USB->Serial adapter and a few other components (see a comprehensive overview from SpenceKonde [here](https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md#a-note-on-breakout-boards)). See [Firmware Flashing](#firmware-flashing) below for UPDI programming instructions |

## PCB Fabrication
The motorFader PCB is designed for JLCPCB SMT assembly -- all surface-mount components are placed by the factory, but the through-hole daisy-chain pin headers are omitted by default. You'll need to order and solder the pin headers separately (or add them to your JLCPCB assembly order):

| Item | Qty | Notes |
|------|-----|-------|
| 5 pin right-angle male pin headers 0.1" spacing | 1 each | Included with Bezek Labs motorFaders, can be purchased separately from electronics suppliers like [LCSC - 40-pin break-apart headers](https://www.lcsc.com/product-detail/C429956.html)|
| 5 pin right-angle female pin headers 0.1" spacing | 1 each | Included with Bezek Labs motorFaders, can be purchased separately from electronics suppliers like [LCSC](https://www.lcsc.com/product-detail/C2935995.html) |


Use the files from the [latest release](https://github.com/scottbez1/motorFader/releases) when ordering!

> [!CAUTION]
> The files below are auto-generated from the current (untested) design files, and are provided for design reference ONLY. They are NOT considered stable for manufacturing. Use the latest stable release linked above! 

Latest auto-generated (untested and likely broken!) artifacts⚠️:
- Review
  - [Schematic](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-schematic.pdf)
  - [Interactive BOM](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-ibom.html)
  - [PCB Packet](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-pcb-packet.pdf)
- Ordering (Configured for JLCPCB)
  - 1.6mm, any color, HASL lead-free
  - [Untested gerbers](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-jlc/gerbers.zip)
  - [Untested BOM csv](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-jlc/bom.csv)
  - [Untested CPL (POS) csv](https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-jlc/pos.csv)



### Assembly

The PCB comes fully assembled from Bezek Labs LLC and JLCPCB. The only soldering required is attaching the PCB to the fader itself and the optional daisy-chaining headers, which are all through-hole connections:

- **2 connections** for the motor
- **4 connections** for the fader potentiometer
- 2 optional mechanical-only connections (I recommend skipping these)
- **5 connections** for the female daisy-chaining pin headers
- **5 connections** for the male daisy-chaining pin headers

No fine-pitch or SMD soldering is required.

## Wiring Overview
### 3.3v MCU direct wired, direct chaining
Most common wiring for 3.3v microcontrollers like ESP32: 5 wires from host to motorFader. 5v powers the motor and 3.3v powers the logic.
| HOST | motorFader |
| --- | ---------- |
| 5v  | Vmot |
| 3.3v | Vio |
| GND | GND |
| SDA | SDA |
| SCL | SCL |
<img width="1028" height="548" alt="motorFader wiring" src="https://github.com/user-attachments/assets/765cf2e1-b493-44a8-8eeb-e8a84db60d3c" />

### 3.3v MCU STEMMA QT, direct chaining
For a microcontroller with a STEMMA QT connector, the motorFader requires an additional 5v wire to power the motor. When more motorFaders are chained directly using the pin headers, no additional wires are required
| HOST | motorFader |
| --- | ---------- |
| STEMMA QT | STEMMA QT |
| 5v  | Vmot |
<img width="1366" height="946" alt="motorFader wiring(2)" src="https://github.com/user-attachments/assets/760fd565-01b0-48c2-9add-047ffbaf16b8" />

### 3.3v MCU STEMMA QT
For a microcontroller with a STEMMA QT connector, the motorFader requires an additional 5v wire to power the motor. When more motorFaders are chained using STEMMA QT, both a STEMMA QT cable AND a 5v wire are required between each motorFader board.
| HOST | motorFader |
| --- | ---------- |
| STEMMA QT | STEMMA QT |
| 5v  | Vmot |
<img width="1554" height="724" alt="motorFader wiring(1)" src="https://github.com/user-attachments/assets/f2c44eea-cfc9-4fe5-9796-83bf9db8263c" />

### 5v STEAMMA QT
If you have a device that has 5v IO and a 5v STEMMA QT connector (e.g. MCP2221 with the solder jumper bridged to set 5v IO)
| HOST | motorFader |
| --- | ---------- |
| STEMMA QT | STEMMA QT |
<img width="1508" height="726" alt="motorFader wiring(4)" src="https://github.com/user-attachments/assets/ef7156ee-8c56-41c3-901f-95f443d2d1f1" />

(Attribution: ESP32-C3 supermini image from StudioPieters, MIT Licensed; Adafruit Qt Py and Adafruit MCP2221 images from Adafruit, Creative Commons Attribution/Share-Alike)

## Firmware Flashing

The motorFader PCBs ship from JLCPCB with a blank ATtiny1616 microcontroller -- you'll need to flash the firmware yourself using a UPDI programmer. This only needs to be done once per board (unless you want to update the firmware later).

### Connecting the UPDI Friend

On the back of the motorFader PCB, there are three through-hole test points labeled for UPDI programming, laid out at 0.1" spacing. You can solder on standard pin headers for repeated use, or simply hold jumper wires against the pads for a one-time flash.

Connect the Adafruit UPDI Friend to the test points as follows:

| UPDI Friend wire | motorFader pad |
|------------------|----------------|
| Black (GND)      | **-**          |
| Red (VCC)        | **+**          |
| White (UPDI)     | **U**          |

TODO: photo of UPDI Friend connected to the back of the board

### Flashing Steps

1. **Install PlatformIO:**

   If you don't already have PlatformIO installed, the easiest way is via VS Code: install the [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode).

2. **Connect the UPDI Friend** to the motorFader board's UPDI and GND pads, and plug it into your computer via USB.

3. **Build and upload the firmware:**
   Use the "Upload" option in the PlatformIO extension. PlatformIO will automatically download the required toolchain, compile the firmware, and upload it to the ATtiny1616 via the UPDI Friend.

4. **Verify** the board is working by connecting it to an I2C bus -- it should appear at its configured address (default 0x20). You can verify with the ESPHome I2C scan (`scan: true`) or the WebHID debug tool.

### Troubleshooting Firmware Upload

- **"No device found" or upload fails:** Check your UPDI and GND connections. UPDI requires only a single data line plus ground.
- **Wrong serial port:** PlatformIO should auto-detect, but you can set the upload port explicitly in `platformio.ini` or via command line: `pio run --target upload --upload-port /dev/ttyUSB0`
- **Permission denied (Linux):** You may need to add your user to the `dialout` group: `sudo usermod -a -G dialout $USER` (then log out and back in).

## ESPHome Integration

The motor fader integrates seamlessly with [ESPHome](https://esphome.io/) for Home Assistant automation. The custom component provides:
- Bidirectional (input & output) position sync with Home Assistant entities
- Layer-aware automation triggers (manual_move, touch_change, double_tap)
- Per-layer haptic configuration
- Multiple faders on a single ESP32 via I2C

See [ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md) for setup instructions and examples.


## Documentation
- **[ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md)** - ESPHome setup and API reference
- **[ABOUT_LAYERS.md](ABOUT_LAYERS.md)** - Layer architecture and implementation details

### Project Structure
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


## Production Testing

The `production_tools/programAndTest` directory contains unsupported (source-provided) code for programming and quality control of motor fader boards during production.
