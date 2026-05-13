# motorFader

The motorFader is a modular control board for 60mm motorized linear potentiometers, making it dead simple to add
one (or many) to a project with just 2 I/O pins for I2C!

<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_perspective.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_perspective.png" width="300" />
</a>

![motor_fader_demo_tiny](https://github.com/user-attachments/assets/fc8dd191-fca0-4ac6-80d8-bb88dc9d0a7a)

The control board allows you to **read** the fader position, **move** the fader to a specified position, and it can even provide **haptic feedback** and virtual detents (kind of like a linear version of my [SmartKnob](https://github.com/scottbez1/smartknob) project), all easily controlled over an I2C bus from a host controller like an ESP32.

An [ESPHome component](ABOUT_ESPHOME_INTEGRATION.md) allows
motorFader assemblies to be seamlessly integrated into Home Assistant with just a few lines of yaml! 

The 0.1" pitch headers make them easily chainable with 18-19mm spacing between modules,
and STEMMA QT/QWIIC-compatible connectors make it easy to hook motorFaders to the rest of your design (see [wiring diagrams](#wiring-overview) below for examples).


<img width="400" alt="motorFader wiring" src="https://github.com/user-attachments/assets/765cf2e1-b493-44a8-8eeb-e8a84db60d3c" />


The onboard ATtiny1616 microcontroller handles all the real-time logic (closed-loop motor control and capacitive
touch handling) and provides a simple I2C bus interface - no need to wire a motor driver/H-bridge or implement a PID control loop for each motor fader yourself.

<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_top.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_top.png" width="300" />
</a>
<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_bottom.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_bottom.png" width="300" />
</a>

## ESPHome Integration

The motor fader integrates seamlessly with [ESPHome](https://esphome.io/) for Home Assistant automation. The custom component provides:
- Bidirectional (input & output) position sync with Home Assistant entities
- Layer-aware automation triggers (manual_move, touch_change, double_tap)
- Per-layer haptic configuration
- Multiple faders on a single ESP32 via I2C

A simple example looks like:
```yaml
motor_fader:
  - id: my_fader
    on_manual_move:
      then:
        - lambda: |-
            ESP_LOGD("fader", "Fader moved to %d on layer %d", x, layer);
```

and to move the fader from a lambda:
```cpp
id(my_fader).remote_move_to(position, layer);
```

That's pretty much all there is to it!

See [ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md) for setup instructions and many more examples.

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
| UPDI programmer | 1 total | [Adafruit UPDI Friend](https://www.adafruit.com/product/5879) is recommended, but you can also build an UPDI programmer with a USB->Serial adapter and a few other components (see a comprehensive overview from SpenceKonde [here](https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md#a-note-on-breakout-boards)). See [Firmware Flashing](ABOUT_UPDATING_FIRMWARE.md) for UPDI programming instructions |


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

## About the Board Design
The most plug-and-play option (if you're in the US) is to buy the motorFader PCBs pre-assembled from my Bezek Labs store, which come with firmware already flashed and the hardware tested and helps support this project and future development!

To fabricate or assemble the PCBs yourself, see [ABOUT_PCB_FABRICATION.md](ABOUT_PCB_FABRICATION.md) for more info on ordering.


The board is relatively straightforward:
* ATtiny1616 MCU (supports 3.3v or 5v operation)
* DRV8837 motor driver
* JST connectors for I2C (QWIIC/STEMMA QT-compatible)
* 0.1" pin headers for directly chaining I2C bus
* LEDs
* supporting components


<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-schematic.pdf">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-schematic.png" width="600" />
</a>

<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_top.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_top.png" width="300" />
</a>
<a href="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_bottom.png">
    <img src="https://motorfader-artifacts.s3.amazonaws.com/master/electronics/motor_fader_main-3D_bottom.png" width="300" />
</a>

## Additional Documentation
This README covers the basics, but there are a number of additional pages with more detailed documentation:

- **[ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md)** - ESPHome setup and API reference
- **[ABOUT_LAYERS.md](ABOUT_LAYERS.md)** - Layer architecture and implementation details
- **[ABOUT_MF60T_LOW_PROFILE_MOD.md](ABOUT_MF60T_LOW_PROFILE_MOD.md)** - Optional instructions for modifying the MF60T faders so they can fit in smaller areas
- **[ABOUT_PCB_FABRICATION.md](ABOUT_PCB_FABRICATION.md)** - How to order and assemble your own motorFader boards directly
- **[ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md)** - How to upload firmware to the motorFader microcontroller using UPDI

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

## License

motorFader is released under the [Apache 2.0 License](LICENSE.txt).

In plain terms: you're free to use, modify, and distribute this project — including in commercial products — as long as you include the license and indicate any changes you made. You don't have to share your modifications, but you can't claim the original work is yours or use the project name/branding to endorse your product.
