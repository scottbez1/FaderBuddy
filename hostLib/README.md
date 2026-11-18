# FaderBuddyHost

Host-side driver for [FaderBuddy](https://github.com/scottbez1/motorFader)
motorized faders over I2C. It is what the ESPHome component and the production
test jig are built on, and can be used directly from any Arduino or PlatformIO
project.

```cpp
#include <Wire.h>
#include <FaderBuddyHost.h>

faderbuddy::WireTransport transport(Wire, 0x20);
faderbuddy::ArduinoPlatform platform(&Serial);
faderbuddy::FaderBuddy fader(transport, platform);

void setup() {
  Wire.begin();
  fader.set_poll_interval(10);
  fader.begin();
  fader.remote_move_to(200);
}

void loop() { fader.loop(); }
```

See `examples/basic` for events (touch, moves, double-tap).

## Layers

| Header | |
|---|---|
| `fader_buddy_host.h` | `FaderBuddy`: one fader as an application sees it. Mirrors the ESPHome component's interface - layers, moves with speed, haptics, touch/move/double-tap events with per-layer rate limiting, self-calibration status, firmware updates. |
| `fader_buddy_updater.h` | `FirmwareUpdater`: installs an application image through the I2C bootloader. Non-blocking. |
| `fader_buddy_device.h` | `FaderBuddyDevice`: typed access to every register, no policy. |
| `fader_buddy_protocol.h` | Wire-format codecs and the firmware-version feature rules. |
| `fader_buddy_platform.h` | `Transport` and `Platform`: what a host provides. |
| `fader_buddy_arduino.h` | `Transport`/`Platform` for Arduino `TwoWire`. |

Nothing outside `fader_buddy_arduino.h` includes a framework header, so the
library builds under Arduino, ESP-IDF and natively. For another framework,
implement `Transport` (three I2C transactions) and `Platform` (a millisecond
clock and a short delay).

## Development

The protocol headers (`i2c_data.h`, `bootloader_protocol.h`) are copies of
`firmware/src/shared/`, and the ESPHome component carries copies of this
library's sources. Edit the originals, then:

```bash
python3 ci/util/check_i2c_data_sync.py --fix   # from the repo root; CI runs it without --fix
```

Unit tests run natively against a simulated fader (application and bootloader):

```bash
pio test -d hostLib -e native
pio ci hostLib/examples/basic --lib hostLib --board esp32dev
```

Publishing to the PlatformIO registry (maintainers): bump `version` in
`library.json`, add a `CHANGELOG.md` entry, then `pio pkg publish hostLib`.
