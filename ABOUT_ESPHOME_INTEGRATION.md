# FaderBuddy ESPHome Integration

This guide shows you how to use the FaderBuddy with ESPHome to create smart motorized controls for Home Assistant or other home automation systems.

## Hardware Setup

### Wiring

Connect your FaderBuddy board(s) to your ESP32 via I2C:

- **SDA** → ESP32 GPIO pin (e.g., GPIO8)
- **SCL** → ESP32 GPIO pin (e.g., GPIO9)
- **GND** → ESP32 GND
- **Vio** → 3.3V power supply
- **Vmot** → 5V power supply

**I2C Addressing**: Each FaderBuddy has a configurable I2C address (default 0x20, configurable via hardware jumpers to 0x20-0x27). When using multiple faders on the same I2C bus, ensure each has a unique address.

### I2C Bus Configuration

In your ESPHome YAML, configure the I2C bus:

```yaml
i2c:
  sda: GPIO8
  scl: GPIO9
  scan: true  # Optional: helps verify faders are detected
```

## Adding the Component

### External Component Reference

Add the fader_buddy component to your ESPHome configuration using the GitHub repository:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/scottbez1/FaderBuddy.git
      ref: master  # or specify a specific tag/branch
    components: [fader_buddy]
```

## Basic Configuration

### Single Fader Setup

Here's a minimal configuration for one FaderBuddy:

```yaml
fader_buddy:
  - id: my_fader
    address: 0x20          # I2C address (default 0x20)
    update_interval: 10ms  # How often to poll the fader
```

### Multiple Faders

To use multiple faders, add multiple entries with unique IDs and addresses:

```yaml
fader_buddy:
  - id: fader_1
    address: 0x20
    update_interval: 10ms

  - id: fader_2
    address: 0x21
    update_interval: 10ms

  - id: fader_3
    address: 0x22
    update_interval: 10ms
```

## Complete Example: Light Brightness Control

This example shows bidirectional control - a single fader controlling a Home Assistant light's brightness and responding to remote changes to that light's brightness:

```yaml
fader_buddy:
  - id: brightness_fader
    address: 0x20
    update_interval: 10ms
    layer_haptics:
      - layer: 0
        mode: smooth
        default_speed: 120  # move deliberately rather than instantly

    # User moves fader → update light brightness
    on_manual_move:
      then:
        - homeassistant.action:
            action: light.turn_on
            data:
              entity_id: light.living_room
              brightness: !lambda 'return x;'
              transition: "0"

# Light brightness changes in Home Assistant → move fader
sensor:
  - platform: homeassistant
    id: living_room_brightness
    entity_id: light.living_room
    attribute: brightness
    internal: true
    on_value:
      then:
        - lambda: |-
            id(brightness_fader).remote_move_to(isnan(x) ? 0 : x);
```

For additional complete working examples, see the `esphome/examples/` directory:

- **multi-fader-display.yaml** - Three faders with an LVGL display, showing haptic configuration, layer setup, and Home Assistant integration

## Configuration Options

### Component Configuration

- **id** (required): Unique identifier for this fader instance, used to reference it in automations and lambdas
- **address** (optional, default: `0x20`): I2C address of the fader (0x20-0x27)
- **update_interval** (optional, default: `50ms`): How often to poll the fader for state updates
- **invert** (optional, default: `false`): Reverse the fader direction (position 0 becomes 255, and vice versa)
- **layer_haptics** (optional): List of haptic configurations for specific layers. See `ABOUT_LAYERS.md` for more details on layers
  - **layer** (required): Layer index (0-7) for this list item
  - **mode** (required): Haptic mode - one of:
    - `smooth`: No haptic feedback, completely smooth movement
    - `smooth_with_magnets`: Smooth with magnetic endpoints that pull the fader to min/max
    - `detents`: Creates distinct "notches" along the fader's travel (requires `detent_count`)
  - **detent_count** (optional, default: `0`): Number of detents (1-15, for detents mode only)
  - **detent_strength** (optional, default: `0`): Detent force feedback strength (0-7, for detents mode only)
  - **default_speed** (optional, default: `255`, requires fader firmware 1.1+): Move speed for this layer, 0-255, used by `fader_buddy.remote_move_to` and by lambda calls to `remote_move_to()` that don't pass a speed of their own. `255` is full speed; see the `speed` parameter of `fader_buddy.remote_move_to` below. This one is tracked in ESPHome rather than on the fader — the component looks up the active layer's value and sends it with each move — so changing it costs no extra I2C traffic.
  - **value_change_min_interval** (optional, default: `0ms`): Rate limiting for `on_manual_move` trigger on this layer. Useful to reduce traffic when controlling networked devices like zigbee lights. Set to `0ms` for no rate limiting. Keep as low as possible.
- **firmware** (optional): Released fader firmware to package for I2C updates. See [Firmware updates](#firmware-updates) below. Mutually exclusive with `firmware_image`.
- **firmware_image** (optional): Path to a locally built application image, for iterating on an unreleased build. Mutually exclusive with `firmware`.
- **max_update_attempts** (optional, default: `3`): How many times a failed update is retried for a given target version before the component refuses to try again. The count persists across ESP32 reboots, so a fader that consistently fails to take an update stops being retried rather than looping forever.

### Triggers

The component provides four triggers that fire in response to fader state changes:

#### on_manual_move

Fires when the user moves the fader. Only fires during `MODE_INPUT_ACTIVE` or `MODE_INPUT_IDLE` (i.e. not during remote motor movements). Respects `value_change_min_interval` rate limiting configured in `layer_haptics`. Provides the current position (0-255) and active layer index.

```yaml
fader_buddy:
  - id: my_fader
    on_manual_move:
      then:
        - lambda: |-
            // x = position (0-255)
            // layer = active layer (0-7)
            ESP_LOGD("fader", "Fader moved to %d on layer %d", x, layer);
```

#### on_touch_change

Fires when the user touches or releases the fader. Provides touch state (true/false) and active layer index.

```yaml
fader_buddy:
  - id: my_fader
    on_touch_change:
      then:
        - lambda: |-
            // x = touch state (true/false)
            // layer = active layer (0-7)
            ESP_LOGD("fader", "Touch: %s (on layer %d)", x ? "pressed" : "released", layer);
```

#### on_double_tap

Fires when the user double-taps the fader. Provides the active layer index.

```yaml
fader_buddy:
  - id: my_fader
    on_double_tap:
      then:
        - lambda: |-
            // layer = active layer (0-7)
            ESP_LOGD("fader", "Double tap on layer %d", layer);
```

#### on_raw_position_update

_Not recommended for general use!_ Fires on every position change detected by the hardware, regardless of the current mode and without any rate limiting. This includes position changes during remote motor movements (`MODE_REMOTE_MOVEMENT_IN_PROGRESS`), not just user-initiated moves. Provides the current position (0-255, after applying `invert`) and active layer index.

For most use cases, prefer `on_manual_move`. Use `on_raw_position_update` only when you specifically need to track the fader's physical position at all times — for example, to update a display that should reflect position even while the motor is actively moving.

```yaml
fader_buddy:
  - id: my_fader
    on_raw_position_update:
      then:
        - lambda: |-
            // x = position (0-255)
            // layer = active layer (0-7)
            ESP_LOGD("fader", "Raw position: %d on layer %d", x, layer);
```
## Actions

### fader_buddy.remote_move_to

Command the fader to move to a specific position.

```yaml
# Example: Move fader to position 128
- fader_buddy.remote_move_to:
    id: my_fader
    position: 128
    layer: 0  # Optional, defaults to layer 0

# Example: move gently rather than at full speed
- fader_buddy.remote_move_to:
    id: my_fader
    position: 200
    speed: 80
```

**Position:** 0-255 (0 = bottom, 255 = top, unless inverted)

**Layer:** Optional layer index (0-7). If the specified layer is not currently active, the position is stored and will be restored when that layer becomes active.

**speed:** Optional. How fast to move, as a unitless **0-255** value: `255` is full speed, `0` is the slowest the fader moves smoothly. Omit it to use the layer's `default_speed`.

Requires fader firmware **1.1 or newer**. On older firmware the component logs a warning once and moves at full speed instead; it checks the firmware version at startup, so a `default_speed` that cannot be honoured is reported then rather than on the first move.

The scale is linear in velocity, and both ends are usable — `0` is the slowest speed the mechanism sustains without creeping in stick-slip steps (roughly 700ms for full travel), and `255` removes the limit entirely. There is nothing outside the range worth reaching for.

This caps the fader's speed rather than scheduling the move, so a shorter move takes proportionally less time — at a given speed, a half-scale move takes about half as long as a full-scale one. Slowing moves down is mostly useful when several faders move at once, or when you want motion to read as deliberate rather than instant.

Treat it as a limit rather than a precise speed. It is realised through a friction-dependent mechanism, so actual velocity lands within about 15% of nominal, with around 7% difference between moving up and moving down.

Moving a layer that is not currently active also stores the speed, so it applies when that layer is restored later.

### fader_buddy.set_active_layer

Switch to a different layer (0-7). The fader will automatically move to that layer's last position.

```yaml
# Example: Button to switch to layer 1
binary_sensor:
  - platform: gpio
    pin: GPIO4
    on_press:
      - fader_buddy.set_active_layer:
          id: my_fader
          layer: 1
```

### fader_buddy.set_layer_haptic_config

Dynamically change the haptic configuration for a layer.

```yaml
# Example: Set layer 2 to detents mode with 10 detents
- fader_buddy.set_layer_haptic_config:
    id: my_fader
    layer: 2
    mode: detents
    detent_count: 10
    detent_strength: 5
```

### fader_buddy.run_self_calibration

Trigger the fader's self-calibration routine. The fader will automatically move to both endpoints to calibrate its potentiometer range.

```yaml
# Example: Calibration button
button:
  - platform: template
    name: "Calibrate Fader"
    on_press:
      - fader_buddy.run_self_calibration:
          id: my_fader
```

## Firmware updates

The fader's own ATtiny1616 firmware can be updated over I2C through its bootloader,
without a UPDI programmer. The image is embedded in the ESP32 build and streamed to
the fader when you call `fader_buddy.update_firmware`.

### Referencing a released image

Application images are published as assets on a GitHub release, tagged
`releases/firmware/v<major>.<minor>` — the same tag scheme the electronics artifacts
use. Naming the version is enough; the download URL follows from it:

```yaml
fader_buddy:
  - id: my_fader
    firmware: "1.3"
```

The image is downloaded at **compile time**, verified against the `sha256` recorded
in `KNOWN_FIRMWARE` (in `esphome/components/fader_buddy/__init__.py`), cached by
hash, and compiled into the ESP32 binary. Nothing is fetched at runtime.

The hash is the point: it pins the exact bytes, so a re-uploaded or substituted
release asset fails the build rather than being flashed onto a fader. A version with
no `KNOWN_FIRMWARE` entry is an error unless you supply the hash yourself:

```yaml
    firmware:
      version: "1.3"
      url: https://example.com/fader_buddy_app_v1.3.bin   # optional, defaults to the release asset
      sha256: 2d2e56fa...                                  # required if not in KNOWN_FIRMWARE
```

Two things are checked at config time, so problems surface from `esphome config`
rather than partway through a compile or — worse — on the fader:

- the image is the expected size and page-aligned, i.e. actually an app image and
  not a truncated download or an Intel-hex;
- the `FW_VERSION` baked into the image's last two bytes matches the version you
  asked for, catching a mislabelled or wrongly attached release asset.

### Using a locally built image

For firmware you haven't released yet, build and point at the file directly:

```bash
source ~/.platformio/penv/bin/activate
python3 firmware/tools/export_app_image.py --output fader_app.bin
```

```yaml
fader_buddy:
  - id: my_fader
    firmware_image: fader_app.bin
```

The binaries are **not** checked into the repository — they are build artifacts,
reproducible from any tagged commit.

### Triggering an update

A fader with `firmware:` or `firmware_image:` configured gets a **Firmware
Update** button automatically, so there is nothing to write for the common case —
it shows up in Home Assistant under the device's settings
(`entity_category: config`). Without a configured image there is nothing to
install, so no button is created. Rename or hide it on the hub:

```yaml
fader_buddy:
  - id: my_fader
    firmware: "1.3"
    firmware_update:
      name: "Update Fader Firmware"
      # internal: true          # to keep it out of Home Assistant entirely
```

A press is ignored unless there is genuinely something to install — a fader
already running the packaged version, or firmware too old to reach its
bootloader, just logs a line and does nothing, rather than tying up the I2C bus
for tens of seconds to reach the same answer. That check happens at press time
because whether an update is pending is runtime state rather than something the
yaml knows.

The equivalent action, for driving an update from an automation:

```yaml
button:
  - platform: template
    name: "Update Fader Firmware"
    on_press:
      - fader_buddy.update_firmware:
          id: my_fader
```

Updates only ever happen when the button is pressed or the action runs — there is
no automatic update mode. The component refuses to start if the fader is being
touched, if the target version is already installed, if `max_update_attempts` has
been reached, or if the fader's firmware predates I2C bootloader entry
(`FW_VERSION` below 1.3), which needs a one-time UPDI migration.

The action **returns immediately** — the transfer then runs a slice at a time from
the main loop, so the device stays responsive throughout. The outcome arrives on
`on_firmware_update_result`, not when the action returns.

### Watching an update

The **Firmware Version** text sensor doubles as the update's status field — a fader
mid-update has no version to report, so it says what it is doing instead:
`waiting for fader` → `entering bootloader` → `erasing` → `writing 25%` … →
`verifying` → `starting app`, then the new version (e.g. `1.5`). A failed update
ends on the fader's real state with the reason appended, e.g. `1.3 (update failed)`
or `bootloader (no app) (update failed)` if it was stranded mid-write. A fader that
is sitting in its bootloader at startup reads `bootloader (no app)` from the outset.

How much of the progress Home Assistant actually renders depends on `api:
batch_delay:`. Entity states are batched and deduplicated per entity, so at the
default 100ms HA sees the steps that happen to straddle a flush rather than all of
them. Set `batch_delay: 0ms` to see each one. The ESPHome log always shows every
step, and the terminal states always get through either way.

The serial number sensor is re-read after a successful update too — a fader that
booted into its bootloader could never report one, since `REG_SERIAL` is an app
register.

One case is updatable even though no version can be read: a fader sitting in its
bootloader with no working app image (see the forced-entry strap in
`ABOUT_I2C_BOOTLOADER.md`). The component notices that at startup, logs it, and
keeps the button live so the fader can be recovered.

### Cutting a release

```bash
git tag releases/firmware/v1.3 && git push origin releases/firmware/v1.3
```

CI builds the image, refuses to publish if the tag and the firmware's own
`FW_VERSION` disagree, attaches `fader_buddy_app_v1.3.bin` to the release, and
prints the `KNOWN_FIRMWARE` line to paste into the component.

## Text Sensors

### serial_number

You can expose the microcontroller's serial number as a text sensor, which is handy for identifying a specific board for diagnostics. The serial is read once at startup and published as an uppercase hex string.


```yaml
text_sensor:
  - platform: fader_buddy
    fader_buddy_id: my_fader
    serial_number:
      name: "Fader Serial Number"
```

The `serial_number` block accepts the standard ESPHome text sensor options (e.g. `name`, `id`, `icon`). It defaults to the `diagnostic` entity category.

## C++ API (for Lambdas)

When writing lambda expressions, you can call these methods directly on the component:

```cpp
// Layer management
id(my_fader).set_active_layer(layer_index);       // Switch to layer 0-7
uint8_t layer = id(my_fader).get_active_layer();  // Get current active layer

// Position control
id(my_fader).remote_move_to(position, layer);     // Move on specific layer
uint8_t pos = id(my_fader).get_position(layer);   // Get position for layer

// Haptic configuration
id(my_fader).set_layer_haptic_config(layer, mode, detent_count, detent_strength);

// Calibration
id(my_fader).run_self_calibration();

// Serial number (empty until read at startup)
std::string serial = id(my_fader).get_serial_number();
```

## Troubleshooting

**Fader not detected:**
- Check I2C wiring (SDA, SCL, GND, Vio, Vmot)
- Verify I2C address matches your hardware configuration
- Enable `scan: true` in the I2C config to see detected addresses in logs

**Fader moves in wrong direction:**
- Set `invert: true` in the component configuration

**Trigger fires too frequently or sporadic movement in a bidirectional setup:**
- Use `value_change_min_interval` in `layer_haptics` to rate limit
- Example: `value_change_min_interval: 100ms` limits to 10 updates per second
- This can be useful if you have a bidirectional setup (i.e. moving the fader controls a light in HASS, and changing a light in HASS moves the fader) and the on_manual_move trigger takes a while to be confirmed/reflected, for example when updating a high-roundtrip-latency light like a Zigbee light. Without rate-limiting, this can often result in weird motor movements after manually moving the fader, as HASS may queue up brightness changes that happen too quickly and then deliver them for while after you've already let go, causing the motor to almost replay your previous movement.

**Fader doesn't move to commanded position:**
- Run self-calibration: `fader_buddy.run_self_calibration`
- Check that you're not in an error state (power cycle if needed)

## Additional Resources

- **Layer Architecture:** See `ABOUT_LAYERS.md` for detailed information about the layer system
- **I2C Protocol:** See `firmware/src/shared/i2c_data.h` for low-level protocol details
- **Example Configurations:** See `esphome/examples/` directory
