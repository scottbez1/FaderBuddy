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
- **firmware** (optional, default: the newest firmware known to this component): Fader firmware to offer as an update. Most setups should leave this unset; see [Firmware updates](#firmware-updates) and [Choosing the firmware image](#choosing-the-firmware-image).
- **firmware_image** (optional): Path to a locally built firmware image, for testing unreleased firmware. Mutually exclusive with `firmware`.
- **firmware_update** (optional): Options for the auto-created firmware update entity. Not created with `firmware: none`. Config category.
- **serial_number** (optional): Options for the auto-created serial number text sensor (`name`, `id`, `icon`, `internal`, …). Diagnostic category.
- **status** (optional): Options for the auto-created status text sensor, which shows the firmware version, update availability and update progress. Diagnostic category.
- **self_calibration** (optional): Options for the auto-created self-calibration button. Config category.

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

The scale is linear in velocity, and both ends are usable — `0` is the slowest speed the mechanism sustains without creeping in stick-slip steps (roughly 700ms for full travel), and `255` removes the limit entirely.

This sets a speed limit, so a half-scale move takes about half as long as a full-scale move. Lower speeds can help when several faders move at once.

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

Each fader shows up in Home Assistant as a **Firmware** update entity, and can be
updated over I2C from there, with no programmer needed.

Each release of this component includes the newest fader firmware available at
the time. After you update the component and reflash your ESP32, Home Assistant
shows an update for any fader running older firmware. Updates never install on
their own: press **Install** in Home Assistant, or skip the update if you don't
want it.

During an update the entity shows a progress bar, and the fader's **Status**
sensor shows info about the update, including any failure info.

Faders with firmware older than 1.3 can't be updated over I2C. They still show an
update in Home Assistant, but installing it does nothing; the Status sensor says
the fader needs a one-time reflash with a UPDI programmer (see
`ABOUT_I2C_BOOTLOADER.md`).

To choose a different firmware version, or drive updates from automations, see
[Advanced: firmware updates](#advanced-firmware-updates).

## Text Sensors

Each fader creates two diagnostic text sensors automatically:

- **serial_number** — the microcontroller's 10-byte factory ID as an uppercase
  hex string, handy for identifying a specific board. Read once at startup.
- **status** — the fader's state as text, e.g. `Firmware 1.4`,
  `Firmware 1.3 - update to 1.4 available`, `Not responding`, or progress
  during an update.

Both take the standard ESPHome text sensor options, on the hub:

```yaml
fader_buddy:
  - id: my_fader
    serial_number:
      name: "Fader Serial Number"
    status:
      name: "Fader Status"
      # internal: true        # to keep it out of Home Assistant entirely
```

The older `text_sensor: platform: fader_buddy` form still works for
`serial_number` and will be removed in component 0.5.0.

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

// Firmware update (see the update.perform action for the usual way to do this)
bool pending = id(my_fader).firmware_update_available();
id(my_fader).start_firmware_update();   // pass true to force
id(my_fader).refresh_firmware_state();  // re-read the version / re-probe
```

## Troubleshooting

**Fader not detected:**
- Check I2C wiring (SDA, SCL, GND, Vio, Vmot)
- Verify I2C address matches your hardware configuration
- Enable `scan: true` in the I2C config to see detected addresses in logs
- Two faders strapped to the same address look like this on a chain: the
  duplicated address returns garbage, and the address nobody is using goes
  silent. The boot scan is the quickest way to rule it out - count the
  addresses, not the faders

**A fader's serial number and status both read Unknown:**

Both sensors are published during initialization, which only runs once the
fader answers a probe of `REG_VERSION`, so Unknown means that probe never got a
usable answer. The boot log says which of the two cases it was:

- `Init: no response from the fader at 0xNN after 5 attempts` - it NAKed, or
  isn't there. The component keeps re-probing in the background (backing off
  to every 30 seconds), so once the fader is connected it initializes itself
  without a reboot. Until then the status sensor reads `Not responding`. If the
  fader is connected but still doesn't answer, try installing the **Firmware**
  update, which can recover a fader with corrupted firmware.
- `Init: Incompatible I2C protocol version ... got N` - it answered, with a
  protocol older than v5. That firmware also predates I2C bootloader entry
  (firmware 1.3), so it needs a one-time UPDI reflash; no update over I2C can
  reach it. The component is marked failed.

A fader sitting in its bootloader with no application is *neither* of these -
it answers the probe with its own marker, reports `Bootloader - no application
installed` on its status sensor, and is recovered by installing the
**Firmware** update.

**Fader moves in wrong direction:**
- Set `invert: true` in the component configuration

**Trigger fires too frequently or sporadic movement in a bidirectional setup:**
- Use `value_change_min_interval` in `layer_haptics` to rate limit
- Example: `value_change_min_interval: 100ms` limits to 10 updates per second
- In a bidirectional setup, slow devices such as Zigbee lights can cause Home Assistant to queue brightness updates. These may move the fader after you let go, replaying your input. Rate limiting reduces this backlog.

**Fader doesn't move to commanded position:**
- Run self-calibration: `fader_buddy.run_self_calibration`
- Check that you're not in an error state (power cycle if needed)

## Advanced: firmware updates

### Choosing the firmware image

By default each fader packages the newest version listed in `KNOWN_FIRMWARE` (in
`esphome/components/fader_buddy/__init__.py`). The `firmware` and
`firmware_image` options change that:

```yaml
fader_buddy:
  - id: fader_a
    firmware: "1.3"               # a specific released version
  - id: fader_b
    firmware: none                # no image and no update entity
  - id: fader_c
    firmware_image: fader_app.bin # a locally built image
```

- **Pinning a version** is rarely needed. Updates are never installed
  automatically, so you can skip one in Home Assistant, or pin the component
  itself with `ref:` in `external_components`.
- **`firmware: none`** removes the update entity. It also saves about 14 KB of
  ESP32 flash per distinct firmware version, and the download at build time.
- **`firmware_image`** is for testing unreleased firmware. Build the image with
  `python3 firmware/tools/export_app_image.py --output fader_app.bin` (with the
  PlatformIO environment activated).

Released images are downloaded from the GitHub release tagged
`releases/firmware/v<major>.<minor>` at compile time, checked against the sha256
in `KNOWN_FIRMWARE`, and cached, so only the first build needs network access.
Nothing is downloaded at runtime. For a version that isn't in `KNOWN_FIRMWARE`,
give the hash yourself:

```yaml
    firmware:
      version: "1.3"
      url: https://example.com/fader_buddy_app_v1.3.bin   # optional, defaults to the release asset
      sha256: 2d2e56fa...                                  # required if not in KNOWN_FIRMWARE
```

`esphome config` also checks that the image is a complete application image, and
that the firmware version inside it matches the version you asked for.

### Updating from automations

```yaml
# Same as pressing Install in Home Assistant:
- update.perform: my_fader_firmware_update_id

# force_update is needed to downgrade, which Home Assistant won't offer. A fader
# already running the packaged version is still left alone.
- update.perform:
    id: my_fader_firmware_update_id
    force_update: true

# The component's own action, the same as a forced update.perform:
- fader_buddy.update_firmware:
    id: my_fader
```

Use the `update.is_available` condition to check whether an update is available.

These actions return immediately and the update runs in the background. To act
on the result, use the hub's `on_firmware_update_result` trigger, which gets
`success` (bool) and `message` (the failure reason):

```yaml
fader_buddy:
  - id: my_fader
    on_firmware_update_result:
      - logger.log:
          format: "Fader update %s: %s"
          args: ['success ? "succeeded" : "failed"', 'message.c_str()']
```

### Reported versions

Home Assistant decides whether to offer an update by comparing version strings,
so the update entity always reports a valid version. Extra detail goes in a
suffix that doesn't affect the comparison:

| Reported | Means |
| --- | --- |
| `1.4` | running firmware 1.4 |
| `1.1+updi-required` | running 1.1, too old to update over I2C |
| `1.0+updi-required` | running 1.0 or older (no version register) |
| `0.0+bootloader` | in its bootloader, no application installed |
| `0.0+unreachable` | never answered a probe |

Home Assistant's "check for updates" re-reads the fader's version (or re-probes
it if it never answered), which picks up a fader that was reflashed over UPDI.

### Update progress in Home Assistant

With the default `api: batch_delay:` of 100ms, Home Assistant only sees some of
the progress steps. Set `batch_delay: 0ms` to see all of them. The ESPHome log
always shows every step.

### Cutting a firmware release

```bash
git tag releases/firmware/v1.3 && git push origin releases/firmware/v1.3
```

CI builds the image, checks that the tag matches the firmware's `FW_VERSION`,
attaches `fader_buddy_app_v1.3.bin` to the release, and prints the
`KNOWN_FIRMWARE` line to add to the component.

## Additional Resources

- **Layer Architecture:** See `ABOUT_LAYERS.md` for detailed information about the layer system
- **I2C Protocol:** See `firmware/src/shared/i2c_data.h` for low-level protocol details
- **Example Configurations:** See `esphome/examples/` directory
