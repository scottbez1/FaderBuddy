# Motor Fader ESPHome Integration

This guide shows you how to use the motor fader with ESPHome to create smart motorized controls for Home Assistant or other home automation systems.

## Hardware Setup

### Wiring

Connect your motor fader(s) to your ESP32 via I2C:

- **SDA** → ESP32 GPIO pin (e.g., GPIO12)
- **SCL** → ESP32 GPIO pin (e.g., GPIO13)
- **GND** → ESP32 GND
- **Vio** → 3.3V power supply
- **Vmot** → 5V power supply

**I2C Addressing**: Each motor fader has a configurable I2C address (default 0x20, configurable via hardware jumpers to 0x20-0x27). When using multiple faders on the same I2C bus, ensure each has a unique address.

### I2C Bus Configuration

In your ESPHome YAML, configure the I2C bus:

```yaml
i2c:
  sda: GPIO12
  scl: GPIO13
  scan: true  # Optional: helps verify faders are detected
```

## Adding the Component

### External Component Reference

Add the motor fader component to your ESPHome configuration using the GitHub repository:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/scottbez1/FaderBuddy.git
      ref: main  # or specify a specific tag/branch
    components: [motor_fader]
```

## Basic Configuration

### Single Fader Setup

Here's a minimal configuration for one motor fader:

```yaml
motor_fader:
  - id: my_fader
    address: 0x20          # I2C address (default 0x20)
    update_interval: 10ms  # How often to poll the fader
```

### Multiple Faders

To use multiple faders, add multiple entries with unique IDs and addresses:

```yaml
motor_fader:
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
  - **value_change_min_interval** (optional, default: `0ms`): Rate limiting for `on_manual_move` trigger on this layer. Useful to reduce traffic when controlling networked devices. Set to `0ms` for no rate limiting.

### Triggers

The component provides three triggers that fire when the user interacts with the fader:

#### on_manual_move

Fires when the user moves the fader or when the fader position changes. Provides the current position (0-255) and active layer index.

```yaml
motor_fader:
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
motor_fader:
  - id: my_fader
    on_touch_change:
      then:
        - lambda: |-
            // x = touch state (true/false)
            // layer = active layer (0-7)
            ESP_LOGD("fader", "Touch: %s (on layer %d", x ? "pressed" : "released", layer);
```

#### on_double_tap

Fires when the user double-taps the fader. Provides the active layer index.

```yaml
motor_fader:
  - id: my_fader
    on_double_tap:
      then:
        - lambda: |-
            // layer = active layer (0-7)
            ESP_LOGD("fader", "Double tap on layer %d", layer);
```

## Actions

### motor_fader.remote_move_to

Command the fader to move to a specific position.

```yaml
# Example: Move fader to position 128
- motor_fader.remote_move_to:
    id: my_fader
    position: 128
    layer: 0  # Optional, defaults to layer 0
```

**Position:** 0-255 (0 = bottom, 255 = top, unless inverted)

**Layer:** Optional layer index (0-7). If the specified layer is not currently active, the position is stored and will be restored when that layer becomes active.

### motor_fader.set_active_layer

Switch to a different layer (0-7). The fader will automatically move to that layer's last position.

```yaml
# Example: Button to switch to layer 1
binary_sensor:
  - platform: gpio
    pin: GPIO4
    on_press:
      - motor_fader.set_active_layer:
          id: my_fader
          layer: 1
```

### motor_fader.set_layer_haptic_config

Dynamically change the haptic configuration for a layer.

```yaml
# Example: Set layer 2 to detents mode with 10 detents
- motor_fader.set_layer_haptic_config:
    id: my_fader
    layer: 2
    mode: detents
    detent_count: 10
    detent_strength: 5
```

### motor_fader.run_self_calibration

Trigger the fader's self-calibration routine. The fader will automatically move to both endpoints to calibrate its potentiometer range.

```yaml
# Example: Calibration button
button:
  - platform: template
    name: "Calibrate Fader"
    on_press:
      - motor_fader.run_self_calibration:
          id: my_fader
```

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
```

## Complete Example: Light Brightness Control

This example shows a single fader controlling a Home Assistant light's brightness:

```yaml
motor_fader:
  - id: brightness_fader
    address: 0x20
    update_interval: 10ms
    layer_haptics:
      - layer: 0
        mode: smooth
        value_change_min_interval: 100ms  # Limit updates to 10/second

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
    entity_id: light.living_room
    attribute: brightness
    internal: true
    on_value:
      then:
        - lambda: |-
            id(brightness_fader).remote_move_to(isnan(x) ? 0 : x);
```

## Examples

For complete working examples, see the `esphome/examples/` directory:

- **multi-fader-display.yaml** - Three faders with an LVGL display, showing haptic configuration, layer setup, and Home Assistant integration

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
- Run self-calibration: `motor_fader.run_self_calibration`
- Check that you're not in an error state (power cycle if needed)

## Additional Resources

- **Layer Architecture:** See `ABOUT_LAYERS.md` for detailed information about the layer system
- **I2C Protocol:** See `firmware/src/shared/i2c_data.h` for low-level protocol details
- **Example Configurations:** See `esphome/examples/` directory
