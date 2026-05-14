# FaderBuddy Layers Architecture

## Overview

The FaderBuddy system implements **8 independent layers** per physical fader, allowing a single fader to represent multiple logical controls. Each layer maintains its own position and haptic configuration, enabling use cases like multi-scene mixing, parameter banks, or controlling multiple Home Assistant entities from one fader.

## Key Concepts

### What is a Layer?

A layer is an independent logical control context stored in the fader's volatile memory and fully managed within the fader firmware. Think of it as a separate "preset" that the fader can switch between:

- **8 layers per fader** (indexed 0-7)
- **Per-layer restore position**: Each layer remembers where its last-known "target" position is
- **Per-layer haptic config**: Each layer can have different haptic feedback (smooth, detents, magnetic endpoints)
- **Independent state**: Layers are isolated - changing one doesn't affect others

### Active Layer

Only one layer is **active** at any time. The active layer determines:
- What position the physical fader should be at
- What haptic feedback the user feels
- Which layer's state is reported in the STATE register

The firmware manages layer switching and ensures smooth transitions between layers.

## Architecture

### Firmware-Managed Layers

Layer state lives entirely in the **ATtiny1616 firmware**. The firmware maintains:

- Haptic configuration for each of the 8 layers
- Target/restore position for each of the 8 layers
- Currently active layer index (0-7)
- Pending layer change (for deferred switching)

**Firmware location**: `firmware/src/main.cpp`

### I2C Protocol

Layer operations use **layer-addressed registers**:

- **REG_ACTIVE_LAYER**: Read/write the active layer index (0-7)
- **REG_LAYER_TARGET**: Read/write target position for a specific layer
- **REG_LAYER_HAPTIC_CONFIG**: Read/write haptic config for a specific layer

See `firmware/src/shared/i2c_data.h` for complete protocol definitions including register addresses, STATE register bit fields, and haptic configuration format.

#### Layer-Addressed Register Protocol

Reading/writing specific layers uses a protocol where the layer index (0-7) is included in the transaction:

**Write to a layer's target position:**
```
I2C Write: [REG_LAYER_TARGET, layer_index, position_value]
```

**Read a layer's haptic config:**
```
I2C Write: [REG_LAYER_HAPTIC_CONFIG, layer_index]  // Set register and layer to query
I2C Read:  [config_bytes...]                       // Firmware returns config for specified layer
```

### STATE Register

The STATE register includes the active layer index (0-7) as one of its packed fields. All state reports indicate which layer is currently active, allowing the host to correlate position/touch/mode information with the correct layer context.

## Layer Switching Behavior

### Deferred Layer Changes

Layer changes are **deferred** during user input to prevent interrupting the user:

**MODE_INPUT_ACTIVE** (user is touching/moving fader):
- Layer change request is stored as pending
- Firmware continues operating on current layer
- When user releases fader and it transitions to INPUT_IDLE, the pending layer change applies

**MODE_INPUT_IDLE or MODE_REMOTE_MOVEMENT_IN_PROGRESS**:
- Layer change applies immediately
- Fader moves to the new layer's restore position
- Remote movements can be interrupted by layer changes

**MODE_SELF_CALIBRATION**:
- Layer change deferred until calibration completes
- Applies when calibration finishes

**MODE_ERROR**:
- Layer change requests are ignored (not deferred)
- Error must be cleared before layer switching is allowed

### Restore Position Updates

Each layer's restore position is updated automatically:

**During user input** (MODE_INPUT_ACTIVE):
- `restore_position[active_layer]` continuously tracks actual fader position
- Captures the user's intended position for this layer

**When starting remote movement**:
- `restore_position[active_layer]` set to the target position
- Not updated during the movement itself

**When writing to inactive layer**:
- `restore_position[layer]` updated without moving physical fader
- Position restored when switching to that layer

## ESPHome Component

The ESPHome component acts as a **thin protocol wrapper**, allowing static initial layer configurations to be defined in yaml, and forwarding layer operations to the firmware:

**Component location**: `esphome/components/fader_buddy/`

### Key Methods
These can be called from within lambdas.

```cpp
// Switch to a different layer (may be deferred by firmware)
void set_active_layer(uint8_t layer_index);

// Get currently active layer from firmware
uint8_t get_active_layer() const;

// Set target position for a specific layer
void remote_move_to(uint8_t position, uint8_t layer = 0);

// Get restore position for a specific layer
uint8_t get_position(uint8_t layer = 0) const;

// Set haptic configuration for a specific layer
void set_layer_haptic_config(uint8_t layer, uint8_t mode,
                              uint8_t detent_count, uint8_t detent_strength);
```

### Automation Triggers

All triggers include the active layer index:

```yaml
on_manual_move:
  - lambda: |-
      // x = position, layer = active layer index
      ESP_LOGD("fader", "Layer %d moved to %d", layer, x);

on_touch_change:
  - lambda: |-
      // x = touch state (bool), layer = active layer index

on_double_tap:
  - lambda: |-
      // layer = active layer index when double-tap occurred
```

### Example: Two-Light Controller

This example shows a single fader controlling two different lights based on the active layer:

```yaml
esphome:
  name: fader-controller

fader_buddy:
  - id: my_fader
    address: 0x20
    update_interval: 10ms

    # Configure different haptic feels per layer
    layer_haptics:
      - layer: 0
        mode: smooth
      - layer: 1
        mode: detents
        detent_count: 10
        detent_strength: 4

    # When user moves the fader, update the appropriate light
    on_manual_move:
      - lambda: |-
          float brightness = x / 2.55;  // Convert 0-255 to 0-100

          if (layer == 0) {
            auto call = id(living_room_light).turn_on();
            call.set_brightness(brightness / 100.0);
            call.perform();
          } else if (layer == 1) {
            auto call = id(bedroom_light).turn_on();
            call.set_brightness(brightness / 100.0);
            call.perform();
          }

light:
  - platform: homeassistant
    id: living_room_light
    entity_id: light.living_room

  - platform: homeassistant
    id: bedroom_light
    entity_id: light.bedroom

# Buttons to switch between layers
binary_sensor:
  - platform: gpio
    pin: GPIO4
    name: "Living Room Button"
    on_press:
      - fader_buddy.set_active_layer:
          id: my_fader
          layer: 0

  - platform: gpio
    pin: GPIO5
    name: "Bedroom Button"
    on_press:
      - fader_buddy.set_active_layer:
          id: my_fader
          layer: 1
```

When a layer switch occurs, the fader automatically moves to that layer's last position, so switching to "bedroom" will restore the bedroom light's previous brightness setting.

If you wanted the motor faders to respond to external changes in light brightness, you could add a homeassistant sensor that monitors the brightness and calls remote moves on the appropriate layer for each light.

## Haptic Configuration

Each of the 8 layers has its own haptic configuration containing:

- **Haptic mode**: smooth, smooth with magnetic endpoints, or detents
- **Detent count**: Number of detent positions (for detents mode)
- **Detent strength**: Force feedback strength

Haptic configs can be set statically in YAML or changed dynamically via actions.

## Example Use Cases

### Multi-Scene Control
- Layer 0: Living room lights
- Layer 1: Bedroom lights
- Layer 2: Kitchen lights
- Each layer remembers its last brightness setting

### Parameter Banks
- Layer 0-7: Different synthesizer parameters
- Switch layers to control different aspects of sound
- Each parameter can have different haptic feedback (smooth vs stepped)

### Mixing Scenes
- Store complete mixer states across layers
- Recall different mixes by switching all faders to the same layer
- Each scene preserves fader positions independently

## Implementation Details

### Firmware

**Layer management functions** (`firmware/src/main.cpp`):
- `request_layer_change()`: Handles layer change requests with deferral logic
- `apply_layer_change()`: Executes layer switch and initiates movement
- `write_layer_target()`: Handles target position writes for any layer
- `write_layer_haptic_config()`: Handles haptic config writes for any layer

**I2C handlers**: Implement layer-addressed register protocol using `queried_layer` state variable to track two-step read operations.

**Mode transitions**: Check for `pending_layer_change` when entering MODE_INPUT_IDLE and apply if present.

### ESPHome

**Configuration** (`esphome/components/fader_buddy/__init__.py`):
- Python schema for `layer_haptics` YAML configuration
- Code generation for actions and triggers with layer parameters

**Component** (`fader_buddy.cpp`):
- I2C wrapper methods that forward operations to firmware
- Trigger firing with active layer extracted from STATE register
- No layer state management - firmware is source of truth

## References

- **I2C Protocol and register definitions**: `firmware/src/shared/i2c_data.h`
- **Firmware implementation**: `firmware/src/main.cpp`
- **ESPHome component**: `esphome/components/fader_buddy/`
- **Complete example with display**: `esphome/examples/multi-fader-display.yaml`
