# Motor Fader ESPHome Component - Layers Design Document

## Overview

This document describes the design for implementing "layers" in the motor fader ESPHome component. Layers allow a single physical motor fader to represent multiple independent logical controls, each with its own haptic configuration and position state.

### Goals

- Support 8 independent layers (indices 0-7) per motor fader
- Each layer maintains its own:
  - Haptic configuration (detents, magnetic endpoints, etc.)
  - Target position and last-known actual position
  - Independent state tracking
- Provide mechanism to switch between layers
- Handle latency between layer commands and firmware response using haptic config nonce
- Each fader in a multi-fader setup has independent layer state

### Use Cases

- Single fader controlling 8 different Home Assistant lights (one per layer)
- Multi-parameter control with different haptic feels per parameter
- Banking/paging interface for DAW or lighting control

## Component API Changes

### New/Modified Public Methods

```cpp
// Switch to a different layer
// Automatically sends the cached haptic config for the new layer to the fader
void set_active_layer(uint8_t layer_index);

// Get current active layer index
uint8_t get_active_layer() const { return active_layer_; }

// Send movement command (layer-aware)
// If layer is not specified, defaults to layer 0
// If layer is active, command is sent immediately to fader
// If layer is inactive, position is stored as restore target
void remote_move_to(uint8_t position, uint8_t layer = 0);

// Get the current target/restore position for a layer
// If layer is not specified, defaults to layer 0
uint8_t get_position(uint8_t layer = 0) const;

// Set haptic configuration for a specific layer
// If layer is active, config is sent immediately to fader
// If layer is inactive, config is cached and sent when layer becomes active
// mode: 0 = no haptics (smooth), 1 = smooth with magnetic ends, 2 = detents
void set_layer_haptic_config(
    uint8_t layer,
    uint8_t mode,
    uint8_t detent_count = 0,
    uint8_t detent_strength = 0,
    uint8_t target_position = 0
);
```

**API Design Note**: Layer parameter defaults to 0 (not the active layer) to avoid confusion from mixing layer-aware and non-layer-aware calls. This provides predictable behavior regardless of the current active layer.

### Python Configuration Schema

Optional configuration for static layer haptic settings:

```yaml
motorFaderESPHomeComponent:
  - id: motor_fader
    address: 0x20
    update_interval: 10ms

    # Optional: Define haptic configuration for each layer
    # If not specified, all layers default to smooth (mode 0)
    layer_haptics:
      - layer: 0  # Layer index (0-7)
        mode: smooth  # smooth, smooth_with_magnets, or detents

      - layer: 1
        mode: smooth_with_magnets

      - layer: 2
        mode: detents
        detent_count: 8      # Required for detents mode
        detent_strength: 5   # Required for detents mode

      # Layers 3-7 not specified, will use default (smooth)
```

**Schema Details:**
- `layer_haptics` is optional - if omitted, all layers use default smooth haptic config
- Each entry specifies configuration for one layer
- Unspecified layers retain the default smooth configuration
- Layer indices must be 0-7
- Mode values: `smooth` (0), `smooth_with_magnets` (1), `detents` (2)
- `detent_count` and `detent_strength` are optional, only used when `mode: detents`
- Configuration is applied during component `setup()` before any automation runs
- Runtime API (`set_layer_haptic_config` action) can override YAML configuration at any time

### Automation Actions

```yaml
# Action to switch layers
- motorFaderESPHomeComponent.set_active_layer:
    id: motor_fader
    layer: 2

# Action to move fader (layer parameter optional, defaults to 0)
- motorFaderESPHomeComponent.remote_move_to:
    id: motor_fader
    position: 128
    layer: 3  # optional - if omitted, defaults to layer 0

# Action to set haptic config for a layer
- motorFaderESPHomeComponent.set_layer_haptic_config:
    id: motor_fader
    layer: 1
    mode: 2  # 0=smooth, 1=smooth_with_magnets, 2=detents
    detent_count: 8  # optional, only used for detents mode
    detent_strength: 5  # optional, only used for detents mode
    target_position: 0  # optional, reserved for future use
```

### Enhanced Automation Triggers

The existing triggers (`on_manual_move`, `on_touch_change`, `on_double_tap`) will be enhanced to include layer metadata:

```yaml
on_manual_move:
  - lambda: |-
      // x contains the position
      // layer contains the active layer index when movement occurred
      ESP_LOGD("fader", "Layer %d moved to %d", layer, x);
```

Implementation: Triggers will pass the active layer index as an additional parameter.

## Internal State Management

### Per-Layer State Structure

```cpp
struct LayerState {
  // Position to restore when switching to this layer
  // Updated when:
  //   1. Switching AWAY from this layer (captures actual position)
  //   2. remote_move_to() called while layer is inactive
  uint8_t restore_position = 0;

  // The haptic config nonce value for this layer
  // Maps directly to layer index (layer 0 = nonce 0, layer 1 = nonce 1, etc.)
  uint8_t expected_nonce;

  // Cached haptic configuration for this layer (32-bit packed value)
  // Sent to fader when switching to this layer
  // Default: fully smooth (mode=0, detent_count=0)
  uint32_t haptic_config = 0;  // Will be initialized with proper nonce in constructor

  // Last confirmed actual position from fader when this layer was active
  // Used for change detection and manual movement triggers
  uint8_t last_confirmed_position = 0;

  // Last known position nonce when this layer was active
  uint8_t last_position_nonce = 0;

  // Flags
  bool has_been_initialized = false;  // Track if layer has ever been active
};
```

### Component State Variables

```cpp
class MotorFaderESPHomeComponent : public PollingComponent, public i2c::I2CDevice {
  // ... existing members ...

  // Layer management
  uint8_t active_layer_ = 0;                    // Currently active layer (0-7)
  LayerState layer_state_[8];                   // State for all 8 layers

  // Pending layer switch
  std::optional<uint8_t> pending_layer_switch_; // Layer we're switching to
  uint32_t layer_switch_timeout_ = 0;           // Timeout for layer switch confirmation

  static constexpr uint32_t LAYER_SWITCH_TIMEOUT_MS = 500;  // Max time to wait for nonce confirmation
};
```

### State Initialization

In the constructor, initialize the expected nonce and default haptic config for each layer:

```cpp
MotorFaderESPHomeComponent::MotorFaderESPHomeComponent() {
  for (uint8_t i = 0; i < 8; i++) {
    layer_state_[i].expected_nonce = i;

    // Default haptic config: fully smooth (mode=0, detent_count=0)
    // Set nonce to match layer index
    layer_state_[i].haptic_config = make_haptic_config_internal_(
        i,    // nonce = layer index
        0,    // mode = HAPTIC_NO_HAPTICS (smooth)
        0,    // detent_count
        0,    // detent_strength
        0     // target_position
    );
  }
}
```

Helper function to construct haptic config value (internal):

```cpp
uint32_t MotorFaderESPHomeComponent::make_haptic_config_internal_(
    uint8_t nonce,
    uint8_t mode,
    uint8_t detent_count,
    uint8_t detent_strength,
    uint8_t target_position) {

  uint32_t config = 0;
  config |= (nonce & 0x07) << HAPTIC_NONCE_bp;
  config |= (mode & 0x07) << HAPTIC_MODE_bp;
  config |= (detent_count & 0x0F) << HAPTIC_DETENT_COUNT_bp;
  config |= (detent_strength & 0x07) << HAPTIC_DETENT_STRENGTH_bp;
  config |= ((uint32_t)target_position & 0xFF) << HAPTIC_TARGET_POSITION_bp;
  return config;
}
```

## Layer Switching Logic

### set_active_layer() Implementation

```cpp
void MotorFaderESPHomeComponent::set_active_layer(uint8_t layer_index) {
  if (layer_index > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer_index);
    return;
  }

  if (layer_index == active_layer_) {
    return;  // Already on this layer
  }

  // Step 1: Capture current position from active layer (if it's been read)
  if (layer_state_[active_layer_].has_been_initialized) {
    layer_state_[active_layer_].restore_position = last_position_;
    ESP_LOGD(TAG, "Stored position %d for layer %d", last_position_, active_layer_);
  }

  // Step 2: Switch to new layer
  uint8_t old_layer = active_layer_;
  active_layer_ = layer_index;
  pending_layer_switch_ = layer_index;
  layer_switch_timeout_ = millis() + LAYER_SWITCH_TIMEOUT_MS;

  ESP_LOGI(TAG, "Switching from layer %d to layer %d", old_layer, layer_index);

  // Step 3: Send haptic config for new layer
  // This includes the correct nonce for the layer
  write_haptic_config_(layer_state_[layer_index].haptic_config);

  // Step 4: Command movement to restore position for new layer
  uint8_t target = layer_state_[layer_index].restore_position;
  ESP_LOGD(TAG, "Moving to restore position %d for layer %d", target, layer_index);
  remote_move_to(target, layer_index);
}
```

### remote_move_to() Implementation

```cpp
void MotorFaderESPHomeComponent::remote_move_to(uint8_t position, uint8_t layer) {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  if (layer == active_layer_) {
    // Active layer - send command immediately to fader hardware
    write_target_position_(position);
  } else {
    // Inactive layer - store as restore position
    layer_state_[layer].restore_position = position;
    ESP_LOGD(TAG, "Stored position %d for inactive layer %d", position, layer);
  }
}
```

### get_position() Implementation

```cpp
uint8_t MotorFaderESPHomeComponent::get_position(uint8_t layer) const {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return 0;
  }
  return layer_state_[layer].restore_position;
}
```

### set_layer_haptic_config() Implementation

```cpp
void MotorFaderESPHomeComponent::set_layer_haptic_config(
    uint8_t layer,
    uint8_t mode,
    uint8_t detent_count,
    uint8_t detent_strength,
    uint8_t target_position) {

  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  // Build haptic config with nonce matching layer index
  uint32_t config = make_haptic_config_internal_(
      layer,            // nonce = layer index
      mode,
      detent_count,
      detent_strength,
      target_position
  );

  // Cache the config for this layer
  layer_state_[layer].haptic_config = config;

  ESP_LOGD(TAG, "Set haptic config for layer %d: mode=%d, detents=%d, strength=%d",
           layer, mode, detent_count, detent_strength);

  // If this is the active layer, send config immediately
  if (layer == active_layer_) {
    write_haptic_config_(config);
  }
}
```

### State Report Processing

Modify `read_sensor_data_()` to check haptic config nonce:

```cpp
void MotorFaderESPHomeComponent::read_sensor_data_() {
  // ... existing I2C read logic ...

  uint8_t reported_nonce = state.haptic_config_nonce;

  // Check if state report is for the active layer
  if (pending_layer_switch_.has_value()) {
    // We're in the middle of a layer switch
    if (reported_nonce == layer_state_[active_layer_].expected_nonce) {
      // Firmware has confirmed the layer switch
      ESP_LOGD(TAG, "Layer switch to %d confirmed (nonce=%d)", active_layer_, reported_nonce);
      pending_layer_switch_.reset();
    } else if (millis() > layer_switch_timeout_) {
      // Timeout waiting for layer switch confirmation
      ESP_LOGW(TAG, "Layer switch timeout - expected nonce %d, got %d",
               layer_state_[active_layer_].expected_nonce, reported_nonce);
      pending_layer_switch_.reset();
      // Continue processing - maybe the nonce is correct now
    } else {
      // Still waiting for confirmation - ignore this state report
      ESP_LOGV(TAG, "Ignoring state report during layer switch (expected nonce %d, got %d)",
               layer_state_[active_layer_].expected_nonce, reported_nonce);
      return;
    }
  }

  // Verify nonce matches active layer
  if (reported_nonce != layer_state_[active_layer_].expected_nonce) {
    ESP_LOGD(TAG, "Ignoring state report for wrong layer (expected nonce %d, got %d)",
             layer_state_[active_layer_].expected_nonce, reported_nonce);
    return;
  }

  // Process state report for active layer
  layer_state_[active_layer_].has_been_initialized = true;

  // ... existing trigger and position tracking logic ...
  // Update layer's last confirmed position
  layer_state_[active_layer_].last_confirmed_position = position;
  layer_state_[active_layer_].last_position_nonce = state.position_nonce;
}
```

## Firmware Changes

### Haptic Config Nonce Behavior

The firmware currently uses a 3-bit haptic config nonce that is **reserved for future use**. To support layers, the firmware needs to be modified:

#### Required Changes:

1. **Nonce Write Handling** (in firmware `src/main.cpp`):
   ```cpp
   // When REG_HAPTIC_CONFIG is written via I2C:
   void handle_haptic_config_write(uint32_t new_config) {
     // Extract all fields from new config
     uint8_t new_nonce = (new_config >> HAPTIC_NONCE_bp) & 0b111;
     uint8_t new_mode = (new_config >> HAPTIC_MODE_bp) & 0b111;
     uint8_t new_detent_count = (new_config >> HAPTIC_DETENT_COUNT_bp) & 0b1111;
     uint8_t new_detent_strength = (new_config >> HAPTIC_DETENT_STRENGTH_bp) & 0b111;
     uint8_t new_target_pos = (new_config >> HAPTIC_TARGET_POSITION_bp) & 0xFF;

     // Update the haptic config nonce in state
     current_haptic_config_nonce = new_nonce;

     // Apply haptic settings (mode, detent count, strength)
     haptic_mode = new_mode;
     haptic_detent_count = new_detent_count;
     haptic_detent_strength = new_detent_strength;
     // target_pos reserved for future use
   }
   ```

2. **Nonce in State Reports**:
   - The firmware must include the current haptic config nonce in every STATE register report
   - This is already defined in the protocol but needs to be populated with the actual nonce value
   - The nonce should persist across all state changes until a new HAPTIC_CONFIG is written

3. **Mode Transitions**:
   - When haptic config nonce changes, the firmware should transition to `MODE_REMOTE_MOVEMENT_IN_PROGRESS` if the target position differs from current position
   - This ensures the fader moves to the new layer's restore position
   - The haptic config change and target position write will typically arrive in quick succession during layer switch

#### No Protocol Changes Required

The I2C protocol already defines the haptic config nonce field (3 bits in both STATE register and HAPTIC_CONFIG register). No register map changes are needed - only firmware implementation.

#### Haptic Config Integration

The component uses the haptic config nonce to identify layers:
- Layer 0 → nonce 0
- Layer 1 → nonce 1
- ...
- Layer 7 → nonce 7

When switching layers, the component sends the cached haptic config (with appropriate nonce) followed by the target position. The firmware responds with state reports containing the new nonce, which the component validates before processing position updates.

### Firmware Testing Requirements

After firmware changes, verify:
- Writing HAPTIC_CONFIG with different nonce values correctly updates the nonce in STATE reports
- All haptic config fields (mode, detent_count, detent_strength) are correctly extracted and applied
- Changing nonce triggers appropriate mode transitions
- Nonce persists correctly across motor movements and state changes
- Different haptic modes produce distinct physical feedback (smooth vs detents vs magnetic ends)
- Haptic config changes take effect immediately without requiring a power cycle

## Implementation Sequence

### Phase 1: Component State Management
1. Add `LayerState` struct and layer state array to component header
2. Implement `get_active_layer()` and `get_position()` accessors
3. Update constructor to initialize layer nonce values

### Phase 2: Layer Switching
1. Implement `set_active_layer()` method
2. Update `remote_move_to()` method with layer parameter (default = 0)
3. Modify `read_sensor_data_()` to validate nonce and handle layer switches
4. Add timeout mechanism for layer switch confirmation

### Phase 3: Firmware Updates
1. Implement haptic config nonce write handling in firmware
2. Ensure nonce is correctly reported in STATE register
3. Add mode transition logic for nonce changes
4. Test firmware changes with production test fixture

### Phase 4: Python/ESPHome Bindings
1. Add Python schema for `layer_haptics` configuration parameter
2. Add schema for new/modified automation actions
3. Update `remote_move_to` action schema to include optional layer parameter (default 0)
4. Generate code for `set_active_layer` and `set_layer_haptic_config` actions
5. Generate setup code to apply `layer_haptics` configuration during component initialization
6. Extend trigger lambdas to include layer parameter
7. Update examples with layer usage patterns

#### Python Schema Implementation (`__init__.py`)

Add configuration schema for layer haptics:

```python
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

# Define haptic mode enum
HapticMode = cg.global_ns.enum("HapticMode")
HAPTIC_MODES = {
    "smooth": HapticMode.HAPTIC_NO_HAPTICS,
    "smooth_with_magnets": HapticMode.HAPTIC_SMOOTH_WITH_MAGNET_ENDS,
    "detents": HapticMode.HAPTIC_DETENTS,
}

CONF_LAYER_HAPTICS = "layer_haptics"
CONF_LAYER = "layer"
CONF_MODE = "mode"
CONF_DETENT_COUNT = "detent_count"
CONF_DETENT_STRENGTH = "detent_strength"

# Schema for a single layer haptic configuration
LAYER_HAPTIC_SCHEMA = cv.Schema({
    cv.Required(CONF_LAYER): cv.int_range(min=0, max=7),
    cv.Required(CONF_MODE): cv.enum(HAPTIC_MODES, lower=True),
    cv.Optional(CONF_DETENT_COUNT, default=0): cv.int_range(min=0, max=15),
    cv.Optional(CONF_DETENT_STRENGTH, default=0): cv.int_range(min=0, max=7),
})

# Add to component schema
CONFIG_SCHEMA = cv.Schema({
    # ... existing schema fields ...
    cv.Optional(CONF_LAYER_HAPTICS): cv.ensure_list(LAYER_HAPTIC_SCHEMA),
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    # ... existing setup code ...

    # Apply layer haptic configurations if specified
    if CONF_LAYER_HAPTICS in config:
        for haptic_config in config[CONF_LAYER_HAPTICS]:
            layer = haptic_config[CONF_LAYER]
            mode = haptic_config[CONF_MODE]
            detent_count = haptic_config[CONF_DETENT_COUNT]
            detent_strength = haptic_config[CONF_DETENT_STRENGTH]

            cg.add(var.set_layer_haptic_config(
                layer, mode, detent_count, detent_strength, 0
            ))
```

### Phase 5: Testing and Documentation
1. Create example YAML configuration demonstrating layers
2. Test multi-layer scenarios with latency simulation
3. Test rapid layer switching
4. Document layer behavior and limitations

## Edge Cases and Considerations

### Rapid Layer Switching
If the user switches layers rapidly (faster than firmware can respond):
- Each `set_active_layer()` call sets a new `pending_layer_switch_`
- Only the most recent layer switch is tracked
- Previous pending switches are abandoned
- This is acceptable - the final layer switch will eventually confirm

### Power Loss / Component Restart
- Layer state is not persisted to flash/EEPROM
- After restart, all layers reset to `restore_position = 0`
- Active layer resets to 0
- Applications requiring persistence should use ESPHome's `globals` with `restore_value: true`

### Invalid Nonce Values
- If firmware reports a nonce that doesn't match any layer, log a warning and ignore the state report
- This could indicate:
  - Firmware/component version mismatch
  - Corrupted I2C communication
  - Firmware bug

### I2C Communication Errors
- If I2C write of HAPTIC_CONFIG fails, the layer switch will timeout
- Component should log error and remain on previous layer
- Application can detect timeout via logs or by checking that triggers stop firing

### Multi-Fader Independence
- Each fader instance maintains its own `active_layer_` and `layer_state_[]` array
- Faders do not synchronize layers automatically
- Application is responsible for coordinating layer switches if desired:
  ```yaml
  # Switch all faders to layer 2
  - motorFaderESPHomeComponent.set_active_layer:
      id: motor_fader_1
      layer: 2
  - motorFaderESPHomeComponent.set_active_layer:
      id: motor_fader_2
      layer: 2
  ```

## Example Usage

### Basic Layer Switching with Haptic Config

```yaml
# Configure haptic settings for each layer via YAML
motorFaderESPHomeComponent:
  - id: motor_fader
    address: 0x20
    update_interval: 10ms

    # Static layer haptic configuration
    layer_haptics:
      - layer: 0
        mode: smooth  # Default, can be omitted

      - layer: 1
        mode: smooth_with_magnets

      - layer: 2
        mode: detents
        detent_count: 8
        detent_strength: 5

# Switch layers with buttons
binary_sensor:
  - platform: gpio
    id: layer_button_0
    on_press:
      - motorFaderESPHomeComponent.set_active_layer:
          id: motor_fader
          layer: 0

  - platform: gpio
    id: layer_button_1
    on_press:
      - motorFaderESPHomeComponent.set_active_layer:
          id: motor_fader
          layer: 1

  - platform: gpio
    id: layer_button_2
    on_press:
      - motorFaderESPHomeComponent.set_active_layer:
          id: motor_fader
          layer: 2
```

### Multiple Lights on One Fader with Different Haptics

```yaml
motorFaderESPHomeComponent:
  - id: motor_fader
    address: 0x20

    # Different haptic feel for each light
    layer_haptics:
      - layer: 0  # Living room
        mode: smooth

      - layer: 1  # Bedroom - discrete brightness levels
        mode: detents
        detent_count: 10
        detent_strength: 4

sensor:
  - platform: homeassistant
    entity_id: light.living_room_brightness
    id: living_room_brightness
    on_value:
      - motorFaderESPHomeComponent.remote_move_to:
          id: motor_fader
          position: !lambda "return x * 2.55;"  # 0-100 to 0-255
          layer: 0

  - platform: homeassistant
    entity_id: light.bedroom_brightness
    id: bedroom_brightness
    on_value:
      - motorFaderESPHomeComponent.remote_move_to:
          id: motor_fader
          position: !lambda "return x * 2.55;"
          layer: 1

motorFaderESPHomeComponent:
  - id: motor_fader
    on_manual_move:
      - lambda: |-
          // Send fader position to appropriate light based on active layer
          uint8_t layer = id(motor_fader).get_active_layer();
          float brightness = x / 2.55;  // 0-255 to 0-100

          if (layer == 0) {
            id(living_room_light).turn_on().set_brightness(brightness).perform();
          } else if (layer == 1) {
            id(bedroom_light).turn_on().set_brightness(brightness).perform();
          }
```

### Scene Recall with Layers

```yaml
# Store current mix as scene/layer
button:
  - platform: template
    name: "Save Scene 1"
    on_press:
      # Capture current positions and store in layer 1
      - lambda: |-
          id(fader1).remote_move_to(id(fader1).get_position(0), 1);
          id(fader2).remote_move_to(id(fader2).get_position(0), 1);

  - platform: template
    name: "Recall Scene 1"
    on_press:
      # Switch all faders to layer 1 (restores saved positions)
      - motorFaderESPHomeComponent.set_active_layer:
          id: fader1
          layer: 1
      - motorFaderESPHomeComponent.set_active_layer:
          id: fader2
          layer: 1
```

### Dynamic Haptic Config Changes

The runtime API allows changing haptic configs dynamically, even for configurations defined in YAML:

```yaml
motorFaderESPHomeComponent:
  - id: motor_fader
    layer_haptics:
      - layer: 0
        mode: smooth
      - layer: 1
        mode: detents
        detent_count: 8
        detent_strength: 5

button:
  - platform: template
    name: "Toggle Layer 1 Detents"
    on_press:
      # Dynamically change layer 1 from 8 detents to 4 detents
      - motorFaderESPHomeComponent.set_layer_haptic_config:
          id: motor_fader
          layer: 1
          mode: detents
          detent_count: 4
          detent_strength: 7

  - platform: template
    name: "Reset Layer 1 to Smooth"
    on_press:
      # Override YAML config at runtime
      - motorFaderESPHomeComponent.set_layer_haptic_config:
          id: motor_fader
          layer: 1
          mode: smooth
```

## Open Questions

1. **Layer Change Triggers**: Should there be an `on_layer_change` trigger that fires when the active layer changes? Use case: updating a display to show which layer is active.

2. **Position Sync**: Should there be a mode where the fader doesn't physically move when switching layers? (i.e., "pickup" mode where the layer position jumps to fader position on first touch)

3. **Global vs Per-Instance Layers**: Current design assumes independent layers per fader. Should there be an option for "global" layers where all faders switch together?

4. **Haptic Config Validation**: Should the component validate haptic config parameters (e.g., detent_count <= 15, mode <= 2) or trust the user input?

## Success Criteria

The layers feature will be considered complete when:
- [ ] All 8 layers can store independent positions and haptic configs
- [ ] Default haptic config for each layer is fully smooth (mode=0, detent_count=0)
- [ ] Haptic config can be set per-layer via YAML `layer_haptics` configuration
- [ ] Haptic config can be dynamically changed via `set_layer_haptic_config()` action
- [ ] YAML-configured haptic settings are applied during component setup
- [ ] Switching between layers automatically sends the cached haptic config for the new layer
- [ ] Switching between layers moves the fader to the correct restore position
- [ ] Commands sent to inactive layers are stored and don't affect the physical fader
- [ ] State reports with mismatched nonces are correctly ignored
- [ ] Firmware correctly updates and reports haptic config nonce
- [ ] Example configurations demonstrate practical layer usage with different haptic feels
- [ ] Layer switching works reliably with realistic I2C latency (10-50ms)
- [ ] Multi-fader setups maintain independent layer state per fader
