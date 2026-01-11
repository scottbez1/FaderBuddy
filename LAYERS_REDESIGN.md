# Motor Fader Layers Redesign - Firmware-Managed Layers

## Overview

This document describes a redesign of the layer management architecture, moving layer state and control logic from the ESPHome component into the ATtiny1616 firmware. This makes the firmware responsible for managing the 8 layers as distinct logical devices, while the ESPHome component becomes a thin protocol wrapper.

## Motivation

The current design has the ESPHome component managing all layer state (active layer tracking, restore positions, haptic configs, nonce validation). This creates complexity in the ESPHome component and requires careful synchronization between ESPHome and firmware state.

The new design moves layer management into firmware because:

1. **Deferred Layer Switching**: Critical requirement to defer layer changes when user input is active, which requires firmware-level mode awareness
2. **Simplified Protocol**: Each layer can be addressed independently like a distinct I2C device
3. **Reduced ESPHome Complexity**: ESPHome becomes a simple protocol wrapper instead of a state manager
4. **Better Encapsulation**: Firmware owns all motor fader state, ESPHome just forwards commands

## Key Behavioral Requirements

### 1. Layer Change Requests

**When MODE_INPUT_IDLE or MODE_REMOTE_MOVEMENT_IN_PROGRESS:**
- Apply layer change immediately
- Move fader to restore position of new layer
- Start reporting state for new layer
- If interrupting a remote movement, the old layer's restore position is already set to the movement destination

**When MODE_INPUT_ACTIVE:**
- **Defer** the layer change (do not apply)
- Continue reporting state for current active layer
- No movement is attempted
- Only the most recent pending layer change is kept (not queued)
- When mode transitions to MODE_INPUT_IDLE: apply the deferred layer change

**When MODE_SELF_CALIBRATION:**
- **Defer** the layer change (do not apply)
- Continue calibration process on current layer
- When calibration completes and transitions to MODE_INPUT_IDLE or MODE_REMOTE_MOVEMENT_IN_PROGRESS: apply the deferred layer change

**When MODE_ERROR:**
- **Ignore** the layer change request entirely
- Remain in error state on current layer
- Layer changes are not accepted until error is cleared via REG_CLEAR_ERROR
- No deferred layer change is stored

**Note:** Reading REG_ACTIVE_LAYER may not immediately reflect a write if the layer change was deferred. The register will return the currently active layer, not the pending layer change.

### 2. Target Position Writes

**Active Layer, MODE_INPUT_IDLE:**
- Set `restore_position[active_layer] = target`
- Start remote movement to target
- Transition to MODE_REMOTE_MOVEMENT_IN_PROGRESS

**Active Layer, MODE_REMOTE_MOVEMENT_IN_PROGRESS:**
- Update `restore_position[active_layer] = target`
- Update `target_adc` to new target
- Continue movement to new target

**Active Layer, MODE_INPUT_ACTIVE:**
- Ignore the target write
- User input takes priority
- `restore_position[active_layer]` continues to be updated by user input

**Non-Active Layer:**
- Update `restore_position[layer] = target`
- No physical change to fader
- Stored for future layer switch

### 3. Haptic Config Writes

**Active Layer:**
- Apply haptic configuration immediately
- Applies even during MODE_INPUT_ACTIVE

**Non-Active Layer:**
- Store configuration for that layer
- Applied when layer becomes active

### 4. Restore Position Updates

The restore position represents "where this layer should be" and is updated by:

**During MODE_INPUT_ACTIVE on active layer:**
- Continuously update `restore_position[active_layer]` to match actual fader position
- User input is the source of truth

**When starting remote movement on active layer:**
- Set `restore_position[active_layer] = target`
- Do NOT update during the movement
- Even if layer change is deferred, restore position stays at target

**When writing target to non-active layer:**
- Set `restore_position[layer] = target`

### 5. Example Scenarios

**Scenario A: Interrupted Remote Movement**

Demonstrating restore position behavior when layer changes during remote movement:

1. Layer 0 active, fader at position 0
2. Write target 158 to layer 0 → `restore_position[0] = 158`, start moving
3. Write active layer to 2 (while still moving to 158) → Layer change applied immediately
4. Movement is interrupted, switches to layer 2
5. Fader moves to `restore_position[2]` instead
6. `restore_position[0]` remains 158 (even though we never arrived)
7. If later switch back to layer 0 → Resume movement to 158

**Scenario B: Deferred Layer Change During Input**

Demonstrating layer change deferral during user input:

1. Layer 0 active, user moves fader to position 100
2. User is still touching/moving fader (MODE_INPUT_ACTIVE)
3. Write active layer to 3 → Layer change deferred
4. Reading REG_ACTIVE_LAYER still returns 0 (not 3)
5. `restore_position[0]` continues updating with user input (currently 100)
6. User releases fader, idle timeout expires → MODE_INPUT_IDLE
7. Deferred layer change applied: switch to layer 3
8. Fader moves to `restore_position[3]`
9. `restore_position[0]` saved at final user position (e.g., 120 if user moved before releasing)

## Protocol Changes

### I2C Transaction Patterns

The new layer-addressed registers introduce a two-step protocol for reading layer-specific data.

#### Simple Registers (No Layer Parameter)

**Read Transaction:**
1. Master writes register address (1 byte)
2. Master reads N bytes of data

**Write Transaction:**
1. Master writes register address + data (1 + N bytes)

**Applies to:** REG_VERSION, REG_STATE, REG_UPTIME, REG_TOUCH_RAW, REG_SERIAL, REG_TOUCH_DELTA, REG_TOUCH_REF, REG_TOUCH_RECAL, REG_ACTIVE_LAYER

#### Layer-Addressed Registers (New Protocol)

**Read Transaction:**
1. Master writes register address + layer index (2 bytes total)
2. Master reads N bytes of data for that layer

**Write Transaction:**
1. Master writes register address + layer index + data (1 + 1 + N bytes)

**Applies to:** REG_LAYER_TARGET, REG_LAYER_HAPTIC_CONFIG

**Example - Reading layer 3's restore position:**
```
I2C Write: [REG_LAYER_TARGET, 0x03]        // Set register and layer to query
I2C Read:  [position_byte]                 // Firmware returns layer_restore_positions[3]
```

**Example - Writing layer 5's haptic config:**
```
I2C Write: [REG_LAYER_HAPTIC_CONFIG, 0x05, config_hi, config_lo]
```

**Rationale:** The layer parameter in the write allows a single register address to access data for any of the 8 layers without requiring 8 separate register addresses per layer. The firmware must maintain internal state (`queried_layer`) to remember which layer was queried between the write and the subsequent read.

### Modified STATE Register (REG_STATE = 0x01)

The STATE register is **repurposed** to use the haptic config nonce field for active layer index:

**Previous bit allocation (bits 4-6):**
- Haptic config nonce (3 bits) - used to identify which layer config was active

**New bit allocation (bits 4-6):**
- Active layer index (3 bits, values 0-7) - explicitly identifies active layer

**Complete STATE register layout:**
```
Bit 0:       Touch (1 bit)
Bits 1-3:    Mode (3 bits)
Bits 4-6:    Active Layer Index (3 bits) ← CHANGED from haptic_config_nonce
Bits 7-14:   Position (8 bits) - position of active layer
Bits 15-16:  Position nonce (2 bits)
Bits 17-27:  Raw ADC (11 bits)
Bits 28-29:  Double tap nonce (2 bits)
Bits 30-31:  Unused (2 bits)
```

**Rationale:** Haptic config nonce was previously used to identify layers via nonce matching. With explicit layer management in firmware, the nonce is no longer needed. Active layer index provides the same information more directly.

### New Registers

**REG_ACTIVE_LAYER (0x0D)** - Read/Write, u8
- **Read**: Returns currently active layer index (0-7)
  - **Note:** May not immediately reflect a write if layer change was deferred during MODE_INPUT_ACTIVE
- **Write**: Request layer change to specified layer (0-7)
  - If MODE_INPUT_IDLE or MODE_REMOTE_MOVEMENT_IN_PROGRESS: Apply immediately, move to restore position
  - If MODE_INPUT_ACTIVE: Defer until idle
  - Multiple pending requests: only keep the most recent

**REG_LAYER_TARGET (0x0E)** - Read/Write
- **Write**: `[layer_index (u8), target_position (u8)]`
  - Behavior depends on layer and mode (see "Target Position Writes" above)
- **Read**: First write `[layer_index (u8)]`, then read returns `restore_position[layer]` (u8)

**REG_LAYER_HAPTIC_CONFIG (0x0F)** - Read/Write
- **Write**: `[layer_index (u8), config (u16 big-endian)]`
  - If layer == active: Apply haptic config immediately (even during input)
  - If layer != active: Store config for that layer
- **Read**: First write `[layer_index (u8)]`, then read returns `haptic_config[layer]` (u16 big-endian)

### Updated Haptic Config Format

The haptic configuration is now **16 bits** (2 bytes) instead of 32 bits.

**Haptic Config Bit Layout (16 bits total):**
```
Bits 0-2:   Haptic Mode (3 bits)
            0 = HAPTIC_NO_HAPTICS (smooth)
            1 = HAPTIC_SMOOTH_WITH_MAGNET_ENDS
            2 = HAPTIC_DETENTS
Bits 3-6:   Detent Count (4 bits, 0-15, only used for HAPTIC_DETENTS mode)
Bits 7-9:   Detent Strength (3 bits, 0-7, scales haptic force)
Bits 10-15: Reserved (6 bits, must be set to 0)
```

**Removed fields from previous protocol:**
- **HAPTIC_NONCE** (previously 3 bits): Layer identification is now explicit via active_layer in STATE register
- **HAPTIC_TARGET_POSITION** (previously 8 bits): Movement is triggered explicitly via REG_ACTIVE_LAYER or REG_LAYER_TARGET

## Firmware Implementation

### New Firmware State Variables

```c
// Layer storage (26 bytes total)
uint16_t layer_haptic_configs[8];      // 16 bytes - cached haptic config per layer
uint8_t layer_restore_positions[8];    // 8 bytes - target/restore position per layer
uint8_t active_layer;                  // 1 byte - currently active layer (0-7)
uint8_t pending_layer_change;          // 1 byte - deferred layer change (0xFF = none, 0-7 = layer)
```

Memory overhead: 26 bytes (well within ATtiny1616's 2KB RAM)

**Additional state for I2C protocol:**
```c
uint8_t queried_layer;  // 1 byte - tracks which layer was queried for two-step read protocol
```

Total memory overhead: 27 bytes

### Initialization

```c
void setup() {
  // Initialize all layers with default haptic config (smooth)
  for (uint8_t i = 0; i < 8; i++) {
    // Default haptic config: mode=0 (smooth), detent_count=0, detent_strength=0
    layer_haptic_configs[i] = 0;  // All zeros = smooth mode with no haptics
    layer_restore_positions[i] = 0;  // Default position
  }

  active_layer = 0;
  pending_layer_change = 0xFF;  // No pending change

  // Load active layer's haptic config into current haptic_config (global variable)
  haptic_config = layer_haptic_configs[0];
}
```

### I2C Handler Updates

The I2C handlers must distinguish between read setup transactions (2 bytes: register + layer) and write transactions (3+ bytes: register + layer + data).

**onI2cReceive() - REG_ACTIVE_LAYER:**
```c
case REG_ACTIVE_LAYER:
  if (howMany == 2) {  // register + 1 byte layer index (write)
    uint8_t new_layer = Wire.read() & 0x07;  // Clamp to 0-7
    request_layer_change(new_layer);
  }
  break;
```

**onI2cReceive() - REG_LAYER_TARGET:**
```c
case REG_LAYER_TARGET:
  if (howMany == 2) {
    // Read setup: register + layer index
    // Save layer index for subsequent read request
    queried_layer = Wire.read() & 0x07;
  } else if (howMany == 3) {
    // Write: register + layer + target position
    uint8_t layer = Wire.read() & 0x07;
    uint8_t target = Wire.read();
    write_layer_target(layer, target);
  }
  break;
```

**onI2cReceive() - REG_LAYER_HAPTIC_CONFIG:**
```c
case REG_LAYER_HAPTIC_CONFIG:
  if (howMany == 2) {
    // Read setup: register + layer index
    // Save layer index for subsequent read request
    queried_layer = Wire.read() & 0x07;
  } else if (howMany == 4) {
    // Write: register + layer + 2 bytes config
    uint8_t layer = Wire.read() & 0x07;
    uint16_t config = ((uint16_t)Wire.read() << 8) | Wire.read();
    write_layer_haptic_config(layer, config);
  }
  break;
```

**onI2cRequest() - REG_ACTIVE_LAYER:**
```c
case REG_ACTIVE_LAYER:
  Wire.write(active_layer);
  break;
```

**onI2cRequest() - REG_LAYER_TARGET:**
```c
case REG_LAYER_TARGET:
  // Return restore position for the previously queried layer
  Wire.write(layer_restore_positions[queried_layer]);
  break;
```

**onI2cRequest() - REG_LAYER_HAPTIC_CONFIG:**
```c
case REG_LAYER_HAPTIC_CONFIG:
  // Return haptic config for the previously queried layer (16 bits, big-endian)
  uint16_t config = layer_haptic_configs[queried_layer];
  Wire.write((config >> 8) & 0xFF);  // High byte
  Wire.write(config & 0xFF);          // Low byte
  break;
```

### Layer Management Functions

**request_layer_change():**
```c
void request_layer_change(uint8_t new_layer) {
  if (new_layer == active_layer) {
    return;  // Already on this layer
  }

  Mode mode = get_mode();
  if (mode == MODE_INPUT_IDLE || mode == MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
    // Apply immediately - remote movements are interruptible
    apply_layer_change(new_layer);
  } else if (mode == MODE_INPUT_ACTIVE || mode == MODE_SELF_CALIBRATION) {
    // Defer until idle - user input or calibration takes priority
    pending_layer_change = new_layer;
  } else if (mode == MODE_ERROR) {
    // Ignore layer change requests while in error state
    // Layer switching is blocked until error is cleared
    return;
  }
}
```

**apply_layer_change():**
```c
void apply_layer_change(uint8_t new_layer) {
  // Save current position to outgoing layer's restore position
  // (only if currently in input mode - remote movements already set restore position)
  Mode mode = get_mode();
  if (mode == MODE_INPUT_ACTIVE) {
    layer_restore_positions[active_layer] = position;  // Current actual position
  }

  // Switch to new layer
  active_layer = new_layer;
  pending_layer_change = 0xFF;

  // Load new layer's haptic config
  haptic_config = layer_haptic_configs[new_layer];

  // Start movement to new layer's restore position
  target_adc = BOUNDED_LERP_UINT16(
    layer_restore_positions[new_layer],
    0, 255,
    input_calib_min,
    input_calib_max
  );

  set_mode(MODE_REMOTE_MOVEMENT_IN_PROGRESS);
  remote_movement_start = millis();
  remote_movement_start_position = input_ewma;
  remote_movement_steady_start = millis();
}
```

**write_layer_target():**
```c
void write_layer_target(uint8_t layer, uint8_t target) {
  if (layer == active_layer) {
    Mode mode = get_mode();
    if (mode == MODE_INPUT_IDLE) {
      // Start remote movement
      layer_restore_positions[layer] = target;
      target_adc = BOUNDED_LERP_UINT16(target, 0, 255, input_calib_min, input_calib_max);
      set_mode(MODE_REMOTE_MOVEMENT_IN_PROGRESS);
      remote_movement_start = millis();
      remote_movement_start_position = input_ewma;
      remote_movement_steady_start = millis();
    } else if (mode == MODE_INPUT_ACTIVE) {
      // Ignore - user has control
      // restore_position will be updated by user input
    } else if (mode == MODE_REMOTE_MOVEMENT_IN_PROGRESS) {
      // Update target of in-progress movement
      layer_restore_positions[layer] = target;
      target_adc = BOUNDED_LERP_UINT16(target, 0, 255, input_calib_min, input_calib_max);
      // Extend timeout
      remote_movement_start = millis();
      remote_movement_start_position = input_ewma;
      remote_movement_steady_start = millis();
    }
  } else {
    // Non-active layer - just update restore position
    layer_restore_positions[layer] = target;
  }
}
```

**write_layer_haptic_config():**
```c
void write_layer_haptic_config(uint8_t layer, uint16_t config) {
  // Validate config (same as existing validation)
  HapticMode mode = static_cast<HapticMode>((config & HAPTIC_MODE_bm) >> HAPTIC_MODE_bp);
  if (mode == HAPTIC_DETENTS) {
    uint8_t detent_count = (config & HAPTIC_DETENT_COUNT_bm) >> HAPTIC_DETENT_COUNT_bp;
    if (detent_count < 1 || detent_count > 10) {
      return;  // Invalid config, reject
    }
  }

  // Store config for layer
  layer_haptic_configs[layer] = config;

  // If this is the active layer, apply immediately
  if (layer == active_layer) {
    haptic_config = config;
    // Note: This applies even during MODE_INPUT_ACTIVE
  }
}
```

### Mode Transition Updates

**Entering MODE_INPUT_IDLE (from MODE_INPUT_ACTIVE, MODE_REMOTE_MOVEMENT_IN_PROGRESS, or MODE_SELF_CALIBRATION):**
```c
// In motor_update(), when transitioning to MODE_INPUT_IDLE:
if (pending_layer_change != 0xFF) {
  // Apply deferred layer change
  apply_layer_change(pending_layer_change);
  return;  // apply_layer_change sets mode to REMOTE_MOVEMENT_IN_PROGRESS
}
```

**Note:** MODE_SELF_CALIBRATION typically transitions to MODE_REMOTE_MOVEMENT_IN_PROGRESS (to restore target position after calibration), but if a layer change is pending, it should be applied first. The apply_layer_change() function will handle the movement to the new layer's restore position.

**During MODE_INPUT_ACTIVE:**
```c
// Continuously update restore position for active layer
layer_restore_positions[active_layer] = position;
```

**When starting MODE_REMOTE_MOVEMENT_IN_PROGRESS:**
```c
// Restore position already set by write_layer_target() or apply_layer_change()
// Do NOT update during movement
```

### STATE Register Updates

**In motor_update(), pack active layer into STATE:**
```c
// Pack active layer index into state (bits 4-6, replacing haptic_config_nonce)
state &= ~STATE_HAPTIC_CONFIG_NONCE_bm;  // Clear old nonce field
state |= ((uint32_t)active_layer << STATE_HAPTIC_CONFIG_NONCE_bp) & STATE_HAPTIC_CONFIG_NONCE_bm;
```

**Rename constant for clarity:**
```c
// In i2c_data.h, add alias or comment:
#define STATE_ACTIVE_LAYER_bp STATE_HAPTIC_CONFIG_NONCE_bp
#define STATE_ACTIVE_LAYER_bs STATE_HAPTIC_CONFIG_NONCE_bs
#define STATE_ACTIVE_LAYER_bm STATE_HAPTIC_CONFIG_NONCE_bm
```

## ESPHome Component Changes

### Removed State Variables

```cpp
// Remove all layer state management:
// - LayerState layer_state_[8];
// - uint8_t active_layer_;
// - optional<uint8_t> pending_layer_switch_;
// - uint32_t layer_switch_timeout_;
```

### Simplified Methods

**set_active_layer():**
```cpp
void MotorFaderESPHomeComponent::set_active_layer(uint8_t layer_index) {
  if (layer_index > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer_index);
    return;
  }

  // Simple write to firmware
  uint8_t buffer[] = {REG_ACTIVE_LAYER, layer_index};
  auto result = this->write(buffer, 2);
  if (result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to set active layer: %d", result);
  }
}
```

**remote_move_to():**
```cpp
void MotorFaderESPHomeComponent::remote_move_to(uint8_t position, uint8_t layer) {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  // Convert user-facing position to hardware position
  uint8_t hw_position = invert_ ? (255 - position) : position;

  // Write to firmware
  uint8_t buffer[] = {REG_LAYER_TARGET, layer, hw_position};
  auto result = this->write(buffer, 3);
  if (result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to write layer target: %d", result);
  }
}
```

**set_layer_haptic_config():**
```cpp
void MotorFaderESPHomeComponent::set_layer_haptic_config(
    uint8_t layer,
    uint8_t mode,
    uint8_t detent_count,
    uint8_t detent_strength) {

  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return;
  }

  // Build haptic config (16 bits)
  uint16_t config = 0;
  config |= (mode & 0x07) << 0;           // Bits 0-2: mode
  config |= (detent_count & 0x0F) << 3;   // Bits 3-6: detent count
  config |= (detent_strength & 0x07) << 7; // Bits 7-9: detent strength
  // Bits 10-15: reserved (0)

  // Write to firmware (3 bytes: register + layer + config)
  uint8_t buffer[4];
  buffer[0] = REG_LAYER_HAPTIC_CONFIG;
  buffer[1] = layer;
  buffer[2] = (config >> 8) & 0xFF;  // High byte
  buffer[3] = config & 0xFF;         // Low byte

  auto result = this->write(buffer, 4);
  if (result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to write layer haptic config: %d", result);
  }
}
```

**get_active_layer():**
```cpp
uint8_t MotorFaderESPHomeComponent::get_active_layer() const {
  // Option 1: Read from REG_ACTIVE_LAYER
  uint8_t reg = REG_ACTIVE_LAYER;
  uint8_t buffer = 0;

  auto write_result = this->write(&reg, 1, false);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
    return 0;  // Default to layer 0 on error
  }

  auto read_result = this->read(&buffer, 1);
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    return 0;
  }

  return buffer & 0x07;

  // Option 2: Extract from last STATE read (if cached)
  // return (last_state_ & STATE_ACTIVE_LAYER_bm) >> STATE_ACTIVE_LAYER_bp;
}
```

**get_position():**
```cpp
uint8_t MotorFaderESPHomeComponent::get_position(uint8_t layer) const {
  if (layer > 7) {
    ESP_LOGE(TAG, "Invalid layer index: %d", layer);
    return 0;
  }

  // Write layer index to query
  uint8_t write_buffer[] = {REG_LAYER_TARGET, layer};
  auto write_result = this->write(write_buffer, 2, false);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to write layer index: %d", write_result);
    return 0;
  }

  // Read restore position for that layer
  uint8_t read_buffer = 0;
  auto read_result = this->read(&read_buffer, 1);
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to read layer position: %d", read_result);
    return 0;
  }

  // Convert hardware position to user-facing position
  return invert_ ? (255 - read_buffer) : read_buffer;
}
```

### Simplified read_sensor_data_()

```cpp
bool MotorFaderESPHomeComponent::read_sensor_data_() {
  uint8_t reg = REG_STATE;
  uint8_t buffer[4];

  auto write_result = this->write(&reg, 1, false);
  if (write_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to write register address: %d", write_result);
    return false;
  }

  auto read_result = this->read(buffer, 4);
  if (read_result != esphome::i2c::ErrorCode::NO_ERROR) {
    ESP_LOGE(TAG, "Failed to read data: %d", read_result);
    return false;
  }

  // Parse STATE register
  uint32_t state = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];

  Mode mode = static_cast<Mode>((state & STATE_MODE_bm) >> STATE_MODE_bp);
  uint16_t hw_position = (state & STATE_POSITION_bm) >> STATE_POSITION_bp;
  uint8_t position_nonce = (state & STATE_POSITION_NONCE_bm) >> STATE_POSITION_NONCE_bp;
  bool touch = (state & STATE_TOUCH_bm) >> STATE_TOUCH_bp;
  uint16_t raw_adc = (state & STATE_RAW_ADC_bm) >> STATE_RAW_ADC_bp;
  uint8_t double_tap_nonce = (state & STATE_DOUBLE_TAP_NONCE_bm) >> STATE_DOUBLE_TAP_NONCE_bp;
  uint8_t active_layer = (state & STATE_ACTIVE_LAYER_bm) >> STATE_ACTIVE_LAYER_bp;

  // No more nonce validation or layer switching logic!
  // Just process triggers with layer parameter

  if (hw_position != last_hw_position_ || position_nonce != last_position_nonce_) {
    if (mode == MODE_INPUT_ACTIVE || mode == MODE_INPUT_IDLE) {
      uint8_t user_position = invert_ ? (255 - hw_position) : hw_position;

      // Fire trigger with active layer from STATE register
      on_manual_move_->trigger(user_position, active_layer);
    }
    last_hw_position_ = hw_position;
    last_position_nonce_ = position_nonce;
  }

  if (touch != last_touch_) {
    on_touch_change_->trigger(touch, active_layer);
    last_touch_ = touch;
  }

  if (double_tap_nonce != last_double_tap_nonce_) {
    on_double_tap_->trigger(active_layer);
    last_double_tap_nonce_ = double_tap_nonce;
  }

  last_state_ = state;
  return true;
}
```

**Removed:**
- All nonce validation logic
- Layer switching timeout logic
- pending_layer_switch_ tracking
- LayerState updates

**Retained:**
- Trigger firing (now with active_layer from STATE register)
- Position inversion (user-facing vs hardware)
- Rate limiting for value changes

### Setup Simplification

```cpp
void MotorFaderESPHomeComponent::setup() {
  // Version check (unchanged)
  // ...

  // No need to initialize layer states or send initial haptic config
  // Firmware initializes itself with default layer configs

  // If YAML specified layer_haptics config, apply them now:
  // (This will be in Python-generated code, calling set_layer_haptic_config)
}
```

## Migration Path

### Phase 1: Firmware Updates
1. Add layer storage variables to firmware
2. Implement new I2C register handlers (REG_ACTIVE_LAYER, REG_LAYER_TARGET, REG_LAYER_HAPTIC_CONFIG)
3. Update STATE register to include active layer index
4. Implement layer switching logic with deferral
5. Update mode transition logic to check for pending layer changes
6. Test firmware changes with production test fixture

### Phase 2: Protocol Updates
1. Update `i2c_data.h` with new register definitions
2. Update bit position constants (STATE_ACTIVE_LAYER_* aliases)
3. Update I2C protocol version to 5
4. Document protocol changes

### Phase 3: ESPHome Component Updates
1. Remove LayerState and layer management code
2. Simplify methods to forward to firmware
3. Update read_sensor_data_() to extract active layer from STATE
4. Update setup() to remove layer initialization
5. Test with firmware

### Phase 4: Python/ESPHome Bindings
1. Update action schemas (no changes needed - same API)
2. Update generated code to use new register addresses
3. Test YAML configurations

## Benefits of New Design

1. **Firmware Encapsulation**: All motor fader state is owned by firmware, including layer management
2. **Deferred Layer Switching**: Critical requirement satisfied at firmware level with mode awareness
3. **Simpler ESPHome Component**: Reduced from ~150 lines of layer management to ~30 lines of I2C forwarding
4. **Better Semantics**: Each layer acts like an independent I2C device
5. **Reduced Latency Issues**: No need for ESPHome-firmware synchronization via nonce timeouts
6. **Cleaner Protocol**: Layer-addressed operations are explicit, not inferred from nonces

## Open Questions

1. **EEPROM Persistence**: Should layer configs and restore positions be persisted to EEPROM? Currently not planned, but could be added.

2. **Layer Change Notifications**: Should there be an `on_layer_change` trigger in ESPHome when active layer changes? The active layer is included in all triggers, so this may not be needed.

3. **Default Layer on Boot**: Firmware always starts on layer 0. Should this be configurable?

4. **Invert Setting**: Currently handled in ESPHome (user-facing vs hardware positions). Should this be moved to firmware as a per-fader setting? Proposal: keep in ESPHome for simplicity.

## Testing Requirements

### Firmware Testing
- [ ] Layer storage initialization
- [ ] REG_ACTIVE_LAYER read/write
- [ ] REG_LAYER_TARGET layer-addressed read (write register+layer, then read)
- [ ] REG_LAYER_TARGET write for all layers
- [ ] REG_LAYER_HAPTIC_CONFIG layer-addressed read (write register+layer, then read)
- [ ] REG_LAYER_HAPTIC_CONFIG write for all layers (u16 format)
- [ ] Layer change immediate application during MODE_INPUT_IDLE
- [ ] Layer change immediate application during MODE_REMOTE_MOVEMENT_IN_PROGRESS (interrupts movement)
- [ ] Layer change deferral during MODE_INPUT_ACTIVE
- [ ] Layer change deferral during MODE_SELF_CALIBRATION
- [ ] Layer change rejection during MODE_ERROR (ignored, no deferral)
- [ ] Multiple pending layer changes (only last one kept)
- [ ] Restore position updates during input
- [ ] Restore position set at start of remote movement (not updated during movement)
- [ ] Active layer index in STATE register
- [ ] Haptic config application on layer switch
- [ ] Target position writes to non-active layers
- [ ] REG_ACTIVE_LAYER read returns current layer (may differ from write if deferred)
- [ ] queried_layer state tracking for layer-addressed reads

### ESPHome Component Testing
- [ ] set_active_layer() forwards to firmware
- [ ] remote_move_to() with layer parameter
- [ ] set_layer_haptic_config() with layer parameter
- [ ] get_active_layer() reads from firmware
- [ ] get_position() reads from firmware
- [ ] Triggers fire with correct active layer
- [ ] Invert setting works correctly
- [ ] Rate limiting still functions

### Integration Testing
- [ ] Layer switching during idle (applies immediately)
- [ ] Layer switching during user input (deferred until idle)
- [ ] Layer switching during remote movement (applies immediately, interrupts movement)
- [ ] Rapid layer switching (only last request kept when deferred)
- [ ] Multi-fader independence
- [ ] YAML layer_haptics configuration
- [ ] Dynamic haptic config changes at runtime
- [ ] Haptic config format updated to u16 (2 bytes)

## Success Criteria

- [ ] All 8 layers maintain independent haptic configs in firmware (u16 format)
- [ ] All 8 layers maintain independent restore positions in firmware
- [ ] Layer changes defer correctly during MODE_INPUT_ACTIVE only
- [ ] Layer changes apply immediately during MODE_REMOTE_MOVEMENT_IN_PROGRESS
- [ ] Restore positions update correctly in all modes
- [ ] STATE register includes active layer index (replacing haptic_config_nonce)
- [ ] ESPHome component successfully simplified (layer management removed)
- [ ] Protocol version updated to 5
- [ ] Haptic config reduced to 16 bits (nonce and target_position fields removed)
- [ ] Production test firmware updated and verified
