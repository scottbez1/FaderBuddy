# Changelog

FaderBuddy has three pieces that version independently: the **hardware**, the
**fader firmware** (ATtiny1616), and the **ESPHome component** that drives it
from a host. They are deliberately decoupled - a fader in the field can be
running older firmware than the host talking to it - so this file records what
each release changed and which combinations work together.

Separately, the **I2C protocol version** (reported at `REG_VERSION`) describes
the wire format of the register map. It is bumped only when the layout of an
existing register changes, never for additive features that an older host can
simply not use. Hosts should feature-detect on the firmware version rather than
the protocol version; see `firmware/src/shared/i2c_data.h`.

## Compatibility

Any firmware works with any component 0.2.0 or later - a component newer than
the firmware just logs a warning and skips features the firmware can't do
(e.g. move speed, motor characterisation). Component 0.1.0 needs the fader's
protocol version to match exactly, so it won't start against a future
firmware that bumps the protocol.

Firmware 1.0 predates `REG_FW_VERSION` and reports no version at all; hosts
that check it will read `0xFFFF` and should treat that as firmware 1.0.

---

## Hardware

### v1

All boards built to date - all are essentially identical, with minor changes
for production purposes.

- **v1.3.1** (pre-release) - adds a panelized gerber/BOM/CPL export for
  ordering 10 boards per panel. Only board change is two vias moved 0.75mm
  for mousebite clearance.
- **v1.3** - design files migrated to KiCad 10 (from 8); back silkscreen
  updated to say "FaderBuddy" instead of "motor fader".
- **v1.2** - fixed the JLCPCB BOM part number for the microcontroller, which
  had pointed at the ATtiny816 (1 ADC) instead of the ATtiny1616 (2 ADCs) this
  design needs.
- **v1.1 and earlier** (v0.1-v1.0) - initial bring-up; error in JLC part numbers.

---

## Firmware (ATtiny1616)

### 1.2 - unreleased

- Remote movement now uses cascade control (position loop sets a velocity
  reference, an inner loop realises it) with a friction feedforward derived
  from the plant model rather than a fixed duty. See
  `firmware/ABOUT_MOTOR_CONTROL.md` for why.
- Self-calibration now also characterises the motor (per-direction breakaway
  duty, speed/duty slope) and derives the feedforward, take-up ceiling, and
  on-target deadband from it, so tuning holds across faders with different
  friction or torque.
- New `REG_MOTOR_CAL` (0x12), reporting those measurements. Read-only and
  diagnostic.
- Calibration EEPROM format changed - the first boot after updating falls
  back to default endpoints until self-calibration is re-run.
- Self-calibration now returns the fader to the active layer's position using
  the newly measured endpoints. It previously re-used a target derived from the
  bounds it had just replaced, so the fader settled slightly off.
- The position nonce in `STATE` is now actually incremented on local input. It
  had no producer, so a user move that ended on the same 8-bit position as it
  started was invisible to the host.
- The backlash take-up ceiling is re-armed on a direction reversal rather than
  on every `LAYER_TARGET` write. A host streaming position updates previously
  held the fader at the take-up duty - roughly half speed - for the whole
  gesture.
- Detent haptics no longer pull toward the top of travel when the fader is
  below the calibrated minimum (reachable with a stale calibration).
- Fixed the hysteresis window being half-width when a move ended at the very top
  of travel, which made a fader parked there twice as likely to report spurious
  user input.

### 1.1 - unreleased

- Rewritten remote-movement control: PD on position with a per-direction
  friction feedforward, a stiction ramp, and a backlash take-up ramp. Moves are
  quieter and no longer overshoot. See `firmware/ABOUT_MOTOR_CONTROL.md`.
- `LAYER_TARGET` (0x0E) accepts an optional 4th byte setting move speed
  (0-255, 255 = full speed). Three-byte writes are unchanged, so this is
  backwards compatible.
- New `REG_FW_VERSION` (0x11), reporting a packed `(major << 8) | minor`.
  This is the first firmware that reports a version.
- Debug-only registers moved from 0x10-0x12 to 0xF0-0xF2, so production
  registers can keep growing from 0x10.
- On a movement timeout, a fader that is essentially in position now goes idle
  quietly instead of latching `MODE_ERROR`.

### 1.0

The last unversioned firmware - everything released before `REG_FW_VERSION`
existed. Protocol v5: firmware-managed layers, 16-bit haptic config, and the
layer-addressed registers.

---

## ESPHome component

### 0.3.0 - unreleased

- The hub now creates its own diagnostic text sensors - serial number and
  firmware version - so a bare `fader_buddy:` block reports what it is with no
  entity yaml at all. Names default to `<hub id> Serial Number` / `<hub id>
  Firmware Version`; override `name:`, or set `internal: true` /
  `disabled_by_default: true`, on the hub's `serial_number:` /
  `firmware_version:` key. Firmware version had no sensor before, only a log
  line.
- **Deprecated:** `text_sensor: platform: fader_buddy`, to be removed in 0.5.0.
  It still works and still wins over the hub's own serial number sensor, so
  there are never two, but it now logs a deprecation warning. To migrate,
  delete the `text_sensor:` block and move any `name:`/`icon:` onto the hub's
  `serial_number:` key.
- Fixed `remote_move_to` at speed 255 re-using the previous speed limit. Full
  speed was sent as a 3-byte write, which leaves the layer's stored speed
  alone, so a move after any slower one silently kept the old limit. The speed
  byte is now sent on every move when the firmware supports it.
- Reads `REG_MOTOR_CAL` at startup and logs what the fader measured about its
  motor, or a note that it hasn't been characterised yet. Diagnostic only.
- Reports the fader's mode changes, which previously went unlogged entirely:
  self-calibration starting and finishing (re-reading `REG_MOTOR_CAL` on
  completion, so the run's own measurements are logged rather than the ones
  read at startup), and a warning whenever the fader latches `MODE_ERROR` -
  including a failed endpoint sweep, which was silent.

### 0.2.0 - unreleased

- `fader_buddy.remote_move_to` takes `speed:` (0-255), and `layer_haptics`
  takes `default_speed:` for moves that don't name one.
- Reads and logs the fader's firmware version at startup, and warns instead
  of silently doing nothing when a config asks for a move speed the fader's
  firmware can't honour (the move runs at full speed instead).
- Accepts a fader reporting a newer protocol version, warning instead of
  failing to start. Only an older protocol is still treated as incompatible.

### 0.1.0

The last unversioned component - everything before this file existed. Layer
management, haptic config, rate-limited triggers, the serial number text
sensor, and position/touch/double-tap handling.
