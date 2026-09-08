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

| Hardware | Firmware | I2C protocol | ESPHome component | Notes |
|---|---|---|---|---|
| Rev A | 1.1 | 5 | 0.2.0 | Current |
| Rev A | 1.1 | 5 | 0.1.0 | Works; host cannot use move speed |
| Rev A | 1.0 | 5 | 0.2.0 | Works; move speed logs a warning and falls back to full speed |
| Rev A | 1.0 | 5 | 0.1.0 | Works |

Firmware 1.0 does not report a version at all (`REG_FW_VERSION` did not exist),
so a host reads `0xFFFF` and treats the fader as 1.0.

Component 0.1.0 requires the fader's protocol version to match exactly, so it
will refuse to start against any future firmware that bumps the protocol.
0.2.0 accepts equal-or-newer and warns instead - meaning a protocol bump is
only safe once hosts have moved to 0.2.0 or later.

---

## Hardware

### Rev A

All boards built to date. These are silkscreened with the build commit hash and
date rather than a revision number; "Rev A" is a label for the changelog, not
something printed on the board.

---

## Firmware (ATtiny1616)

### 1.1 - unreleased

- Rewritten remote-movement control: PD on position with a per-direction
  friction feedforward, a stiction ramp, and a backlash take-up ramp. Moves are
  quieter and no longer overshoot. See `firmware/ABOUT_MOTOR_CONTROL.md`.
- `LAYER_TARGET` (0x0E) accepts an optional 4th byte setting move speed, as a
  unitless 0-255 value across the validated speed range (255 = full speed).
  Three-byte writes are unchanged, so this is backwards compatible.
- New `REG_FW_VERSION` (0x11), reporting a packed `(major << 8) | minor`.
  This is the first firmware that reports a version.
- Debug-only registers moved from 0x10-0x12 to 0xF0-0xF2, so production
  registers can keep growing from 0x10. They remain compiled out of production
  builds.
- On a movement timeout, a fader that is essentially in position now goes idle
  quietly instead of latching `MODE_ERROR` and needing a host round trip.

### 1.0

The last unversioned firmware - everything released before `REG_FW_VERSION`
existed. Protocol v5: firmware-managed layers, 16-bit haptic config, and the
layer-addressed registers.

---

## ESPHome component

### 0.2.0 - unreleased

- `fader_buddy.remote_move_to` takes `speed:` (0-255), and `layer_haptics`
  takes `default_speed:` for moves that don't name one. The default is tracked
  host-side and sent with each move rather than stored on the fader.
- Reads and logs the fader's firmware version at startup, and warns rather
  than silently doing nothing when a config asks for a move speed the fader's
  firmware cannot honour (the move runs at full speed instead).
- Accepts a fader reporting a newer protocol version, warning instead of
  failing to start. Only an older protocol is still treated as incompatible.

### 0.1.0

The last unversioned component - everything before this file existed. Layer
management, haptic config, rate-limited triggers, the serial number text
sensor, and position/touch/double-tap handling.
