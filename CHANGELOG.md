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
| Rev A | 1.2 | 5 | 0.3.0 | Current |
| Rev A | 1.2 | 5 | 0.2.0 | Works; host does not log the motor characterisation |
| Rev A | 1.1 | 5 | 0.3.0 | Works; no motor characterisation to report |
| Rev A | 1.1 | 5 | 0.2.0 | Works |
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

### 1.2 - unreleased

- Remote movement restructured as explicit cascade control: the position loop
  sets a velocity reference and an inner velocity loop realises it. The old
  flat `KP*e - KD*v + FF` was algebraically the same loop, but writing it out
  exposed two defects it was hiding.
- The feedforward is now the plant model inverted, `breakaway + v_ref/k`,
  instead of a fixed duty. A fixed feedforward commands a fixed speed, so no
  slower speed was reachable without subtracting drive back off - which is what
  the one-sided velocity governor did, and why slow moves limit-cycled
  (overshoot, brake, fall under, accelerate). The governor and its companion
  error clamp are both gone; the speed limit is now a clamp on the reference.
- The velocity loop gain is scaled by `1/k`. Its open-loop gain is `k*KV`, so a
  higher-torque motor previously closed a proportionally hotter loop and rang
  through the lag of the velocity estimate - visible as the movement chugging
  even at full speed, on a fader with more torque than the reference unit.
- The stall-escape ramp is shed over ~20 ms rather than dropped in one tick,
  which was a relay that chattered around the stall threshold.
- The on-target window is sized from the time it takes to actually stop
  (reaction + detection + coast, 15 ms) rather than from the coast alone
  (6 ms), and is clamped to 8-20 ADC counts. The old figure produced a window a
  low-friction fader could not land inside, so it dithered at the end of a move
  instead of settling. The default window goes from 6 to 8 ADC counts.

- Self-calibration now characterises the motor as well as the endpoints,
  measuring per-direction breakaway duty, the speed/duty slope above it, and
  the speed motion starts at, then deriving the friction feedforward, the
  take-up ceiling and the on-target deadband from them. This is what makes the
  tuning hold across faders whose friction or torque differs from the unit the
  constants were centred on; a fader with materially lower stiction previously
  hunted at the deadband and could not settle. See
  `firmware/ABOUT_MOTOR_CONTROL.md`.
- New `REG_MOTOR_CAL` (0x12), reporting those measurements and the gains
  derived from them. Read-only and diagnostic; a host that ignores it loses
  nothing.
- Calibration is stored in one EEPROM record together with the endpoints. The
  record's magic changed, so the first boot after this update falls back to
  default endpoints until self-calibration is run - which it needs to be
  anyway, to measure the motor.
- Unrelated but necessary to fit the above: dropped an unused
  `<megaTinyCore.h>` include that was pulling the whole serial stack into a
  build with serial disabled, and moved several hot paths off floating point.
  Net flash is now below the pre-1.1 firmware despite the added feature.

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

### 0.3.0 - unreleased

- Reads `REG_MOTOR_CAL` at startup and logs what the fader measured about its
  motor, plus the gains derived from it, or a note that the unit has not been
  characterised yet. Diagnostic only - nothing else in the component depends
  on it - but it is the only way to see those numbers on a bench with no test
  jig attached.

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
