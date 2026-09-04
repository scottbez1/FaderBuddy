# About: motor control and closed-loop tuning

Notes for whoever next touches the movement code in `src/main.cpp`. Covers the
plant model, why the control law is shaped the way it is, what each constant
does, and how to re-measure everything on hardware.

## Units

Two different position scales appear in the code and in measurements:

- **ADC counts** (0-1023). What the firmware works in: `input_ewma`,
  `input_calib_min/max`, `target_adc`, all `MOVE_*` gains.
- **raw** (0-2046). What the `STATE` register reports, because ADC1 runs with
  2x accumulation. `raw = 2 * ADC`.

Full travel is about 1956 raw over 60 mm, so **1 mm is about 33 raw / 16 ADC counts**.

"Duty" always means the 0-254 value handed to `motor_set()`, matching the TCA0
split-mode period.

## The plant

Measured on a jig fader. Numbers vary between units; the shapes do not.

**It is a velocity source with a friction deadband, not a force source.**
Below a breakaway duty nothing moves at all. Above it, speed is roughly
proportional to duty. There is no useful "push gently" region: at breakaway the
speed jumps discontinuously to ~900 raw/s (Stribeck effect). This single fact
drives almost every design decision below.

**Breakaway duty depends strongly on PWM decay mode, not on frequency:**

| drive scheme            | dir A (rising ADC) | dir B (falling) |
|-------------------------|--------------------|-----------------|
| fast decay @ 306 Hz     | ~55                | ~75             |
| fast decay @ 19.6 kHz   | ~190               | ~200            |
| slow decay @ 19.6 kHz   | ~70                | ~90             |
| slow decay @ 39.2 kHz   | ~68                | ~88             |

Direction asymmetry (~20 duty counts) is consistent and real; the two
directions get separate feedforward constants.

**Velocity above breakaway** rises ~58 raw/s per duty count going up, ~94 going
down. Full duty is ~16000 raw/s (~490 mm/s).

**Coast-down is first order with tau ~5 ms.** Stopping distance after drive is
cut is about `v * 6 ms`: 7 raw from 2300 raw/s, 80 raw from 12000 raw/s.
Braking (both low sides on) only shortens this by ~25% - the mechanism is
already heavily friction-damped, so **braking is not the anti-overshoot lever**.
Cutting drive early is.

## Why slow decay

The DRV8837 takes IN1/IN2 as 1/0 forward, 0/1 reverse, 0/0 coast, 1/1 brake.

- **Fast decay** (what the code used to do): PWM one pin, hold the other low, so
  each cycle alternates drive and coast. Winding current decays through the body
  diodes every cycle. At a high carrier the average current ends up far below
  what the duty implies, so low-duty torque collapses - hence the 190 breakaway
  in the table above.
- **Slow decay** (what `motor_set(..., slow_decay=true, ...)` does): hold the
  leading pin high from PORT, PWM the trailing pin between drive and brake, so
  its high fraction is the inverse duty. Current keeps circulating through the
  bridge, average current tracks duty, and low-duty torque survives.

This is why movement no longer drops the carrier to 306 Hz. That switch existed
purely to buy torque back, and it was the audible buzz. There is now one
permanently inaudible carrier (19.6 kHz) for everything.

**Haptics deliberately still use fast decay.** `HAPTIC_BASE_PWM` and
`get_strength_max_pwm()`'s 189 floor were tuned against the fast-decay force
curve. Switching haptics to slow decay would make them far stronger and change
the feel. If you ever do switch them, those constants must be retuned together.

## The control law

In `motor_update()`, `MODE_REMOTE_MOVEMENT_IN_PROGRESS`:

```
error = target_adc - input_ewma
if |error| <= MOVE_DEADBAND:  brake, and start/continue the settle timer
else:
    u  = MOVE_KP * error                      # proportional
    u -= MOVE_KD * velocity_ewma              # damping
    u += sign(error) * MOVE_FF_{RISING,FALLING}   # friction feedforward
    u += sign(error) * stiction_ramp          # stall escape
    clamp |u| to MOVE_TAKEUP_DUTY while stalled, else MOVE_MAX_DUTY
```

Four ideas, each solving a specific measured problem:

**Feedforward** supplies the breakaway duty so the P term only has to set speed
rather than fight stiction. Deliberately set *above* the breakaway measured on
any single fader, and centred in the range that keeps every move settling - not
tuned for best accuracy on the reference unit. See "Tuning for hardware
variance" below; this is the single most important thing to understand before
changing these numbers.

**Derivative** is what prevents overshoot. Drive falls below breakaway when
`KP*error ≈ KD*velocity`, i.e. at `v = (KP/KD)*error`. With KP 2.0 and KD 0.04
that is 50/s, against a coast limit of `1/tau ≈ 200/s`. The controller
therefore cuts drive about four stopping distances short and coasts in. The old
code had no D term and a constant `+80` offset, so it drove at full authority
right up to the deadband and arrived at ~12000 raw/s - which is precisely the
58-80 raw overshoot it used to show.

**Stiction ramp** fixes a genuine failure, not just feel. Because FF sits below
breakaway, small errors produce `FF + KP*error` below breakaway: the controller
commands motion it cannot produce, sits stuck outside the deadband, and after
8 s falls into `MODE_ERROR`. The ramp adds drive at `MOVE_RAMP_RATE` only while
commanded-but-stalled, capped at `MOVE_RAMP_MAX`, and is dropped the moment the
carriage moves so it never contributes to overshoot. The cap also means a
jammed fader cannot wind up to full drive.

**Take-up limit** addresses belt backlash. A direction-reversed move otherwise
starts at full duty into slack: the rotor free-runs unloaded, then snaps taut
with an audible click and a jerk. While stalled, drive is capped at
`MOVE_TAKEUP_DUTY` - enough to cross the slack promptly, gentle on engagement.
Full authority returns as soon as the carriage moves, costing a few ms.

## Fixed-rate loop and the velocity estimate

The control law runs on a `CONTROL_TICK_US` tick, not once per `loop()`.
Previously it ran every iteration, so gains and filter constants were per
*iteration* and drifted with however long touch processing took. `adc_drain()`
still runs every pass, folding every completed ADC conversion into `input_ewma`
on the `RESRDY` flag, so the position filter runs at the ADC's own constant rate
and never re-uses or tears a result.

Velocity is `d(input_ewma)/dt`, smoothed with a fixed time constant
`VELOCITY_TAU_S`. `input_ewma` is a float fed by a dithered ADC, so its
differences stay meaningful well below one ADC count - no need for a longer
baseline. **dt is measured from the previous execution, not the scheduled tick
time**; using the schedule silently skews the estimate whenever the loop cannot
keep up.

At 2 kHz the main loop still runs 6-8 kHz, leaving headroom for `ptc_process`.
At 250 us the loop saturates (loop rate == tick rate) with no margin; don't.

## Constants and how to change them

| constant | value | effect |
|---|---|---|
| `CONTROL_TICK_US` | 500 | 2 kHz. Higher rates gain little; the limit is friction, not sample rate. |
| `MOVE_KP` | 2.0 | Approach speed. 3.0+ oscillates. |
| `MOVE_KD` | 0.04 | Damping. Main speed/robustness dial - see below. |
| `MOVE_FF_RISING/FALLING` | 106 / 124 | Per-direction friction feedforward, centred for hardware variance. Do not lower. |
| `MOVE_DEADBAND` | 6.0 | On-target window. Has a hard floor - see below. Don't lower it. |
| `MOVE_STALL_VELOCITY` | 60 | "Not moving" threshold, for ramp and take-up. |
| `MOVE_RAMP_RATE/MAX` | 250 / 70 | Stall escape strength. |
| `MOVE_TAKEUP_DUTY` | 95 | Backlash gentleness. 0 disables. |
| `MOVE_TIMEOUT_TOLERANCE` | 20 | On timeout, error below this goes idle instead of `MODE_ERROR`. |

**The deadband has a floor set by the plant, not by taste.** Nothing moves
below breakaway, and at breakaway speed jumps straight to ~900 raw/s, so the
smallest correction the fader can make is that speed times the reaction plus
stopping time - about 10-16 raw, i.e. 5-8 ADC counts. Set the deadband below
that floor and a fader whose breakaway does not match the feedforward constants
will step past the window in both directions indefinitely and trip the movement
timeout. This was a real field failure: 2.5 worked on the fader the gains were
tuned against, where fine control was possible, and hung on a different one.
There is nothing to gain by tightening it either - 6 ADC counts is about 1.5 LSB
of the 8-bit position the host actually sees.

`MOVE_TIMEOUT_TOLERANCE` is the backstop for the same problem. On movement
timeout, an error small enough to mean "arrived but hunting" goes quietly idle;
only a genuinely large error (jam, dead motor, target outside the calibrated
range) still latches `MODE_ERROR`, which needs a host round-trip to clear.

**KD is the speed/robustness tradeoff.** Lower is faster but rings sooner;
higher is smoother but slower to settle. Lowering KD is mathematically
equivalent to adding carriage mass, since both make the carriage coast further,
so a KD sweep doubles as an inertia sweep. Measured on the jig:

| effective inertia | 1.0x | 1.5x | 2.0x | 3.0x |
|---|---|---|---|---|
| reversals over the suite | 0 | 6 | 60 | 145 |

So the tuning tolerates roughly 1.5-2x the *total* moving inertia (carriage +
belt + reflected rotor inertia) before ringing returns. A plastic knob cap is a
few grams against that and is not a concern. KD 0.04 rather than 0.03 was chosen
to widen this margin; the cost is ~40 ms on a full-travel move.

## Tuning for hardware variance

This is an open source project and users run whatever fader they have - newer,
older, stiffer, looser. **The gains are therefore centred for the widest range
of hardware that still settles, not for best accuracy on one fader.** A slightly
missed target is barely noticeable. A fader that takes seconds to settle, or
times out into `MODE_ERROR`, is a dealbreaker. Optimise in that order.

The parameter that matters is how the feedforward compares to a given unit's
breakaway duty. Sweeping FF on the reference fader (breakaway 68 rising) shows
where moves stop settling:

| FF (rising) | 46 | 66 | 86 | 106 | 126 | 146 | 166 |
|---|---|---|---|---|---|---|---|
| moves that settled | 3/6 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 | 4/6 |

The window is roughly **66 to 146**, and it is very lopsided: only about 2 counts
of room below the reference breakaway, but ~78 above. So FF is set to 106, the
centre, rather than just under breakaway.

**Why too-low FF is the dangerous direction.** If FF lands below a unit's
breakaway, small errors cannot produce motion at all. The carriage stalls just
outside the deadband, the stiction ramp winds up, and when it finally crosses
breakaway the speed jumps discontinuously to ~900 raw/s - far too fast to stop
inside the window. It overshoots, the error flips sign, and the cycle repeats
until the movement timeout. That is what "motor active, oscillating, never
settles, then error" looks like in the field, and no deadband value fully
rescues it (tested up to 10 ADC counts).

**Why too-high FF is cheap.** `MOVE_TAKEUP_DUTY` caps drive while the carriage
is stalled, so an over-high feedforward is simply clamped instead of producing a
violent escape. Once moving, the D term absorbs the extra.

Measured tolerance at the shipping values, emulated by offsetting FF:

| unit breakaway vs reference | -40 | -30 | -20 | -10 | 0 | +10 | +20 | +30 | +40 |
|---|---|---|---|---|---|---|---|---|---|
| moves that settled | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 7/7 | 6/7 |

So roughly **-40 to +30 duty counts** of breakaway spread is absorbed, with
worst-case steady-state error about 5.6 ADC counts (~0.35 mm, under 1.5 LSB of
the 8-bit position a host sees). The previous "just under breakaway" tuning
failed already at -10.

With FF centred, the design also stops being sensitive to the other constants:
every deadband from 3 to 9 and every take-up value from 70 to 150 settles 7/7.
That insensitivity is the point - if a change makes behaviour depend sharply on
one constant again, that is a regression even if the reference fader looks good.

If a unit ever falls outside this window, the fix is per-unit calibration rather
than re-centring the constants: extend self-calibration to ramp duty until
motion starts in each direction and store the result in EEPROM next to
`calib_min`/`calib_max`, or have the stiction ramp learn and persist an offset.

## Reproducing the measurements

Two extra build environments exist for this; neither ships.

```bash
source ~/.platformio/penv/bin/activate

# ATtiny with REG_DEBUG_DRIVE / STATUS / GAINS (repo root)
pio run -e fader_buddy_lab -t upload

# ESP32 jig as a serial-driven measurement harness, replacing the
# production test state machine
cd production_tools/programAndTest && pio run -e lab -t upload
```

`env:lab` (`src/lab_main.cpp`) gives a line-oriented serial interface at
921600 baud: `step <from> <to> <ms>` captures a step response at ~3.2 kHz,
`openloop` and `stopdist` drive the bridge directly for system ID, `dbg` reads
control-loop internals, and `gain <index> <value>` overrides any tuning constant
at runtime so a gain sweep does not need a reflash per trial. Gain indices are
in `i2c_data.h` (`DEBUG_GAIN_*`); KP, KD and deadband are scaled by 1000.

Restore production firmware with `pio run -e fader_buddy -t upload` and
`pio run -e lilygo-t-display -t upload`.

The debug registers (0x10-0x12) are additive and compiled out unless
`DEBUG_DRIVE` is defined, so the protocol version is unchanged.

## Measurement gotchas

- **Derive the target from `calib_min`/`calib_max`, not from where the fader
  settles.** Read them via `REG_DEBUG_STATUS`. Inferring the mapping from
  settled endpoints bakes in the controller's own error - this produced a
  phantom 22 raw bias early in tuning.
- **The `STATE` raw ADC field is noisy while the motor drives.** Excursions of
  40+ raw appear there that the filtered `pos` field does not show; they are ADC
  noise, not motion. Median-filter before computing overshoot, and cross-check
  against `pos`.
- **Clear `MODE_ERROR` between automated trials.** One latched error otherwise
  makes every later trial report a frozen fader.
- Reported `err` from `REG_DEBUG_STATUS` is the same float the controller uses
  and is the ground truth for steady-state accuracy.
