# About: motor control and closed-loop tuning

Notes for whoever next touches the movement code. Covers the plant model, why
the control law is shaped the way it is, what each constant does, and how to
re-measure everything on hardware.

Three files: `src/motor_control.h` holds the drive primitives and every `MOVE_*`
constant; `src/main.cpp` runs the control law and owns the mode state machine;
`src/motor_cal.cpp` measures the plant during self-calibration and re-derives
the gains from it.

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

In `main.cpp`'s `motor_update()`, `MODE_REMOTE_MOVEMENT_IN_PROGRESS`:

```
error = target_adc - input_ewma
if |error| <= move_deadband:  brake, and start/continue the settle timer
else:
    v_ref = MOVE_VREF_SLOPE * error               # position loop
    |v_ref| clamped to the requested speed limit  # if one was requested
    |v_ref| floored at move_vel_min               # Stribeck floor
    u  = sign * (breakaway + |v_ref| / k)         # feedforward: plant inverted
    u += move_kv * (v_ref - velocity_ewma)        # velocity loop
    u += sign(error) * stiction_ramp              # stall escape
    drive_ceiling += MOVE_TAKEUP_RAMP_RATE * dt   # opens up from move start
    clamp |u| to (drive_ceiling + stiction_ramp), capped at MOVE_MAX_DUTY
```

**This is cascade control**: an outer position loop that asks for a velocity,
and an inner velocity loop that delivers it. Worth seeing in those terms,
because the previous flat form hid it - `u = KP*e - KD*v + FF` factors exactly
as `FF + KD*((KP/KD)*e - v)`, which is the same loop with `v_ref = (KP/KD)*e`
and gain `KD`. `MOVE_VREF_SLOPE` is that `KP/KD` ratio and keeps its old value
of 50/s; `MOVE_KV` is that `KD`.

Two things follow from writing it out, and both were real bugs in the flat form:

- **The inner loop's open-loop gain is `k * KV`**, not `KV`. `k` is the plant's
  ADC counts/sec per duty, so a higher-torque motor closes a proportionally
  hotter loop. Through the lag of the velocity estimate it goes underdamped and
  the fader oscillates *on velocity* - overshoot, brake, fall under,
  re-accelerate - which is not a position overshoot and does not look like one
  on the bench; it looks like the movement is chugging. `KV` is therefore
  scaled by `1/k` on a characterised unit, holding `k*KV` at the 1.16 the
  reference fader was tuned at.
- **The feedforward has to depend on the requested speed.** Inverting the plant
  gives `u_ff = breakaway + v_ref/k` exactly, so at steady state the feedback
  term is zero and the carriage simply travels at `v_ref`. A *fixed*
  feedforward instead commands whatever speed it commands - ~1100 ADC/s on the
  reference unit - and nothing slower is reachable without subtracting drive
  back off, which is what the old one-sided governor did. See "Optional speed
  limiting" below for why that misbehaved.

Five ideas, each solving a specific measured problem:

**Feedforward** is the plant model inverted: `breakaway + v_ref/k`. It supplies
both the breakaway duty and the duty that produces the wanted speed, so the
feedback term only has to correct the error in that model. Both halves are
measured per unit (see "Per-unit motor characterisation"); the compiled-in
defaults are the reference unit's values, so an uncharacterised fader behaves
as it always did.

**The velocity loop** is what prevents overshoot. Because `v_ref` is
`MOVE_VREF_SLOPE * error`, the controller is asking the carriage to be slower
the closer it gets, and drive falls away when it is travelling faster than
that. At 50/s against a coast limit of `1/tau ~ 200/s` it cuts about four
stopping distances short and coasts in. The old code had no such term and a
constant `+80` offset, so it drove at full authority right up to the deadband
and arrived at ~12000 raw/s - precisely the 58-80 raw overshoot it used to
show.

**Stiction ramp** fixes a genuine failure, not just feel. Because FF sits below
breakaway, small errors produce `FF + KP*error` below breakaway: the controller
commands motion it cannot produce, sits stuck outside the deadband, and after
8 s falls into `MODE_ERROR`. The ramp adds drive at `MOVE_RAMP_RATE` only while
commanded-but-stalled, capped at `MOVE_RAMP_MAX`. The cap also means a jammed
fader cannot wind up to full drive.

It is shed at `MOVE_RAMP_DECAY_RATE` once the carriage breaks free - fast
(~20 ms from full, far shorter than the coast time, so it still cannot
contribute to overshoot) but **not instantly**. Dropping up to 70 duty in one
tick is itself a relay, and sitting near `MOVE_STALL_VELOCITY` it chatters:
wind up, break free, dump the drive, stall again. With the feedforward now
derived to clear breakaway this should rarely engage at all during a normal
move; it covers measurement error, a cold or stiff unit, and the
uncharacterised case.

**Take-up ceiling** addresses belt backlash. A direction-reversed move otherwise
starts at full duty into slack: the rotor free-runs unloaded, then snaps taut
with an audible click and a jerk. The drive ceiling therefore starts at
`MOVE_TAKEUP_DUTY` and opens up to full over time at `MOVE_TAKEUP_RAMP_RATE`.

It is re-armed when a move is commanded in the opposite direction to the one in
progress, not on every write to `LAYER_TARGET`. Slack is only taken up on a
reversal, and a host that streams position updates - an LVGL slider being
dragged, say - retargets many times a second; re-arming on each of those would
hold the ceiling at the take-up duty for the whole gesture and cap the fader at
roughly half speed.

Three things about this are easy to get wrong, and all were bugs during
development:

- **It must be time-based, not gated on "are we moving yet".** While crossing
  the slack the rotor *is* moving, it just isn't loaded, so a velocity-gated
  ceiling lifts partway through the slack and full duty still lands on
  engagement. Time is the only available signal that doesn't depend on whether
  the belt has taken up.
- **The ceiling must stay above the feedforward.** It exists to stop drive
  stepping to full duty, not to suppress the feedforward. Set below it, it
  starves the term that gets the carriage moving: the fader stalls, waits for
  the stiction ramp, and breaks free abruptly - which hunts *and* makes the
  click worse. This regressed once already, when the feedforward was re-centred
  upward and the ceiling was left at its old value, which is why the derivation
  now computes the ceiling from the feedforward rather than storing it loose.

**Optional speed limit** (see below) is off by default and changes nothing when
unset.

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
| `MOVE_VREF_SLOPE` | 50 | ADC/s of velocity reference per ADC count of error. The old `KP/KD`. Higher approaches faster and coasts in from further out. |
| `MOVE_KV` | 0.04 | Velocity loop gain. **Default only** - scaled by `1/k` on a characterised unit, because `k*KV` is what sets damping. |
| `MOVE_BD_RISING/FALLING` | 68 / 86 | Assumed per-direction breakaway duty. **Default only** - measured on a characterised unit. |
| `MOVE_K_DEFAULT` | 29 | Assumed ADC counts/sec per duty. **Default only** - measured. |
| `MOVE_VEL_MIN` | 560 | Floor on the velocity reference, and so on the feedforward's clearance above breakaway. |
| `MOVE_DEADBAND` | 8.0 | On-target window - the final error at which a move is declared arrived. Has a hard floor set by the plant - see below. Don't lower it. Derived per unit when characterised. |
| `MOVE_STALL_VELOCITY` | 60 | "Not moving" threshold, for the stiction ramp. |
| `MOVE_RAMP_RATE/MAX` | 250 / 70 | Stall escape strength. |
| `MOVE_RAMP_DECAY_RATE` | 3500 | How fast the stall escape sheds once moving. Finite, not instant - see below. |
| `MOVE_TAKEUP_DUTY` | 130 | Drive ceiling at move start. Must stay above FF, which is why it is derived from FF on a characterised unit. |
| `MOVE_TAKEUP_RAMP_RATE` | 1200 | Duty/sec the ceiling opens up. Lower = gentler start. |
| `MOVE_SPEED_SLOWEST_MS` | 700 | Full-travel time at host speed 0. The slow end of the validated window. |
| `MOVE_SPEED_FASTEST_MS` | 250 | Full-travel time at host speed 254. Faster than this the limiter stops biting. |
| `MOVE_TIMEOUT_TOLERANCE` | 20 | On timeout, error below this goes idle instead of `MODE_ERROR`. |

**The deadband has a floor set by the plant, not by taste.** Nothing moves
below breakaway, and at breakaway speed jumps straight to `v_jump`, so the
smallest correction the fader can make is that speed times the time it takes to
actually stop. A deadband below that floor cannot be satisfied: the controller
steps past the window in both directions forever, which looks like dithering at
the end of a move and eventually trips the movement timeout.

**That stopping time is not the coast time.** Coast-down is first order with
tau ~5 ms, so the carriage travels ~`v * 6 ms` after drive is cut - but the
controller does not cut drive the instant the carriage reaches the window. The
position filter and the control tick both have to notice first, and the
carriage keeps moving throughout. The reference unit's measured minimum
correction of 5-8 ADC counts at 450-560 ADC/s implies **9-18 ms end to end**,
two to three times the coast alone. `MOTORCAL_STOP_TIME_MS` is 15 ms for that
reason; sizing it at 6 produced a window a looser fader could not land in.

The error is worth making in the generous direction. Too small and the fader
dithers and times out; too large and the final position is slightly off, which
at these sizes is 2-5 LSB of the 8-bit position the host sees. The first is a
failure, the second is a rounding difference.

**`k*KV` is the speed/robustness tradeoff**, and it is the product that
matters, not `KV` alone. Lower is faster but rings sooner; higher is smoother
but slower to settle. Lowering it is mathematically equivalent to adding
carriage mass, since both make the carriage coast further, so a sweep of it
doubles as an inertia sweep. Measured on the jig at `k*KV` 1.16:

| effective inertia | 1.0x | 1.5x | 2.0x | 3.0x |
|---|---|---|---|---|
| reversals over the suite | 0 | 6 | 60 | 145 |

So the tuning tolerates roughly 1.5-2x the *total* moving inertia (carriage +
belt + reflected rotor inertia) before ringing returns. A plastic knob cap is a
few grams against that and is not a concern.

**This is why `k` has to be measured.** Leave `KV` at a fixed 0.04 and a fader
with `k` 60 instead of 29 runs `k*KV` of 2.4 - past the 2.0x column above, into
the regime where the loop rings. It shows up as velocity chugging during the
move rather than as position overshoot at the end, which is why it is easy to
misread as a speed-limiter problem.

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

### The other failure: a unit with LESS friction than the reference

Centring FF buys tolerance, but it does not make the design unit-independent,
and the failure at the low-friction end is the mirror image of the one above.
Near the target the slowest motion the controller can command is the
equilibrium of `u = KP*e + FF - KD*v` against the plant:

```
v_min = k * (KP*deadband + FF - breakaway) / (1 + k*KD)
```

and a move can only settle if `v_min * (stopping time)` fits inside the
deadband. On the reference unit (breakaway 68, k 29) that is
`29*(12 + 106 - 68)/2.16 = 670` ADC/s, about 4 counts of stopping distance
inside a 6 count window. On a unit whose breakaway is 40 and whose `k` is 40,
the same constants give `40*(12 + 106 - 40)/2.6 = 1200` ADC/s and 7.2 counts -
**larger than the deadband**. The fader cannot make a correction small enough
to land in the window, so it steps past, reverses, steps past again, and hunts
until the movement timeout.

That is what a brand-new, looser fader looks like on the bench: jerky, worst at
low commanded speeds, and never quite arriving. It is not fixable by raising the
deadband (the window would have to grow past what the host can see) and it is
not fixable by lowering FF globally, because that reintroduces the stall failure
on stiffer units. The gap between FF and breakaway has to be a *per-unit*
quantity.

## Per-unit motor characterisation

Self-calibration therefore measures the plant as well as the endpoints, and the
gains above become defaults that a characterised unit overwrites. Three numbers
per direction, in `MotorCalData`, persisted in the same EEPROM record as
`calib_min`/`calib_max` and reported at `REG_MOTOR_CAL` (0x12):

| measured | what it is |
|---|---|
| `bd` | breakaway duty - the duty at which the carriage first moves at all |
| `k` | ADC counts/sec of cruise gained per duty count above breakaway |
| `vjump` | the speed motion starts at once breakaway is crossed (Stribeck) |

**The measurement.** Each run parks at the end it will travel away from, brakes
for 200 ms (breakaway from a standstill is a different number from the duty
needed to *sustain* motion, so residual coast would corrupt it), then ramps duty
at ~80/sec until speed exceeds a motion threshold - that duty is `bd`. It holds
there for 40 ms taking the peak speed, which is `vjump`, then dwells at `bd+25`
and `bd+50` for 100 ms each, averaging the tail of each hold. The two dwell
speeds give a two-point fit for `k`. Runs alternate direction so each starts
where the last ended, two passes per direction, averaged. Anything implausible
(`bd` outside 10..180, `k` outside 8..90, a run reaching the calibrated ends)
discards the whole characterisation and keeps the compiled-in defaults - a
half-measured fader is worse than an un-measured one.

**What is derived** (`apply_motor_calibration`, in `motor_cal.cpp`):

```
breakaway, k  ->  straight into the feedforward, u_ff = bd + v_ref/k
KV            =  1.16 / k        (holds the inner loop's gain k*KV constant)
vel_min       =  clamp(1.25 * vjump, 200, 1500)
deadband      =  clamp(vel_min * 15 ms, 8, 20)
takeup        =  max(130, max_dir(bd + vel_min/k) + 20)
```

The compiled-in defaults are themselves a plant model - breakaway 68/86, `k`
29, `vjump` 448 - so there is one derivation rather than one per case, and an
uncharacterised fader provably runs the gains the constants were tuned to.

**On a unit that has never been characterised** - or whose measurement failed -
every derived value falls back to the constant in the table above, so the
behaviour is exactly the pre-1.2 firmware. `REG_MOTOR_CAL` byte 0 says which
case a fader is in.

## Flash budget

Worth knowing before adding anything here: **the ATtiny1616's 16 KB is the
binding constraint on this file**, and the I2C bootloader work reserves the top
2 KB, leaving **14336 bytes for the application**. The characterisation above
was written to fit that, which is why the measurement and the derivation are
integer throughout and the ramp is fixed point - the float versions of the same
logic cost roughly 1.3 KB more and did not fit.

Three unrelated savings were taken at the same time and are worth not undoing:

- `main.cpp` no longer includes `<megaTinyCore.h>`. Nothing used it, but its
  ADC-error helpers take `HardwareSerial&` default arguments, which under LTO
  pulls all of `UART0.cpp` - both USART ISRs and the `Print`/`HardwareSerial`
  machinery - into a build with `SERIAL_ENABLED` off. **800 bytes of flash and
  145 of RAM.**
- The heartbeat LED and `nSLEEP` go straight to `VPORTB` rather than through
  `digitalWrite()`, which is 160 bytes of pin lookup for two call sites.
- `onI2cRequest` builds multi-byte replies through `i2c_write_u16`/`u32` helpers
  instead of writing bytes out at each call site.

If a future change does not fit, measure before trimming: build, then
`avr-nm -S --size-sort -C .pio/build/fader_buddy/firmware.elf | tail -20`.

## Optional speed limiting

`LAYER_TARGET` (0x0E) takes an optional 4th byte capping the speed of the move.
Three-byte writes behave exactly as before, so this is backwards compatible and
does not change the protocol version.

A host that uses the byte should send it on every move, 255 included. The byte
sets the layer's *stored* speed, and a 3-byte write leaves that alone - so
falling back to 3 bytes for a full-speed move silently re-applies whatever limit
the layer last held. Send 3 bytes only to firmware predating the byte, which
ignores a 4-byte write entirely rather than moving.

The byte is a **unitless 0-255 speed**: 255 (the default) is unlimited, and 0 is
the slowest the mechanism moves smoothly. Values below 255 map linearly in
*velocity* across `MOVE_SPEED_SLOWEST_MS` .. `MOVE_SPEED_FASTEST_MS`, which are
stated as full-travel times because that is how the measurements below were
taken; the calibrated span turns each into a velocity.

The scale is deliberately unitless rather than a move time in ms. The usable
window is narrow and bounded at both ends by the plant - past the fast end the
limiter stops having any effect, past the slow end the mechanism stick-slips -
so a physical-units parameter mostly offers values that do nothing or cannot be
honoured. Mapping the whole byte onto the validated window means every value a
host can send does something, and the endpoints can be re-measured and moved
without changing the host-facing meaning of the byte. The cost is that "how
long will this move take" is no longer readable off the parameter, which is
honest: it was never accurate to better than ~15% anyway.

**It is realised by clamping the velocity reference**, and nothing else - one
line in the control law. Because the feedforward is `breakaway + v_ref/k`, a
lower reference directly means less drive, so the requested speed is produced
rather than fought for.

That is worth stating against what it replaced, because the earlier design is
an instructive failure. With a *fixed* feedforward the drive alone commanded
~1100 ADC/s regardless of the request, so clamping the error could never slow
the fader below that floor. Two extra parts were bolted on to work around it:
an error clamp to stop the P term saturating, and a **one-sided governor** that
subtracted drive once measured velocity exceeded the limit.

A one-sided correction puts the operating point exactly on its own
discontinuity: below the limit it contributes nothing at all, above it a lagged
correction. That is a limit-cycle generator, and it behaved like one - overshoot
the limit, brake hard, fall under, accelerate again, most visible at low
requested speeds where the cycle is a large fraction of the commanded velocity.
Both parts are gone; the plant-inverting feedforward removed the floor that
made them necessary.

Measured on the reference fader, requesting a full-scale time and timing a move
of 67% of full scale:

| requested full-scale | 250 ms | 400 ms | 600 ms | 800 ms | 1200 ms | 2000 ms |
|---|---|---|---|---|---|---|
| measured | 193 ms | 261 ms | 356 ms | 444 ms | 584 ms | 759 ms |
| vs expected | +16% | -2% | -11% | -17% | -27% | -43% |

**Usable range is roughly 250 ms to 700 ms of full-scale time**, hence those two
endpoints for the speed scale. Those figures were measured against the old
governor, whose tracking error ran to -43% at the slow end; the reference-clamp
form should track considerably better, since the feedforward now produces the
requested speed directly instead of being pulled back down to it. **Re-measure
that table** - it is the clearest single check that this change did what it was
meant to.

Beyond the slow end it saturates: there is a hard floor below which the motor
cannot sustain continuous rotation and creeps in stick-slip steps rather than
moving smoothly. Slower smooth motion is not reachable by tuning - it would need
a different drive scheme.

`move_vel_min` (1.25x the measured Stribeck jump, 560 ADC counts/sec by default)
is the clamp for that floor. It is applied to the velocity reference in the
control law, every tick, and nowhere else - not where the speed byte is decoded,
which would freeze it against a `move_vel_min` that calibration can change
afterwards. It is also the floor the reference is held at right next to the
target, which is what keeps the feedforward clear of breakaway for small errors.

Direction asymmetry is within about 7%.

Treat this as a *limit*, not a precise speed control - it is realised through
the friction-dependent plant, so exact velocity varies with the fader.

The lab jig's `sstep <from> <to> <speed> <log_ms>` command takes the same
unitless byte, so re-measuring the table above means picking speeds rather than
times.

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
in `i2c_data.h` (`DEBUG_GAIN_*`); the reference slope, KV and deadband are
scaled by 1000.

Restore production firmware with `pio run -e fader_buddy -t upload` and
`pio run -e lilygo-t-display -t upload`.

The debug registers (0xF0-0xF2) are additive and compiled out unless
`DEBUG_DRIVE` is defined, so the protocol version is unchanged.

`REG_MOTOR_CAL` (0x12) is *not* debug-only and is present in production builds,
so a unit's measured breakaway, slope and jump speed can be read on a bench with
nothing but an I2C host attached - no jig, no lab firmware, no programmer. The
ESPHome component logs all of it at startup.

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
