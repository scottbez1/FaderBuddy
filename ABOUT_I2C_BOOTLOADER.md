# I2C Bootloader (ATtiny1616)

FaderBuddy firmware can be updated over the I2C bus, with no UPDI programmer
attached. A small bootloader lives in the ATtiny1616's hardware boot section and
writes the application section on command from the host.

This document describes how that works, what it guarantees, and where it is
still weak. Commands for building, flashing and testing live in
[CLAUDE.md](CLAUDE.md).

## 1. Goal and constraints

Without the bootloader, an ATtiny1616 can only be flashed with a physical UPDI
programmer on three test pads (see
[ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md)). The bootloader lets an
ESP32 host push updates over the wires that are already there. The design had to
work under four hard constraints:

- **No pin changes and no new wires.** Host and target share only I2C
  (SDA/SCL on the TWI0 pins).
- **The host cannot control target power.** Anything that needs a power cycle to
  enter update mode is out.
- **Keep the PlatformIO + megaTinyCore toolchain** for the ATtiny firmware.
- **A bad application must never brick a board.** Recovery has to stay possible
  over I2C.

The boot section satisfies all four. It owns the reset vector, so it always runs
first. NVMCTRL self-programming lets it write the application. Hardware write
protection stops the application from damaging it.

The one-time cost is a **single UPDI flash per board**, installing the
bootloader, the boot-section fuses, and an application linked at the post-boot
offset. After that, updates are I2C-only. UPDI stays the bring-up path for blank
chips and the recovery path of last resort.

## 2. Flash memory map

The ATtiny1616 has 16 KB of flash with a 64-byte page size (256 pages), mapped
into the data space at `FLASHSTART = 0x8000`. Two fuses split it into sections:

```
program view   data-space (mapped) view
0x0000         0x8000  ┌────────────────────────┐  ← reset vector (PC = 0x0000)
                       │  BOOT section          │     (always executes first on reset)
                       │  = the I2C bootloader  │
BOOTEND*256    0x8000+BOOTEND*256 ├──────────────┤  ← interrupt vectors + application start
                       │  APPCODE section       │
                       │  = the FaderBuddy app  │
0x4000         0xBFFF  └────────────────────────┘  (16 KB)
```

Two address spaces matter for the flash-write code. The program counter and the
section fuses use offsets from `0x0000`. Loads and stores that fill the page
buffer use the mapped view at `0x8000 + offset`. Everywhere below, "flash
address X" means the program/section offset; the store target is `0x8000 + X`.

- **`BOOTEND`** sets the boot-section size in 256-byte units. It is **`0x06`**
  (1536 bytes) today. The bootloader measures about 1366 bytes, leaving roughly
  170 bytes free.
- **`APPEND`** is `0x00`, so the application section runs to the end of flash.
  There is no separate APPDATA region.
- The application is linked to start at `BOOTEND * 256` = `0x0600`.

`BOOTEND` is not a value to change casually. Six places move in lockstep; the
comment on `BL_BOOTEND` in `firmware/src/shared/bootloader_protocol.h` lists
them.

Application flash is tight. The app has roughly 130 bytes of headroom below the
`.fw_meta` footer, and about 1.1 KB of the image is avr-libc soft-float pulled
in by the float-based control law in `motor_control.h`. Converting the control
hot path to fixed point is the next real source of app flash.

## 3. Boot-section integrity

Per the datasheet (*NVMCTRL — Memory Organization*, §9.3.1.1):

- **The CPU can never write the BOOT section.** This is inherent hardware
  behaviour, independent of any fuse or lock bit. Neither the application nor
  the bootloader itself can rewrite it.
- **Inter-section writes are directional.** BOOT code may write APPCODE and
  APPDATA. APPCODE may write only APPDATA. APPDATA may write neither.
- **`BOOTLOCK` and `APCWP`** are optional `NVMCTRL.CTRLB` lock bits, not fuses,
  and this design sets neither. `BOOTLOCK` also blocks reads and execution of
  the boot section, so setting it would stop the bootloader from running at all.

The consequence is the safety property the whole design rests on: **a buggy or
half-written application can never corrupt the bootloader.** Since the
bootloader runs first on every reset, even a completely broken application still
leaves a route back over I2C. As defence in depth, the bootloader also
bounds-checks every target page address in software.

The flip side is that **the bootloader itself is not field-updatable**. Changing
it always requires UPDI. Keeping it small and stable is therefore a feature, and
any change to it must stay compatible with hosts and application images already
in the field.

## 4. Interrupt vectors

`CPUINT.CTRLA.IVSEL` selects where the interrupt vector table is fetched from.
`IVSEL = 0`, the reset default, points at the start of the application section;
`IVSEL = 1` points at the boot section.

No code touches `IVSEL`. The bootloader uses no interrupts, so it does not care.
The application wants its vectors at the application start, which is exactly
what the reset default gives it, as long as it is linked with its vector table
at `BOOTEND * 256`. Reset always vectors to `0x0000` regardless of `IVSEL`.

## 5. Self-programming via NVMCTRL

Writing flash is a two-step "fill the page buffer, then issue a command" flow.
Per 64-byte page:

1. **Pre-check.** Poll `NVMCTRL.STATUS` until `FBUSY` and `EEBUSY` are clear.
2. **Clear the page buffer** with `NVMCTRL.CTRLA = PBC`. The buffer auto-clears
   after any reset, write, erase, or sleep-wake, so this is cheap insurance
   rather than a requirement.
3. **Fill the buffer** with ordinary stores to the mapped addresses
   (`0x8000 + offset`) of the target page. Stores to `0x0000`-based addresses
   would not reach flash.
4. **Erase and write** with `NVMCTRL.CTRLA = ERWP`.
5. **Check `WRERROR`** afterwards. No explicit wait is needed: the CPU is halted
   for the duration of the flash operation, so the next instruction does not run
   until it completes.

Every write to `NVMCTRL.CTRLA` goes through Configuration Change Protection,
using the **SPM** key `0x9D` (`_PROTECTED_WRITE_SPM`). The IOREG key `0xD8`
(`_PROTECTED_WRITE`) is for protected I/O registers such as `RSTCTRL.SWRR`, and
does not work for `NVMCTRL.CTRLA`.

Command values, from the datasheet `CTRLA.CMD` table (§9.5.1):

| Value | Name   | Meaning                     | Notes |
|:-----:|--------|-----------------------------|-------|
| 0x00  | —      | No command                  | |
| 0x01  | `WP`   | Write page                  | |
| 0x02  | `ER`   | Erase page                  | |
| 0x03  | `ERWP` | Erase and write page        | what page streaming uses |
| 0x04  | `PBC`  | Page buffer clear           | |
| 0x05  | `CHER` | Chip erase (Flash + EEPROM) | **not used** — it would erase EEPROM too, losing calibration |

Because `BOOTEND` counts 256-byte units and 256 is a multiple of the 64-byte
page, the application start is always page-aligned. There is no partial-page
case at the section boundary.

## 6. Wire protocol

The bootloader drives the **TWI0** slave registers directly, polled, with no
ISR and no `Wire` library. That keeps it small and self-contained in the boot
section. The protocol is defined in
[`firmware/src/shared/bootloader_protocol.h`](firmware/src/shared/bootloader_protocol.h),
which is shared with every host.

**Address.** The bootloader answers on the **same address as the application** —
base `0x20` plus the 3-bit hardware address from PC2/PC1/PC0. A fader keeps its
identity in bootloader mode.

**Commands** (multi-byte fields big-endian, matching `i2c_data.h`):

| Opcode | Command             | Payload / behaviour                                                       |
|:------:|---------------------|---------------------------------------------------------------------------|
| `0x01` | `SET_PAGE_ADDR`     | 2-byte flash page address; resets the frame counter and page buffer        |
| `0x02` | `SEND_FRAME`        | 16 data bytes + CRC16; 4 frames fill a 64-byte page, which then auto-writes |
| `0x03` | `RUN_APP`           | leave bootloader mode and start the application                           |
| `0x04` | `ERASE_APP`         | invalidate the application (see below)                                    |
| `0x05` | `GET_STATUS`        | read back `[version, status, last_error]`                                 |
| `0x06` | `GET_VERSION_CRC16` | address + length → bootloader version + CRC16 of that flash range         |

Master-write commands are processed at the **Stop** condition. Master-read
commands are processed at the address match. Reading register `0x00` returns
`BL_VERSION_MARKER` (`0xB0`), which is how a host tells bootloader mode from a
running application (see [§8](#8-version-identity-and-no-app-detection)).

**`ERASE_APP` erases only the first page**, the one holding the reset vector.
That is enough to mark the application invalid, and each streamed page is
written with `ERWP`, which erases it in place anyway. See
[§12](#12-implementation-notes) for why a full-section erase is avoided.

**The update sequence** is:

1. `ERASE_APP`
2. For each page: `SET_PAGE_ADDR`, then `SEND_FRAME` × 4
3. `GET_VERSION_CRC16` over the whole application region, to verify the write
4. `RUN_APP`

Per-frame CRC16 catches transfer errors as they happen. The whole-image CRC
catches anything missed and stops the host from jumping into a bad image.

## 7. Entering the bootloader

There is no power-cycle control, so entry is application-triggered, with a
hardware escape hatch for when the application cannot help.

### From a running application

1. The host writes `REG_ENTER_BOOTLOADER` (0x10) with the magic payload
   `ENTER_BOOTLOADER_MAGIC`. The magic means a stray or corrupt write cannot
   reboot a fader by accident.
2. The I2C ISR only sets a flag. The main loop then writes the **entry token**
   and issues a software reset via `RSTCTRL.SWRR`.
3. The bootloader runs, reads `RSTCTRL.RSTFR` to confirm the reset was a
   software reset (`SWRF`), and checks the token. If both match it stays
   resident. Otherwise it clears the token and starts the application. Either
   way it writes `RSTFR` back to clear the sticky flags.

The entry token is a `.noinit` RAM variable pinned to **`0x3F00`** in both
builds. RAM survives a warm reset, and the C runtime does not clear `.noinit`.
GPIOR registers cannot be used for this: they are cleared by every reset,
including a software reset.

Requiring both the reset-cause flag and the token makes accidental entry
effectively impossible. After a power-on reset, RAM is indeterminate, but that
path has neither `SWRF` set nor a valid token, so it correctly falls through to
the application.

### Forced entry with the TP5 strap

`REG_ENTER_BOOTLOADER` only works if the application runs and answers the bus.
Two failure modes escape it: an image that was partially written but whose reset
vector happens to be programmed, so `app_is_valid()` accepts it; and an
application that starts but wedges before serving I2C. Both leave a fader
looking dead.

**TP5 (PB5)** is the hardware escape. Its net is the MCU pin and one test pad on
the back of the board, and nothing else, so only a deliberate short pulls it
low. To use it:

1. Short TP5 to any ground. The nearest are R2 and J4, about 4 mm away.
2. Power-cycle or reset the board.
3. Release the short. It only has to be held across reset.

The bootloader is then resident and answers `BL_VERSION_MARKER` at register
`0x00`. Update it normally, then send `RUN_APP`.

`strap_requests_bootloader()` enables the internal pull-up, samples the pin
eight times over a few hundred microseconds, and requires every sample to read
low. It then switches the pull-up back off, so the application sees the pin in
its reset state and a permanently grounded pad draws no current. The check costs
62 bytes of boot section and well under a millisecond on every boot.

For anyone revisiting the pin choice: PA3 (TP6) is TCA0 `WO3`, and PA4/PA5 are
`WO4`/`WO5`, the motor outputs, under this part's default `PORTMUX.CTRLC`
mapping. PA7 (TP1) and PB4 (TP4) are the equivalent alternatives. PB5 was taken
because the bootloader already drives PORTB for the heartbeat LED. TP2 and TP3
are RX/TX and are left for serial debug.

## 8. Version identity and "no app" detection

A host needs to tell three states apart on the bus, and needs a version to
compare against when deciding whether to update.

Reading register `0x00` is the universal probe:

| Response at `0x00`            | Meaning                                 |
|-------------------------------|-----------------------------------------|
| A valid protocol version (≥5) | The application is running normally     |
| `0xB0` (`BL_VERSION_MARKER`)  | The bootloader is resident; no usable app |
| NAK / no response             | Device absent, unpowered, or bus wedged |

`REG_VERSION` is a protocol compatibility version and is too coarse to drive
updates, so the application also serves **`REG_FW_VERSION`** (0x11), a packed
`(major << 8) | minor`. The host compares that against the version it has
packaged.

A board ends up bootloader-only in one of two ways: a bootloader flashed without
an application, or an update interrupted partway through. There is no atomic A/B
image — 16 KB of flash cannot hold two — so an update is not transactional. On
every boot the bootloader therefore runs `app_is_valid()`, which today checks
that the application's reset vector is not blank. If the check fails, the
bootloader stays resident and keeps answering with the marker, which is exactly
the state the host detects. That check is weaker than it should be; see
[§11](#11-known-gaps).

**Pre-bootloader firmware.** Boards running firmware older than the bootloader
work answer `0x00` with a normal protocol version but have no bootloader behind
them. The host distinguishes them with `REG_FW_VERSION` against
`FW_VERSION_BOOTLOADER_ENTRY`, the minimum version that honours
`REG_ENTER_BOOTLOADER`. Anything below that, including `FW_VERSION_NONE`, means
a one-time UPDI migration is required. This is a floor, not a guarantee: the
application cannot read the boot section, so a board whose bootloader was
somehow never installed would still report a bootloader-aware version. In
practice the two are installed together, and the host's marker probe catches the
mismatch before anything is erased.

## 9. Build and install

Three PlatformIO environments, all in the root `platformio.ini`:

- **`fb_bootloader_only`** — the bootloader alone. Bare-metal, no Arduino
  framework, linked into the boot section.
- **`fb_app_only`** — the application linked at `BOOTEND * 256`, with its vector
  table at the application start. This is the day-to-day development path: once
  the bootloader is installed it stays installed, so a developer re-flashes only
  the application over UPDI. `USING_OPTIBOOT` strips megaTinyCore's
  reset-flag and reset-loop `.init3` code, which the bootloader has already
  handled.
- **`fb_app_and_bootloader`** — the combined first flash. A post-build hook
  merges the two hex files, which never overlap, and a single
  `firmware/tools/flash_with_fuses.py` invocation writes the merged image **and**
  the `BOOTEND`/`APPEND` fuses over UPDI with `pymcuprog`.

A fourth environment, `fb_legacy_no_bootloader`, builds the application at
`0x0000` with no bootloader at all. It is only for boards that are not getting
the bootloader.

So bringing up a blank chip is one action: select `fb_app_and_bootloader` and
upload, using the same physical setup as
[ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md). That same combined
upload is the recovery path if a board is ever left in a state I2C cannot reach.

## 10. Host side

### The ESPHome component

`esphome/components/fader_buddy/` embeds a packaged application image at codegen
time and drives the update over I2C. Configuration and usage are documented in
[ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md); this section
covers how it works.

**What gets packaged.** Only the application. The image is the exact
page-aligned APPCODE bytes as a raw `.bin`, produced by
`firmware/tools/export_app_image.py`.

**Where the version comes from.** `FW_VERSION` is baked into the image at a
fixed address, the last two bytes of flash (`BL_APP_META_ADDR` in
`bootloader_protocol.h`, `FW_VERSION_FOOTER` in `firmware/src/main.cpp`).
Because the application section always runs to `FLASHEND`, that address never
moves as the application grows. The component reads the version straight from
the packaged `.bin`, so it cannot drift from what the flashed application will
report. The whole-image CRC16 is not baked in, because a CRC cannot cover
itself; it is computed at ESPHome build time from the same bytes.

**Embedding.** `to_code()` emits the image as a `static const uint8_t[] PROGMEM`
array. The component is `MULTI_CONF`, and many faders normally run the same
firmware, so the blob is emitted once per distinct image and shared across
instances rather than duplicating roughly 14 KB per fader.

**The sequence** is the same one the jig uses: probe register `0x00` → if not
already resident, `REG_ENTER_BOOTLOADER` and wait for the marker → `ERASE_APP` →
stream pages → whole-image `GET_VERSION_CRC16` → `RUN_APP` → re-read
`REG_FW_VERSION` to confirm.

**It runs a slice at a time from `loop()`.** `update_tick_()` walks an
`UpdateStage` enum, one stage per call; `update_firmware()` only arms the state
machine and returns. The transfer takes seconds, and blocking for that long
would starve the API and WiFi, and would make progress reporting impossible,
since queued entity states are only flushed from the loop. Three details make
the slicing work:

- **Page streaming is time-budgeted**, not one page per tick. `UPDATE_WRITE`
  keeps writing until `UPDATE_WRITE_BUDGET_MS` is spent, then yields. One page
  per tick would turn a 230-page image into about 4 seconds of pure latency.
- **The two long stalls are split into "ask" and "poll"** — erase and
  whole-image CRC. Both leave the target unable to answer for hundreds of
  milliseconds, and waiting for them inline would have been the one thing still
  blocking the loop.
- **Every polling stage carries a deadline**, compared wraparound-safe.

`update()` early-returns while an update is in flight, since a fader in its
bootloader has no `REG_STATE` to read.

**Guard rails:**

- **Only on a version mismatch.** A fader already running the packaged version
  is a no-op, reported as success.
- **No attempt cap, and no retry.** A failed update is reported and forgotten;
  nothing retries it and no counter persists. That only needs revisiting if
  updates ever become automatic — a loop that retries by itself needs a stop,
  but a human pressing a button already is one.
- **Never interrupt the user.** The run waits, bounded, for the fader to leave
  `MODE_INPUT_ACTIVE` before taking the bus, and reports failure if it is still
  in use.
- **One fader at a time.** A class-static flag refuses a second concurrent
  update across all instances. It is claimed before the first tick and held for
  the whole run, which matters because the sequence is interruptible: without
  it, a second fader could start in a gap between the first one's slices.
- **Manual only.** There is no autoupdate mode. An update runs only when a human
  presses the button or an automation calls the action.

### The production jig

`production_tools/programAndTest/` is the reference implementation and the
regression test. Every board that goes through the jig exercises the full update
path:

- `TEST_FW_BOOTSTRAP` UPDI-flashes the DUT with the **current bootloader, built
  from source on that run**, plus a fixed `FW_VERSION=0` application. That
  establishes "a board with a bootloader, running an old app".
- `TEST_FW_I2C_UPDATE` then drives `REG_ENTER_BOOTLOADER` from that running old
  application and updates it to the current application over I2C.

Building the bootloader fresh on every jig run is deliberate. A checked-in
bootloader image would silently ship a stale bootloader on every board flashed
after a bootloader change, and the bootloader is the one part that cannot be
fixed later over I2C. See
`production_tools/programAndTest/factory_test_images/README.md`.

## 11. Known gaps

Everything below is absent from the current implementation, not broken in it.

- **App validity is only a reset-vector check.** `app_is_valid()` checks that
  the reset vector is not blank. It does not catch a partially written image
  whose first page happened to land. Closing this means a linker-placed CRC
  footer (image length + CRC16 at the end of APPCODE) that the bootloader
  recomputes on boot, reusing the CRC16 already used for post-write verify.
  Streaming page 0 last would also help, since an interrupted update would then
  leave the reset vector blank. Until then, the TP5 strap is the recovery path,
  and it needs physical access.
- **No command timeout in the resident bootloader.** A host that abandons an
  update mid-stream leaves the bootloader resident forever. A watchdog or
  command timeout would let it recover on its own — jumping to the application
  if one is valid, otherwise resetting the TWI interface and continuing to
  serve. On a shared bus this matters more than it would on a point-to-point
  link.
- **No host-side bus recovery.** Neither the jig nor the ESPHome component
  recovers the master after a transaction timeout (`Wire.end()`/`begin()`, or
  clocking nine SCL pulses). This cannot rescue a *slave* holding SCL low, but
  it would protect against other stalls.
- **`ERASE_APP` does not blank pages beyond the new image.** It erases only the
  reset-vector page, and streaming rewrites only the pages the image covers. If
  a new image is smaller than the old one, stale bytes remain above it. This is
  harmless today, since the CRC verify covers exactly the streamed range, but it
  would matter for a CRC footer over the whole section.
- **Unvalidated on hardware:** the combined `fb_app_and_bootloader` upload onto
  a truly blank chip, and updating a multi-fader chain one fader at a time
  without disturbing the others. The individual pieces of both are validated;
  the combinations are not.
- **No CI coverage.** There is no hardware in CI, so every claim above was
  validated on the bench or by the jig.

## 12. Implementation notes

Details that are not obvious from the code, and that are likely to trip up the
next person working in this area.

**The address jumpers decode backwards from the port bits.** The board nets are
`PC0 = A2`, `PC1 = A1`, `PC2 = A0`, and a fitted jumper pulls its pin low, so a
fitted jumper *sets* its address bit. The bootloader and the application must
decode this identically — `twi_slave_init()` here and `setup_i2c()` in
`firmware/src/main.cpp` — because a mismatch does not fail loudly: the fader
simply answers on a different address once it enters the bootloader, and only
for some jumper settings. Reversing the three bits swaps offsets 1↔4 and 3↔6
and leaves 0, 2, 5 and 7 looking perfectly fine, so half a bench of boards works
and the rest vanish exactly when a host tries to update them.

**The bootloader must raise the main clock itself.** The reset default is
OSC20M divided by 6 — `CLKCTRL.MCLKCTRLB` comes up with `PDIV` = 6X and `PEN`
set — so CLK_PER is 3.33 MHz, not the 20 MHz the application runs at. That is
not just slow: the TWI slave is synchronous, and §26.3.2.1 requires
f_CLK_PER ≥ 10 × f_SCL, capping the bus at 333 kHz. A host on the common
400 kHz was out of spec for the whole time a fader sat in its bootloader, which
looks like intermittent NAKs and dropped frames during precisely the operation
that has to work. `run_at_full_speed()` clears the prescaler as the first thing
`main()` does, and `jump_to_app()` restores the reset value on the way out.
Note that the LED heartbeat is *not* affected either way — it runs off the RTC's
own 32.768 kHz oscillator rather than CLK_PER — but the two busy-wait loops
(pull-up settle, strap sampling) are, and were written against 20 MHz.

Bootloaders already on boards keep the old behaviour, since the boot section is
not field-updatable. If updates over I2C are flaky on such a board, drop the
host bus to 100 kHz or reflash the bootloader over UPDI.

**`PIEN` must be set on the TWI slave.** `TWI0.SCTRLA` needs
`TWI_PIEN_bm | TWI_ENABLE_bm`. Per datasheet §26.5.9, `PIEN` gates whether
`APIF` is raised on a Stop condition — the *flag*, not merely the interrupt.
The bootloader processes every master-write command at Stop, so without `PIEN`
commands like `SET_PAGE_ADDR` and `SEND_FRAME` are silently dropped while the
hardware still byte-ACKs them. Reads keep working, because they are processed at
the repeated-start address match, which raises `APIF` regardless. `DIEN` and
`APIEN` gate only interrupts and stay off, since there is no ISR.

**Every TWI event must end with one `SCMD` write.** `APIF` and `DIF` can only be
cleared by writing `SDATA` or `SCMD`; writing `SSTATUS` does not clear them. An
error path that clears flags and returns without issuing an `SCMD` leaves
`CLKHOLD` asserted, which holds SCL low and wedges the bus for everyone.
`COLL` co-occurs with `APIF`/`DIF` at the end of a slave transmit, so it must be
folded into the master-read completion check rather than handled as a separate
early return. `twi_service()` follows megaTinyCore's polled handler: compute one
action, always write `SCTRLB` at the end.

**Do not bulk-erase the application section.** Roughly 224 back-to-back
standalone `ER` commands leave the NVM controller silently unable to write:
subsequent `ERWP` commands no-op with no `WRERROR` set. A single erase is fine,
and many `ERWP` operations are fine. This is why `erase_app()` erases only the
reset-vector page. The exact silicon mechanism is unknown.

**The bootloader cannot trust an immediate read-back of what it just wrote.**
Reading mapped flash right after a write can return the page-buffer value rather
than what is actually in flash. Use a UPDI read as ground truth, or re-read
after other NVM activity.

**UPDI flash readback is the most useful debugging tool here**, because it is
independent of the I2C path. RAM readback is useful too but needs care: confirm
with a sentinel value that the read itself does not reset the part, since unlike
flash, RAM reflects boot state if the tool resets it.

**The entry token is at `0x3F00`, not the top of RAM.** An address near `RAMEND`
sits where the stack starts and would be clobbered by the bootloader's own
startup pushes before it could be read.

**Useful CRC16 signatures.** CRC16-CCITT over the whole application image, for
classifying a bad verify without reaching for UPDI: an all-`0xFF` section (the
write never landed) and an all-`0x00` section each have their own fixed value,
as do byte-swapped writes and even/odd-byte-only writes. Compute the expected
value for the current image size with the same `crc16_ccitt()` the packaging
tools use (`firmware/tools/fb_image/`).

**`i2c_data.h` and `bootloader_protocol.h` are hand-synced across copies** —
firmware, `esphome/components/fader_buddy/`, the jig, and the WebHID JS
constants. `ci/util/check_i2c_data_sync.py` enforces the C copies on every push;
the WebHID copy is still on you.

## Related documentation

- [ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md) — UPDI flashing, the
  bootstrap and recovery path.
- [ABOUT_ESPHOME_INTEGRATION.md](ABOUT_ESPHOME_INTEGRATION.md) — configuring and
  triggering updates from a host.
- [firmware/src/shared/bootloader_protocol.h](firmware/src/shared/bootloader_protocol.h)
  — the wire protocol and memory map.
- [firmware/src/shared/i2c_data.h](firmware/src/shared/i2c_data.h) — the
  application register map, including `REG_ENTER_BOOTLOADER` and
  `REG_FW_VERSION`.
