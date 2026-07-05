# I2C Bootloader Design (ATtiny1616)

> **Status: design / feasibility proposal — not yet implemented.**
> This document describes *how* firmware-over-I2C could work on FaderBuddy. No
> bootloader code, `platformio.ini`, or `i2c_data.h` changes exist yet; this is
> the specification for a future implementation pass. Register bit values and
> datasheet section numbers below should be re-confirmed against the
> *ATtiny1614/16/17 Data Sheet (DS40002204A)* before writing code — the specific
> spots to check are called out inline and collected in
> [§13 Open questions](#13-risks--open-questions).

## 1. Goal & constraints

Today a FaderBuddy's ATtiny1616 can only be flashed with a physical UPDI
programmer touching three test pads (see
[ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md)). We want the ESP32 host
to be able to push firmware updates to each fader **over the existing I2C bus**,
with these hard constraints:

- **No pin changes.** The only wires between host and target are I2C (SDA/SCL on
  the TWI0 pins). UPDI, the debug/serial pins, etc. stay as they are.
- **No new wires.**
- **The host cannot control target power** — so any scheme that needs a power
  cycle to enter update mode is out.
- **Keep the existing PlatformIO + megaTinyCore toolchain** for building the
  ATtiny firmware.

(The firmware image is assumed already available to the host by some means — how
the host obtains the image is out of scope.)

### Feasibility verdict

**Feasible, and a bootloader is the only viable approach.** UPDI is a separate
pin that is not wired to the host, so new firmware *must* travel over I2C. And to
rewrite the running application safely (and recover from a bad flash), the code
that performs the flash write must live in a **protected region that the
application cannot corrupt**. That is exactly what a bootloader in the
ATtiny1616's hardware boot section provides.

Encouragingly, megaTinyCore already ships **Optiboot_x** for this exact chip
(a UART bootloader). That proves the three hardware mechanisms this design relies
on — a boot section at flash start, `CPUINT.CTRLA.IVSEL` vector selection, and
NVMCTRL self-programming — all work within our toolchain. We are essentially
building a small custom equivalent whose communication layer is a **TWI (I2C)
slave** instead of a UART.

### The one-time cost

Every board needs a **single UPDI flash** to install (a) the bootloader, (b) the
boot-section fuses, and (c) an application built to run at the post-boot offset
and aware of the "enter bootloader" command. A dedicated **combined PlatformIO
environment** (see [§8](#8-toolchain--build-integration-staying-in-platformio))
bundles all three into one upload so this bootstrap is a single action. After that,
all future updates are I2C-only. UPDI remains the bring-up path for blank chips and
the recovery path of last resort.

## 2. Two reference inputs (what ported, what didn't)

**The attached ATtiny88 I2C bootloader** (`attiny_i2c_bootloader`) is a solid
conceptual model but a *classic* AVR design. What we reuse:

- Its **wire protocol shape**: page-address → four 16-byte frames (each with a
  CRC16) → auto page-write; plus erase, a "get version + CRC16 of a region" verify
  command, and a "run application" command.
- Its **safety patterns**: bounds-check every write so the boot region can never
  be targeted (its `update_page()` refuses `pageAddress >= BOOT_PAGE_ADDRESS`),
  and a watchdog **command timeout** so a stalled transfer auto-recovers.
- Its **polled TWI slave** structure (no interrupts in the bootloader).

What does **not** port to the ATtiny1616 (and why the modern part is cleaner):

- Its self-programming uses classic `SPM` / `boot_page_fill/erase/write`. The
  ATtiny1616 uses the **NVMCTRL** peripheral instead (see §6).
- It places the bootloader at the *top* of flash and **hand-edits the hex file to
  overwrite the reset vector** with `rjmp`s into the bootloader, then carefully
  preserves the reset vector on page-0 writes. The ATtiny1616 has a **real
  hardware boot section at the *start* of flash** with write-protection, so none
  of that hack is needed (see §3–§4).

**The ATtiny1614/16/17 datasheet** confirms the modern boot architecture we build
on: a fuse-defined boot section, section-based NVM write protection, interrupt
vector base selection, and command-driven self-programming.

## 3. Flash memory map

The ATtiny1616 has **16 KB flash** with a **64-byte page size**, memory-mapped
into the data space. Modern tinyAVR divides flash into three consecutive sections
defined by two fuses (datasheet *NVMCTRL* / *FUSE* chapters — confirm fuse
addresses):

```
0x0000  ┌────────────────────────┐  ← reset + interrupt vectors live here
        │  BOOT section          │     (always runs first on reset)
        │  = the I2C bootloader  │
BOOTEND*256 ├────────────────────┤  ← application starts here
        │  APPCODE section       │
        │  = the FaderBuddy app  │
        │        ...             │
0x4000  └────────────────────────┘  (16 KB)
```

- **`BOOTEND`** fuse: boot-section size, in **256-byte units**. A C bootloader
  doing NVMCTRL + polled TWI + CRC16 is roughly **1–1.5 KB**, so start with
  `BOOTEND = 0x06` (1536 bytes) and shrink after measuring the built size.
  (Optiboot_x fits a UART bootloader in 512 bytes = `BOOTEND = 0x02`, a useful
  lower bound to aim toward.)
- **`APPEND`** fuse: `0x00` → the application section extends to the end of flash
  (no separate APPDATA region needed).
- The **boot section is at flash address `0x0000`**, so it contains the reset
  vector and therefore **always executes first on any reset**. That is the entire
  trick that gives the bootloader control — no vector hijacking required.
- The **application is linked to start at `BOOTEND * 256`** (see §8).

## 4. Boot-section integrity (write protection)

NVMCTRL enforces **section-based write permission**: code executing in the
APPCODE section is not allowed to issue NVM write/erase commands that target the
BOOT section (datasheet *NVMCTRL — Memory Access* / boot-lock description; confirm
exact behavior and any `BOOTPROT`-style bit).

Consequence: **a buggy or half-written application can never corrupt the
bootloader.** The bootloader is recoverable by construction. Because it always
runs first on reset, even a totally broken application still lets the host
re-enter the bootloader and re-flash. The bootloader additionally never
erases/writes its own section and **bounds-checks every target page address**
(mirroring the reference's `BOOT_PAGE_ADDRESS` guard) as defense in depth.

## 5. Interrupt vectors (`CPUINT.CTRLA.IVSEL`)

On classic AVR a bootloader-relocated application needs the `IVSEL` fuse/bit and a
dedicated boot vector area. Modern tinyAVR handles this in the **CPUINT**
peripheral: `CPUINT.CTRLA.IVSEL` selects whether the interrupt vector table is
fetched from the **start of the boot section** or the **start of the application
section** (CCP-protected write; confirm bit position in datasheet *CPUINT*).

- The **bootloader uses no interrupts** — it polls the TWI slave flags. Smallest
  and simplest; nothing to relocate on its side.
- The **application is built with its vector table at the application start** and
  sets `IVSEL` accordingly at startup, so its ISRs (`Wire`/TWI0, `millis()`
  timer, ADC, PTC touch) resolve into the application section. megaTinyCore's
  Optiboot build already does exactly this — see §8.
- **Reset always vectors to `0x0000`** (the boot section) regardless of `IVSEL`,
  which is what guarantees "bootloader first."

## 6. Self-programming via NVMCTRL

Flash is memory-mapped, and writing is a two-step "fill the page buffer, then
issue a command" flow. Per 64-byte page:

1. **Clear the page buffer** — `NVMCTRL.CTRLA = PBC` (page buffer clear).
2. **Fill the buffer** — ordinary stores (`st`) to the mapped flash addresses of
   the target page load the temporary page buffer.
3. **Erase + write** — `NVMCTRL.CTRLA = ERWP` (erase and write page).
4. **Wait** — poll `NVMCTRL.STATUS` until `FBUSY` clears; check the write-error
   flag.

Every write to `NVMCTRL.CTRLA` must go through the **Configuration Change
Protection** unlock: write the SPM signature to `CPU.CCP` immediately before
(`_PROTECTED_WRITE(NVMCTRL.CTRLA, cmd)` does this). The CPU **stalls** during the
flash operation, so — unlike classic AVR — there is **no read-while-write / NRWW
restriction**; the write routine can live anywhere in the boot section.

Command enum to confirm against the datasheet *NVMCTRL — CTRLA.CMD* table (values
below are the expected modern-tinyAVR encoding):

| Command | Meaning                | Expected value |
|---------|------------------------|:--------------:|
| `NOCMD` | no command             | 0x00 |
| `WP`    | write page             | 0x01 |
| `ER`    | erase page             | 0x02 |
| `ERWP`  | erase and write page   | 0x03 |
| `PBC`   | page buffer clear      | 0x04 |
| `CHER`  | chip (flash) erase     | 0x05 |

CCP signature for self-programming (`CPU.CCP`): expected `0x9D` (SPM key) —
confirm in datasheet *CPU — CCP*.

## 7. TWI0 polled slave + wire protocol

The bootloader talks I2C directly against the **TWI0** registers in **slave mode,
polled, no ISR, and without the `Wire` library** (keeps it tiny and self-contained
in the boot section). The relevant registers (datasheet *TWI*):
`TWI0.SADDR`, `TWI0.SCTRLA`, `TWI0.SSTATUS`, `TWI0.SCTRLB`, `TWI0.SDATA`.

Slave loop sketch (confirm the `SSTATUS`/`SCTRLB` bit names & command encodings):

1. Init: `TWI0.SADDR = address << 1`; enable the slave in `TWI0.SCTRLA`.
2. Poll `TWI0.SSTATUS`:
   - **Address match** (`APIF` set, `AP` = address): inspect `DIR`, then ACK via
     `TWI0.SCTRLB` "response" command.
   - **Data, master-write/slave-receive** (`DIF`, `DIR`=0): read `TWI0.SDATA`,
     then ACK the next byte.
   - **Data, master-read/slave-transmit** (`DIF`, `DIR`=1): write `TWI0.SDATA`,
     then issue "response."
   - **Stop** (`APIF`, not `AP`): complete the transaction.
   - **Bus error / collision**: reset the interface and abort (mirrors the
     reference's `abort_twi()`).

**Address:** the bootloader uses the **same address scheme as the application** —
base `0x20` plus the 3-bit hardware address from pins PC2/PC1/PC0
(`firmware/src/main.cpp` `setup_i2c()`, addresses `0x20`–`0x27`). So a fader keeps
its identity in bootloader mode. (Alternatively, a fixed "bootloader address"
could be used to make mode unambiguous to the host — a design choice to settle at
implementation; keeping the app address is simplest.)

**Command set** (adapted from the reference; all multi-byte fields **big-endian**
to match FaderBuddy's existing convention in
[`i2c_data.h`](firmware/src/shared/i2c_data.h) — note this is the opposite of the
reference, which is little-endian):

| Opcode | Command             | Payload / behavior                                                        |
|:------:|---------------------|---------------------------------------------------------------------------|
| `0x01` | `SET_PAGE_ADDR`     | 2-byte flash page address; resets the frame counter and page buffer       |
| `0x02` | `SEND_FRAME`        | 16 data bytes + CRC16; 4 frames fill a 64-byte page, then it auto-writes  |
| `0x03` | `RUN_APP`           | leave bootloader mode and start the application                           |
| `0x04` | `ERASE_APP`         | erase the entire application section (boot section untouched)             |
| `0x06` | `GET_VERSION_CRC16` | args: address + length → returns bootloader version + CRC16 of that range |

**Recommended update sequence** (same as the reference's proven flow):

1. `ERASE_APP`
2. For each page: `SET_PAGE_ADDR`, then `SEND_FRAME` ×4 (each CRC16-checked)
3. `GET_VERSION_CRC16` over the whole application region to verify the write
4. `RUN_APP`

Per-frame CRC16 catches bit errors during transfer; the whole-image CRC verify
before `RUN_APP` catches anything missed and prevents jumping into a bad image.

## 8. Toolchain / build integration (staying in PlatformIO)

**Known limitation:** PlatformIO's megaTinyCore integration does **not** natively
support "build for a bootloader" or "upload via a bootloader." So we structure it
ourselves — still entirely within PlatformIO, as **three environments**:

- **`[env:bootloader]`** — the bootloader's own minimal source (NVMCTRL + polled
  TWI0 + CRC16, no Arduino framework, or a very thin one), linked into the boot
  section and emitted as a hex. Its measured size sets the final `BOOTEND`.
- **`[env:fader_buddy]`** (the existing app env, now built at the boot offset) —
  link `.text` starting at `BOOTEND * 256`, place the vector table at the
  application start, and set `CPUINT.CTRLA.IVSEL` in startup. The cleanest route is
  to **reuse megaTinyCore's existing Optiboot offset machinery** (it already does
  precisely this for Arduino IDE bootloader builds). If that isn't directly
  reachable from PlatformIO, achieve the same with `-Wl,--section-start=.text=<offset>`
  (plus vector placement) and an `extra_scripts` hook — the project already uses
  `extra_scripts` for `firmware/tools/install_pyupdi.py`, so there is precedent.
  **The exact megaTinyCore knobs must be confirmed against the installed core
  version.** This env is *also* the **day-to-day dev-iteration path**: once the
  bootloader is installed it persists, so a developer re-flashes only the app over
  UPDI without touching the bootloader.
- **`[env:fader_buddy_factory]`** (new — the combined first-flash) — see below.

### Do you need two uploads the first time? No — provide a combined env.

Logically there are **three artifacts** (fuses, bootloader hex, offset-app hex),
but you should not have to do multiple manual uploads to bring up a blank chip. The
`fader_buddy_factory` env packages them into **one action**:

1. An `extra_scripts` **post-build merge** combines the bootloader hex and the
   offset-app hex into a single Intel-hex. The two regions never overlap (bootloader
   at `0x0000`, app at `BOOTEND * 256`), so this is a straightforward merge with
   `srec_cat`, `avr-objcopy`, or the `intelhex` Python package.
2. A single `upload_command` writes the merged hex **and sets the `BOOTEND` /
   `APPEND` fuses** in one UPDI invocation. Recommend **`pymcuprog`** here — it has
   better fuse-writing support than `pyupdi`, and `firmware/tools/install_pymcuprog.py`
   already exists in the tree (currently unused); wire it into this env's
   `extra_scripts` exactly as `install_pyupdi.py` is wired into the app env today.

So the first-time flow becomes: *pick the `fader_buddy_factory` env → Upload* — one
step, over UPDI, exactly the [ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md)
physical setup (UPDI Friend on the three pads). That same combined upload is also the
**recovery path** if a board's application is ever left invalid. After the one-time
factory flash, everything is I2C.

## 9. Application-side changes (specified, not implemented)

To let the host *ask* a running fader to drop into the bootloader:

- **Protocol** — add new registers in the free `0x10+` space of
  [`firmware/src/shared/i2c_data.h`](firmware/src/shared/i2c_data.h):
  `REG_ENTER_BOOTLOADER` (takes a **magic payload** so a stray write can't trigger
  it) and `REG_FW_VERSION` (the application build/semantic version — see
  [§11](#11-version-identity--no-application-detection)). **Bump
  `I2C_PROTOCOL_VERSION`**, which also serves as the host's "this firmware supports
  bootloader entry" capability signal.
  > ⚠️ Per [CLAUDE.md](CLAUDE.md), `i2c_data.h` is **hand-synced across four
  > copies**: the firmware source, `esphome/components/fader_buddy/i2c_data.h`,
  > the production-jig header, and the WebHID JS constants. All four must be
  > updated together. (The firmware and esphome copies have already drifted
  > slightly — `1UL` vs `1U` in the `STATE_*_bm` macros — so re-sync carefully.)
- **Firmware** — handle `REG_ENTER_BOOTLOADER` in `onI2cReceive()` by setting a
  `volatile` flag only (it runs in ISR context — the existing code is careful
  about this). In the main loop, when the flag is set, write the **entry token**
  and perform the **software reset** (see §10) outside ISR context.

## 10. Bootloader entry mechanism (no power cycle available)

Because the host cannot power-cycle the target, entry is **application-triggered**:

1. Host writes `REG_ENTER_BOOTLOADER` (with magic) to the running app.
2. The app writes an **entry token** to a location that survives a warm reset, then
   issues a **software reset**:
   `_PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm)` (confirm register/bit in
   datasheet *RSTCTRL*).
3. On reboot the **bootloader runs first** (it owns `0x0000`), reads
   `RSTCTRL.RSTFR` to confirm the reset cause was a **software reset**, and checks
   the entry token. If both match → stay resident as an I2C slave in bootloader
   mode. Otherwise → clear the token and jump to the application.

**Where to store the entry token** — the bootloader and application are separate
programs, so they must agree on a fixed location:

- **Preferred:** a **`.noinit` RAM variable pinned to a fixed absolute address**
  (e.g. just below `RAMEND`) in *both* builds. RAM contents survive a warm reset
  (software/WDT/BOR); the C runtime does **not** clear `.noinit`. This is the
  standard, portable AVR technique.
- **Alternative:** a **GPR / GPIOR register** (fixed peripheral address, trivially
  shared between the two programs) — *if* the datasheet confirms GPRs survive a
  software reset (they are cleared on power-on reset). **Verify this before
  relying on it.**

Combining the reset-cause flag with the token makes accidental entry effectively
impossible.

**Safety fallbacks:**

- A short **post-reset listen window** in the bootloader (wait briefly for I2C
  activity before jumping to the app) as a secondary entry route.
- A **watchdog command timeout** while resident in bootloader mode (mirrors the
  reference's `wdt_enable(WDTO_8S)`), so a stalled/abandoned update auto-recovers
  to the application instead of hanging on the shared bus.

## 11. Version identity & "no application" detection

The host needs to reliably tell three states apart on the bus: **app running**,
**bootloader resident (no usable app)**, and **device absent/wedged** — and it needs
a version to compare against for update decisions. Two complementary pieces:

### `REG_VERSION` (0x00) as a mode/identity read

Reading register `0x00` becomes the universal "who are you" probe:

| Response at `0x00`         | Meaning                                        |
|----------------------------|------------------------------------------------|
| A valid protocol version (≥5) | Application is running normally              |
| A reserved **bootloader marker** (e.g. `0xB0`) | Bootloader is resident — no usable app |
| NAK / no response          | Device absent, unpowered, or bus wedged        |

The marker is chosen to be outside any valid protocol version and `≠ 0xFF`. This
reuses the read the ESPHome component already does in `setup()`
(`esphome/components/fader_buddy/fader_buddy.cpp`) — today it `mark_failed()`s on a
version mismatch; extend it to recognize the marker and treat the device as
**"needs firmware"** rather than simply broken.

### `REG_FW_VERSION` — the application version to compare

`REG_VERSION` is the *protocol/compatibility* version; it is not granular enough to
drive updates. Add a separate app-only **`REG_FW_VERSION`** (build number or
semantic version, in the free `0x10+` space) that the host compares against the
version it has packaged (see [§12](#12-host-side-firmware-packaging--update-policy-esphome))
to decide whether an update is needed.

### How "no app" arises, and how the bootloader guarantees the marker shows

A board can end up bootloader-only in two ways: a bootloader flashed without an app,
or a **failed/partial application update** (e.g. power lost mid-write — recall
[§10](#10-bootloader-entry-mechanism-no-power-cycle-available) has no atomic A/B
image). The bootloader must never jump into a blank or half-written app, so on every
boot it runs an **application-validity self-check**:

- **Minimum:** the application's reset vector at `BOOTEND * 256` is not blank
  (`!= 0xFFFF`; erased flash reads `0xFF`).
- **Robust:** a **CRC footer** embedded in the app image at a fixed
  linker-placed location (end of APPCODE) storing the image length + CRC16; the
  bootloader recomputes the CRC over the app region and compares. This catches
  partial writes, not just a blank chip, and reuses the same CRC16 the
  `GET_VERSION_CRC16` command uses for post-write verification.

If the check fails, the bootloader **stays resident** and keeps answering `0x00`
with the marker — which is exactly the state the host detects. No valid app is ever
executed, and the device advertises that it needs one.

### Pre-bootloader firmware (migration)

Boards still running the current, pre-bootloader firmware answer `0x00` with a
normal protocol version but have **no bootloader** behind them. The host must not
send them `REG_ENTER_BOOTLOADER` expecting an I2C update. It distinguishes them by
the bumped `I2C_PROTOCOL_VERSION` (the capability signal from
[§9](#9-application-side-changes-specified-not-implemented)): a version below the
bootloader-aware threshold means "UPDI migration required, one time." This is the
only remaining case that still needs a physical programmer.

## 12. Host-side firmware packaging & update policy (ESPHome)

Yes — the ESP32 host can carry the fader firmware and drive updates. This describes
how the ESPHome component (`esphome/components/fader_buddy/`) would gain that, still
as a design (not implemented here).

### What gets packaged

Only the **application** is written over I2C. The bootloader is deliberately **not
field-updatable** (it is write-protected from the app per [§4](#4-boot-section-integrity-write-protection),
and it cannot safely rewrite itself while running) — so bootloader changes always
require UPDI, and keeping it small and stable is a feature, not a limitation. The
packaged blob is therefore the **offset application image** (the exact APPCODE
bytes), plus its `REG_FW_VERSION`.

### Embedding the blob (codegen)

The component's `__init__.py` `to_code()` emits the image as a
`const uint8_t[] PROGMEM` array plus a version constant. Because the component is
`MULTI_CONF = True` (many faders, all running the same firmware), emit the blob
**once** — guard it with a module-level "already emitted" flag in `to_code` and
share the symbol across instances rather than duplicating a ~16 KB array per fader.
ESP32 flash has ample room for one copy.

### Config surface

```yaml
fader_buddy:
  - id: fader0
    address: 0x20
    firmware_image: firmware/faderbuddy-app-v6.bin   # bundled image
    autoupdate_firmware: false        # default; opt-in only (see below)
    max_update_attempts: 3            # per-address safety cap
```

### The update flow

Registered as a **manual action** `fader_buddy.update_firmware` (added alongside the
existing actions in `__init__.py` / `fader_buddy.h`), driving the state machine from
[§9](#9-application-side-changes-specified-not-implemented): read version → (if
different) `REG_ENTER_BOOTLOADER` → wait for bootloader marker at `0x00` →
`ERASE_APP` → stream pages (`SET_PAGE_ADDR` + 4× `SEND_FRAME` with CRC16) →
`GET_VERSION_CRC16` whole-image verify → `RUN_APP` → re-read `REG_FW_VERSION` to
confirm. Reuse the existing `write_with_retry_()` helper for the I2C transfers.

### Auto vs. manual — recommendation

**Default to manual** (an explicit `update_firmware` action / button / HA service),
because every update briefly takes a fader offline and writes its flash, and a
human-triggered update is the safest posture. Offer `autoupdate_firmware: true` as
an **opt-in** for users who want faders to converge to the packaged version on their
own. This matches the intuition that automatic flashing deserves extra guard rails.

### Guard rails (both modes, essential for autoupdate)

- **Update only on version mismatch** — never reflash a fader already at the packaged
  `REG_FW_VERSION` (or already showing the bootloader marker / no-app).
- **Per-address attempt tracking in persistent storage** — use ESPHome
  `ESPPreferences` (`global_preferences->make_preference<...>()`), keyed by I2C
  address **and** target version. Increment on failure; once
  `max_update_attempts` consecutive failures is hit for a given target version,
  **stop** and surface the condition (e.g. a `binary_sensor` "update_failed" or a
  `text_sensor` status). Reset the counter on success or when the packaged version
  changes. This is what stops a bad image from endlessly re-flashing.
- **Flash-wear reasoning** — the ATtiny1616's flash endurance is on the order of
  **~10,000 write/erase cycles** (confirm against the datasheet). Updating *only on
  mismatch* means the normal cost is a handful of cycles over a board's life; the
  attempt cap is the real safeguard against a pathological retry loop burning through
  endurance.
- **Don't interrupt the user** — defer an update while the fader is in use
  (`MODE_INPUT_ACTIVE`); wait for idle.
- **One fader at a time** — never run two updates concurrently on the shared bus, so
  a stuck transfer can't wedge neighbors (pairs with the bus-recovery notes in
  [§13](#13-risks--open-questions)).

## 13. Risks & open questions

**Confirm against the datasheet (DS40002204A) before coding** — every value flagged
inline, collected here:

- NVMCTRL `CTRLA.CMD` command values and the `CPU.CCP` SPM signature (§6).
- `BOOTEND` / `APPEND` fuse addresses, units (256-byte blocks), and boot-section
  write-protection semantics (§3–§4).
- `CPUINT.CTRLA.IVSEL` bit position and exact semantics (§5).
- `RSTCTRL.RSTFR` flag bits and `RSTCTRL.SWRR` software-reset trigger (§10).
- Whether **GPR/GPIOR registers survive a software reset** (§10 — affects token
  storage choice).
- TWI0 slave `SSTATUS` / `SCTRLB` bit names and command encodings (§7).
- Flash **write/erase endurance** (cited as ~10k cycles) — confirms the flash-wear
  argument for the host update policy (§12).

**Design/operational risks:**

- **Shared-bus robustness.** A fader stuck mid-update must not wedge the bus for
  its neighbors. Mitigations: WDT command timeout, bus-error recovery/abort, and
  **updating one fader at a time**.
- **No atomic A/B image.** 16 KB flash isn't enough to hold two full images, so an
  update is not transactional — a power loss mid-write leaves a partial
  application. This is acceptable because the **boot section survives** (protected,
  and always runs first), so the host can simply re-enter the bootloader and
  re-flash without UPDI. Document this failure mode for operators.
- **Toolchain uncertainty.** The offset/`IVSEL` application build is the least
  certain integration point; validate it early (see [§14](#14-verification--test-plan-for-the-implementation-pass))
  since everything else depends on the app running correctly from the offset.
- **Deployment / migration.** Existing and newly-fabricated boards need the
  one-time UPDI install of bootloader + fuses + offset app before I2C updates
  become available. Boards already in the field would need one more UPDI visit to
  gain the capability.

## 14. Verification & test plan (for the implementation pass)

There is no way to hardware-test this in CI, so validation happens on real
hardware in stages:

1. **Build the offset application first** and flash it (plus fuses) via UPDI with
   *no* bootloader. Confirm it runs correctly from `BOOTEND * 256` with working
   interrupts (touch, motor, I2C) — this de-risks the toolchain (§8) before any
   bootloader exists.
2. **Build and UPDI-flash the bootloader**; confirm the app still starts (normal
   reset → bootloader → jump to app).
3. **Combined factory env** (§8): confirm `[env:fader_buddy_factory]` merges
   bootloader + app and flashes both **plus fuses** in a single UPDI upload onto a
   blank chip, yielding a running app.
4. **Identity/version reads** (§11): confirm `REG_VERSION` returns the protocol
   version when the app runs and the **bootloader marker** when only the bootloader
   is resident (blank/invalid app), and that `REG_FW_VERSION` reports the expected
   build; confirm the app-validity self-check keeps a blank/partial app from running.
5. **Exercise the entry path**: host writes `REG_ENTER_BOOTLOADER`; confirm the
   device comes back as an I2C slave in bootloader mode (marker at `0x00`,
   `GET_VERSION_CRC16` responds).
6. **Full update cycle**: `ERASE_APP` → stream pages with per-frame CRC16 →
   `GET_VERSION_CRC16` whole-image verify → `RUN_APP`, and confirm the new app runs
   and reports the new `REG_FW_VERSION`.
7. **Interrupted-update recovery**: abort mid-stream, confirm the app-validity check
   + WDT timeout / re-entry recovers and a subsequent full update succeeds.
8. **Host packaging & policy** (§12): with the ESPHome component, verify the
   embedded blob updates a mismatched fader via the manual `update_firmware` action;
   verify it does **not** reflash a fader already at the packaged version; and verify
   the per-address attempt cap stops retries after `max_update_attempts` and surfaces
   the failure status.
9. **Multi-fader**: confirm a chain of faders can each be addressed and updated
   one-at-a-time without disturbing the others.

Any of the existing I2C controllers can drive these tests: the
[WebHID / MCP2221 tool](software/mcp2221-webhid/), the ESP32 production jig
(`production_tools/programAndTest/`), or the ESPHome host itself once its flashing
state machine exists (`esphome/components/fader_buddy/fader_buddy.cpp`, reusing its
`write_with_retry_()` pattern).

## Related documentation

- [ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md) — current UPDI flashing
  procedure (the bootstrap and recovery path).
- [ABOUT_LAYERS.md](ABOUT_LAYERS.md) — layer architecture and the I2C protocol.
- [firmware/src/shared/i2c_data.h](firmware/src/shared/i2c_data.h) — the I2C
  register map this design extends.
