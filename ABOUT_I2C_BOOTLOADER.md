# I2C Bootloader Design (ATtiny1616)

> **Status: design / feasibility proposal — not yet implemented.**
> This document describes *how* firmware-over-I2C could work on FaderBuddy. No
> bootloader code, `platformio.ini`, or `i2c_data.h` changes exist yet; this is
> the specification for a future implementation pass. Register bit values and
> datasheet section numbers below should be re-confirmed against the
> *ATtiny1614/16/17 Data Sheet (DS40002204A)* before writing code — the specific
> spots to check are called out inline and collected in
> [§11 Open questions](#11-risks--open-questions).

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
and aware of the "enter bootloader" command. After that one bootstrap, all future
updates are I2C-only. UPDI remains the bring-up path for blank chips and the
recovery path of last resort.

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
ourselves — still entirely within PlatformIO:

- **Bootloader build** — a **second PlatformIO env** with its own minimal source
  (NVMCTRL + polled TWI0 + CRC16, no Arduino framework, or a very thin one). It is
  linked into the boot section and produced as a hex that is flashed **once via
  UPDI** together with the fuses. Bootloader size determines the final `BOOTEND`.
- **Application build** — the existing `[env:fader_buddy]`, but built to run at the
  boot offset: link `.text` starting at `BOOTEND * 256`, place the vector table at
  the application start, and set `CPUINT.CTRLA.IVSEL` in startup. The cleanest
  route is to **reuse megaTinyCore's existing Optiboot offset machinery** (it
  already does precisely this for Arduino IDE "Upload Using Programmer" +
  bootloader builds). If that isn't directly reachable from PlatformIO, achieve
  the same with `-Wl,--section-start=.text=<offset>` (plus vector placement) and an
  `extra_scripts` hook — the project already uses `extra_scripts` for
  `firmware/tools/install_pyupdi.py`, so there is a precedent. **The exact
  megaTinyCore knobs must be confirmed against the installed core version.**
- **Fuses** — set `BOOTEND` / `APPEND` at UPDI install time via `pyupdi` (or
  `pymcuprog`); the fuse-setting step joins the existing UPDI upload flow.
- **One-time bootstrap** — the first flash (bootloader + fuses + offset app) is
  over UPDI, i.e. the current [ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md)
  procedure with two extra artifacts. Document it as unchanged for board bring-up
  and as the recovery path if an I2C update is ever interrupted.

## 9. Application-side changes (specified, not implemented)

To let the host *ask* a running fader to drop into the bootloader:

- **Protocol** — add a new register in the free `0x10+` space of
  [`firmware/src/shared/i2c_data.h`](firmware/src/shared/i2c_data.h), e.g.
  `REG_ENTER_BOOTLOADER`, that takes a **magic payload** (so a stray write can't
  trigger it), and **bump `I2C_PROTOCOL_VERSION`**.
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

## 11. Risks & open questions

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
  certain integration point; validate it early (see §12) since everything else
  depends on the app running correctly from the offset.
- **Deployment / migration.** Existing and newly-fabricated boards need the
  one-time UPDI install of bootloader + fuses + offset app before I2C updates
  become available. Boards already in the field would need one more UPDI visit to
  gain the capability.

## 12. Verification & test plan (for the implementation pass)

There is no way to hardware-test this in CI, so validation happens on real
hardware in stages:

1. **Build the offset application first** and flash it (plus fuses) via UPDI with
   *no* bootloader. Confirm it runs correctly from `BOOTEND * 256` with working
   interrupts (touch, motor, I2C) — this de-risks the toolchain (§8) before any
   bootloader exists.
2. **Build and UPDI-flash the bootloader**; confirm the app still starts (normal
   reset → bootloader → jump to app).
3. **Exercise the entry path**: host writes `REG_ENTER_BOOTLOADER`; confirm the
   device comes back as an I2C slave in bootloader mode (e.g. `GET_VERSION_CRC16`
   responds).
4. **Full update cycle**: `ERASE_APP` → stream pages with per-frame CRC16 →
   `GET_VERSION_CRC16` whole-image verify → `RUN_APP`, and confirm the new app
   runs.
5. **Interrupted-update recovery**: abort mid-stream, confirm the WDT timeout /
   re-entry recovers and a subsequent full update succeeds.
6. **Multi-fader**: confirm a chain of faders can each be addressed and updated
   without disturbing the others.

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
