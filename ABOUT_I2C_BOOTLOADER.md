# I2C Bootloader Design (ATtiny1616)

> **Status: implemented and hardware-validated.**
> The full I2C update cycle (enter → erase → stream → whole-image CRC verify →
> RUN_APP → app reports `REG_FW_VERSION`) passes end-to-end on real hardware via
> the production jig, with the written flash UPDI-verified against the image.
> Bring-up found two TWI-slave bugs in the bootloader, both fixed (see
> [DEBUGGING_BOOTLOADER.md](DEBUGGING_BOOTLOADER.md)): the slave init was missing
> `PIEN` (so Stop conditions never raised `APIF` and every master-write command
> was silently dropped), and the `twi_service()` error branch could leave
> `CLKHOLD` asserted and wedge the bus. The ESPHome host-side flow (§11) is still
> unimplemented.
> The design below is now realized in code: the bootloader
> (`firmware/src/bootloader/bootloader.c`), the shared protocol
> (`firmware/src/shared/bootloader_protocol.h`), the `bootloader` /
> `fb_app_only` / `fb_app_and_bootloader` PlatformIO environments, the
> application-side entry path and `REG_ENTER_BOOTLOADER`/`REG_FW_VERSION`
> registers (protocol bumped to **v6**), and an I2C firmware-upload test in the
> production jig (`production_tools/programAndTest`, now part of the normal
> `env:lilygo-t-display` test sequence — see BOOTLOADER_NEXT.md). The
> register values, section semantics, and constants have been **validated
> against the *ATtiny1614/16/17 Data Sheet (DS40002204A)***.
>
> **Two deviations from the spec below, both fixing spec bugs:**
> - The warm-reset entry token lives at **`0x3F00`**, not `0x3FFE` (§9). `0x3FFE`
>   is at the top of RAM where the stack starts and would be clobbered by the
>   bootloader's own startup pushes before it could read the token.
> - The bootloader writes `NVMCTRL.CTRLA` via `_PROTECTED_WRITE_SPM` (CCP **SPM**
>   key `0x9D`), not the plain IOREG `_PROTECTED_WRITE` (§5).
>
> **Not yet done:** the ESPHome host-side packaging/update flow (§11) and the
> optional robustness extras (WDT command timeout, CRC-footer app validity, and
> shrinking `BOOTEND` from the initial 0x08 toward the measured ~1.3 KB — §2/§9).

## 1. Goal & constraints

Today a FaderBuddy's ATtiny1616 can only be flashed with a physical UPDI
programmer touching three test pads (see
[ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md)). We want the ESP32 host
to be able to push firmware updates to each fader **over the existing I2C bus**,
with these hard constraints:

- **No pin changes.** The only wires between host and target are I2C (SDA/SCL on
  the TWI0 pins). UPDI, the debug/serial pins, etc. stay as they are.
- **No new wires.**
- **The host cannot control target power** — any scheme that needs a power cycle
  to enter update mode is out.
- **Keep the existing PlatformIO + megaTinyCore toolchain** for building the
  ATtiny firmware.

**Approach:** a bootloader in the ATtiny1616's hardware boot section. The boot
section owns the reset vector (PC = 0x0000), so it always runs first on any reset;
NVMCTRL self-programming lets it flash the application; and hardware write
protection prevents a corrupt application from overwriting the bootloader.

The one-time cost: every board needs a **single UPDI flash** to install (a) the
bootloader, (b) the boot-section fuses, and (c) an application built to run at the
post-boot offset. A combined PlatformIO environment (see [§7](#7-toolchain--build-integration-staying-in-platformio))
bundles all three into one upload. After that, all future updates are I2C-only.
UPDI remains the bring-up path for blank chips and the recovery path of last resort.

(The firmware image is assumed already available to the host by some means — how
the host obtains the image is out of scope.)

## 2. Flash memory map

The ATtiny1616 has **16 KB flash** with a **64-byte page size** (256 pages total),
memory-mapped into the data space at **`FLASHSTART = 0x8000`** (confirmed against
datasheet Table 6-1 and Figure 6-2 / Figure 9-2). Flash is divided into
three consecutive sections defined by two fuses (`FUSE.BOOTEND`, `FUSE.APPEND`):

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

Two address spaces are in play and matter for the flash-write code (§5): the
**program counter / section view** starts at `0x0000`, while **loads/stores that
fill the page buffer use the data-space mapped view at `0x8000 + offset`**.
Everywhere below, "flash address `X`" means program/section offset `X`; the store
target is `0x8000 + X`.

- **`BOOTEND`** fuse: boot-section size, in **256-byte units**. A C bootloader
  doing NVMCTRL + polled TWI + CRC16 is roughly **1–1.5 KB**, so start with
  `BOOTEND = 0x06` (1536 bytes) and shrink after measuring the built size.
  (512 bytes = `BOOTEND = 0x02` is achievable as a lower bound.)
- **`APPEND`** fuse: `0x00` → the application section extends to the end of flash
  (no separate APPDATA region needed).
- The **boot section is at flash address `0x0000`**, so it contains the reset
  vector and therefore **always executes first on any reset**.
- The **application is linked to start at `BOOTEND * 256`** (see §7).

## 3. Boot-section integrity (write protection)

Confirmed against the datasheet (*NVMCTRL — Memory Organization*, §9.3.1.1):

- **The CPU can never write to the BOOT section** — unconditional, inherent
  hardware behavior, independent of any fuse or lock bit. Neither the application
  nor the bootloader itself can rewrite the boot section.
- **Directional inter-section write protection:** BOOT code may write APPCODE and
  APPDATA; APPCODE may write only APPDATA; APPDATA may write neither Flash nor
  EEPROM.
- **`BOOTLOCK` / `APCWP`** are optional lock bits in `NVMCTRL.CTRLB` (effective
  until the next reset), **not** fuses and **not** required for this design. Note
  `BOOTLOCK` additionally blocks *reads and execution* of the boot section — so
  **do not set it** (the boot section must stay executable on every reset). This
  design sets **no** lock bits and relies on the inherent CPU-can't-write-BOOT
  protection.

Consequence: **a buggy or half-written application can never corrupt the
bootloader.** Because the bootloader always runs first on reset, even a totally
broken application still lets the host re-enter the bootloader and re-flash
without UPDI. As defense in depth the bootloader still **bounds-checks every
target page address** so a bad host command is rejected in software before it even
reaches the hardware guard.

## 4. Interrupt vectors (`CPUINT.CTRLA.IVSEL`) — no action needed

`CPUINT.CTRLA.IVSEL` (**bit 6**, CCP-protected; reset = 0) selects where the
interrupt vector table is fetched from (confirmed against datasheet §13.5.1):

- **`IVSEL = 0` (the reset default) → vectors at the start of the *application*
  section.**
- **`IVSEL = 1` → vectors at the start of the *boot* section.**

**No code needs to touch `IVSEL` at all:**

- The **bootloader uses no interrupts** — it polls the TWI slave flags — so it
  doesn't care where the vector base points.
- The **application wants its vector table at the application start**, which is
  exactly what the reset default (`IVSEL = 0`) already gives it. As long as the
  app is linked with its vector table at `BOOTEND * 256` (see §7), its ISRs
  resolve into the application section **with no startup write to `IVSEL`**.
- **Reset always vectors to `0x0000`** (the boot section) regardless of `IVSEL` —
  `IVSEL` only moves the *interrupt* vector base, never the reset vector.

## 5. Self-programming via NVMCTRL

Flash is memory-mapped, and writing is a two-step "fill the page buffer, then
issue a command" flow. Per 64-byte page:

0. **Pre-check** — poll `NVMCTRL.STATUS` until both `FBUSY` (bit 0) and `EEBUSY`
   (bit 1) are clear before issuing any NVM command.
1. **Clear the page buffer** — `NVMCTRL.CTRLA = PBC` (page buffer clear). The
   buffer auto-clears after any reset, write, erase, or sleep-wake, so this is
   strictly optional but cheap insurance.
2. **Fill the buffer** — ordinary stores (`st`) to the **data-space mapped flash
   addresses** (`0x8000 + offset`) of the target page load the temporary page
   buffer. Writing to `0x0000`-based addresses would *not* hit flash.
3. **Erase + write** — `NVMCTRL.CTRLA = ERWP` (erase and write page).
4. **Wait** — the CPU **stalls** during the flash operation (datasheet: "the CPU
   will be halted"), so no read-while-write restriction applies and the next
   instruction after the `_PROTECTED_WRITE` does not execute until the operation
   completes. Check `WRERROR` (STATUS bit 2) afterward.

Every write to `NVMCTRL.CTRLA` must go through the **Configuration Change
Protection** unlock: write the SPM signature to `CPU.CCP` immediately before
(`_PROTECTED_WRITE(NVMCTRL.CTRLA, cmd)` does this).

Command values — **confirmed** against the datasheet *NVMCTRL — CTRLA.CMD* table
(§9.5.1):

| Value | Name   | Meaning                | Notes |
|:-----:|--------|------------------------|-------|
| 0x00  | —      | No command             | |
| 0x01  | `WP`   | Write page             | |
| 0x02  | `ER`   | Erase page             | |
| 0x03  | `ERWP` | Erase and write page   | |
| 0x04  | `PBC`  | Page buffer clear      | |
| 0x05  | `CHER` | Chip erase (Flash + EEPROM) | **Not used by this bootloader** — use per-page `ER` for `ERASE_APP` instead (see §6) |

CCP signatures (datasheet §8.7.1): **`0x9D` = SPM key** (self-programming);
**`0xD8` = IOREG key** (protected I/O registers, e.g. `IVSEL`, `RSTCTRL.SWRR`).

Note: `SET_PAGE_ADDR` (§6) carries a flash **section offset**; the bootloader adds
`0x8000` to derive the store address for the page buffer fill. Because `BOOTEND`
is in 256-byte units and 256 is a multiple of the 64-byte page, every
`BOOTEND * 256` app start is naturally page-aligned — no partial-page edge case at
the section boundary.

## 6. TWI0 polled slave + wire protocol

The bootloader talks I2C directly against the **TWI0** registers in **slave mode,
polled, no ISR, and without the `Wire` library** (keeps it tiny and self-contained
in the boot section). The relevant registers (datasheet *TWI*, §26):
`TWI0.SADDR`, `TWI0.SCTRLA`, `TWI0.SSTATUS`, `TWI0.SCTRLB`, `TWI0.SDATA`.

Slave loop sketch (`SSTATUS`/`SCTRLB` bit names & command encodings confirmed
against datasheet §26):

1. Init: `TWI0.SADDR = address << 1`; enable the slave in `TWI0.SCTRLA`.
2. Poll `TWI0.SSTATUS`:
   - **Address match** (`APIF` set, `AP` = address): inspect `DIR`, then ACK via
     `TWI0.SCTRLB` "response" command.
   - **Data, master-write/slave-receive** (`DIF`, `DIR`=0): read `TWI0.SDATA`,
     then ACK the next byte.
   - **Data, master-read/slave-transmit** (`DIF`, `DIR`=1): write `TWI0.SDATA`,
     then issue "response."
   - **Stop** (`APIF`, not `AP`): complete the transaction.
   - **Bus error / collision**: reset the interface and abort.

**Address:** the bootloader uses the **same address scheme as the application** —
base `0x20` plus the 3-bit hardware address from pins PC2/PC1/PC0
(`firmware/src/main.cpp` `setup_i2c()`, addresses `0x20`–`0x27`). So a fader keeps
its identity in bootloader mode. (Alternatively, a fixed "bootloader address"
could be used to make mode unambiguous to the host — a design choice to settle at
implementation; keeping the app address is simplest.)

**Command set** (all multi-byte fields **big-endian** to match FaderBuddy's existing convention in
[`i2c_data.h`](firmware/src/shared/i2c_data.h)):

| Opcode | Command             | Payload / behavior                                                        |
|:------:|---------------------|---------------------------------------------------------------------------|
| `0x01` | `SET_PAGE_ADDR`     | 2-byte flash page address; resets the frame counter and page buffer       |
| `0x02` | `SEND_FRAME`        | 16 data bytes + CRC16; 4 frames fill a 64-byte page, then it auto-writes  |
| `0x03` | `RUN_APP`           | leave bootloader mode and start the application                           |
| `0x04` | `ERASE_APP`         | erase the entire application section (EEPROM untouched); see note below   |
| `0x06` | `GET_VERSION_CRC16` | args: address + length → returns bootloader version + CRC16 of that range |

**`ERASE_APP` implementation:** Issue `ER` commands page-by-page over the APPCODE
range (`BOOTEND * 256` through `FLASHEND`). Do **not** use `CHER` — chip erase
also erases EEPROM (losing any stored calibration/user data) and its behavior with
respect to the BOOT section when issued from CPU code is not explicitly documented
as safe.

**Recommended update sequence:**

1. `ERASE_APP`
2. For each page: `SET_PAGE_ADDR`, then `SEND_FRAME` ×4 (each CRC16-checked)
3. `GET_VERSION_CRC16` over the whole application region to verify the write
4. `RUN_APP`

Per-frame CRC16 catches bit errors during transfer; the whole-image CRC verify
before `RUN_APP` catches anything missed and prevents jumping into a bad image.

## 7. Toolchain / build integration (staying in PlatformIO)

**Known limitation:** PlatformIO's megaTinyCore integration does **not** natively
support "build for a bootloader" or "upload via a bootloader." So we structure it
ourselves — still entirely within PlatformIO, as **three environments**:

- **`[env:fb_bootloader_only]`** — the bootloader's own minimal source (NVMCTRL + polled
  TWI0 + CRC16, no Arduino framework, or a very thin one), linked into the boot
  section and emitted as a hex. Its measured size sets the final `BOOTEND`.
- **`[env:fb_app_only]`** (the existing app env, now built at the boot offset) —
  link `.text` starting at `BOOTEND * 256`, with the vector table at the
  application start. **No `IVSEL` startup write is needed** — the reset default
  (`IVSEL = 0`) already routes interrupts to the application section (see §4); the
  only requirement is the vector-table placement. The cleanest route is to
  **reuse megaTinyCore's existing bootloader offset machinery** (it already does
  precisely this for Arduino IDE bootloader builds). If that isn't directly
  reachable from PlatformIO, achieve the same with `-Wl,--section-start=.text=<offset>`
  (plus vector placement) and an `extra_scripts` hook — the project already uses
  `extra_scripts` for `firmware/tools/install_pyupdi.py`, so there is precedent.
  **The exact megaTinyCore knobs must be confirmed against the installed core
  version.** This env is *also* the **day-to-day dev-iteration path**: once the
  bootloader is installed it persists, so a developer re-flashes only the app over
  UPDI without touching the bootloader.
- **`[env:fb_app_and_bootloader]`** (new — the combined first-flash) — see below.

### Do you need two uploads the first time? No — provide a combined env.

Logically there are **three artifacts** (fuses, bootloader hex, offset-app hex),
but you should not have to do multiple manual uploads to bring up a blank chip. The
`fb_app_and_bootloader` env packages them into **one action**:

1. An `extra_scripts` **post-build merge** combines the bootloader hex and the
   offset-app hex into a single Intel-hex. The two regions never overlap (bootloader
   at `0x0000`, app at `BOOTEND * 256`), so this is a straightforward merge with
   `srec_cat`, `avr-objcopy`, or the `intelhex` Python package.
2. A single `upload_command` writes the merged hex **and sets the `BOOTEND` /
   `APPEND` fuses** in one UPDI invocation. Recommend **`pymcuprog`** here — it has
   better fuse-writing support than `pyupdi`, and `firmware/tools/install_pymcuprog.py`
   already exists in the tree (currently unused); wire it into this env's
   `extra_scripts` exactly as `install_pyupdi.py` is wired into the app env today.

So the first-time flow becomes: *pick the `fb_app_and_bootloader` env → Upload* — one
step, over UPDI, exactly the [ABOUT_UPDATING_FIRMWARE.md](ABOUT_UPDATING_FIRMWARE.md)
physical setup (UPDI Friend on the three pads). That same combined upload is also the
**recovery path** if a board's application is ever left invalid. After the one-time
factory flash, everything is I2C.

## 8. Application-side changes (specified, not implemented)

To let the host *ask* a running fader to drop into the bootloader:

- **Protocol** — add new registers in the free `0x10+` space of
  [`firmware/src/shared/i2c_data.h`](firmware/src/shared/i2c_data.h):
  `REG_ENTER_BOOTLOADER` (takes a **magic payload** so a stray write can't trigger
  it) and `REG_FW_VERSION` (the application build/semantic version — see
  [§10](#10-version-identity--no-application-detection)). **Bump
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
  and perform the **software reset** (see §9) outside ISR context.

## 9. Bootloader entry mechanism (no power cycle available)

Because the host cannot power-cycle the target, entry is **application-triggered**:

1. Host writes `REG_ENTER_BOOTLOADER` (with magic) to the running app.
2. The app writes an **entry token** to a location that survives a warm reset, then
   issues a **software reset**:
   `_PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm)` (`SWRE` = bit 0, confirmed
   in datasheet §12.5.2; `SWRR` is CCP `IOREG`-protected, hence `_PROTECTED_WRITE`).
3. On reboot the **bootloader runs first** (it owns `0x0000`), reads
   `RSTCTRL.RSTFR` to confirm the reset cause was a **software reset** (`SWRF`, bit
   4, confirmed §12.5.1), and checks the entry token. If both match → stay resident
   as an I2C slave in bootloader mode. Otherwise → clear the token and jump to the
   application. In both cases the bootloader **writes `RSTFR` back to clear the
   flags** (write 1 to clear; flags are sticky until explicitly cleared).

**Where to store the entry token** — the bootloader and application are separate
programs, so they must agree on a fixed location. **Use a `.noinit` RAM variable
pinned to a fixed absolute address** (e.g. `0x3FFE`, just below `RAMEND = 0x3FFF`)
in *both* builds. RAM contents survive a warm reset (software/WDT/BOR) because
SRAM cells retain their state — only a power-on reset leaves them indeterminate.
The C runtime does **not** clear `.noinit` (megaTinyCore excludes it from the
`.bss` clear).

**Do not use GPIOR registers.** The datasheet confirms `GPIORn` has a reset value
of `0x00` and is cleared by every reset including a software reset — it cannot
carry a token across the entry reset. (Confirmed from §6.8.2.)

Combining the reset-cause flag with the token makes accidental entry effectively
impossible. (One `.noinit` caveat: after a true power-on/BOD reset RAM is
indeterminate — but that path has neither `SWRF` set nor a valid token, so it
correctly falls through to the application.)

**Safety fallbacks:**

- A short **post-reset listen window** in the bootloader (wait briefly for I2C
  activity before jumping to the app) as a secondary entry route. If this window
  can be reached after a **power-on reset** (not just the warm SW-reset path),
  remember the NVMCTRL **POR write lockout** (datasheet §9.3.2.5): after POR the
  controller rejects NVM writes until `FBUSY`/`EEBUSY` clear, so poll `FBUSY` (or
  disable the timeout via `SYSCFG0`) before the first flash op.
- A **watchdog command timeout** while resident in bootloader mode, so a
  stalled/abandoned update auto-recovers to the application instead of hanging on
  the shared bus.

## 10. Version identity & "no application" detection

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
version it has packaged (see [§11](#11-host-side-firmware-packaging--update-policy-esphome))
to decide whether an update is needed.

### How "no app" arises, and how the bootloader guarantees the marker shows

A board can end up bootloader-only in two ways: a bootloader flashed without an app,
or a **failed/partial application update** (e.g. power lost mid-write — recall
[§9](#9-bootloader-entry-mechanism-no-power-cycle-available) has no atomic A/B
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
[§8](#8-application-side-changes-specified-not-implemented)): a version below the
bootloader-aware threshold means "UPDI migration required, one time." This is the
only remaining case that still needs a physical programmer.

## 11. Host-side firmware packaging & update policy (ESPHome)

Yes — the ESP32 host can carry the fader firmware and drive updates. This describes
how the ESPHome component (`esphome/components/fader_buddy/`) would gain that, still
as a design (not implemented here).

### What gets packaged

Only the **application** is written over I2C. The bootloader is deliberately **not
field-updatable** (it is write-protected from the app per [§3](#3-boot-section-integrity-write-protection),
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
[§8](#8-application-side-changes-specified-not-implemented): read version → (if
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
  **10,000 write/erase cycles** (confirmed against the datasheet). Updating *only on
  mismatch* means the normal cost is a handful of cycles over a board's life; the
  attempt cap is the real safeguard against a pathological retry loop burning through
  endurance.
- **Don't interrupt the user** — defer an update while the fader is in use
  (`MODE_INPUT_ACTIVE`); wait for idle.
- **One fader at a time** — never run two updates concurrently on the shared bus, so
  a stuck transfer can't wedge neighbors (pairs with the bus-recovery notes in
  [§12](#12-risks--open-questions)).

## 12. Risks & open questions

**Confirmed against the datasheet (DS40002204A)** — these were verified during this
design pass and are settled:

- ✅ Flash: 16 KB, 64-byte pages (256 total), FLASHSTART = 0x8000 (§6 Table 6-1,
  Figure 6-2, Figure 9-2).
- ✅ NVMCTRL `CTRLA.CMD` values (`WP`=0x1, `ER`=0x2, `ERWP`=0x3, `PBC`=0x4,
  `CHER`=0x5) and the `CPU.CCP` SPM signature `0x9D` (§9.5.1, §8.7.1).
- ✅ Flash sections in 256-byte blocks via `FUSE.BOOTEND`/`FUSE.APPEND`; flash is
  mapped into data space at `0x8000` (§9.3.1.1).
- ✅ Boot-section protection: **the CPU can never write BOOT** (inherent, no
  `BOOTPROT` fuse); directional inter-section protection; `BOOTLOCK`/`APCWP` are
  optional `CTRLB` lock bits we deliberately leave unset (§9.3.1.1, §9.5.2).
- ✅ CPU halts during Flash write/erase (no NRWW restriction); check `WRERROR`
  afterward (§9.3.2.4.1, §9.5.3).
- ✅ `CPUINT.CTRLA.IVSEL` is bit 6, CCP-protected; `IVSEL=0` (reset default) →
  app-section vectors, so **no `IVSEL` handling is required** (§13.5.1).
- ✅ `RSTCTRL.RSTFR` `SWRF` = bit 4; `RSTCTRL.SWRR` `SWRE` = bit 0 (§12.5.1,
  §12.5.2).
- ✅ **GPIOR registers are cleared by a software reset** (reset value `0x00`), so
  the token uses `.noinit` RAM instead (§6.8.2).
- ✅ TWI0 slave registers/bits (`SADDR`, `SCTRLA`, `SSTATUS`/`APIF`/`AP`/`DIF`/`DIR`,
  `SCTRLB`/`ACKACT`/`SCMD`, `SDATA`) (§26).
- ✅ Flash write/erase endurance **10,000 cycles**, backing the flash-wear argument
  (§1).

**Still to pin down at implementation time** (toolchain, not silicon behavior):

- Exact megaTinyCore/PlatformIO knobs to build the app at the `BOOTEND * 256`
  offset with correct vector placement (§7) — the least-certain integration point.
- Final `BOOTEND` value, set from the *measured* built bootloader size (§2).
- `FUSE.BOOTEND`/`FUSE.APPEND` addresses for the `pymcuprog` fuse-write step (§7).

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
  certain integration point; validate it early (see [§13](#13-verification--test-plan-for-the-implementation-pass))
  since everything else depends on the app running correctly from the offset.
- **Deployment / migration.** Existing and newly-fabricated boards need the
  one-time UPDI install of bootloader + fuses + offset app before I2C updates
  become available. Boards already in the field would need one more UPDI visit to
  gain the capability.

## 13. Verification & test plan (for the implementation pass)

There is no way to hardware-test this in CI, so validation happens on real
hardware in stages:

1. **Build the offset application first** and flash it (plus fuses) via UPDI with
   *no* bootloader. Confirm it runs correctly from `BOOTEND * 256` with working
   interrupts (touch, motor, I2C) — this de-risks the toolchain (§7) before any
   bootloader exists.
2. **Build and UPDI-flash the bootloader**; confirm the app still starts (normal
   reset → bootloader → jump to app).
3. **Combined factory env** (§7): confirm `[env:fb_app_and_bootloader]` merges
   bootloader + app and flashes both **plus fuses** in a single UPDI upload onto a
   blank chip, yielding a running app.
4. **Identity/version reads** (§10): confirm `REG_VERSION` returns the protocol
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
8. **Host packaging & policy** (§11): with the ESPHome component, verify the
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
