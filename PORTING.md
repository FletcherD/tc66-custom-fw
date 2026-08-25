# The Rust firmware and its C reference

Milestone 0 (self-hosting: USB CDC + `updat`) is **done and working in Rust**
(`rust/`), on the ecosystem USB stack (usb-device/usbd-serial/stm32-usbd) with a
pure-Rust SRAM self-flasher. The C in `reference-c/` stays as the register-level
reference (clock/PA9/flash sequences known-good on hardware) and as the recovery
image source. This doc covers the firmware structure, the hard-won gotchas, and
how the Rust relates to the C — what you need before extending it (M1+).

Read `HARDWARE.md` first for the board facts (memory map, pins, USB registers,
flash + debug mechanics, command protocol, calibration).

## What M0 is

A polled (interrupt-free) app that:
1. Boots our own vector table / reset, sets `VTOR`, inits `.data`/`.bss`.
2. Brings up the 48 MHz clock and the USB device peripheral, and enumerates as a
   CDC-ACM serial device with the meter's USB identity (so host tools bind to it).
3. Runs a poll loop that services USB and dispatches bare-5-byte commands:
   - `query` → `firm`
   - `getva` → a 192-byte reply (M0 uses a static packet; M1 fills it from real measurement)
   - `updat` → copy an SRAM streaming flasher to `0x20001400` and jump to it; it
     erases the app region and programs incoming plaintext 64-byte chunks via the
     bootloader's flash routines, then resets. This is what makes it self-hosting.

## C reference file map (`reference-c/`)

| File | Role |
|---|---|
| `startup.s` | reset handler (VTOR, NVIC clear, `.data`/`.bss`), **complete 64-entry vector table**, capturing fault handlers |
| `linker.ld` | app @ `0x08002c00` (52K), RAM @ `0x20000000` (8K), **`_estack = 0x20000e40`** |
| `usb.c` / `usb.h` | polled USB CDC-ACM: clock/USB power-up, PMA/BTABLE setup, EP0 enumeration state machine, descriptors, the CDC data path (`usb_cdc_send`, `usb_poll`) |
| `main.c` | command dispatcher + `updat` launch |
| `flasher.s` / `flasher.ld` / `flasher_blob.s` | SRAM streaming self-flasher, linked at `0x20001400`, embedded as a blob and copied to SRAM by `updat` |
| `getva_reply.inc` | the static 192-byte `getva` packet used in M0 |
| `Makefile`, `openocd.cfg` | build + SWD debug config |

## The four things that cost real debugging time — do NOT rediscover them

1. **`_estack` must be an in-range SRAM address (`0x20000e40`), NOT top-of-RAM
   `0x20002000`.** Top-of-RAM faults the bootloader's hand-off to the app (the
   resulting fault's BFAR is exactly `0x20002000`). Set the stack top to an
   in-range value in your Rust memory layout.

2. **Set `VTOR = 0x08002c00` early in reset.** The bootloader jumps to the app
   without setting VTOR, so exceptions would otherwise vector through the
   bootloader's table.

3. **PA9 is the USB D+ soft-connect.** This USB device peripheral has **no internal
   D+ pull-up**; PA9 drives D+ through a resistor. The bootloader hands off with the
   bus still electrically connected, so the host won't re-enumerate your app unless
   you **drive PA9 low, bring USB up, then drive PA9 high** — that forces a fresh
   enumeration. Missing this = the device inits perfectly but the host never talks
   to it (ISTR/FNR stay 0).

4. **Complete the vector table + clear NVIC at reset.** The table needs all IRQ
   entries (this part has up to ~IRQ42; provide 48), and the bootloader may leave an
   IRQ (e.g. USBD) enabled. An unhandled IRQ vectors into `.text` → a wild branch →
   `INVSTATE`+`IBUSERR` → lockup. Mask/clear all NVIC IRQs at reset (`cortex-m-rt`
   gives you the full table for free; still clear NVIC).

Plus the USB-IP detail that blocks enumeration if wrong: **clearing an endpoint's
CTR bit must use a mask that leaves the toggle-type bits (STAT/DTOG) untouched**
(the working value for CTR_TX-clear is `0x8F0F`, not `0x8F7F`). A stack that
reuses ST's `_ClearEP_CTR_TX`/`_ClearEP_CTR_RX` gets this right automatically.

## The debug reality (this chip is readout-protected) — read this before debugging

- **SWD/OpenOCD `halt`/`reset`/`step` LOCK UP the core**, and a `mdw` read can
  leave it halted. **A "frozen" SRAM value via `mdw` does NOT mean the app is
  dead.** This wasted a lot of time here.
- **Verify liveness over SERIAL** (`query`→`firm`) with **no debugger attached**.
- **Background SRAM reads do work** while the core runs — use **SRAM progress
  markers** (write a stage code to a fixed address at each step; read with
  `openocd -f openocd.cfg -c "mdw 0x20001000 8" -c shutdown`).
- Clear a stuck debug-halt with a **port power-cycle** (`usb-hub-power cycle b2`).
- Consequence for Rust: **you cannot use `probe-rs`/`cargo-embed`/`defmt`/RTT** (they
  need halt + a live debug session). Flash via the bootloader and debug via SRAM
  markers + serial, same as the C.

## Build / flash / debug workflow

All wrapped in `rust/Makefile`:
- `make flash` — build → objcopy → sanity-gate the vector header (SP must be
  `0x20000e40`, reset vector Thumb + in-slot) → pad to 51840 B → hands-free flash
  via the running app's `updat` (`tools/app_update.py`).
- `make recover` — AES image + `tools/flash_image.py` (hold K1 while plugging in).
- `make markers` — non-halting SRAM marker read via openocd.
- Recovery always: K1 + flash any known-good encrypted image over the bootloader.

Operational quirks seen on hardware:
- Even a background `mdw` marker read can leave the core halted — if serial goes
  quiet after `make markers`, power-cycle the meter's hub port and it comes back.
- (C reference only) its `getva` reply has no short-packet terminator, so Linux
  cdc_acm holds the last 64 B until more data arrives; a stale block makes
  `app_update.py`'s `query`→`firm` gate fail — drain the port (send `query`, read
  everything) and retry. The Rust firmware doesn't have this quirk.

## How the Rust is built (as implemented in `rust/`)

### Startup / linker
**`cortex-m-rt` 0.7** provides the vector table (240 IRQ entries, all
`DefaultHandler` — no PAC/`device` feature needed), `.data`/`.bss` init, and the
reset handler. Board specifics:
- `memory.x`: `FLASH ORIGIN 0x08002C00 LENGTH 51840` (the exact flashable image
  size, so the linker fails before the pad step would) and `RAM LENGTH 0xE40` —
  RAM deliberately *ends* at `0x20000e40`, which both makes `_stack_start` the
  known-good SP and fences statics/stack below the marker window and flasher SRAM.
- The **`set-sp` cargo feature** loads MSP from `_stack_start` first thing;
  **`set-vtor` stays OFF** on purpose: VTOR is set inside our own `global_asm!`
  **`__pre_init`** (`src/startup.rs`) *after* `cpsid i`, C ordering — with
  interrupts still live, a bootloader-armed pending IRQ must vector through the
  bootloader's valid table, not into our spinning DefaultHandler. `__pre_init`
  also masks+clears all NVIC IRQs (ICER0/1, ICPR0/1). `#[pre_init]` (the Rust
  attribute) is deprecated; a strong `__pre_init` asm symbol is the sanctioned hook.
- `DefaultHandler` is also overridden in `global_asm!` (not `#[exception]`) so the
  stacked-PC capture reads the hardware frame at MSP exactly like the C; HardFault
  uses `#[exception]` + `ExceptionFrame`. Both write the same SRAM windows as the C
  (`src/markers.rs` has the map; MK_RA was dropped, MK_PANIC/MK_BUILD added).

### Register access
Raw `write_volatile`/`read_volatile` on named address constants (`src/regs.rs`),
matching `reference-c/usb.c` 1:1 — easiest to diff against the proven C. A future
switch to the community `gd32f1` PAC (has a `gd32f150x8` feature) is an option once
things are stable.

### USB
The **ecosystem stack**: `usb-device` 0.3 + `usbd-serial` 0.2 + `stm32-usbd` 0.8.
`src/usb_bus.rs` implements stm32-usbd's `UsbPeripheral` with raw pointers
(registers `0x40005C00`, PMA `0x40006000`, 512 B, `MemoryAccess::Word16x1` = the
doubled-PMA model; `DP_PULL_UP_FEATURE = false`). `src/board.rs` keeps our proven
clock bring-up and the PA9 low → build() → PA9 high choreography — no USB stack
does the soft-connect for you. Device identity (VID/PID/strings/bcdDevice) is set
via `UsbDeviceBuilder`; endpoint layout is whatever the stack allocates (today:
EP1 = interrupt IN 0x81, EP2 = bulk OUT 0x02 + bulk IN 0x82 — differs from the
stock EP map, which no host tool cares about since they bind the CDC class).
A pleasant side effect: usbd-serial terminates transfers properly, so the 192-B
`getva` arrives in one read (the C/stock firmware's stuck-last-block quirk on
Linux cdc_acm — 3×64 B with no short packet — is gone).
Profile stays `opt-level=1`, `lto=false` (the C's unexplained `-Os` lockup story;
revisit optimization only as an isolated, revertable change).

### `updat` self-flasher (pure Rust)
`src/flasher.rs`: `flasher_main` lives in a dedicated `.flasher` section **linked
to run at `0x20001400`** (VMA in `FLASHER_RAM`, see `memory.x` + `flasher.x`) with
its load image in flash; `launch()` copies it to SRAM and calls it via a forced
register-indirect jump. Being linked at its run address means no
position-independence tricks. It discovers the bulk endpoint's PMA buffers **from
the BTABLE at runtime** (the EP index is found by scanning EPnR for the bulk type),
so it tracks whatever stm32-usbd allocated. Constraints inside `.flasher`, since
it erases the app region it was loaded from: raw pointers only, no panicking ops
(`MaybeUninit` buffer — a zeroed array could emit a memclr call into flash), its
own `asm!` for cpsid/dsb, bootloader routines called through `transmute`d fn
pointers (`blx reg`). **`make flasher-gate`** (part of every build) disassembles
the section and fails if any literal references app flash or any call leaves the
section. Protocol unchanged: erase → `uprdy` → 810 × (64 B → program → `OK`) →
AIRCR reset; `tools/app_update.py` works as-is.

## After M0: the roadmap (see README + HARDWARE.md)
- **M1 — measurement + real `getva`:** hardware I²C0 (PB6/PB7) monitor read +
  calibration (HARDWARE.md formulas + EEPROM constants) + a timer-paced sampler +
  energy integration, filling the real `getva` reply. This is where measurement
  stops depending on host poll rate.
- **M2 — persistence** (EEPROM cal/serial/energy, `gtrec` log). **M3 — display**
  (ST7735, HARDWARE.md). **M4 — BLE / USB-PD trigger.**
