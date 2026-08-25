# Porting the firmware to Rust

Milestone 0 (self-hosting: USB CDC + `updat`) is **done and working in C** — see
`reference-c/`. That C is the authoritative reference: every register value,
descriptor byte, and control-transfer step is known-good on real hardware. This
doc is the map for reimplementing it in Rust and then extending it (M1+).

Read `HARDWARE.md` first for the board facts (memory map, pins, USB registers,
flash + debug mechanics, command protocol, calibration). This doc covers the
firmware structure, the hard-won gotchas, and the Rust approach.

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

## Build / flash / debug workflow (identical in Rust)

- Build → a raw `.bin` (Rust: `cargo build --release` + `cargo objcopy -- -O binary`).
- Flash, first time / recovery (needs K1): `tools/encrypt.py app.bin app.enc.bin` then
  `tools/flash_image.py app.enc.bin` (hold K1, board in bootloader).
- Flash, hands-free once `updat` works: `tools/app_update.py app.bin` (plaintext, no K1).
- Recovery always: K1 + flash a known-good encrypted image over the bootloader.

## Rust approach

### Startup / linker
Use **`cortex-m-rt`**: it provides the vector table (all IRQs), the reset handler,
and `.data`/`.bss` init, driven by a `memory.x`. Two board specifics to add:
- `memory.x`: `FLASH : ORIGIN = 0x08002c00, LENGTH = 52K` and `RAM : ORIGIN =
  0x20000000, LENGTH = 8K`, but **force the stack top to `0x20000e40`**, not the top
  of RAM (see gotcha #1) — set `_stack_start = 0x20000e40;`.
- In a `#[pre_init]` or at the top of `main`, **set `VTOR` = `0x08002c00`** and clear
  NVIC (`cortex-m` `NVIC::` / raw `ICER`/`ICPR`). Target `thumbv7m-none-eabi`, `#![no_std]`, a
  `panic-halt` handler (or a panic handler that writes a SRAM marker).

### Register access
No official GD32F150 PAC. Two options, both fine since we work at register level:
- Raw `core::ptr::write_volatile`/`read_volatile` on the known addresses
  (HARDWARE.md + `reference-c/usb.c`) — simplest, matches the C 1:1.
- Or generate a PAC from a GD32F1x0 SVD (`svd2rust`) for typed access. Optional.

### USB — the main decision
This is the STM32F0/F1-style USB device IP (BTABLE + `EPnR` + PMA, **PMA
doubled-addressed** — each 16-bit slot is 4 CPU-address bytes apart). Two paths:
- **A) Reuse `usb-device` + `usbd-serial`** with a `UsbBus` implementation for this
  peripheral (the `stm32-usbd` crate targets exactly this IP and has a feature for
  the doubled-PMA access model). If it binds to the GD32 register block, you get a
  proven CDC stack and dodge the EPnR-toggle-mask class of bugs. Risk: `stm32-usbd`
  is written against STM32 PACs, so wiring it to raw GD32 registers is some glue.
  **Still apply gotcha #3 (PA9 soft-connect) yourself** — no USB stack does that.
- **B) Hand-roll**, porting `reference-c/usb.c` (clock, PMA/BTABLE, EP0 state
  machine, descriptors, CDC path) to Rust `unsafe` register writes. Lower risk of
  ecosystem friction, more code. The C is a line-for-line guide.

Recommended: **try A first** (it's where Rust pays off); fall back to B using the C
as the reference. Either way, the descriptors and register sequences in
`reference-c/usb.c` are the ground truth.

### `updat` self-flasher
A small routine that must run from **SRAM** (it erases the flash it was running
from) and calls the bootloader's resident flash routines by absolute address
(HARDWARE.md "App self-update path"). In Rust: a `#[unsafe(naked)]` or `#[link_section
= ".sram_flasher"]` function copied to `0x20001400`, or keep `reference-c/flasher.s`
as an assembled blob and `include_bytes!` it (simplest — it's proven). Reuse the C
protocol so `tools/app_update.py` works unchanged.

## After M0: the roadmap (see README + HARDWARE.md)
- **M1 — measurement + real `getva`:** hardware I²C0 (PB6/PB7) monitor read +
  calibration (HARDWARE.md formulas + EEPROM constants) + a timer-paced sampler +
  energy integration, filling the real `getva` reply. This is where measurement
  stops depending on host poll rate.
- **M2 — persistence** (EEPROM cal/serial/energy, `gtrec` log). **M3 — display**
  (ST7735, HARDWARE.md). **M4 — BLE / USB-PD trigger.**
