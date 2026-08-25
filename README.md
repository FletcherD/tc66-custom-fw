# TC66 custom firmware

A from-scratch replacement application for the RuiDeng TC66 / TC66C USB-C power
meter (GigaDevice GD32F150G8, Cortex-M3). It flashes into the board's 52 KB app
slot, keeps the stock bootloader (so recovery always works), and is wire-compatible
with existing host tools (enumerates as the same USB serial device).

## Status

- **Milestone 0 — done & working on hardware, in C** (`reference-c/`). The firmware
  boots our own vector table, brings up USB CDC-ACM from the register level,
  **enumerates as the meter's CDC serial device, answers `query`→`firm`, and
  reflashes itself over USB via `updat` (no button press)** — i.e. it is
  self-hosting and can be iterated hands-free.
- **Next: reimplement M0 in Rust** in this directory, using the C as the reference,
  then extend (measurement, display, etc.). See **PORTING.md**.

## Goal

A clean, fully-understood firmware we control end to end: measurement paced by a
hardware timer (so instantaneous readings don't depend on how fast the host polls),
our own protocol on top of a compatible base, and room to add features. It also
removes the "poll faster → fewer fresh samples" coupling of the original firmware.

## Layout

```
README.md        this file
HARDWARE.md      board reference: memory map, pins, USB registers, flash + debug
                 mechanics, command protocol + getva format, calibration, display
PORTING.md       M0 walkthrough, the hard-won gotchas, and the Rust porting plan
reference-c/     the working M0 firmware in C — the authoritative reference
                 (startup.s, linker.ld, usb.c/.h, main.c, flasher.s, Makefile, ...)
tools/           encrypt.py (raw -> AES image for the bootloader path)
                 flash_image.py (flash over bootloader, needs K1)
                 app_update.py (flash over the app's own 'updat', no K1)
                 tc66lib.py (serial helpers)
rust/            (to be created — the Rust reimplementation)
```

## Quick start

**Build the C reference:**
```
cd reference-c && make          # -> m0.bin (raw app), make pad -> m0_pad.bin (51840 B)
```

**Flash (first time / recovery — needs K1):**
```
tools/encrypt.py reference-c/m0.bin m0.enc.bin
# hold K1 while plugging in USB (board enumerates as bootloader CDC), then:
tools/flash_image.py m0.enc.bin
```

**Flash hands-free once the app has `updat`:**
```
tools/app_update.py reference-c/m0_pad.bin      # plaintext, ~1 s, self-reflash + reset
```

**Verify it's alive (over SERIAL, no debugger — see the debug note below):**
```
# query -> firm  means the custom app is running and self-hosting
```

**Recovery:** hold K1, flash any known-good encrypted image over the bootloader. A
bad app never bricks the board — the bootloader is never written.

## Debugging — read this (the chip is readout-protected)

- **SWD/OpenOCD `halt`/`reset`/`step` lock the core up**, and a `mdw` read can leave
  it halted — a "frozen" SRAM value **does not mean the app is dead**. This is the
  single biggest debugging trap on this target.
- **Verify liveness over serial** (`query`→`firm`) with **no debugger attached**.
- **Background SRAM reads work** while the core runs — debug with **SRAM progress
  markers** and `openocd -f reference-c/openocd.cfg -c "mdw 0x20001000 8" -c shutdown`.
- Clear a stuck debug-halt with a **port power-cycle** (`usb-hub-power cycle b2`).
- This means the usual Rust flow (`probe-rs`/`defmt`/RTT) is **not** usable — flash
  via the bootloader and debug via markers + serial. Full detail in PORTING.md.

## Roadmap

- **M0 — self-hosting:** boot + USB CDC + `query` + `updat`. **Done (C).** Port to Rust next.
- **M1 — measurement + real `getva`:** hardware I²C0 monitor read + calibration + a
  timer-paced sampler + energy integration → correct V/I/P with no poll-rate coupling.
- **M2 — persistence:** EEPROM (cal/serial/energy) + `gtrec` log.
- **M3 — display:** ST7735 bring-up + a minimal V/I/P draw layer.
- **M4 — stretch:** BLE mirror; USB-PD / QC trigger.

## Key gotchas (all detailed in PORTING.md)

- **`_estack` = `0x20000e40`**, not top-of-RAM `0x20002000` (which faults the bootloader hand-off).
- **Set `VTOR = 0x08002c00`** early — the bootloader doesn't.
- **Drive PA9 (USB D+ soft-connect) low→init→high** or the host never enumerates the app.
- **Complete vector table + clear NVIC** at reset (a stray IRQ = wild branch = lockup).
- **Debug via SRAM markers + serial liveness**, never `halt`/`reset`/`step`.
- Keep the app **≤ 52 KB**; never write below `0x08002c00` or into the config page at `0x0800fc00`.
