# TC66 custom firmware

A from-scratch replacement application for the RuiDeng TC66 / TC66C USB-C power
meter (GigaDevice GD32F150G8, Cortex-M3). It flashes into the board's 52 KB app
slot, keeps the stock bootloader (so recovery always works), and is wire-compatible
with existing host tools (enumerates as the same USB serial device).

## Status

- **Milestone 0 — done & working on hardware, in Rust** (`rust/`). The Rust firmware
  boots from the stock bootloader, runs USB CDC-ACM on the ecosystem stack
  (`usb-device` + `usbd-serial` + `stm32-usbd` over a small `UsbPeripheral` impl),
  **enumerates as the meter's CDC serial device, answers `query`→`firm`, serves the
  static `getva`, and reflashes itself over USB via `updat` (no button press)** —
  self-hosting, iterated hands-free with `make -C rust flash`. The self-flasher is
  pure Rust (a `.flasher` section copied to SRAM) that discovers its endpoint/PMA
  layout at runtime. Verified end-to-end: a build flashed a newer build of itself
  via its own `updat` (`MK_BUILD` round-trip).
- `reference-c/` is the original C implementation of M0, kept as the line-for-line
  reference and as a known-good recovery image. The Rust tree is the daily driver.
- **Next: M1** (real measurement + `getva`). See the roadmap below.

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
PORTING.md       the hard-won gotchas + how the Rust port maps onto the C reference
rust/            the firmware (Rust, no_std, cortex-m-rt, gd32f1 PAC) — rust/src/:
                 main.rs (entry, poll loop, command dispatch),
                 usb_bus.rs (UsbPeripheral impl for stm32-usbd; CDC via
                 usb-device + usbd-serial), board.rs (clock + PA9 soft-connect),
                 startup.rs (__pre_init/fault capture), markers.rs,
                 flasher.rs (Rust SRAM self-flasher, linked at 0x20001400 via
                 flasher.x), getva.rs; memory.x, Makefile (build/flash/recover/
                 markers targets + gates: flasher self-containment, init-symbol
                 bounds, image header)
reference-c/     the original M0 firmware in C — line-for-line reference + recovery
                 image source (startup.s, linker.ld, usb.c/.h, main.c, flasher.s, ...)
tools/           encrypt.py (raw -> AES image for the bootloader path)
                 flash_image.py (flash over bootloader, needs K1)
                 app_update.py (flash over the app's own 'updat', no K1)
                 tc66lib.py (serial helpers)
```

## Quick start

**Build + flash the firmware (hands-free, via the running app's `updat`):**
```
cd rust && make flash           # cargo build -> objcopy -> pad to 51840 B -> app_update.py
```

**Flash (first time / recovery — needs K1):**
```
cd rust && make recover         # encrypt.py + flash_image.py
# (hold K1 while plugging in USB first, so the board enumerates as bootloader CDC)
```

**Read the SRAM debug markers (see rust/src/markers.rs for the map):**
```
cd rust && make markers         # background mdw — NO halt; may still wedge the core,
                                # power-cycle the port afterwards if serial goes quiet
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

- **M0 — self-hosting:** boot + USB CDC + `query` + `updat`. **Done (Rust + C reference).**
- **M1 — measurement + real `getva`** (core done): hardware I²C0 (`i2c.rs`) ✓,
  INA226 + EEPROM calibration (`ina.rs`) ✓, real encrypted `getva` with
  CRC16/MODBUS block checksums (`getva.rs`) ✓, SysTick-paced sampler (250 Hz
  ticks → ~10 Hz publish, poll-rate-independent; MK_WINDOWS/MK_I2CERR telemetry)
  ✓ — verified live on hardware. Remaining for M1: energy integration
  (mAh/mWh groups), temperature + D+/D− (on-chip ADC).
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
