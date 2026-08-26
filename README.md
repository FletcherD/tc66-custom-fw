# TC66 custom firmware

A from-scratch replacement firmware (Rust, `no_std`) for the RuiDeng TC66 /
TC66C USB-C power meter (GigaDevice GD32F150G8, Cortex-M3). It flashes into the
board's 52 KB app slot, keeps the stock bootloader (so recovery always works),
and is wire-compatible with existing host tools: it enumerates as the same USB
serial device, answers the stock command set, and serves real, calibrated
measurements in the stock encrypted `getva` format.

## Why

A clean, fully-understood firmware we control end to end: measurement paced by
a hardware timer — so instantaneous readings don't depend on how fast the host
polls (the stock firmware's "poll faster → fewer fresh samples" coupling is
gone) — our own protocol on top of a compatible base, and room to add features.

## Status

- **M0 — self-hosting: done.** Boots from the stock bootloader, USB CDC-ACM on
  the ecosystem stack (`usb-device`/`usbd-serial`/`stm32-usbd`), reflashes
  itself over USB via `updat` with a pure-Rust SRAM flasher — iterate with
  `make -C rust flash`, no button pressing.
- **M1 — real measurement: done.** INA226 over hardware I²C0 with the per-unit
  EEPROM calibration, TIMER5-interrupt sampling at 250 Hz with an
  interleaved read schedule (~10 Hz publish, immune to main-loop blocking),
  NTC temperature (recovered stock β-model) and D+/D− via the
  on-chip ADC, 1 Hz energy integration (RAM-only for now), all served in the
  stock encrypted+checksummed `getva` packet — plus our plaintext `rawva`
  verb with a window sequence number for fast lossless streaming
  (`tools/rawva.py`). Verified live on hardware.
- **M3 — display basics: done.** ST7735S driver + always-on V/I/P main screen
  (webcam-verified), 10-min idle screen-off (burn-in protection).
- **USB architecture: interrupt-serviced, bench-verified.** The USBD_LP ISR
  keeps the bus served regardless of main-loop work; the device is a
  composite that also emulates a ChargerLAB POWER-Z KM003C, so the mainline
  Linux `powerz` hwmon driver binds it and `sensors` just works (0 failures
  in 2000 gap-free reads). The first ~10 s of every boot are a
  **pseudo-bootloader grace window** (only `query`/`updat` served): a fault
  in any later feature is recovered by power-cycle + `make flash` inside the
  window — no button press. The chip fights back in strange ways; QUIRKS.md
  is the measured catalog.
- **Next steps** (see also the open items in QUIRKS.md):
  - **M2 — persistence** (real serial/run-count, energy write-back to the
    wear-leveled EEPROM journal, `gtrec` log, EEPROM writes in the I²C
    driver).
  - Trial + merge the remaining refactor branches: `refactor/sampler-irq`
    (250 Hz sampler ISR, evenly-interleaved shunt/shunt/vbus reads) and
    `refactor/spi-dma` (non-blocking DMA pixel engine; its SPI0_TX→DMA-ch2
    mapping is manual-derived, unverified). Both predate the current master
    — merge master INTO them first; their stack-gate variants lose to
    master's.
  - CDC latency root cause (~8 ms retry quanta; QUIRKS.md §3) and the
    stock-firmware comparison (QUIRKS.md, bottom).
  - Richer M3 screens (energy, temp, D±); **M4 — stretch** (BLE mirror;
    USB-PD / QC trigger via the on-board FUSB302 — the KM003C protocol's
    AdcQueue/PD-capture attributes are the natural wire format).

## Layout

```
README.md           this file
HARDWARE.md         board reference: memory map, pins, peripherals (INA226,
                    EEPROM, ADC/NTC, FUSB302), flash + debug mechanics,
                    command protocol + getva format, calibration model
FIRMWARE.md         firmware internals: module map, boot path, USB (ISR +
                    grace window), the self-flasher, measurement, build
                    gates, debug workflow
QUIRKS.md           measured GD32F150 misbehavior (USB event loss, NAK bus
                    stalls, ...) + the open investigations & experiments
CLAUDE.md           orientation + working rules for agent sessions
rust/               the firmware — see FIRMWARE.md for the module map;
                    Makefile targets: flash / recover / markers / build gates
tools/              app_update.py (hands-free flash via updat)
                    flash_image.py + encrypt.py (K1 bootloader recovery path)
                    i2cread.py, adcread.py (dev verbs), tc66lib.py (helpers)
openocd.cfg         non-halting SRAM reads via a Pico debugprobe (CMSIS-DAP)
eeprom-backup.bin   this unit's factory EEPROM image (32 KB, double-read
                    verified) — restore point for calibration/journal data
```

The original C implementation of M0 (the porting reference) lived in
`reference-c/` and is retired; it remains in git history.

## Quick start

**Build + flash (hands-free, via the running app's `updat`):**
```
cd rust && make flash     # cargo build -> gates -> pad to 51840 B -> app_update.py
```

**Recovery / first flash (needs the K1 button):**
```
cd rust && make recover   # AES image + flash_image.py
# hold K1 while plugging in USB first -> board enumerates as bootloader CDC
```
A bad app can never brick the board — the bootloader is never written.

**Verify it's alive (over serial, no debugger):** send `query` (5 raw bytes,
115200 8N1), expect `firm`. Poll measurements with `getva` (192 B, AES-256-ECB;
key + field map in HARDWARE.md).

**Read the SRAM debug markers:**
```
cd rust && make markers   # background mdw, NO halt; if serial goes quiet
                          # afterwards, power-cycle the meter's port
```

## Debugging — read this first (the chip is readout-protected)

- **SWD `halt`/`reset`/`step` lock the core up**, and even a background `mdw`
  can leave it halted — a frozen SRAM value does **not** mean the app is dead.
- **Verify liveness over serial** (`query`→`firm`), no debugger attached.
- Debug via **SRAM progress markers** (`rust/src/markers.rs` has the map) and
  the host-triggered dev verbs `i2crd`/`adcrd`.
- Clear a stuck debug-halt with a **port power-cycle** (`usb-hub-power` skill,
  port b2).
- `probe-rs`/`defmt`/RTT are **not usable** on this target. Full detail in
  FIRMWARE.md.

## Key gotchas (detailed in FIRMWARE.md / HARDWARE.md)

- **Initial SP = `0x20000e40`**, not top-of-RAM `0x20002000` (faults the
  bootloader hand-off).
- **Set `VTOR = 0x08002c00` early** — the bootloader doesn't.
- **Drive PA9 (USB D+ soft-connect) low → init → high** or the host never
  enumerates the app.
- **Complete vector table + clear NVIC at reset** (a stray IRQ = wild branch).
- Keep the app **≤ 52 KB**; never write below `0x08002c00` or into the config
  page at `0x0800fc00`.
- **The ADC clock (IRC14M) is off at reset** — enable it or ADC calibration
  hangs forever.
- The `.flasher` section must stay self-contained — the build gates enforce it.
