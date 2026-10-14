# Development guide

How the firmware is built, flashed during development, and debugged, plus
the detailed per-milestone status. Board facts live in
[HARDWARE.md](HARDWARE.md), firmware internals in [FIRMWARE.md](FIRMWARE.md),
measured chip misbehavior in [QUIRKS.md](QUIRKS.md).

## Design goals

A clean, fully-understood firmware controlled end to end: measurement paced
by a hardware timer — so instantaneous readings don't depend on how fast the
host polls (the stock firmware's "poll faster → fewer fresh samples" coupling
is gone) — our own protocol on top of a compatible base, and room to add
features.

## Status

- **M0 — self-hosting: done.** Boots from the stock bootloader, USB CDC-ACM on
  the ecosystem stack (`usb-device`/`usbd-serial`/`stm32-usbd`), reflashes
  itself over USB via `updat` with a pure-Rust SRAM flasher — iterate with
  `make -C rust flash`, no button pressing.
- **M1 — real measurement: done.** INA226 over hardware I²C0 with the per-unit
  EEPROM calibration, TIMER5-interrupt sampling at 250 Hz with an
  interleaved read schedule (~10 Hz publish, immune to main-loop blocking),
  NTC temperature (recovered stock β-model) and D+/D− via the
  on-chip ADC (stock channel map), 1 Hz energy integration, all served in the
  stock encrypted+checksummed `getva` packet — plus our plaintext `rawva`
  verb with a window sequence number for fast lossless streaming
  (`tools/rawva.py`). Verified live on hardware.
- **M2 — persistence: done (bench-verified build 113).** Stock-compatible
  EEPROM layout, so stock and custom firmware read each other's data:
  settings (rotation, brightness 0–9, screen, active energy group, screen
  idle-off delay + wake thresholds, plot period + scale, datalog
  on/interval, USB/BLE command disable) read/written with
  `cfgrd`/`cfgwr` (`tools/cfg.py`; BLE `bcfgrd`/`bcfgwr`), coalesced writes;
  run count + config-page serial in getva; both energy groups + run clocks
  in a wear-leveled, CRC-tagged ring (survive power cuts, ≤10 s lost);
  stock datalog + `gtrec` (`tools/gtrec.py`). `tools/eedump.py` dumps and
  decodes the chip. Bench-verified under an ~8 mA load: ring saves continue
  the stock seq, both groups + run clocks + the group setting restore
  across reboots, BLE settings verbs work. Not yet exercised: a real power
  cut mid-session (torn-slot fallback) and the K2-hold reset.
- **M3 — display: done.** ST7735S driver + four always-on screens
  (V/I/P; mAh/mWh + run clock; temperature + D+/D−; a full-screen
  scrolling V/I plot using the panel's hardware scroll — `plot.rs`; period
  and V/I full scale are settings), K1 steps screens, K2
  held 1 s resets energy, idle screen-off (settings: delay + V/I thresholds) (burn-in protection; any
  key press wakes it — `rust/src/keys.rs`). Pixel streaming is
  DMA-driven and non-blocking (SPI0 TX on DMA ch2, pumped from the main
  loop — bench-verified via the MK_DSP markers); the first frame draws
  pre-enumeration, so the screen is live from power-on.
- **BLE: done.** The TC66C's DX-BT24-M module (USART1, 9600) is served
  always-on: the stock `bgetva`/`bnextp`/`blastp`/`brotat` verbs plus our
  `bquery`/`brawva`, RX by circular DMA, TX by DMA so a 200 ms reply never
  stalls the loop. Bench-verified end to end with `tools/ble.py`
  (`getva` decrypts a live packet over the air).
- **Watchdog: done.** FWDGT (~13 s) armed at boot and fed from the main
  loop and the SRAM flasher; a hung loop reboots the meter instead of
  playing dead. Bench-verified (`tools/wdgtest.py`).
- **Fast-flash placement: done.** Hot path pinned below the GD32's 32 KB
  zero-wait flash boundary (`rust/fast.x` + `make fast-gate`): query 0.2 ms,
  rawva 0.6 ms, getva 2.3 ms round-trip (were 4.5/9/49); powerz sustains
  >5000 reads/s. The TFT pixel engine is fast too (full redraw ~30 ms,
  `tools/uitime.py`). Profile-guided via the on-device 250 Hz PC sampler
  (`pcprf` verb + `tools/pcprofile.py` — the chip's DWT is a stub).
- **USB architecture: interrupt-serviced, bench-verified.** The device
  leads with its **POWER-Z KM003C emulation — the priority protocol**: the
  mainline Linux `powerz` hwmon driver binds it, `sensors` just works, and
  the pre-armed reply path sustains >5000 reads/s with zero timeouts. The
  legacy TC66 CDC verbs (`getva` etc.) ride along **for compatibility**
  with existing host tools, plus our dev verbs and the `updat` reflash.
  The USBD_LP ISR (with an EPxCS-conditioned drain loop) keeps the bus
  served regardless of main-loop work. An optional **pseudo-bootloader
  grace window** (`--features grace`: the first ~10 s serve only
  `query`/`updat`) lets a fault in later code be recovered by power-cycle +
  `make flash` inside the window — no button press. QUIRKS.md is the
  measured catalog of chip misbehavior (every entry absorbed or resolved
  in firmware).
- **M4 — USB-PD voltage trigger: working.** A host-armed FUSB302 PD 2.0/3.0
  sink engine (`rust/src/pd.rs`, verbs `pdctl`/`pdsta`/`pdcap`/`pdreq`,
  host tool `tools/pd.py`): attach/orientation, source-capability
  collection, rev negotiation, fixed-PDO voltage triggering, PPS/APDO
  requests, and Hard Reset — bench-verified against two chargers: an Apple
  96 W (fixed PDOs at PD 2.0, 5/9/15/20 V) and a PD 3.0 source with a
  3.3–11 V PPS APDO (rev-3.0 negotiation, PPS fine-stepping across the range,
  keepalive holding the contract). CC-pin voltages (Vcc1/Vcc2) and
  the logic rail (Vdd) are surfaced in the powerz ADC payload. **Quick
  Charge 2.0/3.0** is a separate D+/D− trigger engine (`rust/src/qc.rs`,
  verbs `qctrg`/`qc3ct`/`qcoff`, host tool `tools/qc.py`) — bench-verified
  QC2.0 discrete 5/9/12 V and QC3.0 continuous ±200 mV fine-stepping
  (bidirectional) against real chargers. **Samsung AFC** (`afctr`, same
  module, closed-loop D− serial reading the charger's Sping on PB4) is
  bench-verified 5↔9 V. **Huawei FCP/SCP** (`fcptr`/`fcprd`/`scptr`,
  `dev-verbs` builds only) have framing and reply decoding matched to the
  stock captures but are untested — see the next steps. Captured PD traffic is exposed over
  the powerz vendor endpoint as the **KM003C PdPacket attribute (0x0010)**
  (`tools/pdcap.py`; the transport is verified, event capture awaits a PD
  source on the bench). Nothing runs at boot; a granted request **raises the
  OUT-port voltage** (trigger with nothing downstream).
- **Next steps** (see also the open items in QUIRKS.md):
  - Bench-test a power cut mid-session (torn-slot fallback), the K2-hold
    reset, and D± readings against a known device/charger.
  - Bench-verify the PD sniffer (`pd.py sniff` + `pdcap.py`) with a phone
    on OUT and a PD charger on IN; check the RX FIFO keeps up (`pdsta` err
    0xE5) and what cable orientations pass through.
  - Bench-verify the positive QC and AFC paths of `tools/detect.py`
    (`qcdet` class 2, the `afctr 0x08` probe, `qctrg` after the detecting
    entry) against the QC/AFC charger. Verified so far: a PC host port (all
    negative) and an Apple USB-C PD charger (DCP with D± tied at 1.51 V,
    Rp 3.0 A, PD 5/9/15/20 V; `qctrg`/`qc3ct` refuse with ER 0x21 and VBUS
    stays at 5 V).
  - **M4 extensions:** bench-test FCP/SCP against a Huawei charger (put
    `fcp` back into `DEV_FEATURES` — it is out of the dev build for space)
    (`fcprd` reg 0x21, n 3 first — read-only — then `fcptr`; scope D− and compare
    with `tools/accp_decode.py`); an SCP voltage raise (VSET 0xb8/b9 + a
    fed watchdog — the stock SCP block only sets 5.00 V / 5 A); a
    protocol-detection scan (docs/AFC_DECODE_NOTES.md). Also: a QC3.0 min-voltage clamp, and
    bench-verifying PD event capture through the KM003C PdPacket attribute.
  - The QUIRKS.md open questions (the unexplained mechanisms behind §1
    NAK/APB stalls and §2 event loss — both absorbed in firmware) and the
    optional stock-firmware Tier 2 comparison.
  - On-device settings menu / group switch on the buttons, a PD/CC status
    screen.

## Layout

```
README.md           user-facing overview: install, recovery, usage
CLAUDE.md           orientation + working rules for agent sessions
docs/DEVELOPMENT.md this file
docs/HARDWARE.md    board reference: memory map, pins, peripherals (INA226,
                    EEPROM, ADC/NTC, FUSB302), flash + debug mechanics,
                    command protocol + getva format, calibration model
docs/FIRMWARE.md    firmware internals: module map, boot path, USB (ISR +
                    grace window), the self-flasher, measurement, build
                    gates, debug workflow
docs/QUIRKS.md      measured GD32F150 misbehavior (USB event loss, NAK bus
                    stalls, ...) + the open investigations & experiments
docs/AFC_DECODE_NOTES.md  Samsung AFC / Huawei FCP+SCP capture decoding
rust/               the firmware — see FIRMWARE.md for the module map;
                    Makefile targets: flash / release / gates / recover / markers
tools/              tc66flash.py (user flasher: release images, back to
                    stock, EEPROM backup — standalone uv script)
                    app_update.py (hands-free dev flash via updat)
                    cfg.py (settings), gtrec.py (datalog), getva.py,
                    rawva.py (streaming), eedump.py (EEPROM dump/decode)
                    pd.py, qc.py (charge triggers; trigger_guard.py limits)
                    flash_image.py + encrypt.py (K1 bootloader path)
                    i2cread.py, adcread.py, tft.py, ... (dev verbs)
                    stack_gate.py, fast_gate.py (build gates), mkfont.py
                    (font/: Inconsolata LGC Bold, SIL OFL)
.github/workflows/  ci.yml (build + all gates, both feature sets, every
                    push), release.yml (tag vX.Y.Z -> encrypted image release)
openocd.cfg         non-halting SRAM reads via a Pico debugprobe (CMSIS-DAP)
```

## Feature sets

Cargo features pick what the image serves (rust/Cargo.toml):

| build | features | used for |
|---|---|---|
| release (`make release`, CI release) | default = `triggers` | published images |
| dev (`make flash`, `make recover`) | + Makefile `DEV_FEATURES` (now `dev-board pcprf`) | the bench loop |
| either + `FEATURES=grace` | + boot grace window | risky always-on bring-up |

The product protocol (powerz, `query getva rawva updat cfgrd cfgwr gtrec
eeprd versn rotat`, BLE verbs) is never gated. `triggers` = the PD/QC/AFC
trigger and detection verbs. The dev groups (HARDWARE.md "CDC verb
protocol"): `fcp` (untested FCP/SCP `fcptr`/`fcprd`/`scptr`, ~2.1 KB),
`dev-board` (`i2crd adcrd clkrd keyrd rstrd wdtst idlrd`), `dev-bt` (BLE
bring-up), `dev-tft` (display bring-up); `dev-verbs` = all four, which no
longer fits the image together with everything else — `DEV_FEATURES`
carries only the groups in current use (`make gates` compile-checks all of
them). Disabled verbs match and silently do nothing.

## Versioning

`rust/Cargo.toml` `version` is the semver; `BUILD_ID` in `rust/src/main.rs`
is bumped on every flashed build. Both are shown on the boot splash
(`v0.1.0` / `b130`) and returned by the `versn` verb (`OK0.1.0+130`).
Releasing: bump the Cargo version, commit, push a tag `v<version>` — the
release workflow refuses a tag that does not match.

## Quick start

**Toolchain:** the pinned nightly in `rust/rust-toolchain.toml` (rustup
installs it; needs `rust-src` + the `thumbv7m-none-eabi` target),
`arm-none-eabi` binutils (gates + objcopy), Python 3 with `cryptography`
(encryption), and [uv](https://docs.astral.sh/uv/) for the tools.

**Build + flash (hands-free, via the running app's `updat`):**
```
cd rust && make flash     # cargo build -> gates -> pad to 51840 B -> app_update.py
```

**Release image / CI gates:**
```
cd rust && make release   # -> tc66fw-release.enc.bin (bootloader format)
cd rust && make gates     # both feature sets, every gate (what CI runs)
```

**Recovery / first flash (needs the K1 button):**
```
cd rust && make recover   # AES image + flash_image.py
# hold K1 while plugging in USB first -> board enumerates as bootloader CDC
```
A bad app can never brick the board — the bootloader is never written.

**Verify it's alive (over serial, no debugger):** send `query` (5 raw bytes,
115200 8N1), expect `firm`; `versn` returns the version. Poll measurements
with `getva` (192 B, AES-256-ECB; key + field map in HARDWARE.md).

**Read the SRAM debug markers:**
```
cd rust && make markers   # background mdw, NO halt; if serial goes quiet
                          # afterwards, power-cycle the meter
```

## Debugging — read this first (the chip is readout-protected)

- **SWD `halt`/`reset`/`step` lock the core up**, and even a background `mdw`
  can leave it halted — a frozen SRAM value does **not** mean the app is dead.
- **Verify liveness over serial** (`query`→`firm`), no debugger attached.
- Debug via **SRAM progress markers** (`rust/src/markers.rs` has the map) and
  the host-triggered dev verbs `i2crd`/`adcrd`.
- Clear a stuck debug-halt by **power-cycling the meter** (a switchable
  USB hub port makes this hands-free).
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
