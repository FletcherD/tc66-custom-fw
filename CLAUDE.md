# CLAUDE.md

Custom Rust firmware for the RuiDeng TC66/TC66C USB-C power meter, developed
against real hardware on the bench. M0 (self-hosting USB + self-reflash),
M1 (real calibrated measurement), M2 persistence (eeprom.rs/cfg.rs/
persist.rs: stock-compatible EEPROM layout — settings via `cfgrd`/`cfgwr`
+ tools/cfg.py, energy ring, datalog + `gtrec`, run count/serial in
getva; docs/HARDWARE.md "EEPROM layout", docs/FIRMWARE.md "Persistence"), the M3 basics (ST7735S driver in
tft.rs + six screens in ui.rs — V/I/P, energy + run clock, temp/D±,
plot.rs's full-screen V/I plot on the panel's hardware rolling scroll, and
pdscr.rs's paged PD info screen (pdinfo.rs parser fed at pd.rs's capture
points, 7×13 font_small.rs, scroll bar; entering it starts the sniffer if
PD is off; a host enumerating the meter starts it too — `pdctl 0` holds until the next enumeration, never without a PC (FIRMWARE.md "Sniffer"); `pdinf` verb + `pd.py info`), and detect.rs's
charger detection screen on the same grid (K2 runs tools/detect.py's
sequence on the device as a main-loop-stepped state machine, ≤5 V,
refuses with a load/sink on the other port, ends PD off; `detct` verb +
`detect.py --device`; both screens' strings are ids into text.rs's packed
table, every line + color pinned by tools/dev/host_check/text.golden —
`-- --bless` after an intended change) —
with no-change screen-off after `idle_off_s` (default 15 s); keys.rs polls K1/K2 — K1 next screen, K2
held 1 s resets energy, K2 short press = next page (detection: a run on the last page), any press wakes; `keyrd`/`uiscr` dev verbs), the
FWDGT watchdog (wdg.rs, ~13 s, fed per loop pass + inside the flasher —
bench-verified with tools/dev/wdgtest.py; `rstrd` reads the reset cause), the
BLE link (bt.rs: DX-BT24-M on USART1 at 9600 — its power-on default, it
ignores TTM provisioning — RX DMA ring + TX DMA, always-on dispatch of
`bgetva`/`bquery`/`brawva`/`bnextp`/`blastp`/`brotat`/`bcfgrd`/`bcfgwr`;
bench-verified with
tools/ble.py), and the M4
charge triggers (pd.rs: FUSB302 PD 2.0/3.0 sink — `pdctl`/`pdsta`/`pdcap`/
`pdreq` + tools/pd.py, fixed PDOs + PPS + hard reset, bench-verified
5-20 V and PPS 3.3-11 V, plus a listen-only sniffer `pdctl 3` (captures a
charger and device negotiating through the meter, either orientation, incl.
SOP'/SOP'' cable e-marker traffic, attach/detach, bad-CRC and hard-reset
events; `pdctl 2` while sniffing re-triggers the negotiation; read with
tools/pdcap.py (no sudo with tools/udev/70-tc66.rules installed — it is, on the bench host; starts the sniffer if the engine is off, `--no-start` not) — decodes all PD message types incl. identities,
extended caps/status, chunk reassembly via tools/lib/pd_decode.py (layouts are
the named fields of tools/lib/pd_spec.py, from the USB PD R3.2 spec; the
firmware's are rust/src/pdspec.rs + pdsta.rs — use them, not raw shifts; regression vectors in tools/dev/pd_decode_test.py;
`--save`/`--replay` for offline re-decoding; tools/pdseq.py runs a capture
around a sequence of pd.py steps); tools/dev/pdlog_check.py scores a capture.
The sink also captures the source's cable probe at attach (listen-only, no
AUTO_CRC, until the first caps — FIRMWARE.md "Cable-listen phase"), and in
every build can ask the source for PD 3.x info (`pdask`/`pdans` + `pd.py ask`, `detect.py`;
`pdsnd` in `dev-pd` sends anything; the
FUSB302 sends type 17 as a GoodCRC — TX completes on the FIFO GoodCRC);
bench-verified
with an Apple charger + Pixel/Moto; this unit's OUT-port CC2 contact is
near-open, so sink attach / hard reset only work with the line on CC1 —
a board fault, not firmware: both CC nodes are unloaded at DC and no GPIO
or ADC pin touches CC, per `gpiot` + tools/dev/gpio_cc_sweep.py). The PD engine wakes on the FUSB302's
INT_N (PA15, read as a GPIO) and runs the BMC slicer at Slice 0x58; I²C is
400 kHz; qc.rs: Quick Charge 2.0 D+/D− — `qctrg`/`qcoff` (entry detects QC first; `qcdet` probe)
+ tools/qc.py, bench-verified 5/9/12 V, plus QC3.0 `qc3ct` and Samsung AFC
`afctr` — a CLOSED-LOOP D− serial that reads the charger Sping on PB4,
bench-verified 5↔9 V; **a granted request raises the OUT
voltage — nothing downstream when triggering**) are done. Pixel streaming is DMA-driven and non-blocking (tft.rs engine
on DMA ch2, pumped by ui::service from the main loop — no display
interrupt); the first frame draws pre-enumeration, so the screen is live
from power-on. USB is ISR-serviced and the device is a composite that
emulates a POWER-Z KM003C (mainline Linux `powerz` hwmon binds it; ADC,
PdPacket, ADC+PD chained, empty PD trace, Enable-/DisablePdMonitor — the
rest Rejects; docs/HARDWARE.md "powerz vendor protocol",
tools/dev/km003c_check.py) —
**powerz comms are the priority path; getva/CDC exists for
compatibility**. The sampler runs from the TIMER5 interrupt (2.24 ms, a shunt +
bus read per tick = ~98% of the INA226's 2.2 ms conversion pairs; `ina::ticks()` stays a 4 ms time base; `window_ms` setting = averaging/publish window, energy integrates full-second averages, `ui_period_ms` = screen refresh — ina.rs; measured conversion timing + raw noise per
CONFIG: tools/dev/inaexp.py / `inaxp`, docs/HARDWARE.md "Measurement front-end"), immune to main-loop blocking. The hot
path is pinned below the GD32's 32 KB zero-wait flash boundary
(`rust/fast.x` + `make fast-gate` — docs/FIRMWARE.md "Fast-flash placement";
query 0.41 / rawva 0.73 / getva 2.69 ms (build 175) — tools/dev/verbtime.py, layout-sensitive by ~0.1-0.2 ms, see FIRMWARE.md; the TFT pixel engine too — full
redraw ~30 ms, tools/dev/uitime.py). Slow-region cost is per cold fetch and
layout-sensitive: anything with a polling loop belongs in fast flash
(`#[link_section = ".fasttext.<name>"]` on our fns, fast.x patterns for
other crates/modules; fast-gate fails on any overflow — no silent spill), and
per-pass `service()` calls keep their idle check `#[inline(always)]` in
the fast main loop (pass ~20 µs). Profile with the on-device PC
sampler (`pcprf` verb + tools/dev/pcprofile.py; the DWT is a stub — never
trust PCSR). For lossless host streaming use the plaintext `rawva` verb
(window seq for dedup; tools/rawva.py); getva also carries the seq at
pac3+4. The build is a pinned nightly (rust-toolchain.toml: build-std core
+ panic=immediate-abort — a panic hits UDF/HardFault, MK_PANIC is dead).
**If new code overflows flash or the stack-gate: stop and ask — never
"make room" by changing unrelated working code.** Next: power-cut / K2
tests of the energy ring, M4 extensions (Huawei FCP/SCP: `fcptr`/`fcprd`/`scptr` verbs, `fcp`
feature — currently out of DEV_FEATURES for space —
framing + reply decoding matched to the stock captures (fcp.csv/scp.csv,
tools/dev/accp_decode.py) but never run against an FCP/SCP charger; an SCP
voltage raise (VSET 0xb8/b9 + watchdog) is not implemented — stock SCP is
5 V/5 A; docs/AFC_DECODE_NOTES.md),
the open docs/QUIRKS.md investigations, richer M3 screens. Fonts are generated by tools/dev/mkfont.py; display verification
uses the bench webcam (see the memory notes), or `tools/dev/tftgrab.py grab`
(`tftrd` frame-memory readback, `dev-tft` builds: `make flash FEATURES=dev-tft`;
`--screen N` selects a screen first; bench-verified on all five screens). docs/DEVELOPMENT.md has the
detailed status/roadmap; README.md is the PUBLIC user-facing page (install,
back to stock, recovery) — keep bench/agent details out of it.

**Public repo + releases** (github.com/FletcherD/tc66-custom-fw, MIT): CI
(`.github/workflows/ci.yml`) runs `make gates` — build + every gate on the
dev AND release feature sets — on each push. Pushing a tag `v<BUILD_ID>` (must
equal rust/src/main.rs `BUILD_ID`) runs release.yml: `make release` →
`tc66fw-v<BUILD_ID>.enc.bin` (encrypted, bootloader format) as the only release
asset. Users flash it with the standalone `tools/tc66flash.py` (uv script:
`flash` via K1 bootloader, `stock` downloads RuiDeng's V1.18 from their
server — never redistribute vendor binaries, `backup-eeprom` via `eeprd`,
`version`). **Feature sets**: cargo default = release = `triggers` (PD/QC/
AFC trigger + detection verbs); `make flash`/`make recover` add the
Makefile's `DEV_FEATURES` — dev verb groups `fcp`/`dev-board`/`dev-bt`/
`dev-tft`/`dev-pd`/`dev-ina` (`dev-verbs` = all; they no longer fit together, so carry only the
groups in current use) + `pcprf`; `make release` / `RELEASE=1` builds the
defaults only. New debug verbs go behind a dev group (or a new one).
Charger protocol detection at 5 V: `tools/detect.py` (`qcdet`, `afctr 0x08`
probe, `fcprd`, PD caps) or on-device (screen 5 / `detct`). Flash is nearly
full (build 164: release ~3.7 KB spare, dev ~3.5 KB;
`tools/dev/inline_report.py` lists inlined-copy candidates for
`#[inline(never)]` — FIRMWARE.md "Fast-flash placement"): `DEV_FEATURES` =
`pcprf dev-pd`. **Version**: just BUILD_ID (no semver),
shown on the ~2 s boot splash (ui.rs `SPLASH`) and by the `versn` verb.
Host trigger tools (pd.py/qc.py) go through tools/lib/trigger_guard.py
(default --max-v 9, confirmation unless --yes).

**The escape hatch**: the boot grace window (first ~10 s serve only
`query`/`updat`, MK_PROBE reads 0x6ACE6ACE) is **feature-gated OFF by
default** — default builds dispatch everything immediately and a
boot-path fault needs K1 + `make recover`. When bringing up risky new
always-on code, build with `--features grace` (`make flash FEATURES=grace`)
and put the new code AFTER the window (never in the pre-enumeration boot
path) to restore it: then a fault
after the window is recovered by power-cycling the meter's port and running `make flash`
within ~10 s of power-up, no K1.

## Read these before working

- **docs/HARDWARE.md** — board facts: memory map, bootloader contract, peripherals
  (INA226 @I²C 0x4A, EEPROM @0x50, FUSB302 @0x22, ADC channels/NTC model),
  command protocol, getva wire format, calibration formulas.
- **docs/FIRMWARE.md** — firmware internals: module map, boot-path gotchas, the
  `.flasher` section mechanics, measurement pipeline, build gates, debug
  workflow. The gotchas there cost real debugging time — do not rediscover them.
- **docs/QUIRKS.md** — measured GD32F150 misbehavior (USB event loss, NAK-traffic
  bus stalls, the no-thread-sweeps rule) + open investigations. Read it
  BEFORE changing anything in the USB service paths — several lines of
  main.rs exist only because of what is documented there.
- **docs/CHARGER_PROTOCOLS.md** — charger detection and sniffing protocol by
  protocol (BC1.2/dividers, QC, AFC, FCP/SCP, Type-C Rp, USB PD, e-markers):
  signalling, passive vs 5 V-probe, what's implemented, bench results.
- **docs/AFC_DECODE_NOTES.md** — the AFC / FCP / SCP D− serial protocol as
  reverse-engineered from stock and its captures, plus public FCP/SCP sources.

## The development loop

```
cd rust && make flash      # build -> gates -> hands-free flash via updat (~1 s)
```
Then verify over serial: `query` → `firm`; `getva` → 192-B encrypted packet
(decrypt/parse snippets: docs/HARDWARE.md). Bump `BUILD_ID` in `rust/src/main.rs`
each flash and read it back at marker 0x20001060 to prove the new image runs.
The board is on the bench hub (port **b3** as of 2026-10-13 — it moves; check
`hub_power.py status`) — power-cycle it with the `usb-hub-power` skill when
needed. Host tools: `tools/` = user-facing scripts, `tools/dev/` = dev/bench tools and gates, `tools/lib/` = shared modules only (tools/README.md). K1 can be driven hands-free by the Pico
debugprobe (tools/dev/k1boot.py) when the probe is plugged in.

## Hard rules

- **Never** write flash below `0x08002c00` or the config page at `0x0800fc00`.
  A bad app is always recoverable (K1 + `make recover` — needs the user to
  hold the physical button), but the bootloader/config must stay intact.
- **Never** use SWD `halt`/`reset`/`step` — the chip is readout-protected and
  halting wedges the core. Even `make markers` (background SRAM read) often
  leaves it halted: always follow with a serial liveness check, and
  power-cycle its port if quiet. `probe-rs`/`defmt`/RTT do not work here.
- **EEPROM writes** go only through `eeprom.rs` (page-split, ACK-polled,
  verified, and a hard guard refusing the calibration words). The factory
  calibration lives there; `eeprom-backup.bin` (repo root, gitignored —
  never commit it) is this unit's
  verified factory image — keep it usable as a restore point
  (`tools/eedump.py dump` to back up before experiments).
- **No panics in new code**: panic=immediate-abort turns any reachable
  panic (bounds check, overflow) into a HardFault — check new functions for
  `udf` in the disassembly (docs/FIRMWARE.md "Persistence").
- Bring up new hardware behind a **host-triggered dev verb** first (like
  `i2crd`/`adcrd`): nothing at boot, bounded waits, staged error replies.
  Move it into the always-on path only once proven. This keeps every failure
  recoverable by power-cycle instead of K1.
- Exactly two interrupts are unmasked, both at one NVIC priority (the
  stack-gate assumes no nesting): **USBD_LP services USB** (ISR/thread
  split, shared-state rules and the with_usb critical-section discipline:
  docs/FIRMWARE.md "USB") and the **TIMER5 sampler tick** (ina.rs, unmasked late
  in `ina::start()` — after the grace window when that feature is on). Everything else stays masked
  and polled from the main loop; long work (AES, TFT) never runs in an ISR.
  All thread-mode I2C must run inside `ina::pause(...)`, and all thread-mode
  access to the published measurements goes through `ina::snapshot()`/its
  helpers — the soundness rules are the ina.rs module doc. The build profile
  is `opt-level = "s"` + fat LTO (bench-validated; if inexplicable USB
  lockups appear, suspect it first) — see docs/FIRMWARE.md.
- The `.flasher` section must stay self-contained (no calls/literals into app
  flash); the `flasher-gate`/`init-gate`/`stack-gate`/`fast-gate` Makefile gates
  enforce this (and the static worst-case stack budget and fast-flash fit — a boot-path stack overflow
  bricks invisibly) and run on every build — never bypass them.

## Bench notes

- The meter is powered and reached via a bench hub port; its measurement path
  (IN→OUT) is only live when something actually feeds the IN port — bus
  voltage/current legitimately read 0 otherwise. **While IN is fed, the meter
  is powered from IN and cycling its port does NOT reset it** (check the run count
  via `tools/cfg.py`); reboot it with `make flash` (updat resets it) instead.
- After an `updat` reboot, enumeration intermittently fails for 5–25 s (kernel
  "can't read configurations"; docs/QUIRKS.md §6, open) — if the meter doesn't come
  back, wait, or cycle its hub port when powered from IN; `tools/dev/enumtrial.py`.
- SWD (for `make markers`) is a Pico debugprobe; config is `openocd.cfg` at
  the repo root. Wiring in docs/HARDWARE.md.
- Update the docs (README, docs/*) after functional changes — they
  describe the current state only; history lives in git.
