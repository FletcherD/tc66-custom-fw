# TC66 hardware reference

Board-level reference for writing firmware for the RuiDeng TC66 / TC66C USB-C
power meter. Everything here is what the firmware needs to drive the board.

## MCU

- **GigaDevice GD32F150G8** — ARM Cortex-M3, QFN28.
- **64 KB flash**, **8 KB SRAM**.
- Core clock **48 MHz** (PLL ×12 from IRC8M/2; IRC8M = 8 MHz), as stock's SystemInit also sets it; ours re-does it in `board::clock_init` with the USB clock = PLL/1. Part max is 72 MHz. USB full-speed needs a 48 MHz USB clock.
- Debug access via **SWD** (see "Debug" below). The chip is readout-protected.

## Memory map

| Region | Range | Notes |
|---|---|---|
| Bootloader | `0x08000000 – 0x08002c00` (11 KB) | **Leave untouched.** Provides the flash-update path and flash helper routines we reuse. Never erased by an app flash. |
| **Application (our code)** | `0x08002c00 – 0x0800fc00` (52 KB) | Where our firmware lives and is flashed. |
| Config page | `0x0800fc00 – 0x08010000` | Per-unit config (serial, cal flag, etc.). **Do not erase/overwrite.** |
| SRAM | `0x20000000 – 0x20002000` (8 KB) | |

**Flash speed boundary at `0x0800_8000`:** fetches below it are zero-wait;
above it each fetch takes "a long delay" (UM §1.3.3/§2.2 — measured ~10x on
this unit). The app spans the boundary, so hot code must be placed below it
(FIRMWARE.md "Fast-flash placement", `rust/fast.x` + `make fast-gate`).

### Application requirements (enforced by the bootloader)

The bootloader hands control to the app on a normal boot **only if**:
- The app has a valid Cortex-M vector table at `0x08002c00`, and
- **`vector[0]` (initial SP) is in `0x2000xxxx`** — but **NOT the exact top of RAM `0x20002000`**. Top-of-RAM faults the hand-off. Use an in-range value; **`0x20000e40` is known-good.**

The bootloader does **not** set `VTOR`, so the app's reset handler must set
`VTOR = 0x08002c00` (write `0x08002c00` to `0xE000ED08`) before enabling interrupts.

There is no CRC/magic/version check on the image. A bad app never bricks the board —
the bootloader is untouched, so recovery is always possible (see Flashing).

## Boot buttons

- **K1 = PF0** — held at power-on → bootloader enters USB firmware-update mode. Released → normal boot (jump to app).
- **K2 = PF1** — held at power-on → factory reset.
- Both are also readable by the app (see "Buttons" below).

## Flashing an image

Two paths:

### 1. Bootloader path (needs K1)
1. Hold **K1** while plugging in USB → the board enumerates as a CDC serial device in bootloader mode.
2. Encrypt the raw app: `tools/dev/encrypt.py app.bin app.enc.bin` (AES-256-ECB, image key baked in; pads to 51840 B).
3. `tools/dev/flash_image.py app.enc.bin` (or the user-facing `tools/tc66flash.py flash`) — sends `query`→`boot`, `update`→`uprdy`, then 810 × 64-byte chunks (the bootloader decrypts + writes each), then the board restarts.

This is the **recovery path** and the way to flash the very first app. Always works.

### 2. App self-update path (`updat`, no K1)
While our app is running, later builds flash over USB with no button press
(`make flash` does this):
1. `tools/dev/app_update.py app.bin` — sends `updat`; the app copies a small flasher into SRAM and streams **plaintext** 64-byte chunks to it; the flasher programs them and resets into the new app.

The self-flasher runs from SRAM (because it erases the whole app region) and reuses
the bootloader's flash routines, which are always resident:

| Routine | Address | Signature |
|---|---|---|
| erase pages | `0x0800048f` | `(r0=base, r1=pagesize=0x400, r2=count)` — e.g. `(0x08002c00, 0x400, 52)` |
| unlock FMC | `0x08000ed5` | `()` |
| program word | `0x08000ef5` | `(r0=flash_addr, r1=word)` |
| lock FMC | `0x08000e49` | `()` |

Reset the core when done by writing `0x05fa0004` to `SCB_AIRCR (0xE000ED0C)`.
The self-flasher must only ever write **upward from `0x08002c00`**, so it can never
touch the bootloader or the config page.

## Debug (SWD)

- SWD pads on the board → an SWD probe. A Raspberry Pi Pico running **debugprobe**
  (CMSIS-DAP) works with OpenOCD; wire **SWCLK=GP2, SWDIO=GP3, GND** common
  (debugprobe requires SWCLK/SWDIO on adjacent GPIOs). `openocd.cfg` (repo root) is ready.
- **The chip is readout-protected, which constrains debugging:**
  - **Flash reads over SWD fail** — you cannot read the flashed image back.
  - **`halt` / `reset` / `step` lock the core up** — do not use them.
  - **Background SRAM reads DO work** while the core runs.
- **Therefore: debug with SRAM progress markers.** Have the firmware write a
  progress code to a fixed SRAM address at each stage, and read it with:
  `openocd -f openocd.cfg -c "mdw 0x20001000 8" -c shutdown` (no halt).
  This is the primary debug loop — see FIRMWARE.md "Build / flash / debug workflow".

## Host protocols

The device is a composite with two host protocols. **The powerz vendor
protocol is the PRIORITY path** — it is what the mainline Linux driver
speaks and what new features should serve first. The CDC verb protocol
(getva etc.) is kept **for compatibility** with existing TC66 host tools
plus our dev verbs and `updat`.

### powerz vendor protocol (interface 0 — the priority path)

The ChargerLAB POWER-Z KM003C protocol subset the mainline Linux `powerz`
hwmon driver uses, on the fixed bulk pair **0x01 OUT / 0x81 IN** (the
driver hard-codes them). Requests are 4-B headers: `cmd, tid, attr<<1`
LE; `GetData` (0x0C) with `ATT_ADC` (0x0001) fetches measurements. The
reply is 52 B: an 8-B `PutData` (0x41) + extended header, then the 44-B
ADC payload (built in `powerz_payload()`, main.rs):

| Offset | Field (LE) |
|---|---|
| 0 / 4 | instantaneous Vbus µV (i32) / Ibus µA (i32, signed: + = IN → OUT) |
| 8 / 12 | 1 s average µV / µA |
| 16 / 20 | 1 s nominal-scale ("original") µV / µA |
| 24 | temp, i16, 1/128 °C (NTC, `ntc::temp_c128`) |
| 26, 28, 34 | Vcc1/Vcc2 (CC pins, FUSB302 MDAC sweep, 42 mV steps — live while the PD engine is armed or sniffing, else 0) / Vdd (logic rail via Vrefint) — u16, 0.1 mV |
| 30 / 32 | D+ / D− in 0.1 mV units (u16) |
| 36 / 37 | KM003C `sample_rate_idx` / `flags` — meaning undocumented (36 is probably the graph rate), 0 |
| 38 / 40 / 42 | KM003C "avg" CC2 / D+ / D− in mV (u16) — the same values as above (ours are window means already) |

The reply is **pre-armed** on the IN endpoint (rebuilt on the 4 ms `ina::ticks()` cadence,
re-armed unconditionally per ISR pass) so a host read completes without
NAK-polling — the driver's 5 ms URB budget is the design constraint.
`Connect`/`Disconnect` (0x02/0x03) and `Enable`/`DisablePdMonitor`
(0x10/0x11) get Accept — Enable also starts the sniffer if the PD engine
is off (main loop, `powerz::PD_MONITOR_REQ`); Disable changes nothing (the
real KM003C buffers PD events regardless). `GetData` 0x0011 (ADC + PD)
answers with both as chained logical packets (ext-header `next` bit 15) in
one ≤127-B transfer (two bulk packets); 0x0020 (PD state trace) answers
with both queues empty (count bytes 0 — the KM003C's state codes are
undocumented). Settings, LogMetadata and the rest get Reject.
`tools/dev/km003c_check.py` exercises every reply.
Bench: >5000 reads/s sustained with zero timeouts.

**Vendor session.** `Connect` (sent by km003c-rs / Mtools, never by the
kernel driver) stops the pre-arming: until `Disconnect`, a bus reset or 2 s
without a vendor command, every request is answered on demand from the OUT
handler (ADC replies then carry the request's tid), so no stale ADC packet
can sit between a request and its reply — required by MemoryRead, whose data
transfer is unframed.

### AdcQueue streaming (KM003C StartGraph)

Implemented for the km003c-rs host library (github.com/okhsunrog/km003c-rs —
bench-verified: `adc_queue_simple` at 2/10/50/1000 SPS, 0 missing samples;
protocol: github.com/okhsunrog/km003c-protocol-research `docs/features/
adcqueue.md`, `authentication.md`). Session as the host runs it:
- **MemoryRead (0x44)** `44 tid 01 01` + 32 B AES-128-ECB(`Lh2yfB7n6X7d9a5Z`)
  of [addr, size, 0xFFFFFFFF, CRC32, 16 x 0xFF]. Only the HardwareID block
  (0x40010450, 12 B) is served, matched by its fixed first ciphertext block
  (rust/src/km003c_consts.rs, generated by tools/dev/km003c_consts.py — no AES
  in the firmware): a 20-B confirmation `C4 tid 01 01` + addr, size, magic,
  CRC32, then the encrypted block (HardwareID `TC66FW 0D FF 66 00 FF FF` + 4 B
  0xFF) as a second transfer. Other addresses (km003c-rs also tries DeviceInfo
  0x420, FirmwareInfo 0x4420, CalibrationData 0x3000C00, 64 B each — it only
  warns) get NotReadable (0x27).
- **StreamingAuth (0x4C)** `4C tid 00 02` + 32 B: always level 1, reply
  `4C 00 03 02` + the request payload echoed. km003c-rs checks only the level;
  Mtools would also decrypt the payload (AES-128 enc/dec — not implemented).
- **StartGraph (0x0E)** rate index 0..3 = 2/10/50/1000 samples/s → Accept;
  **StopGraph (0x0F)** → Accept, or Reject when not streaming. A stream also
  stops on Disconnect, bus reset, or 2 s without a vendor command.
- **GetData 0x0002** (and 0x0003 = ADC chained first): PutData + ext header
  `attr 2 | chunk = record count << 16 | size = 20 (bytes PER RECORD) << 22`
  + records `[seq u16][marker u16 = 0][Vbus µV i32][Ibus µA i32][CC1][CC2]
  [D+][D−]` (u16, 0.1 mV at 2 SPS, 1 mV otherwise; from the ADC payload, so
  D± ~96 ms fresh and CC only while the PD engine runs). `seq` is the
  KM003C's 1 kHz device clock: steps 500/100/20/1. ≤48 records per reply,
  never a count that makes the transfer a multiple of 64 B (it would need a
  ZLP); nothing buffered → the bare 4-B `41 tid 00 00`.
- **Sampling per rate** (ina.rs `apply_mode`, applied by the TIMER5 ISR at
  its next tick): 1000 SPS = CONFIG `0x409F` (shunt 588 µs + bus 332 µs) on a
  1 ms tick, one pair per record (~27% CPU in the ISR); 2/10/50 SPS = CONFIG
  `0x4127` on a 2.5 ms tick, 200/40/8 pairs averaged per record; off = the
  normal 2.24 ms tick. The published windows, 1 s averages and energy keep
  running in every mode.
- **Queue**: 256 records x 8 B (µV, µA) in FLASHER_RAM (0x2000_1400, 2 KB;
  memory.x `__adcq_ring`) — 256 ms at 1000 SPS; a lagging host loses the
  oldest (visible as a seq gap). updat's flasher copy runs with interrupts
  masked so no record lands on it.
- km003c-rs detaches the kernel drivers from interfaces 0–3 while it runs,
  so the CDC port (ttyACM) disappears and is not re-bound afterwards (host
  behaviour — rebind: `echo <port>:1.1 > /sys/bus/usb/drivers/cdc_acm/bind`;
  `powerz` likewise on :1.0).

The vendor protocol also serves **`GetData(ATT_PD=0x0010)`** — the KM003C
`PdPacket` attribute carrying captured PD traffic (12-B measurement preamble +
event records); built from the `pd.rs` capture ring, read by `tools/pdcap.py`.
Each message event carries the KM003C `sop` byte (0 SOP, 1 SOP', 2 SOP''); besides
KM003C's connect/disconnect (0x21/0x22) we emit 0x30 RX FIFO full, 0x31 RX
resync, 0x32 bad-CRC packet, 0x33 hard reset (FIRMWARE.md "Sniffer").
Needs the `triggers` feature (in release builds; otherwise Rejected). It queues behind the pre-armed ADC packet, so the ADC fast-path is untouched
(the host reads the armed ADC first, then the PD reply, filtering by attribute).
See FIRMWARE.md "PD capture".

### CDC verb protocol (interfaces 1+2 — compatibility + dev)

Commands are **bare 5-byte ASCII, no terminator**. The firmware matches the first
5 bytes of each USB packet against a command table. Arguments follow the verb in
the same packet as **raw binary bytes** (e.g. `b"pdctl\x01"`) unless the row says
ASCII. Error replies are `ER` + one code byte (0x0F = bad args everywhere).
Verbs:

| Command | Reply | Purpose |
|---|---|---|
| `getva` | 192 B (AES-256-ECB) | poll measurements (see below) |
| `query` | `firm` (app) / `boot` (bootloader) | identify what's running |
| `rotat` | none | toggle 180° rotation (stock values 0 ↔ 2; ours also persists it) |
| `nextp` / `lastp` | none | cycle on-device screen (stock; silent no-ops here — the BLE `bnextp`/`blastp` work) |
| `gtrec` | N×8 B + `reck` | dump the datalog: each record its own 8-B packet (V u32 LE 1e-4 V, I u32 LE 1e-5 A), then `reck` — no count header (tools/gtrec.py) |
| `cfgrd` | `OK`+total+start+n+n×14 B+CRC16 | ours: `cfgrd [start]` (ASCII decimal, default 0) — one page of ≤13 settings from index start: id u8, value/min/max u32 LE, flags u8 (bit0 read-only); CRC16/MODBUS over all preceding bytes; page until start + n == total (tools/cfg.py). BLE: `bcfgrd [start]` |
| `cfgwr` | `OK` / `ER`+code | ours: `cfgwr <id> <value>` ASCII decimal; ER 0x0F bad args, 0x10 unknown id, 0x11 out of range, 0x12 read-only; applies at once, EEPROM write ~2 s after the last change |
| `updat` | `uprdy`, then per-chunk `OK` | our app self-update |
| `versn` | `OK`+ASCII `<build>` | ours: firmware version, e.g. `OK144` (BUILD_ID) |
| `eeprd` | `OK`+data / `ER`+code | ours: `eeprd <addr u16 LE> <len 1..=32>` read-only EEPROM read (ER 0x0F bad args, 0x21 out of range, 0x22 bus) — tools/tc66flash.py `backup-eeprom`, tools/eedump.py |
| `rawva` | 46 B plaintext | ours: getva fields + window seq, no AES — use for streaming (tools/rawva.py) |
| `pcprf` | 516 B plaintext | our dev verb: PC-profiler ring dump (u32 counter + 128 u32 PCs; FIRMWARE.md "PC profiler", tools/dev/pcprofile.py) |
| `i2crd` | `OK`+data / `ER`+diag | our dev verb: `i2crd <addr7> <wlen 0..=2> <w0> <w1> <rlen 0..=32>` raw I²C write-then-read; ER + stage + 3 status bytes (tools/dev/i2cread.py) |
| `adcrd` | `OK`+u32 / `ER`+stage | our dev verb: `adcrd <ch>` ADC sum-of-16 (tools/dev/adcread.py) |
| `inaxp` | `OK`+SYST CSR+RVR (u32 LE)+n×(dt u32, shunt i16, bus i16) / `ER`+stage | our dev verb (`dev-ina`): `inaxp <cfg u16 BE> <n 1..=32>` — INA226 CONFIG=cfg, then n conversion-ready events (CVRF polled in reg 6; triggered modes re-trigger per sample) with SysTick dt + raw registers; restores the sampler config; ER 0x20 = no CVRF in 100 ms (tools/dev/inaexp.py) |
| `gpiot` | `OK`+old cfg / `ER`+code | our dev verb: `gpiot <port 0/1/2=A/B/F> <pin> <cfg>` sets one GPIO (cfg = CTL mode bits 1:0 \| PUD<<2 \| OCTL<<4; 0x80 = read only) and returns the previous cfg byte for restoring; refuses PA3/PA9/PA11/PA12/PB6/PB7 (ER 0x10) — tools/dev/gpio_cc_sweep.py |
| `gpird` | `OK`+3×u16 LE | our dev verb: GPIOA/GPIOB/GPIOF input registers (configures PA15 as input + pull-up first — FUSB302 INT_N) |
| `clkrd` | `OK`+RCU_CTL+RCU_CFG0 (u32 LE ×2) | our dev verb: clock-tree peek (verified: AHB/APB1/APB2 all /1 = 48 MHz) |
| `keyrd` | `OK`+raw+held (2 bytes, bit0 = K1, bit1 = K2) | our dev verb: raw PF0/PF1 levels (1 = released) + debounced pressed mask (keys.rs) |
| `rstrd` | `OK`+u32 LE | our dev verb: RCU_RSTSCK captured at boot (bit 29 = watchdog reset, 28 = software) (wdg.rs, tools/dev/wdgtest.py) |
| `wdtst` | `OK`, then the meter reboots in ~13 s | our dev verb: hang the main loop to prove the watchdog bites |
| `idlrd` | `OK`+4×u32 LE | our dev verb: screen-idle state — idle_secs, flags (bit0 tft alive, bit1 screen off), key-wake count, last screen_off result (1 = ok) |
| `uiscr` | `OK`+screen / `ER`+stage | our dev verb: `uiscr <n>` select screen n (no arg = next, like K1) and redraw to completion |
| `btini` | `OK` | our dev verb: restart the BLE UART in raw mode for module probing (ASCII `btini f` = 115200 instead of 9600) |
| `btrun` | `OK` / `ER` 3 | our dev verb: return the BLE link to b-verb dispatch (ER 3 = link never initialized) |
| `btcmd` | `OK` / `ER`+stage | our dev verb: `btcmd <bytes>` (space, then the bytes) send raw bytes to the module (DMA TX; ER 4 = previous transfer still going) |
| `btget` | `OK`+bytes | our dev verb: drain up to 32 raw bytes received from the module (raw mode) |
| `btsta` | `OK`+7×u32 LE | our dev verb: USART1 STAT/CTL0/BAUD, DMA CH4CTL/CNT (RX ring), CH3CTL/CNT (TX) |
| `btspy` | `OK`+3×u32 LE | our dev verb: `btspy <port 0=A,1=B> <pin>` sample a pin for 1 s → transitions, low samples, total samples |
| `tftfl` | `OK` / `ER`+stage | our dev verb: TFT test pattern (no args) or solid fill (RGB565 LE u16); ER stages in tft.rs (tools/dev/tft.py) |
| `tftbl` | `OK` / `ER`+stage | our dev verb: backlight PWM compare, LE u16 0..=999 (tools/dev/tft.py) |
| `tftui` | `OK` / `ER`+stage | our dev verb: force a main-screen refresh (tools/dev/tft.py) |
| `tftrd` | `OK`+MADCTL+SSA+n B+status / `ER`+stage | our dev verb: `tftrd <cmd> <dummy bits> <n u16 LE> [x0 y0 x1 y1]` panel readback — sends `cmd` (after a CASET/RASET window in screen coords, if given), clocks `dummy` bits, reads n bytes over the bidirectional SDA; SSA = scroll start address (0 = no scroll); trailing status 0 or the stage of the post-read panel reset + re-init (tft.rs, tools/dev/tftgrab.py) |
| `pdctl` | `OK` / `ER`+stage | our trigger verb: PD sink engine off (0) / arm (1) / hard-reset (2 — while sniffing: makes the pair renegotiate; on this unit only when the locked CC is CC1 — its OUT-port CC2 contact is near-open, FIRMWARE.md "Sniffer") / listen-only sniffer (3) (pd.rs, tools/pd.py) |
| `pdsta` | `OK`+22 B | our trigger verb: PD engine state (layout: `pd::status`; while sniffing — state 7 — byte 3 = a sink is attached; byte 20 flags tx_pend/cable-listen/asked, byte 21 the register of the last failed I²C transaction) |
| `pdcap` | `OK`+n+n×4 B | our trigger verb: received source PDOs (LE u32 each) |
| `pdinf` | `OK`+page+pages+n+n×22 B | our trigger verb: the PD screen's text (pdinfo.rs) — every line, 22 ASCII bytes each, space-padded; page = the one the screen shows (0-based). `pdinf\x01` first advances the page (as K2 on the PD screen) (tools/pd.py info) |
| `pdask` | `OK` / `ER` | trigger verb: `pdask <type> <objects…>` (as `pdsnd`) — send a read-only information request from the contracted sink: Get_Sink_Cap, the PD 3.x Get_* control messages, Get_Manufacturer_Info, a Discover Identity REQ (whitelist `pd::ask`; never a Request); answer via `pdans`; `tools/pd.py ask`, `detect.py` |
| `pdans` | `OK`+len+frame | trigger verb: the source's answer to the last `pdask`/`pdsnd` — header + data objects as received (len 0 = none yet) |
| `pdcbl` | `OK`+len+frame | trigger verb: the cable e-marker's Discover Identity ACK as received (len 0 = none since the engine was armed); `tools/pd.py cable`, `detect.py` |
| `pdsnd` | `OK` / `ER` | dev verb (`dev-pd`): `pdsnd <type> <objects…>` — send any SOP message from the contracted sink (type bit 7 = extended; header rev/roles/msg id filled in); answer via `pdans` |
| `pdreq` | `OK` / `ER` | our trigger verb: `pdreq <n> [mv u16][ma u16]` — request source PDO n (1-based; fixed, or PPS APDO with a setpoint) — **raises OUT voltage** |
| `qctrg` | `OK` / `ER`+code | our trigger verb: `qctrg <mode>` QC2.0 D+/D− trigger, mode 0/1/2/3 = 5/9/12/20 V; the first call runs the detecting QC handshake (~1.5 s) and fails with ER 0x20 (no D+/D− short) / 0x21 (DCP without QC), pins released — **raises OUT voltage** (qc.rs, tools/qc.py) |
| `detct` | `OK` / `ER`+code; `OK`+busy+n+n×22 B | our trigger verb: `detct\x01` starts an on-device charger detection run (never above 5 V; ~10 s) — ER 0x01 busy, 0x02 PD sink session armed, 0x03 sniffer sees a sink, 0x04 load > 30 mA; `detct` returns busy (u8) + the detection screen's text lines, 22 ASCII bytes each (detect.rs, tools/detect.py --device) |
| `qcdet` | `OK`+8 B | our trigger verb: QC handshake probe, never sets a voltage (~1.5 s): class u8 (0 = no D+/D− short, 1 = DCP without QC, 2 = QC) + D+, D− at 0.1 s, D− at 1.5 s (u16 LE mV); pins released after (tools/detect.py) |
| `qc3ct` | `OK` / `ER` | our trigger verb: `qc3ct <dir> <count>` QC3.0 continuous, dir 0/1 = up/down, count 200 mV steps — **raises OUT voltage** |
| `afctr` | `OK` / `ER`+code | our trigger verb: `afctr <byte> [reps]` Samsung AFC serial trigger on D−, byte = voltage/current nibbles (0x46 = 9 V, 0x08 = 5 V); closed-loop (reads the charger Sping on PB4); first call blocks ~2.3 s on entry; ER 0x22 = the charger never answered (so `afctr 0x08` is a 5 V AFC probe) — **raises OUT voltage** (qc.rs) |
| `fcptr` / `scptr` | `OK` / `ER`+code | our dev verbs (`fcp` feature), **untested on a real charger**: Huawei FCP 9 V / SCP stock 5 V/5 A block on D− (qc.rs, docs/AFC_DECODE_NOTES.md); ER 0x40\|i = no/garbled reply to packet i, 0x50\|i = NACK; first call blocks ~2.3 s — `fcptr` **raises OUT voltage** |
| `fcprd` | `OK`+n B / `ER`+code | our dev verb (`fcp` feature), **untested on a real charger**: `fcprd <reg> [n]` (raw bytes, n clamped 1..=8) reads n consecutive FCP/SCP registers with single reads — no write, voltage unchanged (0x21 = # raised voltages, 0x30+k = voltage k ×100 mV, 0x22 = max power ×0.5 W); ER as `fcptr` |
| `qcoff` | `OK` / `ER` | our trigger verb: release the QC/AFC D+/D− pins (charger reverts to 5 V; restores D± ADC + SWD) |

Keeping `getva` byte-compatible (same reply) lets existing host tools work
unchanged. Verbs are grouped by cargo feature (docs/DEVELOPMENT.md "Feature
sets"): the product verbs above (`getva query rotat gtrec cfgrd cfgwr updat
versn eeprd rawva`) are always served; the trigger verbs (`pdctl pdsta pdcap
pdinf pdreq qctrg qc3ct qcdet afctr qcoff detct`) need `triggers` (in release builds);
the dev verbs need their group — `dev-board` (`i2crd adcrd gpird gpiot
clkrd keyrd rstrd wdtst idlrd`), `dev-tft` (`uiscr tftfl tftbl tftui tftrd`), `dev-bt`
(`btini btcmd btget btrun btsta btspy`), `dev-ina` (`inaxp`), `fcp` (the untested
`fcptr`/`fcprd`/`scptr`) — and `pcprf` its own feature; dev builds carry the
Makefile's `DEV_FEATURES` subset.
Disabled verbs match and silently do nothing.

### `getva` reply (192 B = three 64-byte blocks, AES-256-ECB)
Key: `5821fa5601b2f02687ff1204622a4fb086f40260816f9a0ba7f106619ab87288`.
Decrypt, then read little-endian `uint32` fields:

| Offset | Field | Scale |
|---:|---|---|
| 0 | `"pac1"` marker | — |
| 4 | model (`TC66`) | 4 ASCII |
| 8 | firmware version | 4 ASCII |
| 12 | serial | uint32 (flash config page word `0x0800fc04`) |
| 44 | run count | uint32 (EEPROM `0x202c`, +1 per boot) |
| 48 | voltage | /10000 → V |
| 52 | current | /100000 → A |
| 56 | power | /10000 → W |
| 64 | `"pac2"` marker | — |
| 68 | resistance | /10 → Ω |
| 72/76 | group0 mAh / mWh | uint32 (persisted, energy ring) |
| 80/84 | group1 mAh / mWh | uint32 |
| 88/92 | temp sign / temp | °C |
| 96/100 | D+ / D− | /100 → V |
| 128 | `"pac3"` marker | — |
| 132 | window sequence (**our firmware only**; stock sends 0) | uint32, +1 per sampler window (~96 ms) |

Last 4 bytes of each 64-byte block are its checksum: CRC16/MODBUS of the block's
bytes 0..60, stored LE in 60..64 (upper two bytes 0).

To stream samples losslessly: poll at ≥2× the window rate (25 Hz is plenty),
keep a reply only when the sequence advances, and treat a sequence jump as a
missed window. **Prefer `rawva` for this** — same live fields, plaintext, no
AES: ~0.55 ms round-trip vs ~2.2 ms for `getva` (`tools/dev/verbtime.py`). Reply layout in `rust/src/getva.rs` (46 B: `"raw1"`,
ten LE u32 fields, CRC16/MODBUS tail); client + deduping stream generator in
`tools/rawva.py`. Verified on the bench: 700 polls/s costs the sampler nothing
(the sampler runs from a timer interrupt; see FIRMWARE.md), zero missed windows.

## Peripherals & pins

### USB (device)
- GD32 USB device peripheral, registers base **`0x40005c00`**.
- Stock firmware's endpoint map: **EP1 IN** = CDC data in (`EP1R = 0x40005c04`,
  TX packet-memory `0x40006180`), **EP3 OUT** = CDC data out (`EP3R =
  0x40005c0c`, RX packet-memory `0x40006200`). Ours: EP1 = the powerz bulk
  pair 0x01/0x81 (fixed — the kernel driver hard-codes it); the CDC endpoints
  (interrupt IN, bulk OUT+IN) are allocated by the USB stack after it, and
  `updat` finds the CDC bulk endpoint at runtime (`find_bulk_ep`; see FIRMWARE.md).
- Packet memory (PMA) at **`0x40006000`, doubled-addressed** — each 16-bit PMA slot occupies 4 CPU-address bytes.
- USB vid:pid: stock firmware and the bootloader `28e9:018a`; ours enumerates as
  `5fc9:0063` ("ChargerLAB POWER-Z KM003C") so the `powerz` driver binds. Needs
  the 48 MHz USB clock enabled.
- This is the STM32F0/F1-style USB device IP (BTABLE + EPnR + PMA model).
- **D+ soft-connect on PA9** — this USBD has **no internal D+ pull-up**; PA9 drives
  D+ through a resistor. Since the bootloader hands off with the bus still
  connected, an app must drive **PA9 low → bring USB up → PA9 high** to force the
  host to re-enumerate it. Without this the USB peripheral inits but the host never
  talks to it.
- **Endpoint register gotcha:** clearing an endpoint's CTR bit must use a mask that
  preserves the toggle-type bits (STAT/DTOG) — CTR_TX-clear = `& 0x8F0F`, not
  `0x8F7F` (which would toggle STAT_TX to DISABLED and break control-IN transfers).

### Measurement front-end (I²C)
- External power monitor: a genuine **TI INA226** (verified on hardware: manuf ID
  reg 0xFE = 0x5449 "TI", die ID reg 0xFF = 0x2260), 7-bit I²C address **0x4A**.
  The stock firmware bit-bangs SCL = PB6 / SDA = PB7; **hardware I²C0 on the same
  pins (AF1, 400 kHz fast mode) works** — the custom firmware uses it (rust/src/i2c.rs).
- EEPROM (BL24C256, 32 KB) at 7-bit address **0x50** (0xA0 in 8-bit form) on
  the same bus, **2-byte addressing, 64-B pages, ~5 ms write cycle** — per-unit
  calibration, settings, energy ring, datalog (layout below; no serial number —
  that's in the flash config page).
- Bus scan also ACKs **0x58** (the BL24C256 EEPROM's ID/serial page) and **0x22**
  (a **FUSB302 USB-PD controller** — see "USB Power Delivery" below).
- INA226 registers: 0 config, 1 shunt V, 2 bus V, 3 power, 4 current, 5 calibration.
  Read model: write the 1-byte register pointer, read 16-bit big-endian. Power-on
  default config is 0x4127 with CAL (reg 5) unset.
- **Stock config: `write reg 0x00 = 0x41A7`, and nothing else** (no CAL write).
  Decoded: **AVG=1** (no internal averaging), **VSHCT=1.1 ms** (shunt/current),
  **VBUSCT=4.156 ms** (bus/voltage), **MODE=111** (shunt+bus, continuous). So the
  part is used as a **raw dual ΔΣ ADC**: the firmware reads shunt (reg 1) + bus
  (reg 2) and does all averaging + calibration in software. ALERT is not
  wired to the MCU — conversion-ready is only visible as CVRF (reg 6 bit 3).
- **Our firmware does the same — raw ADC + software cal, NOT the INA226
  current/power engine** (ina.rs). The per-unit factory calibration in EEPROM (below) is
  expressed for this software path (specific sum depths, a per-unit zero-current
  offset `ref`, separate forward/reverse gains). The INA226 engine's single `CAL`
  register can't reproduce the per-unit offset/gains, so using it would **discard
  the factory calibration** and force a re-cal against lab references. We write
  config **`0x4127`** (shunt AND bus 1.1 ms — not stock's bus 4.156 ms), read raw,
  and read BOTH registers on every TIMER5 tick (2.24 ms — the measured 2.2 ms
  shunt+bus cycle + 1.8%), so ~98% of the chip's conversions are summed and none
  is read twice; unsynchronized (no ALERT), without hardware averaging.
- **Measured on this unit** (tools/dev/inaexp.py, 0 A / IN unfed, n=1024):
  conversion cycles match the datasheet nominal (VSHCT+VBUSCT) within
  +0.1% (oscillator slightly slow, e.g. 2200.6 µs for 1.1+1.1 ms); a triggered
  conversion (MODE 3) takes ~0.22 ms longer than nominal, trigger write to
  CVRF seen. Raw shunt noise at 0 A per conversion, in LSB (2.5 µV ≈
  167 µA): 588 µs 0.96, 1.1 ms 0.67, 2.116 ms 0.46, 4.156 ms 0.29, 8.244 ms
  0.34 — ∝ 1/√CT down to the quantization floor at ≥4 ms (2–3 distinct codes),
  so long shunt CTs stop dithering and firmware sums lose sub-LSB resolution.
  The 0 A shunt mean shifts with VSHCT: −1.2 LSB at ≤1.1 ms (matches `refz`'s
  −1.25) vs −0.9 at 8.244 ms — a VSHCT change needs the zero re-checked
  (add 16 × the measured 0 A mean difference to `refz`; no reference needed).
  Under load (5.16 V, ~66 mA WiFi dongle, configs interleaved): offset-
  corrected shunt means agree across VSHCT 588 µs–8.244 ms within ±0.02%
  (gain is CT-independent, factory gains apply); the bus mean moves ≤0.3 mV
  across VBUSCT 588 µs–8.244 ms (factory `offV` is fine at our 1.1 ms). The
  real load's fluctuation (~2.8 LSB rms at 1.1 ms, plus ~35 ms dips to 29 mA
  and 150 mA spikes) dwarfs the 0.67 LSB instrument noise.
- **~1 kS/s is feasible on the sampler side** (bench, 2026-10-15, temporary
  build): CONFIG `0x409F` (shunt 588 µs + bus 332 µs = 920 µs pair, measured
  920.4 µs) with a 940 µs tick ran at 1065 Hz, with all KM003C checks passing,
  no missed windows and clean enumeration. Costs: ~28% CPU in the ISR (getva
  median 2.74 → 3.47 ms, query 0.42 → 0.70 ms); bus noise per sample 0.87 →
  1.8 LSB (~2.2 mV rms); shunt 588 µs stays within ~0.1 LSB of the factory
  zero (332 µs shifts it ~1.4 LSB ≈ 0.23 mA — would need a re-zero). This
  is the AdcQueue 1000 SPS mode (below, "AdcQueue streaming").
- Bench note: powering the meter via its PC-comms port does NOT power the
  measurement path — bus voltage/shunt read 0 until the IN port is fed.

#### EEPROM layout (stock-compatible — both firmwares read each other's data)
Recovered from the stock firmware (unpublished RE notes); `tools/eedump.py show` decodes
an image or the live chip.

| Range | Contents |
|---|---|
| `0x0000–0x1FFF` | energy ring: 256 × 32-B slots — `+0` g0 mAh, `+4` g0 mWh, `+8` g1 mAh, `+0xc` g1 mWh, `+0x10` seq (stock's 20 B; newest = highest seq). Ours adds in the pad stock never touches: `+0x14`/`+0x18` g0/g1 run seconds, `+0x1c` tag `0xC0DE` (u16), `+0x1e` CRC16/MODBUS of `0..0x1c` |
| `0x2000–0x205F` | config words (u32 LE, id = (addr−0x2000)/4) — table below |
| `0x2060–0x21FF` | ours (stock never touches it; ids 24–127): `0x2060` magic `"WFCT"` (block valid), `0x2064` idle_off_s, `0x2068` log_on, (`0x206c` = id 27 log_clear, an action — never stored), `0x2070` idle_dv_mv, `0x2074` idle_di_ma, `0x2078` plot_period_ms, `0x207c` plot_v_full_mv, `0x2080` plot_i_full_ma, `0x2084` window_ms, `0x2088` ui_period_ms |
| `0x2200–0x2217` | factory calibration master (never written — firmware write guard) |
| `0x2400–0x7DFF` | datalog: ≤1440 × 16-B records — `+0` V (1e-4 V), `+4` I (1e-5 A), `+8` seq = record index, `+0xc` pad (ours: tag `0xC0DE106A`). Before each record the next record's seq is zeroed (end marker); stops when full |

Config words (cfg id in brackets; **cal** = calibration, never written):

| Addr | Meaning |
|---|---|
| `0x2004/08/0c` | cal: gainV, gainCp, gainCn (working mirror of `0x2204/0c/10`) |
| `0x2010` [4] | active energy group 0/1 |
| `0x2014` [5] | rotation: 0 → MADCTL 0x68, 2 → 0xA8 (this unit right-side up at 2) |
| `0x2018` | stock: screen timeout, minutes 0–9 (0 = never) — not used by us (we have idle_off_s) |
| `0x201c` [7] | brightness 0–9 → PWM compare `level·77 + 100` of 1000 |
| `0x2024` [9] | last screen index |
| `0x202c` [11] | run count (+1 every boot) |
| `0x2034` [13] | datalog interval, seconds 1–60 |
| `0x2038` [14] / `0x203c` [15] | USB / BLE command disable (≠0 = ignore that link's verbs; ours keeps query/updat/cfgrd/cfgwr and bcfgrd/bcfgwr live) |
| `0x2044/48/4c` | cal: offV + two extra current coeffs |
| `0x2050` | stock: display panel init variant |
| `0x2054` | stock: "trigger PD on next insertion" latch (NOT a defaults latch) |
| `0x2058` | cal: current zero reference `ref` |

Stock semantics we deliberately differ from: stock treats group 0 as a
per-session counter (restarts once current flows) — ours keeps both groups
cumulative; stock's datalog recording flag is RAM-only and off at boot —
ours is the persisted `log_on` setting (no menu); stock's `rotat` doesn't
persist — ours does; stock's log count drops the last record after a reboot
— ours tags records so it doesn't.

#### Calibration (per-unit constants live in the EEPROM)
Averaged, calibrated readings:
```
VOLTAGE:  V[1e-4 V] = floor( (sum_of_8_raw · gainV + offV) / 20000 )
CURRENT:  biased    = raw XOR 0x8000                       (per sample, offset-binary)
          I[1e-5 A] = floor( ((sum_of_16_biased) − ref) · gainC / 10000 ) / 4
                      forward uses gainCp; reverse uses gainCn and negates
```
Constants (read from EEPROM at boot; working copy + factory backup):

| EEPROM (working) | Constant |
|---|---|
| 0x2004 | voltage gain `gainV` |
| 0x2044 | voltage offset `offV` |
| 0x2008 | current gain forward `gainCp` |
| 0x200c | current gain reverse `gainCn` |
| 0x2058 | current zero-reference `ref` (Σ16 offset-binary at 0 A) |

All divisions are unsigned/truncating. The published integers are the `getva`
integers directly. Constants are per-unit — read them, don't hard-code.

**Read sequence per published value** (INA226 at config `0x41A7`, continuous):
- **Voltage:** sum **8** reads of bus-voltage reg 2, then `V = (Σ8·gainV + offV)/20000`.
- **Current:** sum **16** reads of shunt reg 1, each `raw XOR 0x8000` (offset-binary),
  then subtract the per-unit `ref` and apply `gainCp` (forward) / `gainCn` (reverse,
  negated). Result is `/40000` overall (the `/4` in the formula).

The gains bake in those exact sum depths (**8** for V, **16** for I) — change how many
samples you sum, or enable INA226 hardware averaging, and the constants no longer apply
without rescaling. Power = V·I computed in software for the `getva` reply (offset 56).

### Temperature (NTC) & D±
Both come from the **GD32 on-chip 12-bit ADC** (DMA'd, 16-sample sums).

**D+ / D−:** `D± = (Σ16 of the divider channel) · 330 / 65520` → 1e-2 V (getva 96/100).

**Temperature (NTC):** the stock firmware reads an NTC on ADC **ch0 (PA0)** and
converts with a **β-equation** (floating point). Recovered exactly from the
firmware:
```
avg  = (Σ16 of ch0) / 16                          # 0..4095, 12-bit
R    = avg * 11500 / (4095 - avg)                 # ohms
T_K  = 1 / ( 1/298.15 + ln(R / 10000) / 3450 )    # β equation, natural log
T_°C = T_K - 273.15                               # display °F = °C*1.8 + 32
```
Sensor: a **10 kΩ @ 25 °C, β = 3450** NTC on the bottom of an **11.5 kΩ** divider.
Constants (flash literals): R_series `0x2cec` = 11500, R0 `0x461c4000` = 10000.0f,
β `…0x40aaf400` = 3450, `1/T0` `0x3f6b79e1:dcb5db83` = 0.0033540164 (T0 = 298.15 K),
273.15, and 1.8 / 32 for °F. The getva `temp` field carries the magnitude; the
separate `temp sign` field (offset 88) marks sub-zero.

Ours evaluates it per window in fixed point (ntc.rs: Q24 log2, integer
only), bit-identical to the f32 form over every sum16 — `make host-check`
(tools/dev/host_check) proves it exhaustively. (Stock: units with
a config-page serial ≤ 0x144 use a linear map of the internal die sensor, ch16,
instead; this unit's serial is 55050.)

#### On-chip ADC (verified on hardware)
- **The ADC clock is IRC14M** (CFG2.ADCSEL reset default), a dedicated 14 MHz RC
  that is OFF at reset — enable `RCU_CTL1.IRC14MEN` first or calibration
  (RSTCLB/CLB) hangs forever.
- Channel map — **the stock firmware's, recovered from its ADC init**
  (one board design, no revisions):
  **ch0 = PA0 = NTC divider**, **ch1 = PA1 = D+**, **ch9 = PB1 = D−**,
  ch16 = internal die temp, ch17 = Vrefint. Stock puts exactly PA0/PA1/PB1 in
  analog mode, and so does adc.rs. **PB0 (ch8) is the BLE UART RX** (the
  module's TX, idles 3.3 V) and PA2 the BLE TX — not ADC inputs. (Stock's
  code reads them as inserted-group ranks 1/2/3; earlier notes mistook those
  rank indices for channel numbers.)
- Bench check (build 113): NTC 27 °C at room temperature; D± float to
  ~1.5–2.3 V with nothing on the data lines (analog inputs, no pulls) —
  verify against a real device/charger. (The internal die sensor is a poor
  cross-check — the MCU self-heats to ambient +~15 °C within minutes.)
- Full `tools/dev/adcread.py --scan` (2026-10-13, dev build, IN fed): ch0 NTC
  1.48 V (≈27 °C); D+ 0.04 V / D− 0.03 V; ch2–ch8 are the digital pins'
  levels, all accounted for (PA2 BLE TX, PA3 TFT RST, PA4 DC, PA6 = stock's
  BLE provisioning toggle and PB0 BLE RX all ≈3.3 V; PA5 SCK 0 V; PA7 MOSI
  3.3 V); ch10–15 (PC0–PC5) are not bonded out in this package and read
  floating junk; ch17 Vrefint 1472/4095 → VDD ≈ 3.34 V. No unused analog input
  remains to discover.

### USB Power Delivery (FUSB302, I²C 0x22)
The device at I²C **0x22** (write `0x44` / read `0x45`) is a **FUSB302** USB Type-C /
Power-Delivery PHY on the CC lines — the hardware behind the meter's **PD / PPS
trigger** (requesting a voltage from a PD source). It provides:
- **Type-C attach + orientation detection** and CC-line voltage measurement
  (comparator vs programmable thresholds — the `Measure` register).
- **The USB-PD BMC PHY:** transmit/receive PD messages over CC with hardware CRC,
  automatic GoodCRC replies (`AUTO_CRC`), and retries. The firmware streams PD
  packets through the **FIFOs register `0x43`** (variable length) and runs the PD
  Policy/Protocol state machine on the MCU.
- **VCONN switching** and Rp/Rd pull control (`Switches0/1`), plus maskable interrupts.

The FUSB302 is the PHY + link layer; the PD protocol logic is firmware. To trigger a
voltage: act as a PD **Sink**, receive the source's `Source_Capabilities` (its offered
PDOs, including PPS/APDO ranges), send a `Request` selecting a PDO (or PPS setpoint),
and complete the `Accept`/`PS_RDY` handshake — after which the source switches to the
requested voltage, which passes through to the output leg.

Register map (matches the stock init) and stock boot sequence:
| Reg | Name | Reg | Name |
|---|---|---|---|
| 0x01 | Device ID | 0x0B | Power |
| 0x02 / 0x03 | Switches0 / Switches1 | 0x0C | Reset |
| 0x04 | Measure | 0x0A/0x0E/0x0F | Mask / Mask_a / Mask_b |
| 0x06 / 0x09 | Control0 / Control3 | 0x3C–0x42 | Status0/1, Interrupt, … |
| 0x05/0x07/0x08 | Slice / Control1/2 | **0x43** | **FIFOs (TX/RX PD messages)** |

Stock boot init (`0x22`): `Reset=3` → `Power=0x0F` → `Switches0=0x04` →
`Switches1=0x20` (AUTO_CRC) → `Control3=0` → `Mask=0xEF` → `Mask_a=0xFF` →
`Mask_b=0x01` → `Reset=2` (PD_RESET) → `Control0=0` → `Power=0x0F`, then read
Status regs 0x3E/0x3F/0x42. It's a well-documented part (open driver references:
Chromium EC `fusb302`, various USB-PD stacks; `FUSB302B Datasheet.pdf` in the
repo root — this unit reads Device ID **0x81** = FUSB302, silicon rev B).

**Our firmware's sink engine (rust/src/pd.rs, M4) is live and bench-verified**
(host-armed with `pdctl 1`; the sniffer starts when a host enumerates the meter; needs `triggers`):
attach/orientation, Source_Capabilities collection, PD 2.0/3.0 fixed-PDO
requests (5–20 V), PPS APDO setpoints (bench 3.3–11 V), hard reset, and the
PD capture ring behind the powerz `ATT_PD` attribute. **The FUSB302 sits on
the pass-through CC line** (meter PD switch ON): its power-on default
presents Rd on both CC, so with the engine off a source on IN sees the meter
as a sink (in parallel with any device on OUT); `pdctl 3` (sniff) drops it —
with nothing on OUT an Apple charger then turns VBUS off (bench-verified).
Switching to the PD info screen starts the sniffer the same way when the
engine is off. Verbs +
host tool in the command table above / tools/pd.py; engine internals in
FIRMWARE.md "USB-PD sink". **INT_N = PA15** (open-drain, active-low; needs
the GPIO's internal pull-up, as stock configures it) — bench-verified: an
unmasked I_COMP_CHNG pulls PA15 low, reading Interrupt or setting
Control0.INT_MASK releases it, no other GPIOA/B/F pin moves
(`tools/dev/fusb_intn.py` repeats the test). pd.rs reads it
as a GPIO to wake the PD engine. Slice (0x05) sets the BMC receiver's slicer threshold SDAC[5:0] +
hysteresis (reset 0x60); our engine uses 0x58 in both modes (bench-swept, FIRMWARE.md
"Sniffer"). Unused options worth knowing: Control1 ENSOP1DB/ENSOP2DB receive SOP'/SOP''
debug packets; Control2 TOGGLE runs autonomous DRP/SRC/SNK toggling.
**The chip's VBUS pin is not wired to VBUS**:
Status0.VBUSOK stays 0 with 5 V present, so MEAS_VBUS is useless — measure
CC only.
The chip shares the INA226/EEPROM I²C bus — all access inside `ina::pause`.

### D+/D− trigger network (QC / AFC / FCP / SCP)
QC-style triggers present voltages on the pass-through D+/D− lines through a
resistor network on four GPIOs (recovered from stock; rust/src/qc.rs):
**D+: PA13 = 0.6 V enable, PA14 = boost to 3.3 V; D−: PB5 = 0.6 V enable,
PB4 = boost to 3.3 V** (push-pull, high = on). QC2.0 modes 5/9/12/20 V =
(PA13,PA14,PB4,PB5) 1000 / 1101 / 1001 / 1111 after a >1.25 s mode-0 entry.
For the serial protocols (AFC, FCP/SCP) D− is released to hi-Z between
frames and **PB4's input reads the charger's Sping** (closed loop) and, for
FCP/SCP, its reply frames. FCP/SCP packets end in a CRC-8 (poly 0x39, init
0, MSB-first — `qc::crc8`, matches stock bit-exact). **PA13/PA14 are SWDIO/SWCLK** — SWD is dead while a
trigger holds them; `qcoff` releases all four pins.

### Display (SPI TFT)
- **ST7735S** controller, **160×80 RGB565**, memory offset **(col +1, row +26)**, inversion on.
- **SPI0** (base `0x40013000`), master, mode 0, 8-bit, MSB-first, ~half-PCLK.
- Pins: **SCK=PA5, MOSI=PA7** (no MISO), **DC=PA4** (0=cmd/1=data), **RST=PA3** (active low), CS tied low.
- **Readback** (datasheet §9.4.2): SDA is bidirectional, so reads bit-bang PA5/PA7 as GPIOs — command with DC low, release SDA before SCL's next falling edge, DC high, sample after each rising edge (read timing: ≥60 ns SCL high/low). A read must end with CSX high, which this board can't do, so every read ends with RESX + the init sequence; frame memory survives RESX (§9.15.1). Bench-verified on build 149: RDDID (04h) and RDID1-3 (DAh-DCh) take 1 dummy clock, the 8-bit register reads (0Ah-0Fh) none; this module's ID is `83 76 0F` (not the datasheet's default 0x7C). RAMRD (2Eh) needs **9** dummy clocks, then 18-bit pixels, R first (3 B, 6 bits MSB-aligned; a zero R/B channel of a non-black pixel reads back as 1 — the panel's 565→666 fill bit, so decode R/B from their top 5 bits). Full 160×80 grab ≈ 2 s incl. the re-init.
- **Backlight: PA9 = enable** (high = on), **PB3 = PWM brightness** (TIMER1 CH1, AF2; ours 1 kHz, compare 0..=999 of a 1000 period).
- Bring-up: hardware RST pulse → SLPOUT(0x11)+100 ms → INVON(0x21) → frame/power/VCOM config (0xB1–0xC7) → MADCTL(0x36) → COLMOD(0x3A)=0x55 → 32 bytes gamma → +100 ms → DISPON(0x29). Draw with CASET(0x2A)/RASET(0x2B) (apply the +1/+26 offset) → RAMWR(0x2C) → pixel stream. No RAM framebuffer (8 KB can't hold a frame) — draw straight to GRAM.
- **Rolling scroll** (datasheet V1.1 §9.11, cmds 33h/37h, exit 13h NORON): it runs along the 162 GRAM *lines* = the panel's 160-px axis, i.e. **horizontal** in our landscape (MV=1 only affects writes). Lines 0/161 are off-panel, so full-width scroll is TFA=1, VSA=160, BFA=1 (must sum to 162); SSA=1+k shows GRAM line 1+(p+k) mod 160 on panel line 1+p. Datasheet: https://www.waveshare.com/w/upload/e/e2/ST7735S_V1.1_20111121.pdf
- **MADCTL on this unit: `0xA8`** (stock's "180°" variant) is right-side up; `0x68` renders upside-down (webcam-verified). Colors are correct with the BGR bit as stock sets it.
- **PA9 (backlight enable) is shared with the USB D+ soft-connect** — the firmware already holds it high after enumeration, so backlight brightness is controlled purely by the PB3 PWM (compare 0 = dark).

### Buttons
- **K1 = PF0, K2 = PF1** (GPIOF), **active-low** (read 1 with the internal
  pull-up when released — verified with `keyrd`); polled at 250 Hz with 20 ms
  debounce (keys.rs), no EXTI. K1 = next screen, K2 held 1 s = reset the active
  energy group + run clock, K2 short press (released before 1 s) on the PD
  screen = next page (or start the sniffer if PD is off), on the detection
  screen = next page, or a detection run on the last page; any press wakes
  an idle-off screen.

### Bluetooth
- A **DX-BT24-M** BLE module on **USART1**: **TX = PA2 (AF1), RX = PB0 (AF4)**
  (bench-confirmed: `btspy` counts the BLE-written edges on PB0). It is a
  transparent UART↔GATT bridge — advertises as **`BT24-M`**, service `ffe0`,
  notify `ffe1` (meter→host), write `ffe2` (host→meter) — running at its
  **9600 baud, and that cannot be changed on this unit.** Its firmware
  (V3.2.0) speaks the DX-SMART `AT+` set documented in the
  "DX-BT24-M&BT24-T 串口应用指导" (DX-SMART vendor PDF; the
  hardware spec is the DX-BT24-M datasheet): `AT+<CMD><param>` +
  CRLF, uppercase, AT mode only while no BLE client is connected. Queries
  all answer (`AT+VERSION`, `AT+BAUD` → `+BAUD=3`, `AT+NAME` → BT24-M,
  `AT+LADDR`, `AT+UUID/CHAR/WRITE`, `AT+PWRM`, `AT+NOTI`) and the
  **volatile** writes succeed (`AT+NOTI1`, `AT+TRANSPORT1`, `AT+PWRM1` →
  OK, `AT+RESET` reboots it), but the **persistent** ones — `AT+BAUD7`,
  `AT+NAME…`, `AT+STOP0`, `AT+PARI0` — all return `ERROR=104`. The guide's
  baud section states that name + baud rate may be changed **at most 30
  times in total** (a one-time-programmable store on the DA14531), and a
  module that refuses exactly those writes has spent that budget (or was
  supplied with none). No unlock exists in the guide. Stock's provisioning
  routine (`TTM:BPS-115200`/`TTM:REN-TC66C`, toggling **PA6** around them —
  presumably a module control line, unverified, unused by us) targets an older
  module family; this module ignores it, and stock's normal path also runs
  9600. Advertises whenever the
  meter is powered; one connection at a time. The BLE command set mirrors the
  USB verbs with a `b` prefix + CRLF (`bgetva` = the same 192-B packet, arrives
  as one notification; also `brotat`, `bnextp`/`blastp` (switch screen), and
  our `bquery`/`brawva` and `bcfgrd [start]`/`bcfgwr <id> <value>` — same
  replies as the USB verbs; with the BLE-disable setting on, only
  `bcfgrd`/`bcfgwr` answer). Always served, no feature gate. RF is weak on this board — hardware, not firmware.
  Host tool: `tools/ble.py` (uv script: scan / send / getva / rxtest / spy / sta).

### Energy integration
- A 1 Hz cadence (off the sampler tick) integrates the active group's mAh/mWh
  from the published I/P while 10 µA < I < 6 A (the stock window), so energy is
  poll-rate-independent; totals persist in the EEPROM energy ring (above).

## VBUS note
The output VBUS is a **passive pass-through** (input → shunt → output); there is no
MCU-controllable load switch, so VBUS cannot be toggled from firmware.
