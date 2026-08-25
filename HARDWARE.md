# TC66 hardware reference

Board-level reference for writing firmware for the RuiDeng TC66 / TC66C USB-C
power meter. Everything here is what the firmware needs to drive the board.

## MCU

- **GigaDevice GD32F150G8** — ARM Cortex-M3, QFN28.
- **64 KB flash**, **8 KB SRAM**.
- Core clock as configured by the stock boot: **48 MHz** (PLL ×12 from HSI/2; HSI = 8 MHz). Part max is 72 MHz. USB full-speed needs a 48 MHz USB clock.
- Debug access via **SWD** (see "Debug" below). The chip is readout-protected.

## Memory map

| Region | Range | Notes |
|---|---|---|
| Bootloader | `0x08000000 – 0x08002c00` (11 KB) | **Leave untouched.** Provides the flash-update path and flash helper routines we reuse. Never erased by an app flash. |
| **Application (our code)** | `0x08002c00 – 0x0800fc00` (52 KB) | Where our firmware lives and is flashed. |
| Config page | `0x0800fc00 – 0x08010000` | Per-unit config (serial, cal flag, etc.). **Do not erase/overwrite.** |
| SRAM | `0x20000000 – 0x20002000` (8 KB) | |

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
- Both are also readable by the app (EXTI on GPIOF).

## Flashing an image

Two paths:

### 1. Bootloader path (needs K1)
1. Hold **K1** while plugging in USB → the board enumerates as a CDC serial device in bootloader mode.
2. Encrypt the raw app: `tools/encrypt.py app.bin app.enc.bin` (AES-256-ECB, image key baked in; pads to 51840 B).
3. `tools/flash_image.py app.enc.bin` — sends `query`→`boot`, `update`→`uprdy`, then 810 × 64-byte chunks (the bootloader decrypts + writes each), then the board restarts.

This is the **recovery path** and the way to flash the very first app. Always works.

### 2. App self-update path (`updat`, no K1)
Once our app implements the `updat` command, later builds flash over USB with no
button press:
1. `tools/app_update.py app.bin` — sends `updat`; the app copies a small flasher into SRAM and streams **plaintext** 64-byte chunks to it; the flasher programs them and resets into the new app.

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
  This is the primary debug loop — see README.

## Host command protocol (USB CDC)

Commands are **bare 5-byte ASCII, no terminator**. The firmware matches the first
5 bytes of each USB packet against a command table. Standard verbs:

| Command | Reply | Purpose |
|---|---|---|
| `getva` | 192 B (AES-256-ECB) | poll measurements (see below) |
| `query` | `firm` (app) / `boot` (bootloader) | identify what's running |
| `rotat` / `nextp` / `lastp` | none | rotate / cycle on-device screen |
| `gtrec` | N×8 B + `reck` | dump the on-device ~1 Hz V/I log |
| `updat` | `uprdy`, then per-chunk `OK` | our app self-update |
| `rawva` | 46 B plaintext | our dev verb: getva fields + window seq, no AES — use for streaming (tools/rawva.py) |
| `i2crd` | `OK`+data / `ER`+diag | our dev verb: raw I²C read (tools/i2cread.py) |
| `adcrd` | `OK`+u32 / `ER`+stage | our dev verb: ADC sum-of-16 (tools/adcread.py) |
| `tftfl` | `OK` / `ER`+stage | our dev verb: TFT test pattern (no args) or solid fill (RGB565 LE u16) (tools/tft.py) |
| `tftbl` | `OK` / `ER`+stage | our dev verb: backlight PWM compare, LE u16 0..=999 (tools/tft.py) |
| `tftui` | `OK` / `ER`+stage | our dev verb: force a main-screen refresh (tools/tft.py) |

Keeping `getva` byte-compatible (same reply) lets existing host tools work unchanged.

### `getva` reply (192 B = three 64-byte blocks, AES-256-ECB)
Key: `5821fa5601b2f02687ff1204622a4fb086f40260816f9a0ba7f106619ab87288`.
Decrypt, then read little-endian `uint32` fields:

| Offset | Field | Scale |
|---:|---|---|
| 0 | `"pac1"` marker | — |
| 4 | model (`TC66`) | 4 ASCII |
| 8 | firmware version | 4 ASCII |
| 12 | serial | uint32 |
| 44 | run count | uint32 |
| 48 | voltage | /10000 → V |
| 52 | current | /100000 → A |
| 56 | power | /10000 → W |
| 64 | `"pac2"` marker | — |
| 68 | resistance | /10 → Ω |
| 72/76 | group0 mAh / mWh | uint32 |
| 80/84 | group1 mAh / mWh | uint32 |
| 88/92 | temp sign / temp | °C |
| 96/100 | D+ / D− | /100 → V |
| 128 | `"pac3"` marker | — |
| 132 | window sequence (**our firmware only**; stock sends 0) | uint32, +1 per sampler window (~96 ms) |

Last 4 bytes of each 64-byte block are a checksum.

To stream samples losslessly: poll at ≥2× the window rate (25 Hz is plenty),
keep a reply only when the sequence advances, and treat a sequence jump as a
missed window. **Prefer `rawva` for this** — same live fields, plaintext, no
AES: ~1 ms per reply vs ~25 ms for `getva` (soft AES-256 is that expensive on
this core even optimized). Reply layout in `rust/src/getva.rs` (46 B: `"raw1"`,
ten LE u32 fields, CRC16/MODBUS tail); client + deduping stream generator in
`tools/rawva.py`. Verified on the bench: 700 polls/s costs the sampler nothing
(the sampler runs from a timer interrupt; see FIRMWARE.md), zero missed windows.

## Peripherals & pins

### USB (device, CDC-ACM)
- GD32 USB device peripheral, registers base **`0x40005c00`**.
- Stock firmware's endpoint map: **EP1 IN** = CDC data in (`EP1R = 0x40005c04`,
  TX packet-memory `0x40006180`), **EP3 OUT** = CDC data out (`EP3R =
  0x40005c0c`, RX packet-memory `0x40006200`). (The custom firmware lets its
  USB stack allocate — currently EP1 = interrupt IN, EP2 = bulk OUT+IN — and
  discovers the layout at runtime where it matters; see FIRMWARE.md.)
- Packet memory (PMA) at **`0x40006000`, doubled-addressed** — each 16-bit PMA slot occupies 4 CPU-address bytes.
- USB vid:pid `28e9:018a`. Needs the 48 MHz USB clock enabled.
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
  pins (AF1, 100 kHz) works** — the custom firmware uses it (rust/src/i2c.rs).
- EEPROM (256 Kbit) at 7-bit address **0x50** (0xA0 in 8-bit form) on the same
  bus, **2-byte addressing** — per-unit calibration constants (verified readable
  at the offsets below), energy accumulators, serial number.
- Bus scan also ACKs **0x58** (the BL24C256 EEPROM's ID/serial page) and **0x22**
  (a **FUSB302 USB-PD controller** — see "USB Power Delivery" below).
- INA226 registers: 0 config, 1 shunt V, 2 bus V, 3 power, 4 current, 5 calibration.
  Read model: write the 1-byte register pointer, read 16-bit big-endian. Power-on
  default config is 0x4127 with CAL (reg 5) unset.
- **Stock config: `write reg 0x00 = 0x41A7`, and nothing else** (no CAL write).
  Decoded: **AVG=1** (no internal averaging), **VSHCT=1.1 ms** (shunt/current),
  **VBUSCT=4.156 ms** (bus/voltage), **MODE=111** (shunt+bus, continuous). So the
  part is used as a **raw dual ΔΣ ADC**: the firmware reads shunt (reg 1) + bus
  (reg 2) and does all averaging + calibration in software.
- **Recommended for M1: do the same — raw ADC + software cal, NOT the INA226
  current/power engine.** The per-unit factory calibration in EEPROM (below) is
  expressed for this software path (specific sum depths, a per-unit zero-current
  offset `ref`, separate forward/reverse gains). The INA226 engine's single `CAL`
  register can't reproduce the per-unit offset/gains, so using it would **discard
  the factory calibration** and force a re-cal against lab references. Keep config
  `0x41A7`, read raw, and **pace the reads on a hardware timer** — that gives the
  poll-rate-independent sampling without needing the chip's hardware averaging.
- Bench note: powering the meter via its PC-comms port does NOT power the
  measurement path — bus voltage/shunt read 0 until the IN port is fed.

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
without rescaling. Drive the sampler from a **hardware timer** so the measurement rate
is independent of host polling (this is the whole point of M1). Power = V·I computed in
software for the `getva` reply (offset 56).

### Temperature (NTC) & D±
Both come from the **GD32 on-chip 12-bit ADC** (DMA'd, 16-sample sums).

**D+ / D−:** `D± = (Σ16 of the divider channel) · 330 / 65520` → 1e-2 V (getva 96/100).

**Temperature (NTC):** the stock firmware reads an NTC on ADC **ch3** (on the
hardware rev it targets — **on our unit the divider measurably sits on ch1**:
ch1 is rock-stable and reads room temp on a cool board, while ch3/ch0 float
and drift; see the ADC section below) and converts with a **β-equation**
(floating point). Recovered exactly from the firmware:
```
avg  = (Σ16 of ch3) / 16                          # 0..4095, 12-bit
R    = avg * 11500 / (4095 - avg)                 # ohms
T_K  = 1 / ( 1/298.15 + ln(R / 10000) / 3450 )    # β equation, natural log
T_°C = T_K - 273.15                               # display °F = °C*1.8 + 32
```
Sensor: a **10 kΩ @ 25 °C, β = 3450** NTC on the bottom of an **11.5 kΩ** divider.
Constants (flash literals): R_series `0x2cec` = 11500, R0 `0x461c4000` = 10000.0f,
β `…0x40aaf400` = 3450, `1/T0` `0x3f6b79e1:dcb5db83` = 0.0033540164 (T0 = 298.15 K),
273.15, and 1.8 / 32 for °F. The getva `temp` field carries the magnitude; the
separate `temp sign` field (offset 88) marks sub-zero.

The getva temperature is a single `ln()` per second — trivial, but if you want to
avoid pulling in `libm` you can use a small NTC lookup table instead. (Older hardware,
`CFG+0x48 ≤ 0x144`, uses an integer piecewise-linear curve on a different channel;
not present on current units.)

#### On-chip ADC (verified on hardware)
- **The ADC clock is IRC14M** (CFG2.ADCSEL reset default), a dedicated 14 MHz RC
  that is OFF at reset — enable `RCU_CTL1.IRC14MEN` first or calibration
  (RSTCLB/CLB) hangs forever. This cost a debug cycle.
- Channel map (identified empirically, `adcrd` verb / tools/adcread.py):
  **ch1 = PA1 = NTC divider** (stable across scans; ch0 = PA0 floats/drifts),
  **ch8 = PB0 = D+**, **ch9 = PB1 = D−** (direct connection — USB idle reads
  3.30 V / 0.00 V), ch2 = PA2 = BLE UART TX (idles 3.3 V), ch16 = internal die
  temp (~1.38 V ≈ 41 °C warm), ch17 = Vrefint (1.18 V sanity check).
- The firmware (rust/src/ina.rs) uses the recovered stock NTC model above on
  **ch1**. Verified: 25 °C at room temp on a cool board; ~60 °C after hours
  powered in the enclosure. (The internal die sensor is a poor cross-check —
  the MCU self-heats to ambient +~15 °C within minutes regardless of board
  temperature, and its V25/slope constants are uncalibrated per part.)

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
Chromium EC `fusb302`, various USB-PD stacks), so **M4 PD trigger is tractable** —
reuse this init and add a minimal Sink policy engine.

### Display (SPI TFT)
- **ST7735S** controller, **160×80 RGB565**, memory offset **(col +1, row +26)**, inversion on.
- **SPI0** (base `0x40013000`), master, mode 0, 8-bit, MSB-first, ~half-PCLK.
- Pins: **SCK=PA5, MOSI=PA7** (no MISO), **DC=PA4** (0=cmd/1=data), **RST=PA3** (active low), CS tied low.
- **Backlight: PA9 = enable** (high = on), **PB3 = PWM brightness** (TIMER1 channel, ~1.5 kHz).
- Bring-up: hardware RST pulse → SLPOUT(0x11)+100 ms → INVON(0x21) → frame/power/VCOM config (0xB1–0xC7) → MADCTL(0x36) → COLMOD(0x3A)=0x55 → 32 bytes gamma → +100 ms → DISPON(0x29). Draw with CASET(0x2A)/RASET(0x2B) (apply the +1/+26 offset) → RAMWR(0x2C) → pixel stream. No RAM framebuffer (8 KB can't hold a frame) — draw straight to GRAM.
- **MADCTL on this unit: `0xA8`** (stock's "180°" variant) is right-side up; `0x68` renders upside-down (webcam-verified). Colors are correct with the BGR bit as stock sets it.
- **PA9 (backlight enable) is shared with the USB D+ soft-connect** — the firmware already holds it high after enumeration, so backlight brightness is controlled purely by the PB3 PWM (compare 0 = dark).

### Buttons
- **K1 = PF0, K2 = PF1** (GPIOF), read via EXTI or polled.

### Bluetooth
- A **DX-BT24** BLE module on **USART0** (UART). The BLE command set mirrors the USB
  verbs with a `b` prefix + CRLF. (Note: the module's RF is weak on this board — a
  hardware limitation, not a firmware issue.)

### Energy integration
- A 1 Hz timer integrates the mAh/mWh accumulators from the published V/I, so energy
  is poll-rate-independent.

## VBUS note
The output VBUS is a **passive pass-through** (input → shunt → output); there is no
MCU-controllable load switch, so VBUS cannot be toggled from firmware.
