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
  (debugprobe requires SWCLK/SWDIO on adjacent GPIOs). `src/openocd.cfg` is ready.
- **The chip is readout-protected, which constrains debugging:**
  - **Flash reads over SWD fail** — you cannot read the flashed image back.
  - **`halt` / `reset` / `step` lock the core up** — do not use them.
  - **Background SRAM reads DO work** while the core runs.
- **Therefore: debug with SRAM progress markers.** Have the firmware write a
  progress code to a fixed SRAM address at each stage, and read it with:
  `openocd -f src/openocd.cfg -c "mdw 0x20001000 8" -c shutdown` (no halt).
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
| 128 | `"pac3"` marker | (unused) |

Last 4 bytes of each 64-byte block are a checksum.

## Peripherals & pins

### USB (device, CDC-ACM)
- GD32 USB device peripheral, registers base **`0x40005c00`**.
- **EP1 IN** = CDC data in (`EP1R = 0x40005c04`, TX packet-memory `0x40006180`).
- **EP3 OUT** = CDC data out (`EP3R = 0x40005c0c`, RX packet-memory `0x40006200`).
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
- External power monitor (INA226-class), I²C address **0x4A**, on a **software (bit-banged) I²C bus: SCL = PB6, SDA = PB7** (these are also the hardware I²C0 pins, so hardware I²C0 is an option for a rewrite).
- EEPROM (256 Kbit) at I²C address **0xA0** shares the same bus — holds per-unit calibration constants, energy accumulators, serial number.
- Read model: write the monitor's register/channel select, then read a 16-bit value.

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

Temperature (on-chip ADC + NTC, Steinhart curve) and D+/D− (on-chip ADC dividers,
`D± = sum_of_16 · 330 / 65520` → 1e-2 V) come from the GD32's own ADC, DMA'd.

### Display (SPI TFT)
- **ST7735S** controller, **160×80 RGB565**, memory offset **(col +1, row +26)**, inversion on.
- **SPI0** (base `0x40013000`), master, mode 0, 8-bit, MSB-first, ~half-PCLK.
- Pins: **SCK=PA5, MOSI=PA7** (no MISO), **DC=PA4** (0=cmd/1=data), **RST=PA3** (active low), CS tied low.
- **Backlight: PA9 = enable** (high = on), **PB3 = PWM brightness** (TIMER1 channel, ~1.5 kHz).
- Bring-up: hardware RST pulse → SLPOUT(0x11)+100 ms → INVON(0x21) → frame/power/VCOM config (0xB1–0xC7) → MADCTL(0x36)=0x68 → COLMOD(0x3A)=0x55 → 32 bytes gamma → +100 ms → DISPON(0x29). Draw with CASET(0x2A)/RASET(0x2B) (apply the +1/+26 offset) → RAMWR(0x2C) → pixel stream. No RAM framebuffer (8 KB can't hold a frame) — draw straight to GRAM.

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
