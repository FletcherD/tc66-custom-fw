# Samsung AFC (working) and Huawei FCP/SCP (untested)

`afctr 0x46` drives a Samsung AFC charger from 5 V → 9 V, bench-verified
(8.92 V), reliable across power-cycles; `afctr 0x08` returns it to 5 V. Lives in
`qc.rs` (`qc::afc`); the verb is `afctr <byte> [reps]` (reps default 3; no
host wrapper — send it raw). The stock `charge_protocol_
handshake` mode 4 is replicated register-for-register (per our unpublished stock-firmware RE notes).

## What makes it work

1. **NRZ, not pulse-width.** Each byte is `proto_send_byte`: start pattern
   (D− high 24 µs / low 32 µs / high 32 µs), 8 data bits MSB-first at ~152 µs
   (1 = D− mark high [PB4 hi, PB5 lo], 0 = low), a 9th **odd-parity** bit, stop.
   D+ held 0.6 V (PA13=1) throughout. Verified byte-perfect vs the stock capture.
2. **Interrupt masking of the bit timing.** The 250 Hz sampler ISR (does ~0.5 ms
   I2C reads) firing mid-byte stretches a bit cell and corrupts the frame. The
   byte runs inside `cortex_m::interrupt::free`; the 2.5 ms marks and the reads
   tolerate jitter.
3. **The entry** (`accp_entry`, register-exact from a 200 kS/s stock capture —
   NOT a D+3.3/D−3.3 hold): release all four pins (hi-Z)
   200 ms, then drive **only D+** to its 0.6 V state (PA13=1, PA14=0) with **D−
   left RELEASED** for the whole 2050 ms hold — the D− ≈ 0.6 V→0 a scope sees is
   charger-sourced. Then the request byte ×3, 10 ms apart. AFC has **no sync
   pings** (those are FCP/SCP).
4. **CLOSED-LOOP — the decisive fix.** `proto_send_word` is not a blind send: it
   READS PB4 (D− as input) to follow the charger's Sping. Per word: Mping (drive
   D− high 2.5 ms via `proto_line_idle`, then release) → `proto_wait_sping` →
   byte → Mping → wait Sping → `proto_measure_response` (read the byte-echo) →
   Mping → wait Sping. An open-loop replay engages the charger (it Spings) but
   never completes — the FSM needs the master to read/wait.
   - **Sync pitfall:** `proto_wait_sping` must FIRST wait for
     the released mark to fall low, THEN detect the charger's Sping rising/
     falling. Waiting only "high then low" mistakes the residual mark-high for
     the Sping, so the byte is sent *during* the charger's Sping → contention →
     garbled byte → no switch. Hence the leading `wait_pb4(false,…)`.

**Bench procedure:** the charger's state matters — power-cycle it (mains off ~5 s, cold) before a clean test. Our D+/D−
drive HW is proven independently (QC2.0 `qctrg` switches this charger to 9/12 V).

## FCP / SCP — IMPLEMENTED FROM THE STOCK CAPTURES, UNTESTED ON A CHARGER

qc.rs `fcp()`/`fcp_read()`/`scp()`, verbs `fcptr`/`fcprd`/`scptr`, compiled
only with the `fcp` feature (add it to the Makefile's `DEV_FEATURES`; it is
out of the default dev build for space), never into release images. Shared with
AFC: `accp_entry` (release → D+ 0.6 V / D− released → 2050 ms) and the
Mping/Sping helpers (`proto_line_idle`/`proto_wait_sping`). FCP/SCP have their
own framer (`accp_frame`) and reply reader (`accp_capture` + `accp_decode`);
AFC keeps its bench-verified `afc_start`/`afc_byte_bits`/`proto_send_word`.

**Ground truth: `fcp.csv` / `scp.csv`** (repo root, gitignored): the stock
firmware running modes 2/3 with an FCP charger attached, 100 kS/s (10 µs per
sample), ch1 = D−, ch2 = D+. Decode with `uv run tools/accp_decode.py fcp.csv`
(prints each frame with parity / CRC / SYNC-rule checks). That charger
answered vendor `0x04 = a2`, `0x21 = 02` (two raised voltages), `0x31 = 5a`
(9.0 V), ACKed the 9 V write, and NACKed (`03`) every SCP packet.

### Frame format (every clean frame in both captures matches)
- Byte = 8 data bits MSB-first + **per-byte odd parity**, ~152-160 µs cells
  (D− mark = 1). Master and charger frames use the same format.
- **SYNC** quarters (~40 µs) before every byte: opposite of the previous line
  level P, back to P, and one more opposite quarter if the byte's MSB equals P
  (so each byte starts on an edge). P = low before the first byte, else the
  previous parity bit. After the last byte the same rule runs toward the
  high Mping (next bit = 1). `accp_sync`/`accp_frame`.
- Packets: `<cmd><reg>[<len>]<data…><crc8>`; the 3 leading `01` pings carry
  no CRC and get no reply.

### Transaction (timings from the capture)
Mping (2.5 ms) → charger Sping (~380 µs later, 2.5 ms) → 250 µs → master
frame → Mping → Sping → ~590 µs → **charger frame** `[ACK, data…, CRC]` →
charger end ping (high ~2.5 ms) → 1.7 ms → Mping → Sping. Pings 900 µs
apart, 10 ms before the first packet and between packets. `accp_capture`
records the reply as run lengths (interrupts masked, ≤ 15 ms, stops at a
high ≥ 1.8 ms — a byte's longest high run is ~1.4 ms); `accp_decode` splits
on runs < 90 µs (SYNC), rounds the rest to 155 µs cells, checks parity and
CRC. A Python port of the decoder reads every FCP frame in the capture with
±15 µs of injected jitter.

### Sequences and replies
- **FCP (mode 2), 9 V:** `0c 04` `0c 21` `0c 31` `0b 2c 5a` (VOUT 90 × 100 mV)
  `0c 2c` `0b 2b 01` (apply). Only the two writes are needed to switch; the
  reads identify the charger and verify the write.
- **`fcprd <reg> [n]`:** single reads only — capability readout at 5 V
  (0x21 → N, 0x30+k voltages, 0x50+k currents, 0x22 max power).
- **SCP (mode 3):** `0b a0 c0` `0b a1 06` `1b b0 02 15 7c` `1b b2 02 13 88`
  `0b ca c8` `0b cb 64` = output enable, 3 s watchdog, boundaries 5.5 V / 5 A,
  VSSET 5.00 V, ISSET 5 A — the low-voltage direct-charge mode; it does **not**
  raise the voltage. A raise would write VSET `0xb8/b9` and keep the
  watchdog fed — not implemented.
- Reply ACK = low nibble 8 (`08`/`88`), NACK = `03`. Errors: `ER 0x40|i`
  = no / garbled reply to packet i, `ER 0x50|i` = NACK.

### Open
- Never run against a real FCP/SCP charger (the bench has none).
- The `0x01` pings' purpose is unknown (copied from stock).
- In scp.csv the stock `1c b0 02` burst read decodes with a wrong CRC every
  time (`1c b2 02` is fine) — unexplained, irrelevant to the writes.

## Protocol reference (public sources)

**Physical layer:** UI 160 µs typ (144–176), Mping/Sping 16 UI (2.3–2.8 ms;
the charger copies the Mping length ±1 %, so it takes its UI from ours). Byte
= MSB-first 8 data + odd parity. Transaction: Mping → Sping → master packet →
Mping → Sping → slave reply → Mping → Sping. Sinks accept a Sping of 10–20 UI.

**Commands:** `0x0c` single read `0c reg`, `0x0b` single write `0b reg val`,
`0x1c` burst read `1c reg num`, `0x1b` burst write `1b reg len d…`; CRC-8
(above) on both FCP and SCP (FCP `0x03` SSTAT has CRC/parity error bits).

**FCP registers:** `0x00` DVCTYPE, `0x01` SPEC_VER, `0x03` SSTAT, `0x04`
vendor ID; `0x21` = number of voltages above 5 V (1 → 5/9, 2 → 5/9/12);
`0x22` max power (0.5 W); `0x28` adapter status (UVP/OVP/OCP/OTP); `0x2b`
output control (bit 0 = apply); `0x2c` VOUT (100 mV); `0x2d` IOUT (100 mA);
`0x30+n` voltage n (100 mV), `0x50+n` current n (100 mA). **Capability readout
with no voltage change:** read `0x21` → N, then `0x30+N` and `0x22`.
Back to 5 V: `0x2c=50`, `0x2b=1` (or release D+).

**SCP registers:** `0x7e/0x80/0x81` adapter type; `0x82–0x8b` IDs; `0x8d`
power class; `0x90` max power; `0x92/0x93` min/max VOUT, `0x94/0x95` min/max
IOUT (value = 10^(bits 7:6) × bits 5:0, mV/mA); `0xa0` ctrl (bit 7 output
enable, bit 6 mode, bit 5 reset); `0xa1` (bits 2:0 watchdog × 0.5 s); `0xa8–0xab`
measured V/I; `0xb0/b1` VSET boundary, `0xb2/b3` ISET boundary (mV/mA);
`0xb8/b9` VSET (mV, the high-voltage set); `0xca` VSSET = (mV − 3000)/10, only
≤ 5.5 V; `0xcb` ISSET (50 mA). **The stock "SCP" block (and so `scptr`) is the
5.00 V / 5 A low-voltage direct-charge mode** (`0xca=0xc8` = 5.00 V,
boundaries 5.5 V / 5 A, 3 s watchdog) — it never raises the voltage; a real
SCP raise writes VSET `0xb8/b9`.

**Detection:** after DCP, D+ 0.6 V for ~1 s, require D− < 0.325 V; then pings
(sc8546: 15 tries ~10 ms apart, 82 ms pause every 5). No answer → plain DCP.

**Sources:**
- Huawei hwpower kernel code — `adapter_protocol_{fcp,scp}.h`, `sc8546_scp.{c,h}`,
  `stm32g031_scp.h`: github.com/wutaijieing/android_kernel_LIO_kirin990_103;
  older `hw_scp.c` in LineageOS/android_kernel_huawei_kirin970.
- Samsung bit-banged AFC (shared physical layer): LineageOS/android_kernel_samsung_sm6115
  `drivers/power/supply/afc/gpio_afc.c`, `include/linux/afc/afc.h`.
- Patent US10135263B2 FIG. 5–7 (framing, parity, SYNC): https://patents.google.com/patent/US10135263B2/en
- Southchip SC2021A datasheet (timing table):
  https://uploadcdn.oneyac.com/upload/document/1675152919627_4112.pdf;
  Hynetek HUSB601 datasheet.
- NXP FTF2016 "Smart USB Fast Charging" slides 22–24 (layer overview).

Stock addresses: `scp_set_voltage @0x08004aac` (patches 0xca + CRC, not
ported); modes 2 = FCP (0x0800417c), 3 = SCP (0x08004c60).
