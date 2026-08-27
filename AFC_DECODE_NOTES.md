# Samsung AFC — SOLVED (build 97). Notes for FCP/SCP.

`afctr 0x46` drives a Samsung AFC charger from 5 V → 9 V, bench-verified
(8.92 V), reliable across power-cycles; `afctr 0x08` returns it to 5 V. Lives in
`qc.rs`; host tool sends `afctr <byte> [reps]`. The stock `charge_protocol_
handshake` mode 4 is replicated register-for-register (see
`/home/fletcher/tc66/firmware/ghidra/CHARGE_TRIGGER_FINDINGS.md`).

## What made it work (the four things, in order of discovery)

1. **NRZ, not pulse-width.** Each byte is `proto_send_byte`: start pattern
   (D− high 24 µs / low 32 µs / high 32 µs), 8 data bits MSB-first at ~152 µs
   (1 = D− mark high [PB4 hi, PB5 lo], 0 = low), a 9th **odd-parity** bit, stop.
   D+ held 0.6 V (PA13=1) throughout. Verified byte-perfect vs the stock capture.
2. **Interrupt masking of the bit timing.** The 250 Hz sampler ISR (does ~0.5 ms
   I2C reads) firing mid-byte stretches a bit cell and corrupts the frame. The
   byte runs inside `cortex_m::interrupt::free`; the 2.5 ms marks and the reads
   tolerate jitter.
3. **The real entry** (register-exact, from a fresh 200 kS/s stock capture — the
   findings-doc's "D+3.3/D−3.3 hold" was wrong): release all four pins (hi-Z)
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
   - **Sync subtlety that cost a build:** `proto_wait_sping` must FIRST wait for
     the released mark to fall low, THEN detect the charger's Sping rising/
     falling. Waiting only "high then low" mistakes the residual mark-high for
     the Sping, so the byte is sent *during* the charger's Sping → contention →
     garbled byte → no switch. Fixed by the leading `wait_pb4(false,…)`.

**Bench procedure:** the charger's state matters — power-cycle it (Kasa plug
192.168.1.233, `kasa-plug` skill, `-t 5` cold) before a clean test. Our D+/D−
drive HW is proven independently (QC2.0 `qctrg` switches this charger to 9/12 V).

## FCP / SCP — SCAFFOLDED (qc.rs `fcp()`/`scp()`, verbs `fcptr`/`scptr`), UNTESTED

Share AFC's machinery: `accp_entry` (same release→D+0.6/D−released→2050 ms),
`proto_send_word(buf, flag)` (now multi-byte), the closed-loop reads. Then 3×
`proto_send_word(0x01, flag=0)` pings 900 µs apart, then the stock command
packets (from the scatter-decompressed .data, all CRC-verified):
- **FCP (mode 2), fixed 9 V:** `0c 04` `0c 21` `0c 31` `0b 2c 5a` (set 0x2c=90 →
  9 V, 100 mV units) `0c 2c` `0b 2b 01` (commit).
- **SCP (mode 3):** `0b a0 c0` (enable) `0b a1 06` `1b b0 02 15 7c` `1b b2 02 13
  88` `0b ca c8` (voltage code 0xc8 — raw scale, volts unknown) `0b cb 64`.
- **CRC-8 CONFIRMED:** poly 0x39, init 0, MSB-first, over all preceding bytes
  (`crc8_huawei @0x08003a2c`). `qc::crc8` verified against all 16 RE example
  packets. Appended in `accp_exchange`.

**Three ASSUMPTIONS in the code that need RE (can't bench-test — no FCP/SCP
charger). Each is marked in qc.rs:**
1. **Parity (most critical).** `afc_byte_bits` uses PER-BYTE odd parity. The RE
   calls `proto_send_byte`'s 9th bit a "running-parity" with scratch at
   0x2000029d — for a multi-byte packet it may be a parity RUNNING across the
   whole `proto_send_bytes` call. If wrong, every multi-byte packet's parity is
   wrong → charger rejects all. (AFC unaffected: len 1.)
2. **Per-packet `flag`.** Assumed data packets = flag=1 (measure the charger
   response, like AFC's byte), pings = flag=0. The real flags for the mode-2/3
   read (0c) vs write (0b/1b) packets are unconfirmed.
3. **Inter-packet gap.** Assumed 10 ms between command packets (AFC's repeat
   gap); the pings are the confirmed 900 µs.

Also: SCP `0xca` voltage-code↔volts scale is raw/unknown (stock scan probes
0xc8=200, 0xdc=220); `scp_set_voltage @0x08004aac` (not ported) patches 0xca +
CRC for other voltages. Modes: 2 = FCP (0x0800417c), 3 = SCP (0x08004c60).
