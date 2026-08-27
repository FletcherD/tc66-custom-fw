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

## FCP / SCP — next (share this exact machinery)

HiSilicon ACCP: one PHY for AFC/FCP/SCP. Reuse `proto_send_word`/`afc_byte`/
`proto_wait_sping`. Differences vs AFC:
- **3 sync pings** first: `proto_send_word(0x01)` ×3 with `i2c_bit_delay(900)` =
  900 µs gaps, then the command packets (multi-byte, `flag` differs).
- Grammar: `0c`=read8 `<0c><reg><crc>`, `0b`=write8 `<0b><reg><val><crc>`,
  `1b`=write16. FCP set 9 V = `0b 2c 5a <crc>` then `0b 2b 01 <crc>`. SCP enable
  `0b a0 c0 <crc>`, set volts `1b b8 <hi> <lo> <crc>` (mV, big-endian).
- **CRC unconfirmed** (0x39 is a guess). Solve it from captured frames — the
  stock capture in scratchpad + brute-forcer `crc.py` — before trusting writes.
- `charge_protocol_handshake` modes: 2 = FCP (block 0x0800417c), 3 = SCP
  (0x08004c60), 4 = AFC (0x080034e4); `scp_set_voltage @0x08004aac` patches the
  target byte + checksum and re-sends.
