# AFC/FCP/SCP D− serial — decode notes from working stock captures

The stock TC66's working protocol-trigger output was captured (D− = ch1,
D+ = ch2) with `tools/scope_capture.py` at **100 kS/s (10 µs/sample)**:
`afc.csv`, `fcp.csv`, `scp.csv` (each shows the charger actually switching).
These are ground truth — our `qc.rs` AFC implementation does NOT match them
and does not trigger. **The RE doc's `proto_send_byte` "152 µs NRZ bit" was an
explicitly-unverified estimate and is wrong.**

## What the captures show (vs our implementation)

| | Working stock capture | Our qc.rs (build 86) |
|---|---|---|
| Encoding | **pulse-width**: each symbol is a HIGH pulse, LONG ≈ 2.5 ms vs SHORT ≈ 0.15–0.5 ms, with ~0.37 ms LOW gaps between | fixed 152 µs level per bit (NRZ) — **wrong** |
| D− idle | **low (0 V)** between hold and serial | high (3.3 V mark) — **wrong** |
| D+ | held 0.6 V throughout | held 0.6 V ✓ |
| Structure | 3 pings (~910 µs `L91` gaps) → data packets (~10 ms `L1003` gaps) ✓ matches RE doc | 0x46 ×3 at 10 ms, no pings |

So AFC entry + D+ hold are right; the **serial waveform is a different
physical layer** (pulse-width / pulse-distance), and D− must idle low.

## Decode progress (100 kS/s)

- **LONG high pulse ≈ 250 samples = 2.5 ms**, **SHORT ≈ 15–48 samples =
  0.15–0.48 ms**. Inter-symbol LOW gap ≈ 37 samples (0.37 ms). Inter-ping gap
  ≈ 91 samples (0.91 ms). Inter-packet gap ≈ 1003 samples (10 ms).
- FCP `fcp.csv` serial burst starts ~idx 2,244,000. First three groups are the
  3 pings, each the pattern `0 0 1 0 0` (LONG=0/SHORT=1) separated by 910 µs,
  then a 10.9 ms gap, then the data packets. AFC `afc.csv` serial ~idx
  5,135,700; one command byte ≈ `L L S L L S L L L` (9 symbols).
- **Not yet cleanly solved:** the exact bit-frame (start marker? 8 data +
  parity? the ping's short frame?) and the LONG/SHORT polarity. Naive
  LONG=0/SHORT=1 MSB-first gives FCP-ping-ish `00100` and AFC ≈ 0x48 (expected
  0x46) — close but the framing/boundaries need methodical work.

## To finish (next session)

1. Decode all three captures precisely against the KNOWN bytes
   (`CHARGE_TRIGGER_FINDINGS.md`): FCP `01 01 01`, then `0c 04 e2 / 0c 21 b0 /
   …`; AFC `46 46 46`; SCP `01 01 01` then `0b a0 c0 64 / …` with CRC-8/0x39.
   Solve the frame + LONG/SHORT polarity so all known bytes decode exactly.
2. Rebuild `qc.rs` serial: emit HIGH pulses of the decoded LONG/SHORT widths
   (µs, SysTick-timed — the timing fix stays), D− idling **low**, with the
   3-ping preamble for FCP/SCP. AFC entry (D+ 0.6 V / D− 3.3 V hold) is already
   correct.
3. Verify on the scope (`scope_capture.py`) that our output matches the working
   capture BEFORE plugging the charger, then confirm 5 V→9 V.

Everything else in M4 (PD 2.0/3.0 + PPS, QC2, QC3, KM003C PD capture) is done
and verified; AFC/FCP/SCP are the remaining serial protocols.
