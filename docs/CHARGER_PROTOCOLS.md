# Charger protocols: detection and sniffing

What the meter can learn about a charger, a cable and a device, protocol by
protocol: how each protocol signals, what the meter sees passively, what
needs an active probe, and the tools and verbs for each. Bench results
below are from this unit (TC66C) with an Apple 96 W USB-C charger, an Apple
e-marked cable, a Pixel and two Motorola phones.

Related: [HARDWARE.md](HARDWARE.md) (pins, verb table),
[FIRMWARE.md](FIRMWARE.md) ("USB-PD sink", "Sniffer", "PD capture", "D+/D−
charge triggers"), [AFC_DECODE_NOTES.md](AFC_DECODE_NOTES.md) (AFC/FCP/SCP
wire format).

## The three signal paths

| Line | Carries | Meter hardware |
|---|---|---|
| **VBUS** | power (5–20 V) | INA226 bus voltage + shunt current (published **unsigned** — a charger on OUT still reads positive) |
| **D+ / D−** | legacy charging signatures (BC1.2, Apple/Samsung dividers), QC level/pulse signalling, the AFC/FCP/SCP single-wire serial on D− | ADC: D+ on PA1 (ch1), D− on PB1 (ch9), sampled ~10×/s in normal operation. Drive network: D+ via PA13 (0.6 V) / PA14 (3.3 V), D− via PB5 (0.6 V) / PB4 (3.3 V). **PB4 doubles as a digital D− input** (reads the AFC/FCP charger's pings and replies). PA13/PA14 are also SWD. |
| **CC1 / CC2** | Type-C attach and current advertisement (Rp/Rd), USB-PD messages (BMC, 300 kbit/s), cable e-marker traffic (SOP′/SOP″) | FUSB302 (I²C 0x22, 400 kHz) on the pass-through CC lines, INT_N on PA15. Its VBUS pin is not wired (VBUSOK always 0). No MCU GPIO or ADC pin touches CC. |

All three pass straight through IN ↔ OUT, so detection and sniffing work
with the charger on either port. The **NO PD** slide switch, when set to
NO PD, cuts CC from the FUSB302 (the CC pins then read 0 V). The meter's
own Rd: the FUSB302's power-on default presents Rd on both CC pins, so with
the PD engine off a charger sees the meter as a sink (in parallel with any
device on the other port).

## Protocols

### Dedicated chargers: BC1.2 DCP and divider signatures (D+/D−)

- **BC1.2 DCP:** D+ and D− shorted inside the charger. A device detects it
  by driving D+ and seeing it on D−.
- **Apple:** fixed dividers on each line. D+/D− ≈ 2.0/2.0 V = 0.5 A,
  2.0/2.7 = 1 A, 2.7/2.0 = 2.1 A, 2.7/2.7 = 2.4 A.
- **Samsung:** D+ = D− ≈ 1.2 V.
- **Meter:** passive — the D± ADC readings identify a divider directly
  (with nothing on the other port; a device would load the lines). The
  short is found by the QC probe below (D− follows D+).
- **Bench:** the Apple USB-C charger shows D+ = D− = 1.51 V (lines tied and
  biased) and a DCP short.

### Qualcomm Quick Charge 2.0 / 3.0 (D+/D−)

- **Handshake:** the device holds D+ at 0.6 V for ≥ 1.25 s; a QC (HVDCP)
  charger then opens its D+/D− short and pulls D− low.
- **QC 2.0:** static levels select the voltage.

  | D+ | D− | Voltage |
  |---|---|---|
  | 0.6 V | 0 V | 5 V |
  | 3.3 V | 0.6 V | 9 V |
  | 0.6 V | 0.6 V | 12 V |
  | 3.3 V | 3.3 V | 20 V (Class B only) |

- **QC 3.0:** from D+ 0.6 V / D− 3.3 V, a D+ pulse high steps +200 mV and a
  D− pulse low steps −200 mV.
- **Meter, active (never raises VBUS):** `qcdet` runs the handshake with D−
  released and reads D− on the ADC at 0.1 s and 1.5 s (threshold 300 mV):
  class 0 = no D+/D− short, 1 = DCP without QC, 2 = QC. `qctrg`/`qc3ct` run
  the same check first and refuse non-QC chargers (ER 0x20 / 0x21).
- **Meter, passive (not implemented):** QC 2.0 requests are static D±
  levels — readable from the existing ADC samples (map through the table,
  confirm with VBUS). QC 3.0 pulses are too short for the ~10 Hz sampling;
  D+ has no timer-capable pin (PA1's only timer is TIMER1_CH1, which drives
  the backlight), so faster ADC sampling or inferring steps from VBUS would
  be the route.
- **Bench:** Apple charger → class 1 (DCP, no QC); a PC host port → class 0.

### Samsung AFC (D− serial)

- **Signalling:** D+ held at 0.6 V; single-wire NRZ serial on D−, bit cell
  ~160 µs, MSB first + odd parity, SYNC pulses. Master ping (~2.5 ms high) →
  charger ping → request byte → master ping → charger reply → ping. Request
  byte: high nibble = volts above 5 (0x46 = 9 V), low nibble = current code
  (0x08 = 5 V).
- **Meter, active:** `afctr 0x08` sends a 5 V request; OK = an AFC charger
  answered, ER 0x22 = no answer — a probe without a voltage change.
  `afctr 0x46` triggers 9 V (bench-verified on an AFC charger).
- **Meter, passive (not implemented):** PB4 can be TIMER2_CH0 (AF1) for
  hardware edge timestamps, but TIMER2_CH0's DMA request is channel 3 —
  shared with the BLE TX DMA. Options: time-share channel 3 during an armed
  capture, a timer interrupt (a third interrupt, against the two-interrupt
  rule), or an armed interrupts-masked capture like `accp_capture`. Whether
  a phone's D− signalling crosses PB4's threshold through the resistor
  network is unverified (the charger's is).
- **Bench:** Apple charger and PC port → no answer.

### Huawei FCP / SCP (D− serial)

- Same physical layer as AFC; register packets with a CRC-8 (poly 0x39).
  Read `0x21` = number of raised voltages N, `0x30+k` = voltage k (100 mV),
  `0x22` = max power (0.5 W). Write `0x2c` (VOUT) then `0x2b = 1` to switch.
  SCP adds direct-charge registers (VSET 0xb8/b9 + a 3 s watchdog); the
  stock firmware's SCP block only sets 5 V / 5 A.
- **Meter:** `fcprd` reads registers without changing the voltage
  (capability readout), `fcptr` requests 9 V, `scptr` sends the stock SCP
  block — all behind the `fcp` feature, which is in neither the release
  nor the default dev build. **Never run against a real FCP/SCP charger**;
  the framing and reply decoding match the stock firmware's captures.
  Details: AFC_DECODE_NOTES.md.

### Type-C current advertisement (CC)

- The source's Rp on CC advertises Default USB / 1.5 A / 3.0 A. With a 5.1 kΩ
  Rd the CC voltage lands in vRd ranges ≈ 0.25–0.61 V / 0.70–1.16 V /
  1.31–2.04 V; with no sink the line floats high.
- **Meter:** FUSB302 BC_LVL comparators (< 0.2 / 0.66 / 1.23 V thresholds)
  and a 6-bit MDAC (42 mV steps, ≤ 2.69 V). The sink and the sniffer report
  the level (`pdsta` status0, the PD screen's `CC1 1.61V Rp 3.0A` line);
  `detect.py` reads it with the sink armed.
- **Sniffer attach detection:** with the MDAC parked at 2.69 V, a sink's Rd
  pulls the line below it (attached), the source's Rp alone doesn't.
- **Bench:** Apple charger → 3.0 A on CC1.

### USB Power Delivery (CC)

- **Messages:** BMC on CC, each acknowledged by a GoodCRC from the other
  side (hardware, within ~195 µs). A negotiation: Source_Capabilities →
  Request → Accept → PS_RDY (+ GoodCRCs). Then vendor messages (VDMs), e.g.
  Discover Identity. Hard Reset is an ordered set (no message id) that
  makes both ends start over (the source cycles VBUS).
- **Information available** (decoded by `tools/pd_decode.py`, checked
  against Linux `include/linux/usb/pd.h` / `pd_vdo.h`):

  | Message | Fields |
  |---|---|
  | Source/Sink_Capabilities | fixed PDOs (V, max I, flags), variable/battery, PPS (V range, I), SPR/EPR AVS |
  | Request (RDO) | object position, operating/max current or power, PPS/AVS output voltage, capability mismatch |
  | Accept / Reject / Wait / PS_RDY | the contract outcome |
  | header | power/data role, spec revision (PD 2.0/3.0), message id |
  | Discover Identity (SOP) | the device's ID header (product type, VID), cert XID, product ID; phones often NAK |
  | Discover Identity (SOP′) | the cable's e-marker — see below |

- **Meter as sink** (`pdctl 1`, `pd.py arm`): collects the source's caps
  and takes PDO 1 (5 V) immediately; `pdreq` renegotiates (fixed + PPS,
  bench-verified 5–20 V, PPS 3.3–11 V). Our own traffic is captured.
- **Meter as listen-only sniffer** (`pdctl 3`, `pd.py sniff`, or entering
  the PD info screen): no Rd, no GoodCRC, never transmits; the receiver
  locks onto the CC carrying the source's Rp; every SOP/SOP′/SOP″ frame from
  both sides goes into the KM003C capture ring (`tools/pdcap.py`), and the
  on-device parser (`pdinfo.rs`) keeps the offers, the contract (Request +
  PS_RDY — robust to a missed Accept), the cable e-marker and counters for
  the PD info screen / `pdinf` / `pd.py info`.
  - In sniff mode the meter presents no Rd: a Type-C charger supplies VBUS
    only while a device is on the other port.
  - `pdctl 2` while sniffing sends a third-party Hard Reset: the charger
    cycles VBUS and renegotiates with the device — a re-plug without
    unplugging.
- **Capture quality** (bench, Apple charger + Pixel/Moto):
  - Every negotiation is captured; the residual misses are frames in the
    densest burst (Request → GoodCRC → Accept → GoodCRC within ~1 ms) in
    about half the negotiations — they never reach the FUSB302's FIFO
    (not even a CRC check), a receiver limit firmware can't reach. The
    contract is still derived correctly from Request + PS_RDY.
  - Source_Cap retry storms come through whole at 400 kHz I²C (76/82
    messages with all 4 copies; 33/41 at 100 kHz).
  - The BMC slicer threshold matters: at the FUSB302's reset value the
    charger's first VDM to a Moto drawing 2.8 A was lost every time; Slice
    0x58 (SDAC 0x18) catches it (16/16).
  - Bad-CRC packets are counted and timestamped (the FIFO only holds good
    ones), as are hard resets, RX FIFO full and resync events.
- **Bench observations:**
  - Apple 96 W: PD 2.0, 5/9/15/20 V @ 3 A; 20 V @ 4.7 A once it has read a
    5 A cable e-marker. Probes the cable (SOP′ Discover Identity) after
    attach — 20 tries × 4 retries when nothing answers.
  - Pixel: requests 9 V @ 3 A; sometimes ignores ~4 s of Source_Caps,
    re-attaches, then negotiates. NAKs the charger's Discover Identity.
  - Moto: PD 3.0 GoodCRC; requests 5 V first, then 9 V @ 3 A.
  - A second Moto (dead battery) never got PD: its GoodCRCs failed CRC at
    both the charger and the meter; it charged at 5 V.
  - A non-PD power pack: Source_Caps unanswered (~52 × 4), then 5 V.

### Cable e-marker (CC, SOP′/SOP″)

- **What it is:** a chip in a USB-C cable's plug, powered by VCONN, that
  answers SOP′ (and SOP″ for a second marker at the far end) messages. The
  source must read it before offering more than 3 A; it also reports the
  cable's data speed, length class, max voltage and vendor.
- **Which cable answers:** only the cable plugged into the charger — VCONN
  stops at that cable, so a second cable on the other side of the meter is
  invisible to the charger (and to us). A 5 A marker on the charger-side
  cable lets the charger offer 5 A even if the device-side cable is 3 A.
- **Meter:** the sniffer receives SOP′/SOP″ (Control1 ENSOP1/2) and the
  parser shows `CBL 5A USB2 05AC`; `pdcap.py` decodes the full identity.
- **Bench:** Apple cable → passive cable, VID 0x05AC (Apple), PID 0x1781,
  XID 2598, 5 A, USB 2.0 only, Type-C, latency class ~2 m.

## What we can learn — summary

| Information | How | Passive? | Tool / verb |
|---|---|---|---|
| Apple / Samsung divider, D± bias | D± ADC | yes | `rawva`, screen 3 (D±), `detect.py` |
| DCP short / QC capability | QC handshake | no (5 V only) | `qcdet`, `detect.py` |
| AFC capability | 5 V AFC request | no (5 V only) | `afctr 0x08`, `detect.py` |
| FCP capability + voltages | register reads | no (5 V only) | `fcprd` (`fcp` feature) |
| Type-C Rp current | CC level with Rd | sink: no; sniff with a device: yes | `pdsta`, PD screen, `detect.py` |
| PD offer (all PDOs) | Source_Caps | sink: active (5 V contract); sniff: yes | `pdcap`, `pdinf`, `pdcap.py` |
| Negotiated contract | Request + PS_RDY | sniff: yes | `pdinf`, PD screen, `pdcap.py` |
| PD revision, roles | message headers | yes | `pdcap.py`, `pdinf` |
| Cable rating / vendor / speed | SOP′ Discover Identity | sniff: yes (charger asks) | `pdcap.py`, PD screen |
| Device identity | SOP Discover Identity | sniff: yes when answered | `pdcap.py` |
| QC2 requested level (by a device) | D± levels | yes, not implemented | — |
| AFC/FCP traffic (device ↔ charger) | D− serial | needs timer capture, not implemented | — |
| VBUS / current under the contract | INA226 | yes | everywhere (current unsigned) |

## Tools and verbs

- `tools/detect.py` — full detection at 5 V: dividers, `qcdet`, AFC probe,
  `fcprd` (if built), Type-C Rp + PD caps; nothing on the other port.
- `tools/pd.py` — `arm` / `off` / `sniff` / `hardreset` / `status` / `caps` /
  `req` / `info` (the PD screen's lines).
- `tools/pdcap.py` (sudo) — live KM003C capture with decoding;
  `tools/pd_decode.py` — the decoders (vendor names from the system
  `usb.ids`); `tools/pdlog_check.py` — scores a capture per hard reset
  (missed messages via GoodCRC pairing and message-id gaps).
- `tools/qc.py` — QC triggers; `tools/accp_decode.py` — AFC/FCP/SCP scope
  capture decoder; `tools/fusb_intn.py`, `tools/gpio_cc_sweep.py` — FUSB302
  INT_N and CC wiring checks.
- Verbs: `qcdet`, `qctrg`, `qc3ct`, `afctr`, `qcoff`, `fcprd`/`fcptr`/`scptr`
  (`fcp`), `pdctl` 0–3, `pdsta`, `pdcap`, `pdreq`, `pdinf` — HARDWARE.md
  "CDC verb protocol".

## Known limits on this unit

- **OUT-port CC2 contact barely connects:** with the PD line on CC2 the
  charger's Rp reaches the FUSB302 as ~1.6 V that collapses under 5.1 kΩ —
  the sink can't attach and the sniffer's hard reset doesn't reach the
  charger (receiving can still work). Use the CC1 orientation (flip a
  cable). Board fault; continuity OUT B5 ↔ IN B5 not yet measured.
- Dense-burst frame misses (above) and the 8-slot capture ring (the host
  must poll back-to-back during bursts — `pdcap.py` does).
- The current reading has no direction.

## References

- USB PD / Type-C specifications (usb.org, free download).
- Linux kernel `include/linux/usb/pd.h`, `include/linux/usb/pd_vdo.h`;
  Chromium EC `include/usb_pd_vdo.h` — field layouts.
- `FUSB302B Datasheet.pdf` (repo root) — registers, tokens, slicer.
- System `usb.ids` — vendor/product names.
- AFC/FCP/SCP sources: AFC_DECODE_NOTES.md ("Sources").
