# GD32F150 quirks & open investigations

Bench-measured misbehavior of this TC66's MCU (GigaDevice GD32F150 — an
STM32-compatible part: STM32F0-style peripheral IP around a Cortex-M3; the
`stm32-usbd` stack drives its USBD unmodified except for the base address).
Each entry: what was measured, how confident we are, how the current
firmware absorbs it, and what would settle it. Keep this file the single
home for chip-behavior findings — FIRMWARE.md describes the *firmware*,
this file describes the *silicon* (and where the two are hard to separate,
says so).

Everything here was measured on THIS unit, through the bench VL817 hub
(full-speed device behind a USB3 hub's transaction translator) — host-side
contributions are not fully excluded unless stated.

## 1. NAK traffic stalls CPU bus accesses (~1–5 ms each)

**Confidence: high (three independent measurements).** While a host
NAK-polls any of our bulk endpoints (IN awaiting data, or OUT awaiting
buffer space), CPU accesses to APB peripherals stall for milliseconds:

- Timestamp probes (TIMER5 CNT reads on APB1) bracketing ~300 instructions
  measured 1.1 ms; a 10k-iteration register-only busy loop in the same
  function ran at a normal 9 cycles/iteration — the probes themselves were
  stalling, not the code.
- The polled main loop (one TIMER5 read + one USBD ISTR read per pass)
  degraded to ~70 passes/s under storm — self-sustaining, since a slow loop
  keeps the endpoint unserviced and the storm alive.
- Moving the timebase to SysTick (core-internal, never stalls) fixed the
  loop degradation outright.

On a genuine STM32, NAKs are handled by the SIE for free; only CPU accesses
to the USB's own registers/PMA are documented to wait-state, and by
nanoseconds not milliseconds. Straight-line flash-heavy code (AES) also
appears slowed under storm while tight loops are not — suggesting the stall
reaches instruction fetch, but this is confounded (see §4).

**Firmware absorption:** SysTick timebase; powerz replies pre-armed so IN
storms never form in steady state; ring-full parks the ISR so OUT storms
stay brief.

## 2. USB events are lost / the IRQ line does not re-assert

**Confidence: high.** The USBD's CTR flags should be level-triggered: a
pending endpoint event should refire USBD_LP until serviced. Measured
instead:

- In polled mode, 320 `endpoint_in_complete` callbacks for ~397 consumed
  replies (some may be legitimate coalescing between polls).
- In ISR mode, a command could park in the OUT endpoint for 8–36 ms with
  the ISR never firing — until the *next packet's edge*: a dummy "chaser"
  packet sent 2 ms after a command cut round-trip latency 2–4×.

**Firmware absorption:** `powerz::arm()` writes unconditionally (never
trusts a software armed-latch); `write_all` runs a thread-mode `dev.poll`
sweep every 64 spins (reply drains stalled in ~50 ms steps without it);
the grace window sweeps every pass.

## 3. Two poll consumers interfere destructively

**Confidence: measured effect, unknown mechanism.** A/B on the ISR
architecture: pure ISR gives query 8 ms / rawva 36 ms round-trips; adding
thread-mode `dev.poll` sweeps gives 24 / 93 ms — *identical* at 4 ms-cadence
and per-pass cadence, so the harm is per-transaction (some stateful
ISTR/EPnR interaction between contexts), not timing contention. This is why
the main loop deliberately does NOT sweep (see the comment in main.rs).
Residual latency shows ~8 ms quantization that looks like a host NAK-retry
schedule (possibly the hub TT's).

## 4. Soft AES-256 is ~20 ms when ~1–5 ms is expected

**Confidence: measured, cause unresolved.** Every measurement so far ran
while the host was NAK-polling for the reply (unavoidable in
request/response), so §1's fetch-stall and a genuine flash misconfiguration
are confounded. Note `clock_init` never touches the FMC wait-state/prefetch
config — we inherit whatever the bootloader set.

**Discriminating experiment (cheap, not yet run):** time `getva::build`
with device-side SysTick stamps but *report* the result in a later reply,
so nothing polls during the measurement. Also dump the FMC wait-state
register via a dev verb.

## 5. Not silicon, but adjacent (documented elsewhere, listed for map)

- Bootloader hand-off faults if the flash header's initial SP is
  0x20002000 (predicate unknown; 0x20000E40 is proven — FIRMWARE.md).
- Writing a data endpoint before the first bus reset creates a phantom
  endpoint-0 (EPnR address field still 0) and wedges enumeration — arguably
  correct-per-IP-model, but brutal (FIRMWARE.md).
- NTC divider on ADC ch1 on this unit where stock reads ch3 (board rev
  variation — HARDWARE.md).

## Settling "silicon vs our stack": the stock-firmware comparison

The stock firmware is production code running on this exact silicon —
a uniquely good discriminator, in two tiers:

**Tier 1 — static analysis (free, no flashing):** `firmware/V118.bin`
decrypts with the known image AES key. Read out:
- The vector table: does stock enable/use the USBD interrupt at all, or is
  it purely polled? If stock uses the IRQ, its handler will contain any
  vendor workarounds for §2 — a cheat sheet. If stock is purely polled,
  that is soft evidence the vendor avoided the IRQ path on purpose.
- The clock init: does stock program FMC wait states/prefetch (§4)?
- The USB service loop shape: continuous polling (would mask §1–§3
  entirely, matching our polled builds' behavior).

**Tier 2 — dynamic (costs one K1 cycle to return!):** flash stock, measure
getva RTT distribution at several poll rates through the same hub/host. If
stock also shows the ~8 ms quanta → host/hub-side, not us. If stock is
~1 ms like our polled builds → consistent with polled-masks-everything.
NOTE: stock has no `updat`; returning to custom firmware needs the
bootloader (K1 held on power-up + `make recover`). Do Tier 1 first.

**Limits:** stock not exhibiting a symptom only proves the *path stock
takes* is safe — it cannot exonerate paths stock never exercises (our ISR
mode, powerz's second bulk pair, composite descriptors). The GD32F1x0 user
manual's USBD chapter and GigaDevice errata sheets are the other unread
primary source and should be checked alongside (ISTR/CTR semantics, IRQ
line behavior). Ground truth beyond that: USB-level capture with the bench
logic analyzer (FS = 12 Mb/s; check the SLogic16's usable rate first).
