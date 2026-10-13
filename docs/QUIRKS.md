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

## 1. NAK traffic stalls CPU bus accesses (~1–5 ms each) — ABSORBED

**Status (2026-08-26): the silicon behavior is presumably still there, but
the firmware no longer provokes it** — pre-armed powerz replies (no IN NAK
storms in steady state), the ISR drain loop below (OUT events serviced
promptly), fast-flash fetch (§4) and the SysTick timebase together mean no
sustained NAK phase exists for the stall to bite: powerz sustains ~5000
reads/s with zero timeouts and CDC sits at sub-ms. The underlying stall
was never explained by anything in the manual (the audit found only the
PMA-arbitration note, ns-scale) — treat it as errata-like and re-measure
only if a new traffic pattern reintroduces NAK phases. The clock tree is
exonerated: `clkrd` (2026-08-26) reads RCU_CFG0 = 0x0068000A — AHB/APB1/
APB2 all /1, so APB1 = 48 MHz, double the UM's 24 MHz USBD minimum.

**Original findings:** while a host
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

## 2. USB events are lost / the IRQ line does not re-assert — FIXED IN FIRMWARE

**Status (2026-08-26): closed by adopting the stock firmware's drain-loop
shape** (build 61). The USBD_LP ISR now services in a loop until no
endpoint shows a pending event, so one IRQ entry never strands a second
event behind a re-fire this clone may lose. Crucially the loop condition
is an **EPxCS scan (RX_ST/TX_ST, bits 15/7 — rc_w0, reads side-effect-
free), NOT `ISTR & CTR`** like stock: a bare ISTR read measurably consumes
events here (A/B below). The loop is bounded (4 passes; backpressure — a
full usbd-serial buffer holding RX_ST — is deliberately left for the
re-fire, and the ring-full park is unchanged). Bench: MK_USB_REDRAIN
counted 157 extra same-entry passes over one ~5 kHz powerz run with
MK_USB_RDMAX = 0; callback delivery went from lossy to >2 dispatches per
consumed reply; latency unchanged. The unconditional `arm()` and
write_all sweeps stay as belt-and-braces.

**Original findings:** the USBD's CTR flags should be level-triggered: a
pending endpoint event should refire USBD_LP until serviced. Measured
instead:

- In polled mode, 320 `endpoint_in_complete` callbacks for ~397 consumed
  replies (some may be legitimate coalescing between polls).
- In ISR mode, a command could park in the OUT endpoint for 8–36 ms with
  the ISR never firing — until the *next packet's edge*: a dummy "chaser"
  packet sent 2 ms after a command cut round-trip latency 2–4×.

**Manual audit (2026-08-25):** unlike STM32's RM0008, the GD32F1x0 UM
(§22.7.2) marks STIF access "r" with *no documented clear mechanism* and
never promises it stays asserted as a summary of pending EPxCS ST flags —
the level-triggered guarantee we assumed is an STM32 documentation
property GigaDevice does not make. The UM's own vocabulary (§23.1) even
includes an rc_r "read clears it" flag class.

**A/B experiment (2026-08-25, builds 50–52):** the bare diagnostic
`read_volatile(ISTR)` at USBD_LP entry (MK_USB_ISTR) vs deleted, measured
by pure EP1-IN reads against the powerz dbg counters:
- read present, slow-flash build: **0%** of consumed replies produced an
  `endpoint_in_complete` callback (0/401) — the read consumes the event
  before `dev.poll` sees it;
- read deleted, slow-flash build: **50.4%** (202/401) — better, still lossy;
- read present, **fast-flash build 55** (§4 fix): **>100%** (588/401) —
  with the whole ISR path fetching zero-wait, events flow; the residual
  loss on slow builds was the ISR being too slow to keep up, not only the
  read. CDC latency was identical in all three (the unconditional-arm +
  write_all-sweep absorptions mask event loss completely).

**Firmware absorption:** the EPxCS-conditioned drain loop above (the
vendor's own answer, from the stock firmware's `while (ISTR & CTR)` ISR —
Tier 1 below); plus belt-and-braces: `powerz::arm()` writes
unconditionally (never trusts a software armed-latch), `write_all` runs a
thread-mode `dev.poll` sweep every 64 spins, the grace window sweeps
every pass.

## 3. Two poll consumers interfere destructively — CLOSED STRUCTURALLY

**Status (2026-08-26): the second consumer is gone** (build 62). With the
ISR drain loop (§2) servicing events at the source, `write_all`'s
thread-mode `dev.poll` sweep — the last steady-state second consumer,
kept because reply drains stalled in ~50 ms steps without it on the old
single-pass ISR — was removed and A/B'd: latency identical (query 0.7 /
rawva 1.2 / getva 3.1 ms), the 516-B `pcprf` reply drains in 4 ms with no
stalls, powerz 6.5 k reads/s with zero timeouts. Steady state now has
**exactly one hardware consumer** — the same structural property that
spares the stock firmware from this quirk (its superloop never touches
ISTR/EPnR). The only remaining thread-mode sweep is the grace window,
where escape-hatch robustness outranks everything. The interference
mechanism itself was never identified (candidates: UM §22.6.2's
don't-touch-enabled-EPxCS rule, which STM32 lacks and stm32-usbd's
toggle-write model ignores; or ISTR read-sensitivity, §2) — with no
second consumer it no longer matters. RULE THAT REMAINS: never add
another `dev.poll` caller outside the ISR.

**Original findings:** A/B on the (slow-flash) ISR architecture: pure ISR
gave query 8 ms / rawva 36 ms round-trips; adding thread-mode `dev.poll`
sweeps gave 24 / 93 ms — *identical* at 4 ms-cadence and per-pass cadence,
so the harm was per-transaction, not timing contention. The residual
~8 ms staircase was RESOLVED separately as slow-flash instruction fetch
(§4): with the USB path in the fast region (build 55+), query fell
8 → 0.2 ms, rawva 36 → 0.7 ms, getva 48 → 2.3 ms.

## 4. Soft AES-256 is ~20 ms when ~1–5 ms is expected — RESOLVED

**Documented hardware behavior, fixed (2026-08-25).** The GD32F150 fetches
zero-wait only from the first 32 KB of flash (0x0800_0000–0x0800_7FFF);
above that the datasheet/UM promise "a long delay" per fetch (UM §1.3.3 /
§2.2, datasheet §3.2 — the GD32 serial-flash architecture; FMC_WS is
irrelevant, it configures the fast region). The linker had put the entire
AES implementation above the boundary. **Fix: `rust/fast.x`** pins the hot
path (ISRs, USB stack, AES/getva, main loop) below 0x0800_8000, enforced
by `make fast-gate`. Measured effect (build 53 vs 52, same code): getva
48.6 → 3.4 ms, rawva 9.2 → 1.3 ms, query 4.5 → 0.7 ms; the ~10x slow-fetch
factor also explains most of §1's "AES slowed by NAK storms" confound and
all of §3's ~8 ms staircase.

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

**Tier 1 — static analysis (free, no flashing): DONE (2026-08-25).** Answers
below; full write-up + Ghidra annotations in our unpublished
stock-firmware RE notes.

- **Vector table — does stock use the USBD interrupt?** YES. A real USBD ISR is
  installed at vector idx 53 (**IRQ37 = USBD_LP**), `usbd_lp_isr @0x08007ee0`
  (reads USB_ISTR @0x40005c44, handles CTR/RESET/SUSP/SOF/ESOF/PMAOVR). Stock is
  **fully interrupt-driven** for USB, not polled — so if our polled builds "work"
  it's polling masking the symptom, not the vendor also avoiding the IRQ.
  - **§2 cheat-sheet:** the CTR handler `usb_ctr_drain_loop @0x08007d50` is the
    classic **`while (ISTR & CTR)` drain loop** — re-read ISTR each pass, dispatch
    on `ISTR & 0xf` (EP_ID), service **all** pending EP events per IRQ entry, clear
    CTR per-endpoint via EPnR (never via ISTR). That is the fix for "events lost /
    IRQ doesn't re-assert": don't trust one CTR per IRQ; loop until CTR clears.
- **Clock init — does stock program FMC wait states/prefetch (§4)?** NO.
  `SystemInit @0x08004f70` touches **RCU only** (PLL ×12 from HSI/2 → 48 MHz); it
  never references FMC (0x40022000). The only FMC users are the flash prog/erase
  helpers, none of which write FMC_WS. **Stock inherits the bootloader's FMC config
  exactly like we do** → FMC is *not* a stock-vs-us difference; §4 stays open (read
  the bootloader or `*0x40022000` live). NB: SystemInit computes **48 MHz**, not the
  72 MHz in FINDINGS — this changes the "expected AES time" baseline.
- **USB service-loop shape.** HYBRID: ISR fills a **software** RX buffer
  (`@0x20000370`) + ready flag (`@0x20000001`); the main superloop calls
  `usb_command_dispatcher @0x080054b8` once/pass and reads only that buffer — it
  **never touches ISTR/EPnR**. So there is exactly ONE hw consumer (the ISR), which
  is structurally why stock avoids §3 (two-poll-consumer ISTR/EPnR interference) and
  sidesteps §1's register-access stall in steady state. (getva reply = 3×64 B with a
  fixed `delay_ms(10)` between blocks — the source of the observed cadence.)

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
