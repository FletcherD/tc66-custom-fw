//! Quick Charge 2.0 D+/D− trigger (M4, host-armed dev verb — nothing at boot).
//!
//! QC is NOT a CC/PD protocol: it raises a charger's output by presenting
//! fixed voltages on the pass-through D+/D− data lines. The meter synthesizes
//! those levels through a resistor network driven by four MCU GPIOs — two per
//! data line — recovered from the stock `set_dpm_trigger_gpio @0x08009b8e`:
//!
//!   D+ : PA13 = 0.6 V enable, PA14 = boost to 3.3 V
//!   D− : PB5  = 0.6 V enable, PB4  = boost to 3.3 V
//!   (drive high = on; a line reads 0 V off, 0.6 V enable-only, 3.3 V both)
//!
//! | mode | request | PA13 PA14 PB4 PB5 | D+   D−  |
//! |------|---------|-------------------|----------|
//! |  0   |   5 V   |  1    0    0   0  | 0.6  0   |
//! |  1   |   9 V   |  1    1    0   1  | 3.3  0.6 |
//! |  2   |  12 V   |  1    0    0   1  | 0.6  0.6 |
//! |  3   |  20 V   |  1    1    1   1  | 3.3  3.3 |
//!
//! Entry (`hvdcp_entry`): D+ 0.6 V with D− released for 1.5 s; a QC charger
//! opens its D+/D− short and pulls D− low, which the D− ADC sees — entry fails
//! on anything else, and `qcdet` is the same handshake as a probe. Then any
//! mode sets the voltage.
//! `qcoff` releases the four pins (D± float back to normal, charger reverts
//! to 5 V after its own timeout) and re-enables our D± ADC read.
//!
//! **PA13/PA14 are SWDIO/SWCLK** — SWD (`make markers`) is dead while a QC
//! trigger is active; `qcoff` restores it. GPIO-only, no shared bus, thread
//! mode from the verb handler — no `ina::pause` needed. **A trigger raises
//! the OUT-port voltage: nothing downstream when triggering.**

use gd32f1::gd32f150 as pac;

/// mode -> (PA13, PA14, PB4, PB5) drive levels (true = high). Order matches
/// the decoded stock table above.
const LEVELS: [(bool, bool, bool, bool); 4] = [
    (true, false, false, false), // 0: 5 V
    (true, true, false, true),   // 1: 9 V
    (true, false, false, true),  // 2: 12 V
    (true, true, true, true),    // 3: 20 V
];

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
/// Thread-exclusive (verb dispatch only): set once QC entry has run, so a
/// voltage change skips the 1.25 s re-entry. Cleared by `off()`.
static ENTERED: Racy<bool> = Racy(core::cell::UnsafeCell::new(false));
/// Set once the QC3.0 continuous base (D+ 0.6 V / D− 3.3 V) is established,
/// so a repeated `qc3` call keeps stepping instead of re-basing. Cleared by
/// `off()` and by any discrete `trigger()`.
static IN_QC3: Racy<bool> = Racy(core::cell::UnsafeCell::new(false));

/// PA13/PA14/PB4/PB5 as push-pull outputs (the drive the resistor network
/// expects — mirrors the stock config).
#[inline(never)]
fn config_pins() {
    let rcu = unsafe { &*pac::Rcu::ptr() };
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    rcu.ahben().modify(|_, w| w.paen().enabled().pben().enabled());
    gpioa.ctl().modify(|_, w| w.ctl13().output().ctl14().output());
    gpioa
        .omode()
        .modify(|_, w| w.om13().push_pull().om14().push_pull());
    gpiob.ctl().modify(|_, w| w.ctl4().output().ctl5().output());
    gpiob
        .omode()
        .modify(|_, w| w.om4().push_pull().om5().push_pull());
}

#[inline(never)]
fn drive(levels: (bool, bool, bool, bool)) {
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    let (pa13, pa14, pb4, pb5) = levels;
    /* BOP: low half sets, high half (bits +16) resets — one write per port */
    let a = (if pa13 { 1 << 13 } else { 1 << 29 }) | (if pa14 { 1 << 14 } else { 1 << 30 });
    let b = (if pb4 { 1 << 4 } else { 1 << 20 }) | (if pb5 { 1 << 5 } else { 1 << 21 });
    gpioa.bop().write(|w| unsafe { w.bits(a) });
    gpiob.bop().write(|w| unsafe { w.bits(b) });
}

/// A D± line via its ADC channel (1 = D+, 9 = D−) in mV; 0 on an ADC error.
#[inline(never)]
pub fn line_mv(ch: u8) -> u16 {
    crate::adc::read_sum16(ch).map_or(0, |s| (s * 3300 / 65520) as u16)
}

/// Below this D− counts as released/pulled low, above it as following D+
/// (QC VDAT_REF is 0.25-0.4 V; D+ sits at ~0.6 V).
const DM_LOW_MV: u16 = 300;
/// `ER` codes of a failed QC entry: no D+/D− short (not a dedicated charger),
/// or a dedicated charger that kept the short (no QC).
const E_NO_DCP: u8 = 0x20;
const E_NO_HVDCP: u8 = 0x21;

/// QC (HVDCP) handshake with detection: D+ 0.6 V, D− released, and the ADC
/// reads D− after 100 ms (a dedicated charger's D+/D− short makes it follow
/// D+) and after 1.5 s (a QC charger has opened the short and pulls D− low).
/// Fills `out` = [class, D+ mV, D− early mV, D− late mV] (class 0 = no short,
/// 1 = DCP without QC, 2 = QC); returns the class. Leaves D− released.
fn hvdcp_entry(out: &mut [u16; 4]) -> u16 {
    hvdcp_begin();
    hold_50ms(2);
    out[1] = line_mv(1);
    out[2] = line_mv(9);
    hold_50ms(28);
    out[3] = line_mv(9);
    out[0] = hvdcp_class(out[2], out[3]);
    out[0]
}

/// The handshake's drive: D+ 0.6 V, D− released (detect.rs times the reads).
pub fn hvdcp_begin() {
    config_pins();
    drive(LEVELS[0]);
    dm_drive(false);
}

/// Class from D− at 0.1 s and 1.5 s (see `hvdcp_entry`).
pub fn hvdcp_class(dm_early: u16, dm_late: u16) -> u16 {
    if dm_early < DM_LOW_MV {
        0
    } else if dm_late >= DM_LOW_MV {
        1
    } else {
        2
    }
}

/// QC entry for `trigger`/`qc3` on the first call after `off()`: the
/// detecting handshake; anything but a QC charger releases the pins and
/// fails (`E_NO_DCP` / `E_NO_HVDCP`) instead of driving levels at it.
fn qc_enter() -> Result<(), u8> {
    if unsafe { *ENTERED.0.get() } {
        return Ok(());
    }
    let mut d = [0u16; 4];
    match hvdcp_entry(&mut d) {
        2 => {
            dm_drive(true);
            unsafe { *ENTERED.0.get() = true };
            Ok(())
        }
        c => {
            let _ = off();
            Err(if c == 0 { E_NO_DCP } else { E_NO_HVDCP })
        }
    }
}

/// `qcdet`: the QC handshake as a probe — `out` = [class, D+, D− early, D−
/// late] (see `hvdcp_entry`); then releases the pins, so the charger falls
/// back to 5 V DCP. Never sets a voltage.
pub fn detect(out: &mut [u16; 4]) {
    hvdcp_entry(out);
    let _ = off();
}

/// `qctrg <mode>`: request 5/9/12/20 V (mode 0..3). The first call after
/// `off()` runs the detecting QC handshake (~1.5 s) and fails without a QC
/// charger; then sets the target mode.
pub fn trigger(mode: u8) -> Result<(), u8> {
    if mode as usize >= LEVELS.len() {
        return Err(0x0F);
    }
    config_pins();
    qc_enter()?;
    drive(LEVELS[mode as usize]);
    unsafe { *IN_QC3.0.get() = false };
    Ok(())
}

/// A single QC3.0 continuous-mode step. Exact replicas of the stock
/// `charge_trigger_pulse` pair: `up` = fn1 @0x08009b00 (D+ pulsed high 10 ms
/// = +200 mV), else fn2 @0x08009b48 (D− dipped low 10 ms = −200 mV). Both
/// leave the continuous base D+ 0.6 V / D− 3.3 V.
fn qc3_step(up: bool) {
    // tuple order: (PA13, PA14, PB4, PB5); D− 0.6 V = PB5 on / PB4 off.
    if up {
        drive((true, true, true, true)); // D+ 3.3, D- 3.3
        crate::board::delay_ms(10);
        drive((true, false, true, true)); // D+ 0.6, D- 3.3 (base)
    } else {
        drive((true, false, false, true)); // D+ 0.6, D- 0.6 (PB5 on, PB4 off)
        crate::board::delay_ms(10);
        drive((true, false, true, true)); // D+ 0.6, D- 3.3 (base)
    }
}

/// `qc3 <dir> <count>`: QC3.0 continuous fine-stepping. dir 0 = up (+200 mV),
/// 1 = down. The first call after entry/off runs QC entry (if needed) and
/// establishes the continuous base (D+ 0.6 V / D− 3.3 V); later calls keep
/// stepping from where they left off. `qcoff` or any `qctrg` exits QC3.
pub fn qc3(down: bool, count: u8) -> Result<(), u8> {
    config_pins();
    qc_enter()?;
    if !unsafe { *IN_QC3.0.get() } {
        /* enter continuous mode: settle at the base D+ 0.6 V / D− 3.3 V */
        drive((true, false, true, true));
        crate::board::delay_ms(100);
        unsafe { *IN_QC3.0.get() = true };
    }
    for _ in 0..count {
        qc3_step(!down);
        /* dwell at the base between pulses — back-to-back edges merge into
         * one and the charger counts a single step (bench-confirmed); the
         * stock negotiator likewise spaces its pulse calls */
        crate::board::delay_ms(20);
    }
    Ok(())
}

/* ---- Samsung AFC (Adaptive Fast Charging, stock handshake mode 4) ----
 *
 * CLOSED-LOOP single-wire serial on D− (D+ held at 0.6 V throughout). Replicates
 * the stock `charge_protocol_handshake` mode 4 register-for-register
 * (CHARGE_TRIGGER_FINDINGS.md); the byte framing matches the stock afc.csv
 * capture symbol-for-symbol AND the charger's Sping is read back on PB4.
 *
 * Serial levels (stock set_dpm_trigger_gpio network): mark / "1" = D− PB4 high,
 * PB5 low (~3.3 V); "0" = both low (0 V). D+ = PA13 high (0.6 V) the whole time.
 *
 * Per byte (`proto_send_byte`): start pattern (D− high 24 µs / low 32 µs / high
 * 32 µs), 8 data bits MSB-first at ~152 µs each (NRZ), a 9th ODD-parity bit,
 * then a stop. `proto_send_word` wraps it CLOSED-LOOP: Mping (D− driven high
 * 2.5 ms via `proto_line_idle`, then RELEASED to hi-Z) → `proto_wait_sping`
 * (READ PB4: wait ≤3 ms for the charger's Sping rising edge, ≤3 ms for falling)
 * → byte → Mping → wait Sping → `proto_measure_response` (read the charger's
 * byte-echo) → Mping → wait Sping. An open-loop replay that never reads PB4 does
 * NOT advance the charger's FSM (the charger Spings but the request never
 * completes — observed). Entry: release all 4 pins 200 ms, then drive D+ to
 * 0.6 V with D− RELEASED for 2050 ms, then the request byte (0x46 = 9 V,
 * 0x08 = 5 V) ×3, 10 ms apart. AFC has NO sync pings (FCP/SCP only).
 *
 * The byte's bit timing runs with interrupts masked: the sampler (250 Hz, ~0.5
 * ms I2C reads) or USB ISR firing mid-bit stretches a cell and corrupts the
 * frame. The 2.5 ms marks and the read-based waits tolerate ISR jitter. */

/// Microsecond delay off the free-running SysTick (24-bit, 48 MHz, set up in
/// main). MUST be used for the serial bit timing rather than
/// `cortex_m::asm::delay`: that's a busy-loop and qc.rs lives in slow flash
/// (above the 0x0800_8000 zero-wait boundary), so its loop fetches stretch
/// ~7-10x and the bit period is unpredictable. SysTick counts real time
/// regardless of fetch speed. Max ~349 ms (24-bit wrap) — fine for µs.
#[inline(never)]
fn us(n: u32) {
    let cyc = n * 48; /* 48 cycles/µs */
    let start = cortex_m::peripheral::SYST::get_current();
    while start.wrapping_sub(cortex_m::peripheral::SYST::get_current()) & 0x00FF_FFFF < cyc {}
}

const BIT_US: u32 = 152; /* delay_bit_period @0x08003d42 */

/// Hold the current drive for `n` × 50 ms (SysTick `us` maxes at ~349 ms/call,
/// so multi-second entry holds must loop).
fn hold_50ms(n: u32) {
    for _ in 0..n {
        us(50_000);
    }
}

/// D− to the mark / "1" level = PB4 high, PB5 low (~3.3 V); D+ stays 0.6 V.
#[inline(never)]
fn dm_mark() {
    drive((true, false, true, false));
}
/// D− to "0" = both PB4/PB5 low (0 V); D+ stays 0.6 V.
fn dm_space() {
    drive((true, false, false, false));
}

/// PB4/PB5 (D−) as push-pull outputs (drive) or floating inputs (release, hi-Z).
/// Releasing lets the charger drive D− for its Sping AND lets us read PB4.
/// D+ (PA13/PA14) is untouched.
#[inline(never)]
fn dm_drive(drive: bool) {
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    if drive {
        gpiob.ctl().modify(|_, w| w.ctl4().output().ctl5().output());
    } else {
        gpiob.ctl().modify(|_, w| w.ctl4().input().ctl5().input());
    }
}

/// PA13/PA14 (D+) as push-pull outputs (drive) or floating inputs (released).
fn dp_drive(drive: bool) {
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    if drive {
        gpioa.ctl().modify(|_, w| w.ctl13().output().ctl14().output());
    } else {
        gpioa.ctl().modify(|_, w| w.ctl13().input().ctl14().input());
    }
}

/// Read D− via PB4's input register (valid while released/hi-Z).
fn dm_read() -> bool {
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    (gpiob.istat().read().bits() >> 4) & 1 != 0
}

/// Spin reading PB4 until it equals `want`, or `to_us` µs elapse (SysTick).
/// Returns false on timeout.
fn wait_pb4(want: bool, to_us: u32) -> bool {
    let lim = to_us * 48;
    let start = cortex_m::peripheral::SYST::get_current();
    while dm_read() != want {
        if start.wrapping_sub(cortex_m::peripheral::SYST::get_current()) & 0x00FF_FFFF >= lim {
            return false;
        }
    }
    true
}

/// Stock `proto_wait_sping` @0x08004eb4: after a Mping, wait for the charger's
/// Sping on D−. First let the released mark FALL (so the residual mark-high is
/// not mistaken for the Sping), then wait for the charger's rising edge (≤3 ms)
/// and its falling edge (≤3 ms). Reading this — not blind-delaying — is what
/// advances the charger's protocol FSM, and waiting out the whole Sping keeps
/// the next byte from colliding with it.
/// Returns whether a Sping (rise and fall) was seen.
fn proto_wait_sping() -> bool {
    wait_pb4(false, 3000); /* mark falls / line settles low */
    let rose = wait_pb4(true, 3000); /* charger Sping rising */
    let fell = wait_pb4(false, 3000); /* charger Sping falling */
    rose && fell
}

/// Stock `proto_measure_response` @0x08004ef4: with D− released, wait out the
/// charger's response high pulse (its byte-echo ACK) and its end (~20 ms cap).
/// The caller's preceding `proto_wait_sping` already left the line low.
fn proto_measure_response() {
    wait_pb4(true, 20000); /* charger echo high */
    wait_pb4(false, 20000); /* echo end */
}

/// Stock `proto_line_idle`: drive D− to the mark push-pull for 2.5 ms (the
/// master "Mping"), then release D− (hi-Z) so the charger can Sping.
#[inline(never)]
fn proto_line_idle() {
    dm_drive(true);
    dm_mark();
    us(2500);
    dm_drive(false);
}

/// Stock start pattern (once per word, before the bytes): D− high 24 µs /
/// low 32 µs / high 32 µs. Leaves D− driven push-pull.
fn afc_start() {
    dm_drive(true);
    dm_mark();
    us(24);
    dm_space();
    us(32);
    dm_mark();
    us(32);
}

/// Stock `proto_send_byte` body: 8 data bits MSB-first at BIT_US, a 9th
/// odd-parity bit, then a stop (D− high + 180 µs). No start pattern (the word
/// emits that once). Assumes D− already driven push-pull.
///
/// ASSUMPTION (untested, AFC-derived): parity is PER-BYTE odd parity. The RE
/// calls it a "running-parity bit" with scratch at 0x2000029d — for a
/// multi-byte packet this may instead be a parity running across the whole
/// packet. Confirm against `proto_send_byte @0x080047b4` before trusting
/// FCP/SCP writes. AFC (1 byte) is unaffected (per-byte == running for len 1).
fn afc_byte_bits(b: u8) {
    for i in (0..8).rev() {
        if (b >> i) & 1 != 0 {
            dm_mark();
        } else {
            dm_space();
        }
        us(BIT_US);
    }
    /* 9th bit = odd parity: mark when popcount(b) is even (XOR-fold parity). */
    let mut p = b;
    p ^= p >> 4;
    p ^= p >> 2;
    p ^= p >> 1;
    if p & 1 == 0 {
        dm_mark();
    } else {
        dm_space();
    }
    us(BIT_US);
    /* stop: D− high then a 180 µs gap */
    dm_mark();
    us(180);
}

/// One stock `proto_send_word(buf, len, flag)` — the CLOSED-LOOP handshake
/// (`@0x08004d60`): Mping → wait charger Sping → start + `buf` bytes → [if flag:
/// Mping → wait Sping → measure the charger's echo] → Mping → wait Sping. Only
/// the bit timing is interrupt-masked; the marks and the (read-based) waits
/// tolerate ISR jitter.
/// Returns whether the charger Spinged after every Mping.
fn proto_send_word(buf: &[u8], flag: bool) -> bool {
    proto_line_idle();
    let mut ok = proto_wait_sping();
    us(250);
    cortex_m::interrupt::free(|_| {
        afc_start();
        for &b in buf {
            afc_byte_bits(b);
        }
    });
    if flag {
        proto_line_idle();
        ok &= proto_wait_sping();
        proto_measure_response();
        us(180);
    }
    proto_line_idle();
    ok &= proto_wait_sping();
    ok
}

/// `afctr <byte> [reps]`: Samsung AFC request. First call after entry/off runs
/// the ~2.25 s AFC mode-4 entry, then frames the request byte `reps` times
/// (default 3, 10 ms apart). Later calls re-send without re-entry (fast voltage
/// change). `0x46` = 9 V, `0x08` = 5 V (high nibble = 5 V + n; low = current).
/// `E_NO_AFC` if the charger never completed a closed-loop exchange, so
/// `afctr 0x08` doubles as a 5 V AFC probe.
pub fn afc(cmd: u8, reps: u8) -> Result<(), u8> {
    config_pins();
    accp_entry(); /* mode-4 entry, shared with FCP/SCP */
    let n = if reps == 0 { 3 } else { reps };
    let mut acked = false;
    for k in 0..n {
        acked |= proto_send_word(&[cmd], true);
        if k + 1 < n {
            us(10_000); /* 10 ms gap between repeats */
        }
    }
    /* leave D− released (hi-Z); D+ stays 0.6 V and the charger holds the
     * negotiated voltage. */
    dm_drive(false);
    if acked {
        Ok(())
    } else {
        Err(E_NO_AFC)
    }
}

/// `afctr` ER code: no repeat got a Sping after every Mping (no AFC charger).
const E_NO_AFC: u8 = 0x22;

/* ---- Huawei FCP / SCP (charge_protocol_handshake modes 2 / 3) ----
 *
 * UNTESTED on hardware. Same HiSilicon ACCP physical layer and entry as AFC
 * (release-all → D+ 0.6 V, D− released, 2050 ms), then 3 `01` sync pings
 * 900 µs apart and register packets 10 ms apart, each ending in a CRC-8 (poly
 * 0x39, init 0, MSB-first — `crc8_huawei @0x08003a2c`). Grammar:
 *   0c = read8   <0c><reg><crc>            reply <08><val><crc>
 *   0b = write8  <0b><reg><val><crc>       reply <08><crc>
 *   1b = write16 <1b><reg><len><d0><d1><crc>
 * ACK = low nibble 8 (0x08/0x88), NACK = 0x03. Framing, timing and replies
 * are taken from the stock 100 kS/s captures (fcp.csv/scp.csv, a real FCP
 * charger attached) — docs/AFC_DECODE_NOTES.md "Frame format". AFC keeps its
 * own bench-verified framing (afc_start/afc_byte_bits/proto_send_word).
 * Transaction: Mping → Sping → frame → Mping → Sping → [charger frame →
 * charger end ping (~2.5 ms high) → 1.7 ms] → Mping → Sping. */

/// SYNC quarter-pulse width (UI/4; the captures show ~40 µs).
const SYNC_US: u32 = 40;
/// Charger bit cell used to decode replies (the captures show 150-160 µs).
const RX_UI_US: u32 = 155;

/// Spin `n` µs on SysTick (fast-flash copy of `us` for the framer).
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
#[link_section = ".fasttext.qc_spin_us"]
#[inline(never)]
fn spin_us(n: u32) {
    let cyc = n * 48;
    let start = cortex_m::peripheral::SYST::get_current();
    while start.wrapping_sub(cortex_m::peripheral::SYST::get_current()) & 0x00FF_FFFF < cyc {}
}

/// D− level while driven: mark (PB4 high, PB5 low) or 0 V (both low).
#[inline(always)]
fn dm_level(high: bool) {
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    gpiob
        .bop()
        .write(|w| unsafe { w.bits(if high { 1 << 4 | 1 << 21 } else { 1 << 20 | 1 << 21 }) });
}

/// SYNC before a byte (or the end ping): opposite of the previous level `p`,
/// back to `p`, and one more opposite quarter if the next bit equals `p` — so
/// every byte starts on an edge. Matches every frame in the stock captures.
#[link_section = ".fasttext.qc_accp_sync"]
#[inline(never)]
fn accp_sync(p: bool, next: bool) {
    dm_level(!p);
    spin_us(SYNC_US);
    dm_level(p);
    spin_us(SYNC_US);
    if next == p {
        dm_level(!p);
        spin_us(SYNC_US);
    }
}

/// One master frame: per byte a SYNC, 8 data bits MSB-first and an odd-parity
/// bit (per byte), then a closing SYNC toward the (high) Mping that follows.
/// Interrupts masked: a stretched cell corrupts the frame. Leaves D− driven.
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
#[link_section = ".fasttext.qc_accp_frame"]
#[inline(never)]
fn accp_frame(buf: &[u8]) {
    dm_drive(true);
    cortex_m::interrupt::free(|_| {
        let mut p = false; /* line idles low after the Sping */
        for &b in buf {
            accp_sync(p, b & 0x80 != 0);
            for i in (0..8).rev() {
                dm_level((b >> i) & 1 != 0);
                spin_us(BIT_US);
            }
            p = b.count_ones() % 2 == 0;
            dm_level(p);
            spin_us(BIT_US);
        }
        accp_sync(p, true);
    });
}

/// Record the charger's reply frame as alternating run lengths in µs, the
/// first one high. Waits ≤ 3 ms for the first edge; stops at the charger's end
/// ping (a high ≥ 1.8 ms — a byte's longest high run is 9 bits ≈ 1.4 ms), a
/// 4 ms low, 15 ms total, or a full buffer. D− must be released. Returns the
/// run count (0 = nothing came).
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
#[link_section = ".fasttext.qc_accp_capture"]
#[inline(never)]
fn accp_capture(runs: &mut [u16]) -> usize {
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    let now = cortex_m::peripheral::SYST::get_current;
    let since = |t: u32| t.wrapping_sub(now()) & 0x00FF_FFFF;
    let pb4 = || (gpiob.istat().read().bits() >> 4) & 1 != 0;
    cortex_m::interrupt::free(|_| {
        let t0 = now();
        while !pb4() {
            if since(t0) >= 3000 * 48 {
                return 0;
            }
        }
        let start = now();
        let mut t = start;
        let mut level = true;
        let mut n = 0;
        loop {
            let el = since(t);
            if pb4() != level {
                match runs.get_mut(n) {
                    Some(r) => *r = (el / 48) as u16,
                    None => break,
                }
                n += 1;
                level = !level;
                t = now();
            } else if (level && el >= 1800 * 48) || el >= 4000 * 48 || since(start) >= 15_000 * 48 {
                break;
            }
        }
        n
    })
}

/// Decode `runs` (alternating, first high) into bytes: a run < 90 µs is a SYNC
/// quarter (byte boundary), anything longer is round(run / RX_UI_US) bits.
/// Each byte must be 9 bits with odd parity; the last byte must be the CRC-8
/// of the others. Returns the byte count without the CRC; Err(()) if garbled.
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
fn accp_decode(runs: &[u16], out: &mut [u8]) -> Result<usize, ()> {
    let mut level = true;
    let mut bits: u16 = 0;
    let mut nb = 0u32;
    let mut n = 0usize;
    for &r in runs {
        let r = r as u32;
        if r < 90 {
            if nb > 0 {
                if nb != 9 || bits.count_ones() % 2 != 1 {
                    return Err(());
                }
                *out.get_mut(n).ok_or(())? = (bits >> 1) as u8;
                n += 1;
                bits = 0;
                nb = 0;
            }
        } else {
            for _ in 0..(r + RX_UI_US / 2) / RX_UI_US {
                bits = bits << 1 | level as u16;
                nb += 1;
                if nb > 9 {
                    return Err(());
                }
            }
        }
        level = !level;
    }
    if nb != 0 || n < 2 {
        return Err(());
    }
    let (body, crc) = out.get(..n).ok_or(())?.split_at(n - 1);
    if crc.first() != Some(&crc8(body)) {
        return Err(());
    }
    Ok(n - 1)
}

/// CRC-8, poly 0x39, init 0x00, MSB-first, over `data` (`crc8_huawei`).
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x39
            } else {
                crc << 1
            };
        }
    }
    crc
}

/// FCP/SCP shared entry: release all four pins 200 ms, then drive D+ to 0.6 V
/// with D− released for 2050 ms (identical to AFC — modes 2/3/4 share it).
fn accp_entry() {
    if unsafe { *ENTERED.0.get() } {
        return;
    }
    accp_release();
    hold_50ms(4); /* 200 ms preamble, all released */
    accp_hold();
    hold_50ms(41); /* 2050 ms hold */
    accp_entered();
}

/// Entry phase 1 (`accp_entry`; detect.rs runs the phases from the main
/// loop): all four pins released.
pub fn accp_release() {
    config_pins();
    dp_drive(false);
    dm_drive(false);
}

/// Entry phase 2: D+ 0.6 V, D− released.
pub fn accp_hold() {
    dp_drive(true);
    drive((true, false, false, false));
}

/// Entry complete: the next `afc`/`fcp*` call skips the entry.
pub fn accp_entered() {
    unsafe {
        *ENTERED.0.get() = true;
        *IN_QC3.0.get() = false;
    }
}

/// Mping → Sping → `frame` → Mping → Sping.
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
#[inline(never)]
fn accp_send(frame: &[u8]) {
    proto_line_idle();
    proto_wait_sping();
    us(250);
    accp_frame(frame);
    proto_line_idle();
    proto_wait_sping();
}

/// The 3 leading `01` sync pings (no CRC, no reply), 900 µs apart, then the
/// 10 ms gap before the first packet.
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
fn accp_pings() {
    for k in 0..3 {
        accp_send(&[0x01]);
        if k < 2 {
            us(900);
        }
    }
    us(10_000);
}

/// Error byte for packet `i`: 0x40|i = no / garbled reply, 0x50|i = NACK.
const E_NOREPLY: u8 = 0x40;
const E_NACK: u8 = 0x50;

/// One register transaction: send `pkt` + CRC, read the charger's reply into
/// `reply` (ACK first). Returns the reply length (ACK + data, no CRC); `i`
/// tags the error byte.
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
fn accp_xfer(pkt: &[u8], reply: &mut [u8], i: u8) -> Result<usize, u8> {
    let mut frame = [0u8; 8];
    let n = pkt.len().min(7);
    frame[..n].copy_from_slice(&pkt[..n]);
    frame[n] = crc8(&frame[..n]);
    accp_send(&frame[..n + 1]);
    let mut runs = [0u16; 96];
    let nr = accp_capture(&mut runs);
    wait_pb4(false, 4000); /* charger end ping falls */
    us(1700);
    proto_line_idle();
    proto_wait_sping();
    let len = accp_decode(runs.get(..nr).unwrap_or(&[]), reply).map_err(|_| E_NOREPLY | i)?;
    if reply[0] & 0x0F != 0x08 {
        return Err(E_NACK | i);
    }
    Ok(len)
}

/// Run `packets` as one exchange (pings first), 10 ms apart; stops at the
/// first failed packet.
#[cfg_attr(not(feature = "fcp"), allow(dead_code))]
fn accp_exchange(packets: &[&[u8]]) -> Result<(), u8> {
    accp_pings();
    let mut reply = [0u8; 8];
    for (i, pkt) in packets.iter().enumerate() {
        if i > 0 {
            us(10_000);
        }
        let r = accp_xfer(pkt, &mut reply, i as u8);
        if r.is_err() {
            dm_drive(false);
            return r.map(|_| ());
        }
    }
    dm_drive(false); /* leave D− released; charger holds the voltage */
    Ok(())
}

/// `fcptr`: Huawei FCP 9 V request — the stock mode-2 exchange: read regs
/// 04/21/31, write VOUT 0x2c = 90 (9 V, 100 mV units), read it back, write
/// OUTPUT_CONTROL 0x2b = 1 (apply). Err = 0x40|i / 0x50|i for packet i.
#[cfg(feature = "fcp")] /* untested — not in release builds */
pub fn fcp() -> Result<(), u8> {
    config_pins();
    accp_entry();
    accp_exchange(&[
        &[0x0c, 0x04],
        &[0x0c, 0x21],
        &[0x0c, 0x31],
        &[0x0b, 0x2c, 0x5a],
        &[0x0c, 0x2c],
        &[0x0b, 0x2b, 0x01],
    ])
}

/// `fcprd <reg> [n]`: read `out.len()` consecutive FCP/SCP registers from
/// `reg` with single reads — no write, so the voltage never changes
/// (capability readout: 0x21 = number of raised voltages N, 0x30+k = voltage
/// k in 100 mV, 0x22 = max power in 0.5 W). Err as `fcp` (i = register index).
#[cfg(feature = "fcp")] /* untested — not in release builds */
pub fn fcp_read(reg: u8, out: &mut [u8]) -> Result<(), u8> {
    config_pins();
    accp_entry();
    accp_pings();
    let mut reply = [0u8; 8];
    let mut r = Ok(());
    for (k, o) in out.iter_mut().enumerate() {
        if k > 0 {
            us(10_000);
        }
        let i = k as u8;
        match accp_xfer(&[0x0c, reg.wrapping_add(i)], &mut reply, i) {
            Ok(2) => *o = reply[1],
            Ok(_) => r = Err(E_NOREPLY | i),
            Err(e) => r = Err(e),
        }
        if r.is_err() {
            break;
        }
    }
    dm_drive(false);
    r
}

/// `scptr`: Huawei SCP — the stock mode-3 block: enable (0xa0 = 0xc0), 3 s
/// watchdog (0xa1 = 0x06), VSET/ISET boundaries 0xb0 = 5500 mV / 0xb2 =
/// 5000 mA, VSSET 0xca = 0xc8 (5.00 V), ISSET 0xcb = 0x64 (5 A) — the 5 V
/// low-voltage direct-charge mode; it does not raise the voltage. Err as `fcp`.
#[cfg(feature = "fcp")] /* untested — not in release builds */
pub fn scp() -> Result<(), u8> {
    config_pins();
    accp_entry();
    accp_exchange(&[
        &[0x0b, 0xa0, 0xc0],
        &[0x0b, 0xa1, 0x06],
        &[0x1b, 0xb0, 0x02, 0x15, 0x7c],
        &[0x1b, 0xb2, 0x02, 0x13, 0x88],
        &[0x0b, 0xca, 0xc8],
        &[0x0b, 0xcb, 0x64],
    ])
}

/// `qcoff`: release D+/D− (pins back to floating input) and forget entry.
/// The charger reverts to 5 V on its own timeout; our D± ADC read works again.
#[inline(never)]
pub fn off() -> Result<(), u8> {
    let rcu = unsafe { &*pac::Rcu::ptr() };
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    rcu.ahben().modify(|_, w| w.paen().enabled().pben().enabled());
    gpioa.ctl().modify(|_, w| w.ctl13().input().ctl14().input());
    gpiob.ctl().modify(|_, w| w.ctl4().input().ctl5().input());
    unsafe {
        *ENTERED.0.get() = false;
        *IN_QC3.0.get() = false;
    }
    Ok(())
}
