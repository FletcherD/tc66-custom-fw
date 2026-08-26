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
//! Entry: hold mode 0 (D+ 0.6 V, D− 0 V) for >1.25 s so a QC charger leaves
//! 5 V mode and detaches its D+/D− short; then any mode sets the voltage.
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

/// `qctrg <mode>`: request 5/9/12/20 V (mode 0..3). Runs QC entry (hold
/// mode 0 for 1.5 s) on the first call after `off()`, then sets the target
/// mode. Blocks up to ~1.5 s on entry (dev-verb, acceptable).
pub fn trigger(mode: u8) -> Result<(), u8> {
    if mode as usize >= LEVELS.len() {
        return Err(0x0F);
    }
    config_pins();
    if !unsafe { *ENTERED.0.get() } {
        drive(LEVELS[0]); /* D+ 0.6 V, D− 0 V */
        crate::board::delay_ms(1500);
        unsafe { *ENTERED.0.get() = true };
    }
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
    if !unsafe { *ENTERED.0.get() } {
        drive(LEVELS[0]);
        crate::board::delay_ms(1500);
        unsafe { *ENTERED.0.get() = true };
    }
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
 * AFC is a serial protocol on D− (D+ held at 0.6 V throughout), NOT a level
 * table. Framing recovered from the stock `proto_send_byte @0x080047b4`:
 *   serial "1" = D− PB4 high / PB5 low; "0" = both low.
 *   8 data bits MSB-first, ~152 µs each, then a 9th ODD-parity bit (set so the
 *   total number of 1-bits incl. parity is odd), then a stop.
 * Entry (stock mode-4): 200 ms D+3.3/D−3.3 preamble, D+→0.6 V held ~2050 ms
 * (D− 3.3 V), then the request byte framed on D−, repeated.
 * Request byte = voltage nibble << 4 | current nibble (public AFC spec):
 *   5V=0x00, 9V=0x46, 12V=0x66 (current nibble 6 ≈ 1.65 A). Host passes it.
 * (CHARGE_TRIGGER_FINDINGS.md; the command byte is per the public AFC spec —
 * the stock builds it in RAM cfg+0x279.) */

const BIT_US: u32 = 152; /* delay_bit_period @0x08003d42 */

/// Microsecond delay off the free-running SysTick (24-bit, 48 MHz, set up in
/// main). MUST be used for the serial bit timing rather than
/// `cortex_m::asm::delay`: that's a busy-loop and qc.rs lives in slow flash
/// (above the 0x0800_8000 zero-wait boundary), so its loop fetches stretch
/// ~7-10x and the bit period is unpredictable. SysTick counts real time
/// regardless of fetch speed. Max ~349 ms (24-bit wrap) — fine for µs.
fn us(n: u32) {
    let cyc = n * 48; /* 48 cycles/µs */
    let start = cortex_m::peripheral::SYST::get_current();
    while start.wrapping_sub(cortex_m::peripheral::SYST::get_current()) & 0x00FF_FFFF < cyc {}
}

/// One serial bit on D− (D+ stays 0.6 V). "1" = PB4 high only, "0" = both low.
fn afc_bit(one: bool) {
    drive((true, false, one, false));
    us(BIT_US);
}

/// Send one byte as a stock proto_send_word: start pattern, 8 data bits
/// MSB-first, odd-parity 9th bit, stop.
fn afc_byte(b: u8) {
    /* start pattern: D− 1 (24 µs) / 0 (32 µs) / 1 (32 µs) */
    drive((true, false, true, false));
    us(24);
    drive((true, false, false, false));
    us(32);
    drive((true, false, true, false));
    us(32);
    let mut ones = 0u32;
    for i in (0..8).rev() {
        let one = (b >> i) & 1 != 0;
        if one {
            ones += 1;
        }
        afc_bit(one);
    }
    /* 9th bit = odd parity: make the total count of 1-bits odd */
    afc_bit(ones % 2 == 0);
    /* stop: D− high then a 180 µs gap (i2c_bit_delay(180)) */
    drive((true, false, true, false));
    us(180);
}

/// `afctr <byte> [reps]`: Samsung AFC request. First call after entry/off runs
/// the ~2.25 s AFC entry, then frames the request byte `reps` times (default 3,
/// 10 ms apart). Later calls re-send without re-entry (fast voltage change).
pub fn afc(cmd: u8, reps: u8) -> Result<(), u8> {
    config_pins();
    if !unsafe { *ENTERED.0.get() } {
        /* mode-4 entry: 3.3/3.3 preamble, then D+ 0.6 V held with D− 3.3 V */
        drive((true, true, true, true));
        crate::board::delay_ms(200);
        drive((true, false, true, true)); /* D+ 0.6 V, D− 3.3 V */
        crate::board::delay_ms(2050);
        unsafe { *ENTERED.0.get() = true };
        unsafe { *IN_QC3.0.get() = false };
    }
    /* AFC (stock FUN_080034e4): send the request byte ×3 with 10 ms gaps —
     * NO sync pings (those are FCP/SCP only; the disassembly's mode-4 path
     * goes straight from the 2050 ms hold to proto_send_word(cmd) ×3). */
    let n = if reps == 0 { 3 } else { reps };
    for k in 0..n {
        afc_byte(cmd);
        if k + 1 < n {
            crate::board::delay_ms(10);
        }
    }
    /* park D− back at 3.3 V (AFC idle between requests) */
    drive((true, false, true, true));
    Ok(())
}

/// `qcoff`: release D+/D− (pins back to floating input) and forget entry.
/// The charger reverts to 5 V on its own timeout; our D± ADC read works again.
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
