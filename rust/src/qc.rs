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
