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
fn wait_pb4(want: bool, to_us: u32) {
    let lim = to_us * 48;
    let start = cortex_m::peripheral::SYST::get_current();
    while dm_read() != want {
        if start.wrapping_sub(cortex_m::peripheral::SYST::get_current()) & 0x00FF_FFFF >= lim {
            break;
        }
    }
}

/// Stock `proto_wait_sping` @0x08004eb4: after a Mping, wait for the charger's
/// Sping on D−. First let the released mark FALL (so the residual mark-high is
/// not mistaken for the Sping), then wait for the charger's rising edge (≤3 ms)
/// and its falling edge (≤3 ms). Reading this — not blind-delaying — is what
/// advances the charger's protocol FSM, and waiting out the whole Sping keeps
/// the next byte from colliding with it.
fn proto_wait_sping() {
    wait_pb4(false, 3000); /* mark falls / line settles low */
    wait_pb4(true, 3000); /* charger Sping rising */
    wait_pb4(false, 3000); /* charger Sping falling */
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
fn proto_send_word(buf: &[u8], flag: bool) {
    proto_line_idle();
    proto_wait_sping();
    us(250);
    cortex_m::interrupt::free(|_| {
        afc_start();
        for &b in buf {
            afc_byte_bits(b);
        }
    });
    if flag {
        proto_line_idle();
        proto_wait_sping();
        proto_measure_response();
        us(180);
    }
    proto_line_idle();
    proto_wait_sping();
}

/// `afctr <byte> [reps]`: Samsung AFC request. First call after entry/off runs
/// the ~2.25 s AFC mode-4 entry, then frames the request byte `reps` times
/// (default 3, 10 ms apart). Later calls re-send without re-entry (fast voltage
/// change). `0x46` = 9 V, `0x08` = 5 V (high nibble = 5 V + n; low = current).
pub fn afc(cmd: u8, reps: u8) -> Result<(), u8> {
    config_pins();
    accp_entry(); /* mode-4 entry, shared with FCP/SCP */
    let n = if reps == 0 { 3 } else { reps };
    for k in 0..n {
        proto_send_word(&[cmd], true);
        if k + 1 < n {
            us(10_000); /* 10 ms gap between repeats */
        }
    }
    /* leave D− released (hi-Z); D+ stays 0.6 V and the charger holds the
     * negotiated voltage. */
    dm_drive(false);
    Ok(())
}

/* ---- Huawei FCP / SCP (charge_protocol_handshake modes 2 / 3) ----
 *
 * Same HiSilicon ACCP physical layer as AFC (D− serial, closed-loop
 * proto_send_word), same entry (release-all → drive D+ 0.6 V, D− released,
 * 2050 ms). Differences: 3 leading `01` sync pings 900 µs apart, then multi-byte
 * register packets, each ending in a CRC-8 (poly 0x39, init 0, MSB-first, over
 * all preceding bytes — `crc8_huawei @0x08003a2c`, verified bit-exact). Grammar:
 *   0c = read8   <0c><reg><crc>
 *   0b = write8  <0b><reg><val><crc>
 *   1b = write16 <1b><reg><len><d0><d1><crc>
 * Packets recovered from the stock scatter-decompressed .data (all CRC-verified).
 *
 * OPEN QUESTIONS (asked the user to RE — currently ASSUMPTIONS, marked below):
 *   (a) per-packet `flag` (does each read/write set flag=1 = measure the
 *       charger response, or flag=0)? Pings are flag=0; assumed data = flag=1.
 *   (b) parity per-byte vs running across the packet (see afc_byte_bits).
 *   (c) inter-packet gap after the 3× 900 µs pings; assumed 10 ms (AFC's gap).
 *   (d) whether a trigger needs the full read+write sequence or just the writes.
 * FCP is fixed-voltage (6 packets: read 04/21/31, write 2c=90→9V, read 2c,
 * write 2b=01 commit). SCP reg 0xca = voltage code (raw scale, unknown volts;
 * the stock scan probes 0xc8=200 and 0xdc=220). */

/// CRC-8, poly 0x39, init 0x00, MSB-first, over `data` (`crc8_huawei`).
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
    dp_drive(false);
    dm_drive(false);
    hold_50ms(4); /* 200 ms preamble, all released */
    dp_drive(true);
    drive((true, false, false, false)); /* D+ 0.6 V, D− released */
    hold_50ms(41); /* 2050 ms hold */
    unsafe { *ENTERED.0.get() = true };
    unsafe { *IN_QC3.0.get() = false };
}

/// Send the 3 leading `01` sync pings (900 µs apart), then each command packet.
/// A packet is `[opcode, reg, (val…)]`; `crc8` is appended here. ASSUMPTION:
/// data packets use flag=true, gap = 10 ms (see the module note).
fn accp_exchange(packets: &[&[u8]]) {
    for k in 0..3 {
        proto_send_word(&[0x01], false); /* sync ping, flag=0 */
        if k < 2 {
            us(900);
        }
    }
    us(10_000);
    let mut frame = [0u8; 8];
    for (i, pkt) in packets.iter().enumerate() {
        let n = pkt.len();
        frame[..n].copy_from_slice(pkt);
        frame[n] = crc8(pkt);
        proto_send_word(&frame[..n + 1], true);
        if i + 1 < packets.len() {
            us(10_000);
        }
    }
    dm_drive(false); /* leave D− released; charger holds the voltage */
}

/// `fcptr`: Huawei FCP 9 V request. Entry, 3 pings, then the stock mode-2
/// 6-packet exchange: read regs 04/21/31, write VOUT_CONFIG 0x2c = 90 (→ 9 V,
/// 100 mV units), read it back, write OUTPUT_CONTROL 0x2b = 1 (commit).
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
    ]);
    Ok(())
}

/// `scptr`: Huawei SCP request. Entry, 3 pings, then the stock mode-3 6-packet
/// exchange: enable (CTRL_BYTE0 0xa0 = 0xc0), 0xa1 = 0x06, write16 regs
/// 0xb0/0xb2, set voltage code 0xca = 0xc8 (raw scale — volts unknown), 0xcb =
/// 0x64. `scp_set_voltage` (not ported) patches 0xca + CRC for other voltages.
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
    ]);
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
