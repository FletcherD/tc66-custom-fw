//! M2 persistence: the two energy groups + run clock (wear-leveled EEPROM
//! ring), the datalogger ring behind `gtrec`, and the per-unit serial
//! number. Layouts are the stock firmware's (docs/HARDWARE.md "EEPROM layout"),
//! so stock and this firmware pick up each other's totals.
//!
//! Energy ring 0x0000..0x2000: 256 slots × 32 B, each
//!   +0x00 g0 mAh, +0x04 g0 mWh, +0x08 g1 mAh, +0x0c g1 mWh, +0x10 seq
//!   (stock: the 20 B it reads and writes), then OUR extension in the pad
//!   stock never touches: +0x14 g0 run seconds, +0x18 g1 run seconds,
//!   +0x1c tag 0xC0DE (u16), +0x1e CRC16/MODBUS of bytes 0..0x1c.
//! The newest slot is the highest seq. A slot carrying our tag must pass
//! its CRC (a write torn by a power cut); if the newest fails, the
//! next-newest is used. Untagged slots (written by stock) are trusted as-is
//! and have no run clock.
//!
//! Both groups are cumulative totals (stock keeps group 1 cumulative but
//! treats group 0 as a per-session counter that restarts once current
//! flows). Like stock, a second only integrates while 10 µA < I < 6 A, so
//! an idle meter neither counts run time nor wears the ring.
//!
//! Datalog 0x2400..0x7E00: up to 1440 records × 16 B, record n =
//!   +0 V (1e-4 V), +4 I (1e-5 A), +8 seq = n, +0xc pad (ours: LOG_TAG)
//! appended every `log_interval_s` while `log_on` is set (stock: RAM flag,
//! off at boot — ours is a persisted setting, as there is no menu). Before
//! each record the NEXT record's seq is zeroed as the end marker; the log
//! stops when full (no wrap, as stock). The count is the chain of records
//! with seq == n from n = 1; our tag in the last record's pad also makes a
//! lone record 0 (and the last record after a reboot — stock hides it)
//! visible.
//!
//! Thread mode only (main loop + verb dispatch).

use crate::eeprom;

const RING_SLOTS: u16 = 256;
const SLOT: u16 = 32;
const TAG: u16 = 0xC0DE;
/* save the ring this often while anything changed (seconds). 256 slots ×
 * 1M cycles at one save / 10 s ≈ 80 years of continuous use. */
const SAVE_SECS: u32 = 10;
/* integrate only inside the stock window: 10 µA < I < 6 A (1e-5 A units) */
const I_MIN: u32 = 1;
const I_MAX: u32 = 600_000;
/* display/stock clamps */
const MAH_MAX: u64 = 99_999;
const MWH_MAX: u64 = 999_999;

const LOG_BASE: u16 = 0x2400;
const LOG_REC: u16 = 16;
pub const LOG_MAX: u16 = 1440;
const LOG_TAG: u32 = 0xC0DE_106A;

/* integration units: 1e-5 A·s and 1e-4 W·s (the published I/P units × 1 s) */
const I_PER_MAH: u64 = 360_000;
const P_PER_MWH: u64 = 36_000;

#[derive(Clone, Copy)]
struct Group {
    acc_i: u64,
    acc_p: u64,
    secs: u32,
}

struct State {
    g: [Group; 2],
    active: usize,
    /* ring position of the next save and its seq */
    next_slot: u16,
    seq: u32,
    /* seconds since the last save, and whether anything changed since */
    since_save: u32,
    dirty: bool,
    loaded: bool,
    /* datalog: next record index (None = not scanned yet), seconds since
     * the last record */
    log_n: Option<u16>,
    log_secs: u32,
}

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
static ST: Racy<State> = Racy(core::cell::UnsafeCell::new(State {
    g: [Group { acc_i: 0, acc_p: 0, secs: 0 }; 2],
    active: 0,
    next_slot: 0,
    seq: 0,
    since_save: 0,
    dirty: false,
    loaded: false,
    log_n: None,
    log_secs: 0,
}));

fn st() -> &'static mut State {
    unsafe { &mut *ST.0.get() }
}

fn mah(g: &Group) -> u32 {
    (g.acc_i / I_PER_MAH).min(MAH_MAX) as u32
}
fn mwh(g: &Group) -> u32 {
    (g.acc_p / P_PER_MWH).min(MWH_MAX) as u32
}

/// (mAh, mWh) of group `n` — getva bytes 72..88.
pub fn group_energy(n: usize) -> (u32, u32) {
    let g = &st().g[n & 1];
    (mah(g), mwh(g))
}

/// Active group's run clock (energy screen).
pub fn run_secs() -> u32 {
    let s = st();
    s.g[s.active & 1].secs
}

fn publish() {
    let s = st();
    let g = &s.g[s.active & 1];
    crate::ina::set_energy(mah(g), mwh(g));
}

/// A decoded ring slot: w = [g0 mAh, g0 mWh, g1 mAh, g1 mWh, seq].
struct Slot {
    w: [u32; 5],
    secs: [u32; 2],
    tagged: bool,
    crc_ok: bool,
}

fn read_slot(n: u16) -> Result<Slot, u8> {
    let mut b = [0u8; SLOT as usize];
    eeprom::read(n * SLOT, &mut b)?;
    let u = |o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
    let tagged = u16::from_le_bytes([b[0x1c], b[0x1d]]) == TAG;
    let crc = u16::from_le_bytes([b[0x1e], b[0x1f]]);
    Ok(Slot {
        w: [u(0), u(4), u(8), u(12), u(16)],
        secs: [u(0x14), u(0x18)],
        tagged,
        crc_ok: crate::getva::crc16_modbus(&b[..0x1c]) == crc,
    })
}

/// Highest-seq slot (skipping erased ones and `skip`). Err on a bus error.
fn newest(skip: Option<u16>) -> Result<Option<(u16, u32)>, u8> {
    let mut best: Option<(u16, u32)> = None;
    for n in 0..RING_SLOTS {
        let seq = eeprom::read_u32(n * SLOT + 0x10)?;
        if seq == u32::MAX || Some(n) == skip {
            continue;
        }
        if best.map_or(true, |(_, b)| seq > b) {
            best = Some((n, seq));
        }
    }
    Ok(best)
}

/// Boot (after USB is up, before the sampler starts — the bus is ours):
/// find the newest ring slot and restore both groups from it. ~256 short
/// seq reads (~0.25 s). A newest slot with our tag but a bad CRC (a write
/// torn by a power cut — its seq may be garbage too) is skipped once and
/// the next-newest used as-is. (Side effect: if stock ran in between and
/// wrote 20 B over one of our slots, its newest save is skipped the same
/// way — one second of stock counting lost.) On a read error nothing is
/// restored and saving stays off for this boot, so a flaky bus never
/// overwrites good history.
#[inline(never)]
pub fn boot() {
    let s = st();
    s.active = (crate::cfg::get(crate::cfg::ID_GROUP) & 1) as usize;
    let Ok(mut best) = newest(None) else { return };
    let mut slot = None;
    if let Some((n, _)) = best {
        let Ok(sl) = read_slot(n) else { return };
        if sl.tagged && !sl.crc_ok {
            let Ok(b2) = newest(Some(n)) else { return };
            best = b2;
            if let Some((n2, _)) = best {
                let Ok(sl2) = read_slot(n2) else { return };
                slot = Some(sl2);
            }
        } else {
            slot = Some(sl);
        }
    }
    if let (Some((n, seq)), Some(slot)) = (best, slot) {
        for k in 0..2 {
            s.g[k] = Group {
                acc_i: slot.w[2 * k] as u64 * I_PER_MAH,
                acc_p: slot.w[2 * k + 1] as u64 * P_PER_MWH,
                secs: if slot.tagged && slot.crc_ok { slot.secs[k] } else { 0 },
            };
        }
        s.next_slot = (n + 1) % RING_SLOTS;
        s.seq = seq.wrapping_add(1);
    }
    s.loaded = true;
    publish();
}

/// Write the current totals to the next ring slot.
#[inline(never)]
fn save() {
    let s = st();
    if !s.loaded {
        return;
    }
    let mut b = [0u8; SLOT as usize];
    let words = [
        mah(&s.g[0]),
        mwh(&s.g[0]),
        mah(&s.g[1]),
        mwh(&s.g[1]),
        s.seq,
        s.g[0].secs,
        s.g[1].secs,
    ];
    for (k, w) in words.iter().enumerate() {
        b[4 * k..4 * k + 4].copy_from_slice(&w.to_le_bytes());
    }
    b[0x1c..0x1e].copy_from_slice(&TAG.to_le_bytes());
    let crc = crate::getva::crc16_modbus(&b[..0x1c]);
    b[0x1e..0x20].copy_from_slice(&crc.to_le_bytes());
    if eeprom::write(s.next_slot * SLOT, &b).is_ok() {
        s.next_slot = (s.next_slot + 1) % RING_SLOTS;
        s.seq = s.seq.wrapping_add(1);
        s.since_save = 0;
        s.dirty = false;
    }
    /* on failure the same slot/seq is retried at the next save point */
}

/// 1 Hz from the main loop: integrate the published I/P into the active
/// group and tick its run clock (only while current flows), save every
/// SAVE_SECS while changed, append the datalog record when due.
#[inline(never)]
pub fn second(v_1e4: u32, i_1e5: u32, p_1e4: u32) {
    let s = st();
    if i_1e5 > I_MIN && i_1e5 < I_MAX {
        let g = &mut s.g[s.active & 1];
        g.acc_i += i_1e5 as u64;
        g.acc_p += p_1e4 as u64;
        g.secs = g.secs.saturating_add(1);
        s.dirty = true;
        publish();
    }
    s.since_save = s.since_save.saturating_add(1);
    if s.since_save >= SAVE_SECS && s.dirty {
        save();
    }
    if crate::cfg::get(crate::cfg::ID_LOG_ON) != 0 {
        s.log_secs += 1;
        if s.log_secs >= crate::cfg::get(crate::cfg::ID_LOG_INTERVAL) {
            s.log_secs = 0;
            log_append(v_1e4, i_1e5);
        }
    }
}

/// K2 hold: zero the active group and its run clock, saved at once.
pub fn reset_active() {
    let s = st();
    s.g[s.active & 1] = Group { acc_i: 0, acc_p: 0, secs: 0 };
    publish();
    save();
}

/// Group setting changed (cfg): switch which group accumulates and is
/// shown.
pub fn set_group(n: usize) {
    st().active = n & 1;
    publish();
}

/// Per-unit serial number: word 1 of the factory config page (0x0800FC04;
/// word 0 is the stock "calibrated" flag). Read-only — this firmware never
/// writes that page.
pub fn serial() -> u32 {
    unsafe { core::ptr::read_volatile(0x0800_FC04 as *const u32) }
}

fn rec_addr(n: u16) -> u16 {
    LOG_BASE + n * LOG_REC
}

/// Number of records in the log (scanning it once on first use: up to
/// 1440 short seq reads, ~1.3 s, each its own sampler pause). None if the
/// EEPROM can't be read.
#[inline(never)]
fn log_count() -> Option<u16> {
    let s = st();
    if let Some(n) = s.log_n {
        return Some(n);
    }
    let mut n: u16 = 0; /* records 0..=n-1 chained */
    while n + 1 < LOG_MAX {
        if eeprom::read_u32(rec_addr(n + 1) + 8).ok()? != (n + 1) as u32 {
            break;
        }
        n += 1;
    }
    /* records 0..n are chained (seq == index); the last one counts when it
     * carries our tag (stock's count stops one short of it) */
    let tag = eeprom::read_u32(rec_addr(n) + 12).ok()?;
    let count = if tag == LOG_TAG { n + 1 } else { n };
    s.log_n = Some(count);
    Some(count)
}

/// Append one record (end marker first, as stock). Full log: stop.
#[inline(never)]
fn log_append(v_1e4: u32, i_1e5: u32) {
    let Some(n) = log_count() else { return };
    if n >= LOG_MAX {
        return;
    }
    if n + 1 < LOG_MAX && eeprom::write_u32(rec_addr(n + 1) + 8, 0).is_err() {
        return;
    }
    let mut b = [0u8; LOG_REC as usize];
    for (k, w) in [v_1e4, i_1e5, n as u32, LOG_TAG].iter().enumerate() {
        b[4 * k..4 * k + 4].copy_from_slice(&w.to_le_bytes());
    }
    if eeprom::write(rec_addr(n), &b).is_ok() {
        st().log_n = Some(n + 1);
    }
}

/// `log_clear` setting action: empty the log (records 0 and 1 seq/pad
/// zeroed — what stock's "clear log" menu writes, plus our tag word).
pub fn log_clear() {
    let ok = eeprom::write(rec_addr(0) + 8, &[0; 8]).is_ok()
        && eeprom::write_u32(rec_addr(1) + 8, 0).is_ok();
    st().log_n = if ok { Some(0) } else { None };
    st().log_secs = 0;
}

/// `gtrec`: hand each record's first 8 B (V, I) to `out`, in order. The
/// reads are 2 records (32 B) per sampler pause. Returns false on a read
/// error (the dump stops there).
#[inline(never)]
pub fn log_dump(mut out: impl FnMut(&[u8])) -> bool {
    let Some(n) = log_count() else { return false };
    let mut k: u16 = 0;
    while k < n {
        let mut b = [0u8; 2 * LOG_REC as usize];
        let m = (n - k).min(2);
        if eeprom::read(rec_addr(k), &mut b[..(m * LOG_REC) as usize]).is_err() {
            return false;
        }
        for r in 0..m as usize {
            out(&b[r * LOG_REC as usize..r * LOG_REC as usize + 8]);
        }
        k += m;
        crate::wdg::feed();
    }
    true
}
