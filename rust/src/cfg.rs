//! Persisted settings + the `cfgrd` / `cfgwr` host verbs (USB CDC, and
//! `bcfgrd` / `bcfgwr` over BLE).
//!
//! A setting's id is its EEPROM word index — address = 0x2000 + 4·id — so
//! the stock config block (HARDWARE.md "EEPROM layout") is used as-is and
//! stock firmware keeps reading our values. Settings the stock firmware
//! doesn't have live in our own block at 0x2060.. (id 24+, never touched by
//! stock), guarded by a magic word at id 24.
//!
//! `set` validates, updates RAM and applies immediately; the EEPROM write is
//! coalesced — `service` writes every dirty word ~2 s after the last change,
//! so a host script setting values in a loop costs one write per word.
//! Thread mode only (verb dispatch + main loop); the ISRs never touch this.

use crate::eeprom;

pub const ID_GROUP: u8 = 4;
pub const ID_ROTATION: u8 = 5;
pub const ID_BRIGHTNESS: u8 = 7;
pub const ID_SCREEN: u8 = 9;
pub const ID_RUNCOUNT: u8 = 11;
pub const ID_LOG_INTERVAL: u8 = 13;
pub const ID_USB_OFF: u8 = 14;
pub const ID_BLE_OFF: u8 = 15;
pub const ID_IDLE_OFF: u8 = 25;
pub const ID_LOG_ON: u8 = 26;
pub const ID_LOG_CLEAR: u8 = 27;
pub const ID_IDLE_DV: u8 = 28;
pub const ID_IDLE_DI: u8 = 29;

const ID_MAGIC: u8 = 24; /* our block's validity word (0x2060) */
const MAGIC: u32 = 0x5443_4657; /* "WFCT" LE */

const BASE: u16 = 0x2000;
const fn addr(id: u8) -> u16 {
    BASE + 4 * id as u16
}

pub const E_ARGS: u8 = 0x0F;
pub const E_ID: u8 = 0x10;
pub const E_VALUE: u8 = 0x11;
pub const E_RO: u8 = 0x12;

const RO: u8 = 1;
/* stored in our own block: a write there also (re)writes the magic word */
const OURS: u8 = 2;
/* not stored: a write triggers an action, reads are always 0 */
const ACTION: u8 = 4;

struct Def {
    id: u8,
    min: u32,
    max: u32,
    def: u32,
    flags: u8,
}

/* Stock defaults except rotation: stock's default 0 (MADCTL 0x68) is
 * upside-down on this unit's mounting; 2 is the bench-verified orientation
 * and what an unreadable EEPROM falls back to. */
const DEFS: [Def; 13] = [
    Def { id: ID_GROUP, min: 0, max: 1, def: 0, flags: 0 },
    Def { id: ID_ROTATION, min: 0, max: 2, def: 2, flags: 0 },
    Def { id: ID_BRIGHTNESS, min: 0, max: 9, def: 6, flags: 0 },
    Def { id: ID_SCREEN, min: 0, max: crate::ui::SCREENS as u32 - 1, def: 0, flags: 0 },
    Def { id: ID_RUNCOUNT, min: 0, max: u32::MAX, def: 0, flags: RO },
    Def { id: ID_LOG_INTERVAL, min: 1, max: 60, def: 1, flags: 0 },
    Def { id: ID_USB_OFF, min: 0, max: 1, def: 0, flags: 0 },
    Def { id: ID_BLE_OFF, min: 0, max: 1, def: 0, flags: 0 },
    Def { id: ID_IDLE_OFF, min: 0, max: 3600, def: 15, flags: OURS },
    Def { id: ID_LOG_ON, min: 0, max: 1, def: 0, flags: OURS },
    Def { id: ID_LOG_CLEAR, min: 0, max: 1, def: 0, flags: ACTION },
    Def { id: ID_IDLE_DV, min: 0, max: 30_000, def: 100, flags: OURS },
    Def { id: ID_IDLE_DI, min: 0, max: 6_000, def: 5, flags: OURS },
];
const N: usize = DEFS.len();
/* config words covered by `load`: ids 0..=max id in DEFS (ours start at 24) */
const WORDS: usize = {
    let mut max = ID_MAGIC as usize;
    let mut k = 0;
    while k < N {
        if DEFS[k].id as usize > max {
            max = DEFS[k].id as usize;
        }
        k += 1;
    }
    max + 1
};

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}

struct State {
    vals: [u32; N],
    dirty: u16,
    changed_at: u32,
    loaded: bool,
}
static ST: Racy<State> = Racy(core::cell::UnsafeCell::new(State {
    vals: [0; N],
    dirty: 0,
    changed_at: 0,
    loaded: false,
}));

fn st() -> &'static mut State {
    unsafe { &mut *ST.0.get() }
}

fn index(id: u8) -> Option<usize> {
    DEFS.iter().position(|d| d.id == id)
}

fn valid(d: &Def, v: u32) -> bool {
    /* rotation uses the stock encoding: 0 or 2 (180°), nothing between */
    v >= d.min && v <= d.max && !(d.id == ID_ROTATION && v == 1)
}

pub fn get(id: u8) -> u32 {
    index(id).map_or(0, |k| st().vals[k])
}

/// Boot, after the grace window (new code stays out of the escape-hatch
/// path): load the config block and fall back to defaults for anything
/// missing or out of range. Read-only — `commit_boot` does the boot-time
/// write. Returns false if the EEPROM could not be read (defaults stay in
/// RAM).
#[inline(never)]
pub fn load() -> bool {
    let s = st();
    for (k, d) in DEFS.iter().enumerate() {
        s.vals[k] = d.def;
    }
    /* stock block 0x2000..0x2060 (24 words) + our block from 0x2060 up to
     * the highest id in DEFS (WORDS is derived from DEFS, so a new setting
     * can never index past the buffer) */
    let mut raw = [0u8; 4 * WORDS];
    if eeprom::read(BASE, &mut raw[..96]).is_err() || eeprom::read(addr(24), &mut raw[96..]).is_err() {
        return false;
    }
    let word = |id: u8| {
        let o = 4 * id as usize;
        raw.get(o..o + 4).map_or(u32::MAX, |w| u32::from_le_bytes([w[0], w[1], w[2], w[3]]))
    };
    let ours_valid = word(ID_MAGIC) == MAGIC;
    for (k, d) in DEFS.iter().enumerate() {
        if d.flags & ACTION != 0 || (d.flags & OURS != 0 && !ours_valid) {
            continue;
        }
        let v = word(d.id);
        if valid(d, v) {
            s.vals[k] = v;
        }
    }
    /* raw run count: commit_boot bumps it */
    if let Some(k) = index(ID_RUNCOUNT) {
        s.vals[k] = word(ID_RUNCOUNT);
    }
    s.loaded = true;
    true
}

/// Boot-time write, once SysTick runs and USB is up: the run-count bump
/// (stock does it every boot — getva byte 44; an erased 0xFFFFFFFF word
/// restarts at 1). Skipped when `load` failed, so a dead bus never writes
/// over real data.
#[inline(never)]
pub fn commit_boot() {
    let s = st();
    if !s.loaded {
        return;
    }
    if let Some(k) = index(ID_RUNCOUNT) {
        let rc = s.vals[k].wrapping_add(1);
        s.vals[k] = if rc == 0 { 1 } else { rc };
        let _ = eeprom::write_u32(addr(ID_RUNCOUNT), s.vals[k]);
    }
}

/// Validate + set + apply; the EEPROM write follows from `service`.
#[inline(never)]
pub fn set(id: u8, v: u32) -> Result<(), u8> {
    let k = index(id).ok_or(E_ID)?;
    let d = &DEFS[k];
    if d.flags & RO != 0 {
        return Err(E_RO);
    }
    if !valid(d, v) {
        return Err(E_VALUE);
    }
    if d.flags & ACTION != 0 {
        if v != 0 {
            apply(id, v);
        }
        return Ok(());
    }
    let s = st();
    if s.vals[k] != v {
        s.vals[k] = v;
        s.dirty |= 1 << k;
        s.changed_at = crate::ina::ticks();
        apply(id, v);
    }
    Ok(())
}

/// Record a value changed by the device itself (e.g. the screen index after
/// a key press) — persisted like a host write, no re-apply.
pub fn note(id: u8, v: u32) {
    if let Some(k) = index(id) {
        let s = st();
        if s.vals[k] != v {
            s.vals[k] = v;
            s.dirty |= 1 << k;
            s.changed_at = crate::ina::ticks();
        }
    }
}

/// Push a setting's value into the subsystem that uses it. Settings read
/// on demand (idle-off, command gates) need nothing here.
fn apply(id: u8, v: u32) {
    match id {
        ID_ROTATION => crate::ui::set_rotation(v as u8),
        ID_BRIGHTNESS => {
            let _ = crate::tft::set_brightness(v as u8);
        }
        ID_SCREEN => crate::ui::set_screen(v as u8),
        ID_GROUP => crate::persist::set_group(v as usize),
        ID_LOG_CLEAR => crate::persist::log_clear(),
        _ => {}
    }
}

/// Apply the display settings at boot (the first frame was drawn with the
/// defaults; rotation repaints only if it differs). The energy group is
/// read by persist::boot itself.
pub fn apply_boot() {
    for id in [ID_ROTATION, ID_BRIGHTNESS, ID_SCREEN] {
        apply(id, get(id));
    }
}

/// Main loop: write dirty settings ~2 s after the last change. A failed
/// word stays dirty and is retried after another quiet period.
#[inline(never)]
pub fn service() {
    let s = st();
    if s.dirty == 0 || crate::ina::ticks().wrapping_sub(s.changed_at) < 500 {
        return;
    }
    for (k, d) in DEFS.iter().enumerate() {
        if s.dirty & (1 << k) == 0 {
            continue;
        }
        let mut ok = eeprom::write_u32(addr(d.id), s.vals[k]).is_ok();
        if ok && d.flags & OURS != 0 {
            ok = eeprom::write_u32(addr(ID_MAGIC), MAGIC).is_ok();
        }
        if ok {
            s.dirty &= !(1 << k);
        }
    }
    s.changed_at = crate::ina::ticks();
}

/// Length of the `cfgrd` reply.
pub const DUMP_LEN: usize = 3 + 14 * N + 2;

/// `cfgrd` reply: "OK" + n + n × (id u8, value, min, max: u32 LE, flags u8)
/// + CRC16/MODBUS (LE) of everything before it.
#[inline(never)]
pub fn dump(out: &mut [u8; DUMP_LEN]) {
    out[..2].copy_from_slice(b"OK");
    out[2] = N as u8;
    for (k, d) in DEFS.iter().enumerate() {
        let o = 3 + 14 * k;
        out[o] = d.id;
        out[o + 1..o + 5].copy_from_slice(&st().vals[k].to_le_bytes());
        out[o + 5..o + 9].copy_from_slice(&d.min.to_le_bytes());
        out[o + 9..o + 13].copy_from_slice(&d.max.to_le_bytes());
        out[o + 13] = d.flags & RO;
    }
    let crc = crate::getva::crc16_modbus(&out[..DUMP_LEN - 2]);
    out[DUMP_LEN - 2..].copy_from_slice(&crc.to_le_bytes());
}

/// Parse "<id> <value>" (ASCII decimal, single spaces) from the bytes after
/// the verb and apply it.
#[inline(never)]
pub fn write_cmd(args: &[u8]) -> Result<(), u8> {
    let mut it = args.split(|&b| b == b' ').filter(|t| !t.is_empty());
    let id = parse_dec(it.next().ok_or(E_ARGS)?)?;
    let v = parse_dec(it.next().ok_or(E_ARGS)?)?;
    if it.next().is_some() || id > 255 {
        return Err(E_ARGS);
    }
    set(id as u8, v)
}

fn parse_dec(t: &[u8]) -> Result<u32, u8> {
    let mut v: u32 = 0;
    for &c in t {
        if !c.is_ascii_digit() {
            return Err(E_ARGS);
        }
        v = v.checked_mul(10).and_then(|v| v.checked_add((c - b'0') as u32)).ok_or(E_ARGS)?;
    }
    Ok(v)
}
