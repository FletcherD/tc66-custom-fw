//! BL24C256 EEPROM (32 KB, 7-bit 0x50, 2-byte addressing, 64-B pages) on the
//! shared I2C0 bus. Layout is the stock firmware's (HARDWARE.md "EEPROM
//! layout"), so stock and this firmware read each other's data.
//!
//! Every bus access runs inside `ina::pause` (the sampler ISR owns the bus
//! otherwise), and each pause is kept to one short transaction (a 32-B page
//! write is ~3 ms at 100 kHz — under one 4 ms sampler tick). The ~5 ms
//! internal write cycle that follows runs with the bus RELEASED: the chip
//! just NACKs its own address until done, so the sampler keeps reading the
//! INA226 meanwhile. Completion is detected by ACK polling (an address-only
//! pointer write — no data byte, so it never starts a write cycle).
//!
//! **Write guard:** the factory calibration master (0x2200..0x2218) and the
//! working calibration mirror words can never be written through here.

use crate::i2c;
use crate::ina;

const ADDR: u8 = 0x50;
pub const SIZE: u16 = 0x8000;
const PAGE: u16 = 64;
/* ACK-poll budget: ~25 x ~0.5 ms polls (+ ~0.2 ms each) covers the 5 ms
 * datasheet tWR with margin */
const POLL_TRIES: u32 = 25;

/* error codes for "ER" replies */
pub const E_GUARD: u8 = 0x20;
pub const E_RANGE: u8 = 0x21;
pub const E_BUS: u8 = 0x22;
pub const E_WRITE_TIMEOUT: u8 = 0x23;
pub const E_VERIFY: u8 = 0x24;

/// Byte ranges that must never be written: factory cal master and the
/// working cal mirror (gainV/gainCp/gainCn, offV + two extra coeffs, ref).
const PROTECTED: [(u16, u16); 4] = [
    (0x2004, 0x2010),
    (0x2044, 0x2050),
    (0x2058, 0x205C),
    (0x2200, 0x2218),
];

fn guarded(addr: u16, len: u16) -> bool {
    let end = addr + len;
    PROTECTED.iter().any(|&(lo, hi)| addr < hi && lo < end)
}

/// Sequential read of `buf.len()` bytes at `addr` (one transaction). Caller
/// keeps reads short (≤ 32 B) once the sampler runs — the pause covers the
/// whole transfer.
#[inline(never)]
pub fn read(addr: u16, buf: &mut [u8]) -> Result<(), u8> {
    if addr as u32 + buf.len() as u32 > SIZE as u32 {
        return Err(E_RANGE);
    }
    ina::pause(|| i2c::transact(ADDR, &addr.to_be_bytes(), buf)).map_err(|_| E_BUS)
}

pub fn read_u32(addr: u16) -> Result<u32, u8> {
    let mut b = [0u8; 4];
    read(addr, &mut b)?;
    Ok(u32::from_le_bytes(b))
}

/// Write `data` at `addr`, split at page boundaries, each chunk waited out
/// by ACK polling and read back. Blocks the caller for ~6 ms per page.
#[inline(never)]
pub fn write(addr: u16, data: &[u8]) -> Result<(), u8> {
    if data.is_empty() || data.len() > 64 || addr as u32 + data.len() as u32 > SIZE as u32 {
        return Err(E_RANGE);
    }
    if guarded(addr, data.len() as u16) {
        return Err(E_GUARD);
    }
    let mut off = 0usize;
    while off < data.len() {
        let a = addr + off as u16;
        let room = (PAGE - a % PAGE) as usize;
        let n = room.min(data.len() - off);
        write_page(a, &data[off..off + n])?;
        off += n;
    }
    Ok(())
}

pub fn write_u32(addr: u16, v: u32) -> Result<(), u8> {
    write(addr, &v.to_le_bytes())
}

/// ~0.5 ms off the free-running SysTick (asm::delay's cycle count depends on
/// which flash region the loop lands in). The iteration cap keeps it bounded
/// even if SysTick were not running yet.
fn wait_half_ms() {
    let start = cortex_m::peripheral::SYST::get_current();
    for _ in 0..100_000u32 {
        if start.wrapping_sub(cortex_m::peripheral::SYST::get_current()) & 0x00FF_FFFF >= 24_000 {
            return;
        }
    }
}

/// One page-contained write: address + data in a single transaction, ACK
/// poll until the internal cycle ends, then verify.
#[inline(never)]
fn write_page(addr: u16, data: &[u8]) -> Result<(), u8> {
    if data.len() > PAGE as usize {
        return Err(E_RANGE);
    }
    let mut frame = [0u8; 2 + PAGE as usize];
    frame[..2].copy_from_slice(&addr.to_be_bytes());
    frame[2..2 + data.len()].copy_from_slice(data);
    let frame = &frame[..2 + data.len()];
    ina::pause(|| i2c::transact(ADDR, frame, &mut [])).map_err(|_| E_BUS)?;
    let mut ready = false;
    for _ in 0..POLL_TRIES {
        wait_half_ms();
        if ina::pause(|| i2c::transact(ADDR, &addr.to_be_bytes(), &mut [])).is_ok() {
            ready = true;
            break;
        }
    }
    if !ready {
        return Err(E_WRITE_TIMEOUT);
    }
    let mut back = [0u8; PAGE as usize];
    read(addr, &mut back[..data.len()])?;
    if back[..data.len()] != *data {
        return Err(E_VERIFY);
    }
    Ok(())
}
