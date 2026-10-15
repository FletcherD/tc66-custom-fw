//! KM003C AdcQueue streaming state (docs/HARDWARE.md "AdcQueue streaming").
//!
//! A host's StartGraph (powerz.rs, USBD_LP ISR) `request`s a stream mode —
//! 1..=4 = StartGraph rate index 0..=3 + 1 = 2/10/50/1000 samples/s, 0 = off
//! (zero-based so every state starts in .bss); the TIMER5 sampler ISR (ina.rs) applies it
//! at its next tick — INA226 CONFIG + tick period — then calls `begin` and
//! `push`es one calibrated sample per sample period. GetData(AdcQueue)
//! (powerz.rs) `take`s up to `MAX_TAKE` samples and expands them to the
//! 20-B wire records on the fly.
//!
//! The ring lives in FLASHER_RAM (memory.x, 2 KB at 0x2000_1400): idle
//! except while `updat` runs the self-flasher there, and flasher::launch()
//! masks interrupts before copying it in, so no sample can land on it.
//!
//! Soundness: the producer (TIMER5) and the consumer (USBD_LP) are the two
//! unmasked IRQs at one NVIC priority — they never preempt each other — and
//! thread mode touches none of this, so plain loads/stores of the atomics
//! below need no stronger ordering than Relaxed.

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// Stream mode "not streaming"; 1..=4 = StartGraph rate index + 1.
pub const OFF: u8 = 0;
/// Stream mode of 1000 samples/s.
pub const FAST: u8 = 4;
/// Samples per reply: the real device's observed maximum.
pub const MAX_TAKE: u32 = 48;
/// Stop streaming after this long without a vendor command (the real
/// device also stops when nobody polls).
const IDLE_STOP_US: u32 = 2_000_000;

const RING_LEN: u32 = 256;

#[derive(Clone, Copy)]
#[repr(C)]
pub struct Sample {
    pub v_uv: i32,
    pub i_ua: i32,
}

extern "C" {
    /* memory.x: start of FLASHER_RAM */
    static mut __adcq_ring: [Sample; RING_LEN as usize];
}

/// Rate requested by the host (USBD_LP writes, TIMER5 applies).
static REQ: AtomicU8 = AtomicU8::new(OFF);
/// Rate the sampler is running (TIMER5 writes).
static ACTIVE: AtomicU8 = AtomicU8::new(OFF);
/// Samples pushed / consumed since `begin` (wrapping counters).
static HEAD: AtomicU32 = AtomicU32::new(0);
static TAIL: AtomicU32 = AtomicU32::new(0);
/// us since the last vendor command (TIMER5 adds, USBD_LP clears).
static IDLE_US: AtomicU32 = AtomicU32::new(0);

/// 1 kHz sequence-counter step per sample by stream mode (the KM003C's
/// device-time counter: 500/100/20/1 ms) — in fast flash, the sampler ISR
/// reads it every tick.
#[link_section = ".fasttext.adcq_steps"]
static STEPS: [u32; 5] = [1, 500, 100, 20, 1];

pub fn seq_step(mode: u8) -> u32 {
    *STEPS.get(mode as usize).unwrap_or(&1)
}

/// USBD_LP: StartGraph(rate) / StopGraph (`OFF`).
pub fn request(rate: u8) {
    REQ.store(rate, Ordering::Relaxed);
}

pub fn requested() -> u8 {
    REQ.load(Ordering::Relaxed)
}

pub fn active() -> u8 {
    ACTIVE.load(Ordering::Relaxed)
}

/// USBD_LP: any vendor command keeps a stream (and the powerz session) alive.
pub fn host_seen() {
    IDLE_US.store(0, Ordering::Relaxed);
}

/// us since the last vendor command (saturating).
pub fn idle_us() -> u32 {
    IDLE_US.load(Ordering::Relaxed)
}

/// TIMER5, every tick: advance the idle clock; an abandoned stream stops.
pub fn tick(us: u32) {
    let idle = IDLE_US.load(Ordering::Relaxed).saturating_add(us);
    IDLE_US.store(idle, Ordering::Relaxed);
    if idle > IDLE_STOP_US && REQ.load(Ordering::Relaxed) != OFF {
        REQ.store(OFF, Ordering::Relaxed);
    }
}

/// TIMER5: the sampler now runs `rate` — empty the ring.
pub fn begin(rate: u8) {
    HEAD.store(0, Ordering::Relaxed);
    TAIL.store(0, Ordering::Relaxed);
    ACTIVE.store(rate, Ordering::Relaxed);
}

/// TIMER5: append one sample (overwrites the oldest when the host lags; the
/// sequence gap shows it).
pub fn push(s: Sample) {
    let h = HEAD.load(Ordering::Relaxed);
    unsafe { (*core::ptr::addr_of_mut!(__adcq_ring))[(h % RING_LEN) as usize] = s };
    HEAD.store(h.wrapping_add(1), Ordering::Relaxed);
}

/// USBD_LP: claim up to `max` of the oldest unread samples; returns the
/// index of the first and the count. Lost (overwritten) samples are skipped;
/// nothing is returned while a mode change is pending (the ring still holds
/// the old mode's samples until the sampler's `begin`).
pub fn take(max: u32) -> (u32, u32) {
    if REQ.load(Ordering::Relaxed) != ACTIVE.load(Ordering::Relaxed) {
        return (0, 0);
    }
    let h = HEAD.load(Ordering::Relaxed);
    let mut t = TAIL.load(Ordering::Relaxed);
    if h.wrapping_sub(t) > RING_LEN {
        t = h.wrapping_sub(RING_LEN);
    }
    let n = h.wrapping_sub(t).min(max);
    TAIL.store(t.wrapping_add(n), Ordering::Relaxed);
    (t, n)
}

/// Sample `idx` (from `take`) and its sequence-counter value.
pub fn get(idx: u32) -> (Sample, u16) {
    let s = unsafe { (*core::ptr::addr_of!(__adcq_ring))[(idx % RING_LEN) as usize] };
    (s, idx.wrapping_mul(seq_step(active())) as u16)
}
