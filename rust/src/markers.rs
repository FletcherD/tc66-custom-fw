//! SRAM progress markers — the only debug channel on this readout-protected
//! chip (halting via SWD wedges the core; background SRAM reads work).
//! Read with `make markers` (a no-halt openocd mdw of this window).

#![allow(dead_code)]

pub const MK_BOOT: usize = 0x2000_1000; // 0xB007B007 once main() runs
pub const MK_STAGE: usize = 0x2000_1004; // furthest milestone (STAGE_*)
// 0x20001008..0x2000101c: HardFault capture window (startup.rs)
pub const MK_HEART: usize = 0x2000_1020; // usb::poll() heartbeat
pub const MK_CTR: usize = 0x2000_1024; // CTR (transfer-complete) count
pub const MK_RESET: usize = 0x2000_1028; // USB bus-reset count
pub const MK_SETUP: usize = 0x2000_102c; // last SETUP: bmReqType<<8|bReq
pub const MK_CMD: usize = 0x2000_1030; // last CDC command (first 4 B, LE)
pub const MK_ENUM: usize = 0x2000_1034; // assigned USB address
pub const MK_CMDCNT: usize = 0x2000_1038; // CDC commands dispatched
pub const MK_PROBE: usize = 0x2000_103c; // last code point reached (freeze locator)
pub const MK_ISTR: usize = 0x2000_1040; // last ISTR value observed
pub const MK_SPEXIT: usize = 0x2000_1048; // SP at usb::poll exit
// 0x2000104c..0x20001054: DefaultHandler capture window (startup.rs)
pub const MK_PANIC: usize = 0x2000_1058; // 0xDEADC0DE on Rust panic
pub const MK_PANIC_LINE: usize = 0x2000_105c; // panicking source line
pub const MK_BUILD: usize = 0x2000_1060; // build id (self-hosting proof)
pub const MK_I2CERR: usize = 0x2000_1064; // sampler I2C error count (ISR; the
                                          // thread's cal-retry bumps run with
                                          // the sampler IRQ paused, so the
                                          // read-modify-write never races)
pub const MK_WINDOWS: usize = 0x2000_1068; // completed sample windows, ~10.4 Hz
                                          // (written by the TIMER5 ISR)

/* ---- Per-subsystem diagnostic blocks (12 words each) ----
 * Convention: word 0 = state/heartbeat (bump it in your service path so a
 * frozen value locates a wedge), word 1 = event/IRQ count, word 2 = last
 * error/status snapshot; words 3..11 subsystem-defined — DOCUMENT every
 * slot you claim here, next to its constant. Read with `make markers`
 * (covers 0x1000..0x1100). Only ever write via mk() (volatile).
 * The window ends at 0x20001400 (flasher); 0x1100..0x1400 is free. */

/* USB interrupt-path block (usb refactor) */
pub const MK_USB_BASE: usize = 0x2000_1070; // +0 state, +4 irq count, +8 status

/* Sampler interrupt block (ina.rs: TIMER5_DAC ISR @ 250 Hz + its main-loop
 * `service()` half). Writers noted per word; single-writer per word except
 * MK_I2CERR (see below). */
pub const MK_SMP_BASE: usize = 0x2000_10a0;
/// +0x00 (ISR) live state: (tick_count << 8) | code. A frozen value locates
/// a wedge by its low byte:
///   0x03        idle tick — waiting for thread-mode cal load
///   0x10+step   starting the I2C read for window step 0..=23
///               (step % 3: 0,1 = shunt, 2 = bus)
///   0x40        window publish (calibrate + PUBLISHED write, step 24)
///   0x50        1 s average drain (every 250 ticks)
///   0x02        ISR exit (healthy resting value)
pub const MK_SMP_STATE: usize = MK_SMP_BASE;
/// +0x04 (ISR) total TIMER5 ISR entries == 4 ms ticks (expect +250/s; also
/// mirrored by the AtomicU32 the main loop derives its cadences from)
pub const MK_SMP_IRQS: usize = MK_SMP_BASE + 0x04;
/// +0x08 (ISR) last in-window I2C failure:
/// (i2c stage << 24) | (STAT0 << 8) | window step
pub const MK_SMP_I2CE: usize = MK_SMP_BASE + 0x08;
/// +0x0c (ISR) seconds drained into the powerz averages (expect +1/s)
pub const MK_SMP_SECS: usize = MK_SMP_BASE + 0x0c;
/// +0x10 (thread, sampler paused) cal status: (attempts << 8) | loaded(0/1)
pub const MK_SMP_CAL: usize = MK_SMP_BASE + 0x10;
/// +0x14 (thread) deferred window-end ADC bursts serviced — should track
/// MK_WINDOWS; a growing gap means the main loop is starved/blocked
pub const MK_SMP_ADCB: usize = MK_SMP_BASE + 0x14;
/// +0x18 (thread) ADC burst failures: (count << 8) | last adc stage
pub const MK_SMP_ADCE: usize = MK_SMP_BASE + 0x18;
/* +0x1c..+0x2c free (5 words) */

/* display/DMA block (spi-dma refactor) */
pub const MK_DSP_BASE: usize = 0x2000_10d0; // +0 state, +4 evt count, +8 status

pub const STAGE_CLOCK: u32 = 1; // 48 MHz clock + USB clock configured
pub const STAGE_USB_INIT: u32 = 2; // USB peripheral powered + connected
pub const STAGE_RESET: u32 = 3; // first USB bus reset serviced
pub const STAGE_ADDRESS: u32 = 4; // SET_ADDRESS handled
pub const STAGE_CONFIG: u32 = 5; // SET_CONFIGURATION handled (enumerated)
pub const STAGE_CMD: u32 = 6; // first CDC command received
pub const STAGE_QUERY: u32 = 7; // answered query->firm
pub const STAGE_UPDAT: u32 = 8; // updat received, launching flasher

#[inline(always)]
pub fn mk(addr: usize, v: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, v) }
}

#[inline(always)]
pub fn mk_get(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}
