//! SRAM progress markers — the only debug channel on this readout-protected
//! chip (halting via SWD wedges the core; background SRAM reads work).
//! Read with `make markers` (a no-halt openocd mdw of this window).

#![allow(dead_code)]

pub const MK_BOOT: usize = 0x2000_1000; // 0xB007B007 once main() runs
pub const MK_STAGE: usize = 0x2000_1004; // furthest milestone (STAGE_*)
// 0x20001008..0x2000101c: HardFault capture window (startup.rs)
pub const MK_HEART: usize = 0x2000_1020; // main-loop pass heartbeat
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
pub const MK_I2CERR: usize = 0x2000_1064; // sampler I2C error count
pub const MK_WINDOWS: usize = 0x2000_1068; // completed sample windows (~10 Hz)

/* ---- Per-subsystem diagnostic blocks (12 words each) ----
 * Convention: word 0 = state/heartbeat (bump it in your service path so a
 * frozen value locates a wedge), word 1 = event/IRQ count, word 2 = last
 * error/status snapshot; words 3..11 subsystem-defined — DOCUMENT every
 * slot you claim here, next to its constant. Read with `make markers`
 * (covers 0x1000..0x1100). Only ever write via mk() (volatile).
 * The window ends at 0x20001400 (flasher); 0x1100..0x1400 is free. */

/* USB interrupt-path block: the USBD_LP ISR + command ring (main.rs).
 * All words are single-writer (ISR, or thread mode under interrupt::free
 * for FULL/CMDS — contexts that are mutually exclusive), so the
 * read-modify-write bumps never race. */
pub const MK_USB_BASE: usize = 0x2000_1070;
pub const MK_USB_STATE: usize = MK_USB_BASE; // ISR phase: 0 idle/exited, 1 entered,
                                             // 2 dev.poll done, 3 powerz serviced,
                                             // 4 CDC drained. Frozen nonzero = the
                                             // ISR wedged in that phase.
pub const MK_USB_IRQS: usize = MK_USB_BASE + 0x04; // total USBD_LP entries
pub const MK_USB_ISTR: usize = MK_USB_BASE + 0x08; // USBD_INTF (ISTR) at last ISR entry
pub const MK_USB_FULL: usize = MK_USB_BASE + 0x0c; // command-ring-full deferrals
                                                   // (backpressure, not data loss)
pub const MK_USB_CMDS: usize = MK_USB_BASE + 0x10; // CDC packets moved into the ring
pub const MK_USB_SPUR: usize = MK_USB_BASE + 0x14; // ISR entries before the USB
                                                   // pointers were published (must
                                                   // stay 0 — nonzero means the IRQ
                                                   // was unmasked too early)
// +0x18..+0x2c reserved for the USB block
/* sampler interrupt-path block (sampler refactor) */
pub const MK_SMP_BASE: usize = 0x2000_10a0; // +0 state, +4 irq count, +8 status
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
