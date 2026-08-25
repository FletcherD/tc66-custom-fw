//! SRAM progress markers — the only debug channel on this readout-protected chip.
//! Read (NO halt) with: openocd -f ../reference-c/openocd.cfg -c "mdw 0x20001000 24" -c shutdown
//! Map is identical to reference-c/usb.h except: MK_RA dropped (no stable-Rust
//! return-address intrinsic), MK_PANIC/MK_PANIC_LINE added.

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
pub const MK_I2CERR: usize = 0x2000_1064; // sampler I2C error count
pub const MK_WINDOWS: usize = 0x2000_1068; // completed sample windows (~10 Hz)

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
