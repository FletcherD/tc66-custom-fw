//! `updat` self-flasher launch — port of launch_flasher() in reference-c/main.c.
//!
//! flasher.bin is the PROVEN SRAM streaming flasher, assembled once from
//! reference-c/flasher.s (commit e744d61: `make -C reference-c flasher.bin`,
//! 316 B, linked for 0x20001400) and committed here. It erases the app region,
//! streams 810 plaintext 64-B chunks from EP3 via the bootloader's resident
//! flash routines, acks each with "OK" on EP1, then resets the core. It depends
//! on usb.rs's exact PMA layout (EP1 TX @0xC0, EP3 RX @0x100).

use crate::markers::{mk, MK_STAGE, STAGE_UPDAT};

static FLASHER_BLOB: &[u8] = include_bytes!("../flasher.bin");
const FLASHER_SRAM: usize = 0x2000_1400;

#[inline(never)]
pub fn launch() -> ! {
    mk(MK_STAGE, STAGE_UPDAT);
    unsafe {
        /* copy the position-fixed flasher into SRAM (outside linker RAM) */
        for (i, &b) in FLASHER_BLOB.iter().enumerate() {
            core::ptr::write_volatile((FLASHER_SRAM + i) as *mut u8, b);
        }
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        cortex_m::interrupt::disable();
        /* jump (thumb). The flasher sets its own SP, arms EP3, streams, resets. */
        let entry: extern "C" fn() -> ! = core::mem::transmute(FLASHER_SRAM | 1);
        entry()
    }
}
