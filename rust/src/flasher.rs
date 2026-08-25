//! `updat` self-flasher, in Rust (replaces the assembled blob from reference-c).
//!
//! flasher_main lives in the `.flasher` section: linked to RUN at 0x20001400
//! (FLASHER_RAM in memory.x / flasher.x) but stored in flash; launch() copies it
//! into SRAM and jumps. Because it's linked at its run address there are no
//! position-independence requirements — but it MUST be fully self-contained:
//! after it erases the app region, any call/reference into app flash is fatal.
//! Hence: raw pointers only, no panicking ops, its own asm! (not cortex-m's asm
//! wrappers, which may be extern blobs in flash), helpers #[inline(always)],
//! reply strings materialized as integer literals. The Makefile's `flasher-gate`
//! disassembles the section and rejects any app-flash address in its literals.
//!
//! Unlike the old blob (fixed EP1/EP3 @ hard-coded PMA addresses), this flasher
//! takes the bulk endpoint index as an argument and reads the endpoint's buffer
//! locations from the BTABLE at runtime, so it works with whatever layout
//! stm32-usbd allocated. Protocol is unchanged (tools/app_update.py):
//! erase -> "uprdy" -> 810 x (recv 64 B -> program -> "OK") -> AIRCR reset.

use crate::markers::{mk, MK_STAGE, STAGE_UPDAT};

/* Bootloader-resident flash routines (always present, never erased) */
const BL_ERASE_PAGES: usize = 0x0800_048f; /* (base, pagesize, count) */
const BL_FMC_UNLOCK: usize = 0x0800_0ed5; /* () */
const BL_FMC_PROGRAM_WORD: usize = 0x0800_0ef5; /* (flash_addr, word) */
const BL_FMC_LOCK: usize = 0x0800_0e49; /* () */

const APP_BASE: u32 = 0x0800_2c00;
const PAGE_SIZE: u32 = 0x400;
const NUM_PAGES: u32 = 52;
const IMAGE_BYTES: u32 = 51840; /* 810 x 64 */

const USB_BASE: usize = 0x4000_5C00;
const PMA_BASE: usize = 0x4000_6000;

#[inline(always)]
unsafe fn epr_rd(ep: u32) -> u32 {
    core::ptr::read_volatile((USB_BASE + 4 * ep as usize) as *const u32)
}
#[inline(always)]
unsafe fn epr_wr(ep: u32, v: u32) {
    core::ptr::write_volatile((USB_BASE + 4 * ep as usize) as *mut u32, v)
}
#[inline(always)]
unsafe fn set_stat_rx_valid(ep: u32) {
    epr_wr(ep, (epr_rd(ep) & 0xBF8F) ^ 0x3000);
}
#[inline(always)]
unsafe fn set_stat_tx_valid(ep: u32) {
    epr_wr(ep, (epr_rd(ep) & 0x8FBF) ^ 0x30);
}
#[inline(always)]
unsafe fn clear_ctr_rx(ep: u32) {
    epr_wr(ep, epr_rd(ep) & 0x0F8F);
}
#[inline(always)]
unsafe fn clear_ctr_tx(ep: u32) {
    epr_wr(ep, epr_rd(ep) & 0x8F0F);
}
/* BTABLE (BTABLE reg = 0): entry for EP n at PMA_BASE + 16n; fields ADDR_TX/
 * COUNT_TX/ADDR_RX/COUNT_RX at CPU offsets 0/4/8/12 (doubled addressing). */
#[inline(always)]
unsafe fn bt_rd(ep: u32, field: u32) -> u32 {
    core::ptr::read_volatile((PMA_BASE + 16 * ep as usize + 4 * field as usize) as *const u16) as u32
}
#[inline(always)]
unsafe fn bt_wr(ep: u32, field: u32, v: u16) {
    core::ptr::write_volatile((PMA_BASE + 16 * ep as usize + 4 * field as usize) as *mut u16, v)
}

/* Send `len` (<=6) bytes packed into three little-endian halfwords on the bulk
 * IN side of `ep`, and wait for the host to fetch them. */
#[inline(always)]
unsafe fn send_reply(ep: u32, tx_pma: *mut u16, h0: u16, h1: u16, h2: u16, len: u16) {
    core::ptr::write_volatile(tx_pma, h0);
    core::ptr::write_volatile(tx_pma.wrapping_add(2), h1);
    core::ptr::write_volatile(tx_pma.wrapping_add(4), h2);
    bt_wr(ep, 1, len); /* COUNT_TX */
    clear_ctr_tx(ep);
    set_stat_tx_valid(ep);
    while epr_rd(ep) & 0x80 == 0 {} /* wait CTR_TX (host present during update) */
    clear_ctr_tx(ep);
}

#[link_section = ".flasher"]
#[inline(never)]
extern "C" fn flasher_main(ep: u32) -> ! {
    unsafe {
        core::arch::asm!("cpsid i");

        /* discover this endpoint's PMA buffers from the BTABLE */
        let tx_pma = (PMA_BASE + 2 * bt_rd(ep, 0) as usize) as *mut u16; /* ADDR_TX */
        let rx_pma = (PMA_BASE + 2 * bt_rd(ep, 2) as usize) as *const u16; /* ADDR_RX */

        /* erase the whole app region (we run from SRAM; bootloader code survives) */
        let erase: extern "C" fn(u32, u32, u32) = core::mem::transmute(BL_ERASE_PAGES);
        erase(APP_BASE, PAGE_SIZE, NUM_PAGES);

        set_stat_rx_valid(ep); /* make sure OUT is armed before inviting chunks */

        /* "uprdy" = 75 70 72 64 79 */
        send_reply(ep, tx_pma, 0x7075, 0x6472, 0x0079, 5);

        let unlock: extern "C" fn() = core::mem::transmute(BL_FMC_UNLOCK);
        let program: extern "C" fn(u32, u32) = core::mem::transmute(BL_FMC_PROGRAM_WORD);
        let lock: extern "C" fn() = core::mem::transmute(BL_FMC_LOCK);

        let mut off: u32 = 0;
        /* MaybeUninit: a plain [0u32; 16] could compile to a __aeabi_memclr call
         * into (about-to-be-erased) flash; every word is written before read. */
        let mut words = core::mem::MaybeUninit::<[u32; 16]>::uninit();
        let wp = words.as_mut_ptr() as *mut u32;
        while off < IMAGE_BYTES {
            /* receive one 64-B chunk */
            while epr_rd(ep) & 0x8000 == 0 {} /* wait CTR_RX */
            let mut i = 0usize;
            while i < 16 {
                /* word i = halfwords at rx_pma[+8i] | [+8i+4] (doubled addressing) */
                let lo = core::ptr::read_volatile(rx_pma.wrapping_add(4 * i)) as u32;
                let hi = core::ptr::read_volatile(rx_pma.wrapping_add(4 * i + 2)) as u32;
                core::ptr::write_volatile(wp.add(i), lo | (hi << 16));
                i += 1;
            }
            clear_ctr_rx(ep);
            set_stat_rx_valid(ep); /* next chunk may arrive while we program */

            unlock();
            let mut i = 0u32;
            while i < 16 {
                program(APP_BASE + off + 4 * i, core::ptr::read_volatile(wp.add(i as usize)));
                i += 1;
            }
            lock();

            /* "OK" = 4F 4B */
            send_reply(ep, tx_pma, 0x4B4F, 0, 0, 2);
            off += 64;
        }

        /* reset into the new app */
        core::ptr::write_volatile(0xE000_ED0C as *mut u32, 0x05FA_0004); /* AIRCR */
        core::arch::asm!("dsb");
        loop {}
    }
}

extern "C" {
    static mut __sflasher: u32;
    static __eflasher: u32;
    static __siflasher: u32;
}

/// Copy the flasher to its SRAM run address and jump. `bulk_ep` is the EPnR
/// index of the CDC bulk data endpoint (discovered by the caller).
#[inline(never)]
pub fn launch(bulk_ep: u32) -> ! {
    mk(MK_STAGE, STAGE_UPDAT);
    unsafe {
        let mut src = core::ptr::addr_of!(__siflasher) as *const u32;
        let mut dst = core::ptr::addr_of_mut!(__sflasher) as *mut u32;
        let end = core::ptr::addr_of!(__eflasher) as usize;
        while (dst as usize) < end {
            core::ptr::write_volatile(dst, core::ptr::read_volatile(src));
            src = src.add(1);
            dst = dst.add(1);
        }
        core::arch::asm!("dsb", "isb", "cpsid i");
        /* force an indirect (blx reg) call to the SRAM address */
        let f: extern "C" fn(u32) -> ! = flasher_main;
        let f = core::ptr::read_volatile(&f);
        f(bulk_ep)
    }
}
