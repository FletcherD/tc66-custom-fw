//! Register map + volatile access helpers, 1:1 with reference-c/usb.c lines 20-51.
//! Raw addresses on purpose (no PAC yet) so the sequences diff cleanly against the
//! proven C. Single-core, IRQs permanently masked — plain volatile is sound.

#![allow(dead_code)]

pub const RCU_BASE: usize = 0x4002_1000;
pub const RCU_CTL0: usize = RCU_BASE + 0x00;
pub const RCU_CFG0: usize = RCU_BASE + 0x04;
pub const RCU_AHBEN: usize = RCU_BASE + 0x14;
pub const RCU_APB1EN: usize = RCU_BASE + 0x1C;

// GPIOA — PA9 is the USB D+ soft-connect (PA9 --resistor--> USB_DP).
pub const GPIOA_BASE: usize = 0x4800_0000;
pub const GPIOA_MODER: usize = GPIOA_BASE + 0x00;
pub const GPIOA_OTYPER: usize = GPIOA_BASE + 0x04;
pub const GPIOA_OSPEED: usize = GPIOA_BASE + 0x08;
pub const GPIOA_PUPDR: usize = GPIOA_BASE + 0x0C;
pub const GPIOA_BOP: usize = GPIOA_BASE + 0x18; // set bits
pub const GPIOA_BC: usize = GPIOA_BASE + 0x28; // clear bits
pub const PA9_MASK: u32 = 1 << 9;

pub const USB_BASE: usize = 0x4000_5C00;
pub const USB_CNTR: usize = USB_BASE + 0x40;
pub const USB_ISTR: usize = USB_BASE + 0x44;
pub const USB_DADDR: usize = USB_BASE + 0x4C;
pub const USB_BTABLE: usize = USB_BASE + 0x50;

#[inline(always)]
pub fn usb_epr(n: u32) -> usize {
    USB_BASE + 4 * n as usize
}

/// PMA: 16-bit slots, doubled-addressed. CPU addr of PMA byte-offset `po`
/// = 0x40006000 + 2*po (each 16-bit slot occupies 4 CPU-address bytes).
pub const PMA_BASE: usize = 0x4000_6000;

#[inline(always)]
pub fn pma_u16(po: u32) -> *mut u16 {
    (PMA_BASE + 2 * po as usize) as *mut u16
}

/// BTABLE field for EP n. field: 0=ADDR_TX 1=COUNT_TX 2=ADDR_RX 3=COUNT_RX.
/// CPU addr = PMA_BASE + 16*n + 4*field — the C's `BT(n)[2*field]` (u16 index
/// steps by 2 because of the doubled addressing).
#[inline(always)]
pub fn bt(n: u32, field: u32) -> *mut u16 {
    (PMA_BASE + 16 * n as usize + 4 * field as usize) as *mut u16
}

#[inline(always)]
pub fn rd(addr: usize) -> u32 {
    unsafe { core::ptr::read_volatile(addr as *const u32) }
}

#[inline(always)]
pub fn wr(addr: usize, v: u32) {
    unsafe { core::ptr::write_volatile(addr as *mut u32, v) }
}

#[inline(always)]
pub fn modify(addr: usize, f: impl FnOnce(u32) -> u32) {
    wr(addr, f(rd(addr)));
}
