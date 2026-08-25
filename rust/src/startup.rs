//! Early startup + fault capture.
//!
//! cortex-m-rt owns the vector table (240 IRQ entries, all DefaultHandler) and
//! .data/.bss init; we hook the two PROVIDE'd symbols it leaves open:
//!  - `__pre_init`: runs before .data/.bss — cpsid i, VTOR, NVIC mask+clear.
//!    VTOR is set AFTER cpsid i (same order as the C): with interrupts still
//!    enabled a bootloader-armed pending IRQ must vector through the bootloader's
//!    valid table, not into our spinning DefaultHandler.
//!  - `DefaultHandler`: asm (not #[exception]) so the stacked-PC capture reads the
//!    hardware exception frame directly at MSP, exactly like the C IRQ_Default.

use core::ptr::{read_volatile, write_volatile};
use cortex_m_rt::{exception, ExceptionFrame};

use crate::markers::{MK_PANIC, MK_PANIC_LINE};

core::arch::global_asm!(
    r#"
    .section .text.__pre_init, "ax"
    .global __pre_init
    .type __pre_init, %function
    .thumb_func
__pre_init:
    cpsid   i
    /* Raise MSP to just below the SRAM marker window: the flash header must
       keep initial SP = 0x20000E40 (the bootloader faults on some SP values
       — 0x20002000 known bad, predicate unknown — so we ship the proven
       one), but the app needs the extra 448 B of stack (see stack-gate).
       Safe here: Reset loads MSP then bl's us with nothing yet stacked.
       KEEP IN SYNC with STACK_TOP in tools/stack_gate.py. */
    ldr     r0, =0x20001000
    msr     msp, r0
    ldr     r0, =0x08002c00         @ VTOR = our vector table
    ldr     r1, =0xE000ED08
    str     r0, [r1]
    ldr     r1, =0xE000E180         @ mask+clear ALL NVIC IRQs (bootloader may
    mvn     r0, #0                  @ have left USBD IRQ37 enabled/pending)
    str     r0, [r1, #0]            @ ICER0
    str     r0, [r1, #4]            @ ICER1
    str     r0, [r1, #0x100]        @ ICPR0 (0xE000E280)
    str     r0, [r1, #0x104]        @ ICPR1
    dsb
    isb
    bx      lr

    /* NMI + any unexpected IRQ/system exception: record IPSR + stacked PC, spin.
       0x2000104c = 0xE7C00000 sentinel, +4 = IPSR, +8 = stacked PC */
    .section .text.DefaultHandler, "ax"
    .global DefaultHandler
    .type DefaultHandler, %function
    .thumb_func
DefaultHandler:
    mrs     r0, msp
    ldr     r1, =0x2000104c
    ldr     r2, =0xE7C00000
    str     r2, [r1, #0]
    mrs     r2, ipsr
    str     r2, [r1, #4]
    ldr     r2, [r0, #0x18]         @ stacked PC
    str     r2, [r1, #8]
1:  b       1b
"#
);

/* HardFault capture window (MemManage/BusFault/UsageFault are not enabled in
 * SHCSR, so they escalate here — same net behavior as the C's vector aliasing):
 *   0x20001008 = 0xFA017FA0 sentinel, +4 = stacked PC, +8 = HFSR, +c = CFSR,
 *   +10 = stacked LR, +14 = IPSR */
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    write_volatile(0x2000_1008 as *mut u32, 0xFA01_7FA0);
    write_volatile(0x2000_100c as *mut u32, ef.pc());
    write_volatile(0x2000_1010 as *mut u32, read_volatile(0xE000_ED2C as *const u32)); // HFSR
    write_volatile(0x2000_1014 as *mut u32, read_volatile(0xE000_ED28 as *const u32)); // CFSR
    write_volatile(0x2000_1018 as *mut u32, ef.lr());
    let ipsr: u32;
    core::arch::asm!("mrs {}, ipsr", out(reg) ipsr);
    write_volatile(0x2000_101c as *mut u32, ipsr);
    loop {}
}

#[panic_handler]
#[inline(never)]
fn panic(info: &core::panic::PanicInfo) -> ! {
    unsafe {
        write_volatile(MK_PANIC as *mut u32, 0xDEAD_C0DE);
        write_volatile(
            MK_PANIC_LINE as *mut u32,
            info.location().map(|l| l.line()).unwrap_or(0),
        );
    }
    loop {}
}
