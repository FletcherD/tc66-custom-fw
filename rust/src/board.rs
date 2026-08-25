//! Clock bring-up + PA9 D+ soft-connect choreography (carried over unchanged
//! from the hand-rolled stack; sequences proven on hardware, see reference-c/usb.c).

use crate::markers::*;
use crate::regs::*;

#[inline(never)]
pub fn clock_init() {
    /* Target: PLL x12 from HSI/2 => 48 MHz, USBDPSC=/1 (48 MHz USB clock).
     * The bootloader hands off with its OWN clock running (PLL as system clock),
     * so switch back to HSI and disable PLL before touching PLLMF — a running
     * PLL ignores config writes. Robust from any state. */
    modify(RCU_CTL0, |v| v | 1 << 0); /* HSIEN */
    while rd(RCU_CTL0) & (1 << 1) == 0 {} /* wait HSISTB */
    modify(RCU_CFG0, |v| v & !0x3); /* SCS = HSI */
    while rd(RCU_CFG0) & 0xC != 0x0 {} /* wait SCSS = HSI */
    modify(RCU_CTL0, |v| v & !(1 << 24)); /* PLLEN = 0 (now allowed) */
    while rd(RCU_CTL0) & (1 << 25) != 0 {} /* wait PLLSTB clear */
    modify(RCU_CFG0, |v| v & !0x003C_0000); /* clear PLLMF[3:0] (bits 21:18) */
    modify(RCU_CFG0, |v| v & !(1 << 27)); /* clear PLLMF[4] */
    modify(RCU_CFG0, |v| v & !(1 << 16)); /* PLLSEL = HSI/2 */
    modify(RCU_CFG0, |v| v | 10 << 18); /* PLLMF=10 -> x12 (=48MHz) */
    modify(RCU_CFG0, |v| (v & !0x00C0_0000) | 0x0040_0000); /* USBDPSC = 01 (CK_PLL/1) */
    modify(RCU_CTL0, |v| v | 1 << 24); /* PLLEN */
    while rd(RCU_CTL0) & (1 << 25) == 0 {} /* wait PLLSTB */
    modify(RCU_CFG0, |v| (v & !0x3) | 0x2); /* SCS = PLL */
    while rd(RCU_CFG0) & 0xC != 0x8 {} /* wait SCSS = PLL */
    mk(MK_STAGE, STAGE_CLOCK);
}

#[inline(never)]
pub fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms * 48_000);
}

#[inline(never)]
pub fn pa9_config_output() {
    modify(RCU_AHBEN, |v| v | 1 << 17); /* GPIOA clock (AHBEN bit17) */
    modify(GPIOA_MODER, |v| (v & !(3 << 18)) | 1 << 18); /* PA9 = output */
    modify(GPIOA_OTYPER, |v| v & !(1 << 9)); /* push-pull */
    modify(GPIOA_OSPEED, |v| (v & !(3 << 18)) | 1 << 18); /* medium speed */
    modify(GPIOA_PUPDR, |v| v & !(3 << 18)); /* no pull */
}

#[inline(never)]
pub fn pa9_low() {
    wr(GPIOA_BC, PA9_MASK); /* D+ low -> host sees disconnect */
}

#[inline(never)]
pub fn pa9_high() {
    wr(GPIOA_BOP, PA9_MASK); /* D+ pull-up -> host enumerates us */
}
