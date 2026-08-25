//! Clock bring-up + PA9 D+ soft-connect choreography, on the gd32f1 PAC.
//! The register sequence is 1:1 with the proven one in reference-c/usb.c
//! (each modify() below corresponds to one RMW step of the C).

use gd32f1::gd32f150 as pac;

use crate::markers::*;

#[inline(never)]
pub fn clock_init(rcu: &pac::Rcu) {
    /* Target: PLL x12 from IRC8M/2 => 48 MHz, USBDPSC=/1 (48 MHz USB clock).
     * The bootloader hands off with its OWN clock running (PLL as system clock),
     * so switch back to IRC8M and disable PLL before touching PLLMF — a running
     * PLL ignores config writes. Robust from any state. */
    rcu.ctl0().modify(|_, w| w.irc8men().on());
    while rcu.ctl0().read().irc8mstb().is_not_ready() {}
    rcu.cfg0().modify(|_, w| w.scs().irc8m());
    while !rcu.cfg0().read().scss().is_irc8m() {}
    rcu.ctl0().modify(|_, w| w.pllen().off());
    while rcu.ctl0().read().pllstb().is_ready() {} /* wait PLLSTB clear */
    rcu.cfg0().modify(|_, w| w.pllmf().mul12()); /* x12 (=48 MHz) */
    rcu.cfg0().modify(|_, w| w.pllmf_msb().clear_bit());
    rcu.cfg0().modify(|_, w| w.pllsel().irc8m_2());
    rcu.cfg0().modify(|_, w| w.usbdpsc().div1()); /* USB clock = CK_PLL/1 */
    rcu.ctl0().modify(|_, w| w.pllen().on());
    while rcu.ctl0().read().pllstb().is_not_ready() {}
    rcu.cfg0().modify(|_, w| w.scs().pll());
    while !rcu.cfg0().read().scss().is_pll() {}
    mk(MK_STAGE, STAGE_CLOCK);
}

#[inline(never)]
pub fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms * 48_000);
}

#[inline(never)]
pub fn pa9_config_output(rcu: &pac::Rcu, gpioa: &pac::Gpioa) {
    rcu.ahben().modify(|_, w| w.paen().enabled()); /* GPIOA clock */
    gpioa.ctl().modify(|_, w| w.ctl9().output());
    gpioa.omode().modify(|_, w| w.om9().push_pull());
    gpioa.ospd().modify(|_, w| w.ospd9().speed10m());
    gpioa.pud().modify(|_, w| w.pud9().floating());
}

#[inline(never)]
pub fn pa9_low(gpioa: &pac::Gpioa) {
    gpioa.bc().write(|w| w.cr9().set_bit()); /* D+ low -> host sees disconnect */
}

#[inline(never)]
pub fn pa9_high(gpioa: &pac::Gpioa) {
    gpioa.bop().write(|w| w.bop9().set_bit()); /* D+ pull-up -> host enumerates us */
}
