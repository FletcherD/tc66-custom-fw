//! Free watchdog timer (FWDGT): a hung main loop reboots the meter instead
//! of sitting dead until someone power-cycles it.
//!
//! Clocked by IRC40K (30-60 kHz, nominally 40) through /128 with a 4095
//! reload: ~13 s nominal, ~8.7 s worst case. Every thread-mode blocking
//! path is far shorter (AFC/QC entry holds <= 2.5 s, the `updat` flasher
//! feeds it from SRAM itself), so a bite always means a genuine hang.
//! Once started it cannot be stopped — only a reset clears it. `feed` is a
//! single register write, called once per main-loop pass (and from the
//! grace window loop when that feature is on).

use gd32f1::gd32f150 as pac;

/// RCU_RSTSCK as captured at boot (bit 29 = FWDGT reset, 26 = POR,
/// 27 = NRST pin, 28 = software), flags then cleared for the next boot.
static RESET_CAUSE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

#[inline(never)]
pub fn start(rcu: &pac::Rcu) {
    let rs = rcu.rstsck().read().bits();
    RESET_CAUSE.store(rs, core::sync::atomic::Ordering::Relaxed);
    rcu.rstsck().modify(|_, w| w.rstfc().set_bit());
    rcu.rstsck().modify(|_, w| w.irc40ken().set_bit());
    while rcu.rstsck().read().irc40kstb().bit_is_clear() {}
    let w = unsafe { &*pac::Fwdgt::ptr() };
    w.ctl().write(|x| x.cmd().enable());
    w.psc().write(|x| x.psc().divide_by128());
    w.rld().write(|x| x.rld().bits(4095));
    w.ctl().write(|x| x.cmd().start());
    w.ctl().write(|x| x.cmd().reset());
}

#[inline(always)]
pub fn feed() {
    unsafe { core::ptr::write_volatile(0x4000_3000 as *mut u32, 0xAAAA) };
}

#[cfg_attr(not(feature = "dev-verbs"), allow(dead_code))]
pub fn reset_cause() -> u32 {
    RESET_CAUSE.load(core::sync::atomic::Ordering::Relaxed)
}
