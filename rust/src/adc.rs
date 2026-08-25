//! Polled on-chip ADC (F1-style: CLB calibration, SWRCST software trigger).
//! Used for the NTC temperature divider and the D+/D- dividers (HARDWARE.md:
//! published D± = sum_of_16 * 330 / 65520 -> 1e-2 V). Channels are identified
//! empirically via the `adcrd` dev verb (tools/adcread.py --scan).
//!
//! Same safety pattern as i2c.rs: bounded waits, Option returns, lazy init.

use gd32f1::gd32f150 as pac;

const TIMEOUT: u32 = 100_000;

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
static INITED: Racy<bool> = Racy(core::cell::UnsafeCell::new(false));

fn ensure_init() -> Result<(), u8> {
    unsafe {
        if *INITED.0.get() {
            return Ok(());
        }
    }
    let rcu = unsafe { &*pac::Rcu::ptr() };
    let adc = unsafe { &*pac::Adc::ptr() };

    rcu.apb2en().modify(|_, w| w.adcen().enabled());
    /* CK_ADC comes from the dedicated IRC14M oscillator (CFG2.ADCSEL reset
     * default) — which is OFF at reset; without it RSTCLB/CLB hang forever. */
    rcu.ctl1().modify(|_, w| w.irc14men().on());
    let mut ok = false;
    for _ in 0..TIMEOUT {
        if rcu.ctl1().read().irc14mstb().is_ready() {
            ok = true;
            break;
        }
    }
    if !ok {
        return Err(4); /* IRC14M never stabilized */
    }
    rcu.cfg2().modify(|_, w| w.adcsel().irc14m()); /* explicit, matches reset */

    /* ADCON must be set by a write that changes NO other CTL1 bits (F1-style
     * quirk), so: power on alone, wait t_stab, then the sensor enable. */
    adc.ctl1().modify(|_, w| w.adcon().set_bit());
    cortex_m::asm::delay(96_000); /* t_stab, generous 2 ms */
    adc.ctl1().modify(|_, w| w.tsvren().set_bit()); /* internal temp/vref ch16/17 */
    cortex_m::asm::delay(96_000);

    /* max sampling time (239.5 cycles) on all channels — NTC/divider sources
     * are high-impedance; 3-bit fields, all-ones across both SAMPT regs */
    adc.sampt0().write(|w| unsafe { w.bits(0x00FF_FFFF) });
    adc.sampt1().write(|w| unsafe { w.bits(0x3FFF_FFFF) });

    /* calibration: reset then calibrate, both self-clearing */
    adc.ctl1().modify(|_, w| w.rstclb().set_bit());
    let mut ok = false;
    for _ in 0..TIMEOUT {
        if adc.ctl1().read().rstclb().bit_is_clear() {
            ok = true;
            break;
        }
    }
    if !ok {
        return Err(1); /* RSTCLB stuck */
    }
    adc.ctl1().modify(|_, w| w.clb().set_bit());
    ok = false;
    for _ in 0..TIMEOUT {
        if adc.ctl1().read().clb().bit_is_clear() {
            ok = true;
            break;
        }
    }
    if !ok {
        return Err(2); /* CLB stuck */
    }

    /* regular-group trigger = SWRCST, trigger enabled, sequence length 1 */
    adc.ctl1()
        .modify(|_, w| w.etsrc().swrcst().eterc().set_bit());
    adc.rsq0().modify(|_, w| w.rl().bits(0));

    unsafe { *INITED.0.get() = true };
    Ok(())
}

fn convert_once(ch: u8) -> Result<u16, u8> {
    let adc = unsafe { &*pac::Adc::ptr() };
    adc.rsq2().modify(|_, w| unsafe { w.rsq1().bits(ch) });
    adc.ctl1().modify(|_, w| w.swrcst().set_bit());
    for _ in 0..TIMEOUT {
        if adc.stat().read().eoc().is_complete() {
            return Ok(adc.rdata().read().bits() as u16); /* read clears EOC */
        }
    }
    Err(3) /* EOC timeout */
}

/// Sum of 16 conversions of `ch` (the wire formulas all work on sum-of-16).
/// Err carries a stage: 1 RSTCLB stuck, 2 CLB stuck, 3 EOC timeout.
#[inline(never)]
pub fn read_sum16(ch: u8) -> Result<u32, u8> {
    ensure_init()?;
    let mut sum: u32 = 0;
    for _ in 0..16 {
        sum += convert_once(ch)? as u32;
    }
    Ok(sum)
}
