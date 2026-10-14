//! Polled I2C0 master on PB6=SCL / PB7=SDA (AF1, open-drain; the board has
//! external pull-ups — this is the stock firmware's bit-banged bus, driven here
//! by the hardware peripheral instead). GD32F1x0's I2C is the STM32F1-style IP
//! (SBSEND/ADDSEND/BTC + the POAP/ACKEN two-byte dance, per ST AN2824 and
//! gd32f1x0-hal's implementation).
//!
//! Safety-first design for bring-up (M1 step 1):
//!  - nothing here runs at boot — only when the host sends `i2crd`, so a fault
//!    or wedge is always recoverable by power-cycling (boot path untouched);
//!  - every flag wait is bounded and returns an error snapshot (stage + STAT0 +
//!    STAT1) that the reply carries back to the host for diagnosis;
//!  - the peripheral is SRESET + reconfigured on every transaction: stateless,
//!    and self-healing after most bus wedges.

use gd32f1::gd32f150 as pac;

pub struct I2cErr {
    /// which wait timed out; bit7 set = bus error/NACK detected at that stage
    pub stage: u8,
    pub stat0: u16,
    /* read only by the i2crd verb's ER reply */
    #[cfg_attr(not(feature = "dev-board"), allow(dead_code))]
    pub stat1: u16,
}

/* stages (for the ER reply): 1 busy-idle 2 SBSEND(w) 3 ADDSEND(w) 4 TBE
 * 5 BTC(w) 6 SBSEND(r) 7 ADDSEND(r) 8 RBNE 9 BTC(r) 0x0F bad-args */
const TIMEOUT: u32 = 100_000;
/* The bus must be idle at transaction start; when it is stuck busy (wedged
 * slave) that must fail FAST — the sampler ISR pays this wait on every tick,
 * and the original 100k iterations (~30 ms) starved the thread + USB into a
 * system-wide freeze (build 65). ~1.5 ms still covers any legitimate
 * in-flight STOP tail. */
const BUSY_TIMEOUT: u32 = 5_000;

fn snapshot(stage: u8, i2c: &pac::i2c0::RegisterBlock) -> I2cErr {
    I2cErr {
        stage,
        stat0: i2c.stat0().read().bits() as u16,
        stat1: i2c.stat1().read().bits() as u16,
    }
}

fn wait_n(
    i2c: &pac::i2c0::RegisterBlock,
    stage: u8,
    limit: u32,
    cond: impl Fn(&pac::i2c0::stat0::R) -> bool,
) -> Result<(), I2cErr> {
    for _ in 0..limit {
        let s = i2c.stat0().read();
        if cond(&s) {
            return Ok(());
        }
        /* NACK / bus error / lost arbitration: no point spinning out the clock */
        if s.aerr().is_error() || s.berr().is_error() || s.lostarb().is_lost() {
            return Err(snapshot(stage | 0x80, i2c));
        }
    }
    Err(snapshot(stage, i2c))
}

fn wait(
    i2c: &pac::i2c0::RegisterBlock,
    stage: u8,
    cond: impl Fn(&pac::i2c0::stat0::R) -> bool,
) -> Result<(), I2cErr> {
    wait_n(i2c, stage, TIMEOUT, cond)
}

#[inline(never)]
fn setup(i2c: &pac::i2c0::RegisterBlock) {
    let rcu = unsafe { &*pac::Rcu::ptr() };
    let gpiob = unsafe { &*pac::Gpiob::ptr() };

    rcu.ahben().modify(|_, w| w.pben().enabled());
    rcu.apb1en().modify(|_, w| w.i2c0en().enabled());

    /* PB6/PB7: alternate function 1, open-drain */
    gpiob.afsel0().modify(|_, w| w.sel6().af1().sel7().af1());
    gpiob.omode().modify(|_, w| w.om6().open_drain().om7().open_drain());
    gpiob.pud().modify(|_, w| w.pud6().floating().pud7().floating());
    gpiob.ctl().modify(|_, w| w.ctl6().alternate().ctl7().alternate());

    /* full reset then 100 kHz standard mode from PCLK1 = 48 MHz
     * (i2cclk = 48, clkc = 48 MHz / (2 * 100 kHz) = 240, risetime = 48 + 1) */
    i2c.ctl0().write(|w| w.sreset().set_bit());
    i2c.ctl0().write(|w| w.sreset().clear_bit());
    i2c.ctl1().write(|w| unsafe { w.i2cclk().bits(48) });
    i2c.ckcfg().write(|w| w.clkc().bits(240));
    i2c.rt().write(|w| w.risetime().bits(49));
    i2c.ctl0().write(|w| w.i2cen().set_bit());
}

/// Manually clock the bus free: 9 SCL pulses (releases a slave stuck mid-read
/// driving SDA) then a STOP, PB6/PB7 as plain GPIO. ~120 us, bounded, no
/// side effects beyond the pins — setup() reclaims them as AF1 afterwards.
#[inline(never)]
fn unstick() {
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    gpiob.bop().write(|w| w.bop6().set_bit().bop7().set_bit()); /* both released */
    gpiob.omode().modify(|_, w| w.om6().open_drain().om7().open_drain());
    gpiob.ctl().modify(|_, w| w.ctl6().output().ctl7().output());
    let half = || cortex_m::asm::delay(250); /* ~5 us @ 48 MHz */
    half();
    for _ in 0..9 {
        gpiob.bc().write(|w| w.cr6().set_bit()); /* SCL low */
        half();
        gpiob.bop().write(|w| w.bop6().set_bit()); /* SCL high */
        half();
    }
    /* STOP: SDA low -> high while SCL is high */
    gpiob.bc().write(|w| w.cr7().set_bit());
    half();
    gpiob.bop().write(|w| w.bop7().set_bit());
    half();
}

/// One transaction: optional register-pointer write (no STOP), then a
/// repeated-START read of `rbuf.len()` bytes. `wbuf` empty = pure read;
/// `rbuf` empty = pure write (with STOP).
#[inline(never)]
pub fn transact(addr7: u8, wbuf: &[u8], rbuf: &mut [u8]) -> Result<(), I2cErr> {
    let i2c = unsafe { &*pac::I2c0::ptr() };
    setup(i2c);

    let mut r = do_transact(i2c, addr7, wbuf, rbuf);
    if let Err(ref e) = r {
        if e.stage & 0x7F == 1 {
            /* bus stuck busy before we even started: clock it free and retry
             * once (self-healing for a slave left mid-transaction) */
            unstick();
            setup(i2c);
            r = do_transact(i2c, addr7, wbuf, rbuf);
        }
    }
    if r.is_err() {
        /* abort: force STOP so a next attempt starts from idle */
        i2c.ctl0().modify(|_, w| w.stop().set_bit());
    }
    /* Let the final STOP finish on the wires before returning. The caller
     * may be about to unmask the sampler tick, whose transaction begins
     * with an SRESET — resetting the peripheral mid-STOP abandons the STOP
     * and wedges the bus busy (this froze build 65: tick timeouts then
     * starved the whole system). Bounded, normally a few us. */
    for _ in 0..2_000 {
        if i2c.stat1().read().i2cbsy().is_not_busy() {
            break;
        }
    }
    r
}

fn do_transact(
    i2c: &pac::i2c0::RegisterBlock,
    addr7: u8,
    wbuf: &[u8],
    rbuf: &mut [u8],
) -> Result<(), I2cErr> {
    wait_n(i2c, 1, BUSY_TIMEOUT, |_| {
        i2c.stat1().read().i2cbsy().is_not_busy()
    })?;

    if !wbuf.is_empty() {
        i2c.ctl0().modify(|_, w| w.start().set_bit());
        wait(i2c, 2, |s| s.sbsend().is_start())?;
        i2c.data().write(|w| unsafe { w.bits((addr7 as u16) << 1) });
        wait(i2c, 3, |s| s.addsend().is_match())?;
        let _ = i2c.stat0().read();
        let _ = i2c.stat1().read(); /* clear ADDSEND */
        for &b in wbuf {
            wait(i2c, 4, |s| s.tbe().is_empty())?;
            i2c.data().write(|w| unsafe { w.bits(b as u16) });
        }
        wait(i2c, 5, |s| s.btc().is_finished())?;
        if rbuf.is_empty() {
            i2c.ctl0().modify(|_, w| w.stop().set_bit());
            return Ok(());
        }
    } else if rbuf.is_empty() {
        return Ok(());
    }

    /* master receive, ST AN2824 sequences per length */
    let n = rbuf.len();
    if n == 2 {
        /* 2 bytes: POAP=next + ACK before START; after ADDSEND clears: ACK off,
         * wait BTC, STOP, read both from DR + shift register */
        i2c.ctl0().modify(|_, w| w.poap().set_bit().acken().set_bit());
    } else if n > 2 {
        i2c.ctl0().modify(|_, w| w.acken().set_bit());
    }
    i2c.ctl0().modify(|_, w| w.start().set_bit());
    wait(i2c, 6, |s| s.sbsend().is_start())?;
    i2c.data().write(|w| unsafe { w.bits(((addr7 as u16) << 1) | 1) });
    wait(i2c, 7, |s| s.addsend().is_match())?;

    match n {
        1 => {
            i2c.ctl0().modify(|_, w| w.acken().clear_bit());
            let _ = i2c.stat0().read();
            let _ = i2c.stat1().read(); /* clear ADDSEND */
            i2c.ctl0().modify(|_, w| w.stop().set_bit());
            wait(i2c, 8, |s| s.rbne().is_not_empty())?;
            rbuf[0] = i2c.data().read().bits() as u8;
        }
        2 => {
            let _ = i2c.stat0().read();
            let _ = i2c.stat1().read(); /* clear ADDSEND */
            i2c.ctl0().modify(|_, w| w.acken().clear_bit());
            wait(i2c, 9, |s| s.btc().is_finished())?;
            i2c.ctl0().modify(|_, w| w.stop().set_bit());
            rbuf[0] = i2c.data().read().bits() as u8;
            rbuf[1] = i2c.data().read().bits() as u8;
        }
        _ => {
            let _ = i2c.stat0().read();
            let _ = i2c.stat1().read(); /* clear ADDSEND */
            let mut i = 0;
            while n - i > 3 {
                wait(i2c, 8, |s| s.rbne().is_not_empty())?;
                rbuf[i] = i2c.data().read().bits() as u8;
                i += 1;
            }
            wait(i2c, 9, |s| s.btc().is_finished())?; /* N-2 in DR, N-1 in shift */
            i2c.ctl0().modify(|_, w| w.acken().clear_bit());
            rbuf[i] = i2c.data().read().bits() as u8; /* N-2 */
            i2c.ctl0().modify(|_, w| w.stop().set_bit());
            rbuf[i + 1] = i2c.data().read().bits() as u8; /* N-1 */
            wait(i2c, 8, |s| s.rbne().is_not_empty())?;
            rbuf[i + 2] = i2c.data().read().bits() as u8; /* N */
        }
    }

    /* restore defaults for the next transaction */
    i2c.ctl0().modify(|_, w| w.poap().clear_bit().acken().clear_bit());
    Ok(())
}
