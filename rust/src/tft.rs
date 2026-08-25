//! ST7735S TFT driver (160x80 RGB565, offset col+1/row+26) on SPI0.
//! The transport, pins, init sequence, and offsets are the stock firmware's,
//! recovered in tc66/firmware/ghidra/TFT_FINDINGS.md:
//!   SCK=PA5, MOSI=PA7 (AF0), DC=PA4, RST=PA3, CS tied low (SW-NSS).
//!   Backlight: enable=PA9 (already high — it doubles as the USB D+
//!   soft-connect, board.rs), brightness=PB3 TIMER1 CH1 PWM.
//!
//! Same safety pattern as i2c.rs/adc.rs: nothing runs at boot — only via the
//! `tftfl`/`tftbl` dev verbs; every SPI wait is bounded and returns a stage.

use gd32f1::gd32f150 as pac;

use crate::board;

pub const WIDTH: u16 = 160;
pub const HEIGHT: u16 = 80;
const COL_OFF: u8 = 1;
const ROW_OFF: u8 = 26;

/* stages (for the ER reply): 1 TBE timeout, 2 shift-out (TRANS) timeout */
const TIMEOUT: u32 = 100_000;

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
static INITED: Racy<bool> = Racy(core::cell::UnsafeCell::new(false));

#[inline(never)]
fn spi_write_byte(b: u8) -> Result<(), u8> {
    let spi = unsafe { &*pac::Spi0::ptr() };
    spi.data().write(|w| w.data().bits(b as u16));
    for _ in 0..TIMEOUT {
        if spi.stat().read().tbe().is_empty() {
            return Ok(());
        }
    }
    Err(1)
}

/// Wait until the shifter is idle — required before flipping DC, since TBE
/// only means the data register emptied, not that the last bit left the pin.
#[inline(never)]
fn spi_wait_idle() -> Result<(), u8> {
    let spi = unsafe { &*pac::Spi0::ptr() };
    for _ in 0..TIMEOUT {
        if spi.stat().read().trans().is_idle() {
            return Ok(());
        }
    }
    Err(2)
}

fn write_command(cmd: u8) -> Result<(), u8> {
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    spi_wait_idle()?;
    gpioa.bc().write(|w| w.cr4().set_bit()); /* DC low = command */
    spi_write_byte(cmd)
}

fn write_data(bytes: &[u8]) -> Result<(), u8> {
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    spi_wait_idle()?;
    gpioa.bop().write(|w| w.bop4().set_bit()); /* DC high = data */
    for &b in bytes {
        spi_write_byte(b)?;
    }
    Ok(())
}

#[inline(never)]
fn cmd(c: u8, data: &[u8]) -> Result<(), u8> {
    write_command(c)?;
    write_data(data)
}

/* Stock ST7735S init, factory-tuned values verbatim (TFT_FINDINGS.md), as a
 * walkable table: cmd byte, ndata|flags, data...; bit7 of ndata = 100 ms delay
 * after the command. Data-driven to keep the code size down at opt-level 1. */
const DELAY: u8 = 0x80;
#[rustfmt::skip]
const INIT_SEQ: &[u8] = &[
    0x11, DELAY,                        /* SLPOUT */
    0x21, 0,                            /* INVON — IPS panel */
    0xB1, 3, 0x05, 0x3C, 0x3C,
    0xB2, 3, 0x05, 0x3C, 0x3C,
    0xB3, 6, 0x05, 0x3C, 0x3C, 0x05, 0x3C, 0x3C,
    0xB4, 1, 0x03,
    0xC0, 3, 0xE0, 0x00, 0x07,
    0xC1, 1, 0xC5,
    0xC2, 2, 0x0A, 0x00,
    0xC4, 2, 0x8D, 0xEE,
    0xC5, 1, 0x03,
    0xC7, 1, 0x76,
    0x36, 1, 0xA8,                      /* MADCTL: landscape, BGR, 180° — the
                                         * stock flip variant; 0x68 is upside-
                                         * down for this unit's mounting */
    0x3A, 1, 0x55,                      /* COLMOD: RGB565 */
    0xE0, 16, 0x27, 0x0E, 0x07, 0x04, 0x11, 0x0B, 0x06, 0x0C,
              0x0E, 0x14, 0x1B, 0x3E, 0x06, 0x25, 0x07, 0x1F,
    0xE1, 16 | DELAY,
              0x27, 0x0E, 0x07, 0x04, 0x11, 0x0B, 0x06, 0x0C,
              0x0E, 0x14, 0x1B, 0x3E, 0x30, 0x25, 0x07, 0x1F,
    0x29, 0,                            /* DISPON */
];

#[inline(never)]
fn ensure_init() -> Result<(), u8> {
    unsafe {
        if *INITED.0.get() {
            return Ok(());
        }
    }
    let rcu = unsafe { &*pac::Rcu::ptr() };
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    let spi = unsafe { &*pac::Spi0::ptr() };

    rcu.ahben().modify(|_, w| w.paen().enabled());
    rcu.apb2en().modify(|_, w| w.spi0en().enabled());

    /* PA5/PA7 -> AF0 (SPI0 SCK/MOSI); PA3/PA4 -> outputs (RST/DC) */
    gpioa.afsel0().modify(|_, w| w.sel5().af0().sel7().af0());
    gpioa
        .omode()
        .modify(|_, w| w.om3().push_pull().om4().push_pull().om5().push_pull().om7().push_pull());
    gpioa.ospd().modify(|_, w| {
        w.ospd3()
            .speed50m()
            .ospd4()
            .speed50m()
            .ospd5()
            .speed50m()
            .ospd7()
            .speed50m()
    });
    gpioa
        .pud()
        .modify(|_, w| w.pud3().floating().pud4().floating().pud5().floating().pud7().floating());
    gpioa
        .ctl()
        .modify(|_, w| w.ctl3().output().ctl4().output().ctl5().alternate().ctl7().alternate());

    /* SPI0: master, mode 0, 8-bit, MSB-first, SW-NSS held high, PSC /2
     * (APB2 = 48 MHz -> SCLK 24 MHz; stock ran /2 off 72 MHz) */
    spi.ctl0().write(|w| {
        w.mstmod()
            .master()
            .swnssen()
            .software()
            .swnss()
            .set_bit()
            .ckpl()
            .idle_low()
            .ckph()
            .first_edge()
            .ff16()
            .eight_bit()
            .lf()
            .msbfirst()
            .psc()
            .div2()
    });
    spi.ctl0().modify(|_, w| w.spien().enabled());

    /* hardware reset pulse on PA3 (no SW-reset 0x01 in the stock sequence) */
    gpioa.bc().write(|w| w.cr3().set_bit());
    board::delay_ms(100);
    gpioa.bop().write(|w| w.bop3().set_bit());
    board::delay_ms(100);

    let mut i = 0;
    while i < INIT_SEQ.len() {
        let n = (INIT_SEQ[i + 1] & !DELAY) as usize;
        cmd(INIT_SEQ[i], &INIT_SEQ[i + 2..i + 2 + n])?;
        if INIT_SEQ[i + 1] & DELAY != 0 {
            board::delay_ms(100);
        }
        i += 2 + n;
    }

    unsafe { *INITED.0.get() = true };
    fill_rect(0, 0, WIDTH - 1, HEIGHT - 1, 0x0000)?; /* clear to black */
    backlight_init();
    Ok(())
}

/// CASET/RASET with the panel's GRAM offset, then RAMWR; pixels follow as data.
fn set_window(x0: u16, y0: u16, x1: u16, y1: u16) -> Result<(), u8> {
    cmd(
        0x2A,
        &[0, x0 as u8 + COL_OFF, 0, x1 as u8 + COL_OFF],
    )?;
    cmd(
        0x2B,
        &[0, y0 as u8 + ROW_OFF, 0, y1 as u8 + ROW_OFF],
    )?;
    write_command(0x2C)
}

#[inline(never)]
pub fn fill_rect(x0: u16, y0: u16, x1: u16, y1: u16, color: u16) -> Result<(), u8> {
    ensure_init()?;
    set_window(x0, y0, x1, y1)?;
    let n = (x1 - x0 + 1) as u32 * (y1 - y0 + 1) as u32;
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    spi_wait_idle()?;
    gpioa.bop().write(|w| w.bop4().set_bit()); /* DC high, stays high */
    for _ in 0..n {
        spi_write_byte((color >> 8) as u8)?;
        spi_write_byte(color as u8)?;
    }
    spi_wait_idle()
}

/// Bring-up pattern: white border + R/G/B bands + a black square in the
/// top-left corner. Verifies the GRAM offset (border reaches all four edges),
/// color order, and — via the asymmetric square — both rotation axes.
#[inline(never)]
pub fn test_pattern() -> Result<(), u8> {
    ensure_init()?;
    fill_rect(0, 0, WIDTH - 1, HEIGHT - 1, 0xFFFF)?;
    let bands: [(u16, u16); 3] = [(0xF800, 2), (0x07E0, 28), (0x001F, 54)];
    for &(color, y) in bands.iter() {
        fill_rect(30, y, WIDTH - 3, y + 23, color)?;
    }
    fill_rect(2, 2, 25, 25, 0x0000)
}

/// TIMER1 CH1 PWM on PB3 (AF2) for the backlight LED driver. 48 MHz / 48 /
/// 1000 = 1 kHz (stock: 1.5 kHz off 72 MHz — the rate is uncritical).
#[inline(never)]
fn backlight_init() {
    let rcu = unsafe { &*pac::Rcu::ptr() };
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    let tim = unsafe { &*pac::Timer1::ptr() };

    rcu.ahben().modify(|_, w| w.pben().enabled());
    rcu.apb1en().modify(|_, w| w.timer1en().enabled());

    gpiob.afsel0().modify(|_, w| w.sel3().af2());
    gpiob.omode().modify(|_, w| w.om3().push_pull());
    gpiob.ospd().modify(|_, w| w.ospd3().speed10m());
    gpiob.pud().modify(|_, w| w.pud3().floating());
    gpiob.ctl().modify(|_, w| w.ctl3().alternate());

    tim.psc().write(|w| w.psc().bits(47));
    tim.car().write(|w| w.car().bits(999));
    tim.chctl0_output().modify(|_, w| {
        w.ch1ms()
            .output()
            .ch1comctl()
            .pwm_mode0()
            .ch1comsen()
            .set_bit()
    });
    tim.chctl2().modify(|_, w| w.ch1en().set_bit());
    tim.ch1cv().write(|w| w.ch1val().bits(499)); /* ~50% */
    tim.swevg().write(|w| w.upg().set_bit()); /* latch PSC/CAR now */
    tim.ctl0().modify(|_, w| w.cen().enabled());
}

/// Set backlight PWM compare directly (0 = dark .. 999 = full).
#[inline(never)]
pub fn backlight_set(compare: u16) -> Result<(), u8> {
    ensure_init()?;
    let tim = unsafe { &*pac::Timer1::ptr() };
    let c = if compare > 999 { 999 } else { compare };
    tim.ch1cv().write(|w| w.ch1val().bits(c as u32));
    Ok(())
}
