//! ST7735S TFT driver (160x80 RGB565, offset col+1/row+26) on SPI0.
//! The transport, pins, init sequence, and offsets are the stock firmware's,
//! recovered in tc66/firmware/ghidra/TFT_FINDINGS.md:
//!   SCK=PA5, MOSI=PA7 (AF0), DC=PA4, RST=PA3, CS tied low (SW-NSS).
//!   Backlight: enable=PA9 (already high — it doubles as the USB D+
//!   soft-connect, board.rs), brightness=PB3 TIMER1 CH1 PWM.
//!
//! Pixel streaming is DMA-driven and non-blocking: commands (CASET/RASET/
//! RAMWR, init) stay polled 8-bit writes, but bulk pixel data goes out as
//! 16-bit SPI frames fed by DMA channel 2 (the fixed SPI0_TX request on this
//! part). Callers kick a job (`start_char`, or the blocking `fill_rect`) and
//! the main loop pumps `service()` to advance it; nothing busy-waits on
//! pixels outside the bounded dev-verb/boot pumps. Jobs:
//!   - fill: one DMA transfer from a single staged color word with the
//!     memory-increment disabled — zero staging RAM;
//!   - glyph: per-row staging — one 18-px row (36 B) is expanded 1 bpp ->
//!     RGB565 into ROWBUF and DMA'd; the next row is staged on completion.
//!
//! Same safety pattern as i2c.rs/adc.rs: nothing runs at boot — only via the
//! `tftfl`/`tftbl`/`tftui` verbs and the pre-enumeration first frame; every
//! wait is bounded and returns a stage; a failed job tears the session down
//! (DMA off, SPI back to 8-bit) so the next attempt starts clean.

use gd32f1::gd32f150 as pac;

use crate::board;
use crate::font;
use crate::markers::*;

pub const WIDTH: u16 = 160;
pub const HEIGHT: u16 = 80;
const COL_OFF: u8 = 1;
const ROW_OFF: u8 = 26;

/* stages (for the ER reply):
 *   1 SPI TBE timeout               (polled command path)
 *   2 SPI shift-out (TRANS) timeout (polled command path / session teardown)
 *   3 DMA completion timeout        (FTF flag never set)
 *   4 DMA transfer-error flag       (ERRIF — bad bus access)
 *   5 engine busy                   (start_* called while a job runs)
 *   6 pump budget exhausted         (blocking wrapper gave up) */
const TIMEOUT: u32 = 100_000;

/* service() polls allowed per engine phase before the job is declared
 * wedged. A full-screen fill is ~8.5 ms of DMA; a tight pump loop polls
 * every ~0.2 us, so the legitimate worst case is ~45k polls — 300k is a
 * comfortable bound that still fails fast on a dead SPI/DMA. */
const PHASE_BUDGET: u32 = 300_000;

/* engine phases (also the MK_DSP_STATE low byte) */
const PH_IDLE: u8 = 0;
const PH_FILL: u8 = 1; /* fill DMA running */
const PH_GLYPH: u8 = 2; /* glyph row DMA running (row in state byte 1) */
const PH_DRAIN: u8 = 3; /* data done; waiting shifter idle, restore 8-bit */

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
static INITED: Racy<bool> = Racy(core::cell::UnsafeCell::new(false));
/* screen-off state + the backlight compare to restore on wake */
static SCREEN_OFF: Racy<bool> = Racy(core::cell::UnsafeCell::new(false));
static LAST_BL: Racy<u16> = Racy(core::cell::UnsafeCell::new(499));

struct Engine {
    phase: u8,
    row: u8, /* next glyph row to stage */
    budget: u32,
    glyph_off: u16,
    fg: u16,
    bg: u16,
}
static ENGINE: Racy<Engine> = Racy(core::cell::UnsafeCell::new(Engine {
    phase: PH_IDLE,
    row: 0,
    budget: 0,
    glyph_off: 0,
    fg: 0,
    bg: 0,
}));

/* DMA staging: one glyph row of RGB565 (refilled between row transfers),
 * and the single color word a fill streams with memory-increment off. */
const ROW_PX: usize = font::ADV as usize;
static ROWBUF: Racy<[u16; ROW_PX]> = Racy(core::cell::UnsafeCell::new([0; ROW_PX]));
static FILL_WORD: Racy<u16> = Racy(core::cell::UnsafeCell::new(0));

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
    rcu.ahben().modify(|_, w| w.dmaen().enabled());
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
    /* raw variant: fill_rect's ensure_init guard would recurse into this
     * function, which the static stack-gate (correctly) refuses to bound */
    fill_rect_raw(0, 0, WIDTH - 1, HEIGHT - 1, 0x0000)?; /* clear to black */
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

/* ---------------- DMA pixel engine ---------------- */

/// Switch the SPI frame format. FF16 may only change while SPIEN is clear
/// (and the caller must have seen TRANS idle first, or bits get chopped).
fn spi_frame16(on: bool) {
    let spi = unsafe { &*pac::Spi0::ptr() };
    spi.ctl0().modify(|_, w| w.spien().disabled());
    spi.ctl0().modify(|_, w| {
        if on {
            w.ff16().sixteen_bit()
        } else {
            w.ff16().eight_bit()
        }
    });
    spi.ctl0().modify(|_, w| w.spien().enabled());
}

/// Program + enable DMA channel 2 (the fixed SPI0_TX request): `cnt`
/// halfword transfers from `maddr` (incrementing if `minc`) into SPI0 DATA.
#[inline(never)]
fn dma_kick(maddr: u32, cnt: u16, minc: bool) {
    let dma = unsafe { &*pac::Dma::ptr() };
    let spi = unsafe { &*pac::Spi0::ptr() };
    dma.ch2ctl().modify(|_, w| w.chen().disabled());
    dma.intc().write(|w| {
        w.gifc2()
            .clear()
            .ftfifc2()
            .clear()
            .htfifc2()
            .clear()
            .errifc2()
            .clear()
    });
    dma.ch2paddr()
        .write(|w| unsafe { w.paddr().bits(spi.data().as_ptr() as u32) });
    dma.ch2maddr().write(|w| unsafe { w.maddr().bits(maddr) });
    dma.ch2cnt().write(|w| w.cnt().bits(cnt));
    dma.ch2ctl().write(|w| {
        let w = w
            .dir()
            .from_memory()
            .pnaga()
            .fixed()
            .pwidth()
            .bits16()
            .mwidth()
            .bits16()
            .prio()
            .high();
        if minc {
            w.mnaga().increment()
        } else {
            w.mnaga().fixed()
        }
    });
    dma.ch2ctl().modify(|_, w| w.chen().enabled());
    mk(MK_DSP_KICKS, mk_get(MK_DSP_KICKS) + 1);
}

/// Open a pixel session: window commands (polled, 8-bit), DC high, SPI to
/// 16-bit frames, TX DMA request on. On error nothing is left half-switched.
fn pixel_session_begin(x0: u16, y0: u16, x1: u16, y1: u16) -> Result<(), u8> {
    set_window(x0, y0, x1, y1)?;
    spi_wait_idle()?;
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    gpioa.bop().write(|w| w.bop4().set_bit()); /* DC high = data */
    spi_frame16(true);
    let spi = unsafe { &*pac::Spi0::ptr() };
    spi.ctl1().modify(|_, w| w.dmaten().enabled());
    Ok(())
}

/// Close the pixel session: DMA request off, channel off, SPI back to 8-bit.
/// Caller must have seen TRANS idle (drain phase) on the success path.
fn pixel_session_end() {
    let dma = unsafe { &*pac::Dma::ptr() };
    let spi = unsafe { &*pac::Spi0::ptr() };
    dma.ch2ctl().modify(|_, w| w.chen().disabled());
    spi.ctl1().modify(|_, w| w.dmaten().disabled());
    spi_frame16(false);
}

/// Tear the job down after a failure and record the diagnostics snapshot:
/// stage | SPI_STAT<<8 | DMA ch2 INTF nibble<<24 (MK_DSP_ERR).
#[inline(never)]
fn abort(stage: u8) -> u8 {
    let dma = unsafe { &*pac::Dma::ptr() };
    let spi = unsafe { &*pac::Spi0::ptr() };
    mk(
        MK_DSP_ERR,
        stage as u32
            | (spi.stat().read().bits() as u32 & 0xFFFF) << 8
            | ((dma.intf().read().bits() >> 8) & 0xF) << 24,
    );
    pixel_session_end();
    let e = unsafe { &mut *ENGINE.0.get() };
    e.phase = PH_IDLE;
    mk(MK_DSP_STATE, PH_IDLE as u32);
    stage
}

/// Expand one 1-bpp glyph row into ROWBUF as RGB565. The 16-bit SPI frame
/// shifts MSB-first, so native u16 values go out hi-byte-first as the panel
/// expects — no byte swizzling.
fn stage_row(row: usize) {
    let e = unsafe { &*ENGINE.0.get() };
    let buf = unsafe { &mut *ROWBUF.0.get() };
    let bpr = (font::ADV as usize + 7) / 8;
    let base = e.glyph_off as usize + row * bpr;
    for (c, px) in buf.iter_mut().enumerate() {
        let ink = font::DATA[base + c / 8] & (0x80 >> (c % 8)) != 0;
        *px = if ink { e.fg } else { e.bg };
    }
}

/// Advance the pixel engine one poll. Ok(true) = idle, Ok(false) = a job is
/// still running, Err(stage) = the job failed and was torn down (the engine
/// is idle again; the caller decides what to do about the display).
/// Cheap when idle — the main loop calls this every pass.
#[inline(never)]
pub fn service() -> Result<bool, u8> {
    let e = unsafe { &mut *ENGINE.0.get() };
    match e.phase {
        PH_FILL | PH_GLYPH => {
            let dma = unsafe { &*pac::Dma::ptr() };
            let f = dma.intf().read();
            if f.errif2().is_error() {
                return Err(abort(4));
            }
            if f.ftfif2().is_complete() {
                if e.phase == PH_GLYPH && (e.row as u16) < font::CELL_H {
                    /* FTF = the last halfword reached the SPI, so ROWBUF is
                     * free to restage; the shifter drains while we refill */
                    stage_row(e.row as usize);
                    e.row += 1;
                    e.budget = PHASE_BUDGET;
                    mk(MK_DSP_STATE, PH_GLYPH as u32 | (e.row as u32) << 8);
                    dma_kick(ROWBUF.0.get() as u32, ROW_PX as u16, true);
                } else {
                    e.phase = PH_DRAIN;
                    e.budget = PHASE_BUDGET;
                    mk(MK_DSP_STATE, PH_DRAIN as u32);
                }
                Ok(false)
            } else {
                e.budget -= 1;
                if e.budget == 0 {
                    return Err(abort(3));
                }
                Ok(false)
            }
        }
        PH_DRAIN => {
            let spi = unsafe { &*pac::Spi0::ptr() };
            if spi.stat().read().trans().is_idle() {
                pixel_session_end();
                e.phase = PH_IDLE;
                mk(MK_DSP_STATE, PH_IDLE as u32);
                mk(MK_DSP_DONE, mk_get(MK_DSP_DONE) + 1);
                Ok(true)
            } else {
                e.budget -= 1;
                if e.budget == 0 {
                    return Err(abort(3));
                }
                Ok(false)
            }
        }
        _ => Ok(true), /* PH_IDLE */
    }
}

/// Pump service() until the engine goes idle. Terminates because every
/// phase carries a poll budget. Blocking — dev verbs / boot only.
#[inline(never)]
fn pump_idle() -> Result<(), u8> {
    loop {
        if service()? {
            return Ok(());
        }
    }
}

/// Kick an async solid fill. Engine must be idle (callers pump first).
fn start_fill(x0: u16, y0: u16, x1: u16, y1: u16, color: u16) -> Result<(), u8> {
    let e = unsafe { &mut *ENGINE.0.get() };
    if e.phase != PH_IDLE {
        return Err(5);
    }
    pixel_session_begin(x0, y0, x1, y1)?;
    unsafe { *FILL_WORD.0.get() = color };
    let n = (x1 - x0 + 1) as u32 * (y1 - y0 + 1) as u32; /* max 12800 < 2^16 */
    e.phase = PH_FILL;
    e.budget = PHASE_BUDGET;
    mk(MK_DSP_STATE, PH_FILL as u32);
    mk(MK_DSP_FILLS, mk_get(MK_DSP_FILLS) + 1);
    dma_kick(FILL_WORD.0.get() as u32, n as u16, false);
    Ok(())
}

/// Kick an async glyph blit (font.rs, 1 bpp expanded row-by-row) at (x, y).
/// The full cell is overwritten, so no clear, no flicker. Engine must be
/// idle; unknown characters draw nothing (engine stays idle, Ok).
#[inline(never)]
pub fn start_char(x: u16, y: u16, ch: u8, fg: u16, bg: u16) -> Result<(), u8> {
    ensure_init()?;
    let e = unsafe { &mut *ENGINE.0.get() };
    if e.phase != PH_IDLE {
        return Err(5);
    }
    let Some(&(_, off)) = font::GLYPHS.iter().find(|&&(c, _)| c == ch) else {
        return Ok(());
    };
    pixel_session_begin(x, y, x + font::ADV - 1, y + font::CELL_H - 1)?;
    e.glyph_off = off;
    e.fg = fg;
    e.bg = bg;
    stage_row(0);
    e.row = 1;
    e.phase = PH_GLYPH;
    e.budget = PHASE_BUDGET;
    mk(MK_DSP_STATE, PH_GLYPH as u32 | 1 << 8);
    mk(MK_DSP_GLYPHS, mk_get(MK_DSP_GLYPHS) + 1);
    dma_kick(ROWBUF.0.get() as u32, ROW_PX as u16, true);
    Ok(())
}

/// Blocking solid fill (dev verbs / test pattern / init clear): waits out
/// any running job, kicks the fill, pumps it to completion. Bounded.
#[inline(never)]
pub fn fill_rect(x0: u16, y0: u16, x1: u16, y1: u16, color: u16) -> Result<(), u8> {
    ensure_init()?;
    fill_rect_raw(x0, y0, x1, y1, color)
}

#[inline(never)]
fn fill_rect_raw(x0: u16, y0: u16, x1: u16, y1: u16, color: u16) -> Result<(), u8> {
    pump_idle()?;
    start_fill(x0, y0, x1, y1, color)?;
    pump_idle()
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

/// Set backlight PWM compare directly (0 = dark .. 999 = full). Also the
/// wake-restore value for the idle screen-off.
#[inline(never)]
pub fn backlight_set(compare: u16) -> Result<(), u8> {
    ensure_init()?;
    let tim = unsafe { &*pac::Timer1::ptr() };
    let c = if compare > 999 { 999 } else { compare };
    unsafe { *LAST_BL.0.get() = c };
    tim.ch1cv().write(|w| w.ch1val().bits(c as u32));
    Ok(())
}

/// Idle screen-off: DISPOFF (source outputs go to a fixed level — no static
/// image driven into the liquid crystal, the actual burn-in mechanism) plus
/// backlight to 0. GRAM writes still work while off, so ui::draw can keep
/// running; wake is a single DISPON + backlight restore. No-ops when already
/// in the requested state.
#[inline(never)]
pub fn screen_off() -> Result<(), u8> {
    ensure_init()?;
    if unsafe { *SCREEN_OFF.0.get() } {
        return Ok(());
    }
    /* a pixel session may be in flight (16-bit frames, DC high, DMA on) —
     * a command write would corrupt it; drain first (bounded) */
    pump_idle()?;
    write_command(0x28)?; /* DISPOFF */
    let tim = unsafe { &*pac::Timer1::ptr() };
    tim.ch1cv().write(|w| w.ch1val().bits(0)); /* raw: keep LAST_BL intact */
    unsafe { *SCREEN_OFF.0.get() = true };
    Ok(())
}

#[inline(never)]
pub fn screen_on() -> Result<(), u8> {
    ensure_init()?;
    if !unsafe { *SCREEN_OFF.0.get() } {
        return Ok(());
    }
    /* see screen_off: never send a command mid-pixel-session */
    pump_idle()?;
    write_command(0x29)?; /* DISPON */
    let tim = unsafe { &*pac::Timer1::ptr() };
    let bl = unsafe { *LAST_BL.0.get() };
    tim.ch1cv().write(|w| w.ch1val().bits(bl as u32));
    unsafe { *SCREEN_OFF.0.get() = false };
    Ok(())
}
