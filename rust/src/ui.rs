//! Main screens: three 8-cell lines each, switched with K1 (main.rs).
//!
//!   0  V / I / P            (green / yellow / cyan)
//!   1  mAh / mWh / hh:mm:ss (energy since power-on or the last K2 reset)
//!   2  temperature / D+ / D−
//!   3  scrolling V/I plot   (plot.rs — full screen, no cells)
//!
//! Fixed layout of 8 monospace font cells per line (font.rs, 18x24). Cells
//! are always fully overdrawn by the tft glyph engine, and only cells whose
//! character changed since the last draw are touched, so updates are
//! flicker-free and cheap (idle screen redraws nothing). A screen switch
//! marks every cell dirty (same char, different color must still repaint).
//!
//! Drawing is incremental: `draw` only formats the target text and marks
//! changed cells dirty (cheap, never blocks); `service` — pumped every main
//! loop pass — draws at most one dirty cell per pass, finishing its glyph
//! DMA before returning (~4.5 ms; a full 24-cell screen ~125 ms).
//! `flush` (boot first frame, `tftui` verb) pumps to completion and is
//! bounded by the engine's per-phase poll budgets.

use crate::font;
use crate::ina;
use crate::plot;
use crate::tft;

const COLS: usize = 8;
const X0: u16 = 8; /* 8 + 8*18 = 152, centered-ish on 160 */
const Y0: u16 = 1;
const Y_STEP: u16 = font::CELL_H + 3;

const BG: u16 = 0x0000;
pub const SCREENS: u8 = 4;
pub const PLOT: u8 = 3;
/* per-screen line colors (RGB565) */
const COLORS: [[u16; 3]; SCREENS as usize] = [
    [0x07E0, 0xFFE0, 0x07FF], /* V green, I yellow, P cyan (V/I as on the plot) */
    [0xFD20, 0xF81F, 0xFFFF], /* mAh orange, mWh magenta, time white */
    [0xFBEA, 0xFFFF, 0xAFFF], /* temp salmon, D+ white, D- light blue */
    [0; 3],                   /* plot: no cells */
];

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
/* target characters; 0 never matches, so the first draw paints all */
static LAST: Racy<[[u8; COLS]; 3]> = Racy(core::cell::UnsafeCell::new([[0; COLS]; 3]));
/* bit (row*8+col) set = cell in LAST not yet drawn; cleared when its glyph
 * job is kicked (LAST holds the char that was kicked — a later draw() that
 * changes the cell re-marks it, so an in-flight glyph is never stale-final) */
static DIRTY: Racy<u32> = Racy(core::cell::UnsafeCell::new(0));
static SCREEN: Racy<u8> = Racy(core::cell::UnsafeCell::new(0));

pub fn screen() -> u8 {
    unsafe { *SCREEN.0.get() }
}

/// Select a screen (wraps). Poisons the char cache so the next `draw`
/// marks every cell dirty (same char in a new color must repaint too).
/// Entering or leaving the plot is deferred to `service` (tft commands
/// need an idle engine).
pub fn set_screen(n: u8) {
    let n = n % SCREENS;
    if n == PLOT {
        plot::enter();
    } else {
        plot::leave();
    }
    unsafe {
        *SCREEN.0.get() = n;
        *LAST.0.get() = [[0; COLS]; 3];
        *DIRTY.0.get() = 0;
    }
}

/// Rotation setting: flip the panel, then repaint every cell.
pub fn set_rotation(rot: u8) {
    let _ = tft::set_rotation(rot);
    set_screen(screen());
}

pub fn next_screen() {
    set_screen(screen() + 1);
}

/// right-aligned decimal digits of `v` into `buf`, zero-padded
fn digits(buf: &mut [u8], mut v: u32) {
    for b in buf.iter_mut().rev() {
        *b = b'0' + (v % 10) as u8;
        v /= 10;
    }
}

/// right-aligned decimal digits with leading zeros blanked (last kept)
fn digits_sp(buf: &mut [u8], v: u32) {
    digits(buf, v);
    let n = buf.len() - 1;
    for b in buf.iter_mut().take(n) {
        if *b != b'0' {
            break;
        }
        *b = b' ';
    }
}

/// "xx.ffffU" — two int digits (space-padded), 4-digit fraction, unit
fn fmt2_4(line: &mut [u8; COLS], val: u32, unit: u8) {
    let int = (val / 10_000).min(99);
    digits(&mut line[0..2], int);
    if line[0] == b'0' {
        line[0] = b' ';
    }
    line[2] = b'.';
    digits(&mut line[3..7], val % 10_000);
    line[7] = unit;
}

/// "x.fffffU" — one int digit, 5-digit fraction, unit
fn fmt1_5(line: &mut [u8; COLS], val: u32, unit: u8) {
    digits(&mut line[0..1], (val / 100_000).min(9));
    line[1] = b'.';
    digits(&mut line[2..7], val % 100_000);
    line[7] = unit;
}

/// Format the current screen from the published measurements (own
/// snapshot — the caller's frame stays small) and `run_secs` (elapsed time
/// since power-on / energy reset), then mark changed cells dirty. Never
/// touches the hardware; `service`/`flush` do the drawing.
#[inline(never)]
pub fn draw(run_secs: u32) {
    if screen() == PLOT {
        return;
    }
    let m = ina::snapshot();
    let mut lines = [[b' '; COLS]; 3];
    match screen() {
        0 => {
            let (p, _) = ina::derive(m.v_1e4, m.i_1e5);
            fmt2_4(&mut lines[0], m.v_1e4, b'V');
            fmt1_5(&mut lines[1], m.i_1e5, b'A');
            fmt2_4(&mut lines[2], p, b'W');
        }
        1 => {
            digits_sp(&mut lines[0][0..5], m.mah.min(99_999));
            lines[0][5..8].copy_from_slice(b"mAh");
            digits_sp(&mut lines[1][0..5], m.mwh.min(99_999));
            lines[1][5..8].copy_from_slice(b"mWh");
            let t = &mut lines[2];
            digits(&mut t[0..2], (run_secs / 3600).min(99));
            t[2] = b':';
            digits(&mut t[3..5], run_secs / 60 % 60);
            t[5] = b':';
            digits(&mut t[6..8], run_secs % 60);
        }
        _ => {
            let t = &mut lines[0];
            t[0] = b'T';
            t[4] = if m.temp_c < 0 { b'-' } else { b' ' };
            digits_sp(&mut t[5..7], m.temp_c.unsigned_abs().min(99));
            t[7] = b'C';
            for (row, (sign, d)) in [(b'+', m.dp_1e2), (b'-', m.dm_1e2)].iter().enumerate() {
                let l = &mut lines[1 + row];
                l[0] = b'D';
                l[1] = *sign;
                digits(&mut l[3..4], (d / 100).min(9));
                l[4] = b'.';
                digits(&mut l[5..7], d % 100);
                l[7] = b'V';
            }
        }
    }

    let last = unsafe { &mut *LAST.0.get() };
    let dirty = unsafe { &mut *DIRTY.0.get() };
    for (row, line) in lines.iter().enumerate() {
        for (col, &ch) in line.iter().enumerate() {
            if last[row][col] != ch {
                last[row][col] = ch;
                *dirty |= 1 << (row * COLS + col);
            }
        }
    }
}

/// One pump pass: advance the tft engine; when it is idle, draw the next
/// dirty cell to completion (at most one glyph per call, ~4.5 ms).
/// Ok(true) = engine idle and nothing left to draw. Err = a job failed and
/// was torn down (caller drops the always-on refresh, as before).
#[inline(never)]
pub fn service() -> Result<bool, u8> {
    if !tft::service()? || !plot::service()? {
        return Ok(false);
    }
    if screen() == PLOT {
        return Ok(true);
    }
    let dirty = unsafe { &mut *DIRTY.0.get() };
    if *dirty == 0 {
        return Ok(true);
    }
    let n = dirty.trailing_zeros() as usize;
    let (row, col) = (n / COLS, n % COLS);
    let ch = unsafe { (*LAST.0.get())[row][col] };
    tft::start_char(
        X0 + col as u16 * font::ADV,
        Y0 + row as u16 * Y_STEP,
        ch,
        COLORS[screen() as usize][row],
        BG,
    )?;
    *dirty &= !(1 << n);
    tft::pump_idle()?;
    Ok(false)
}

/// Pump until the screen is fully drawn. Blocking — boot first frame and the
/// `tftui` dev verb only. Terminates: every engine phase carries a poll
/// budget, and the dirty set only shrinks here.
#[inline(never)]
pub fn flush() -> Result<(), u8> {
    loop {
        if service()? {
            return Ok(());
        }
    }
}
