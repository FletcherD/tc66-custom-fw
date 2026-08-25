//! M3 main screen: voltage, current, power on three colored lines.
//!
//! Fixed layout of 8 monospace font cells per line (font.rs, 18x24). Cells
//! are always fully overdrawn by the tft glyph engine, and only cells whose
//! character changed since the last draw are touched, so updates are
//! flicker-free and cheap (idle screen redraws nothing).
//!
//! Drawing is asynchronous: `draw` only formats the target text and marks
//! changed cells dirty (cheap, never blocks); `service` — pumped every main
//! loop pass — advances the tft DMA engine and kicks the next dirty cell's
//! glyph when it goes idle. `flush` (boot first frame, `tftui` verb) pumps
//! to completion and is bounded by the engine's per-phase poll budgets.

use crate::font;
use crate::tft;

const COLS: usize = 8;
const X0: u16 = 8; /* 8 + 8*18 = 152, centered-ish on 160 */
const Y0: u16 = 1;
const Y_STEP: u16 = font::CELL_H + 3;

const BG: u16 = 0x0000;
/* line colors: V yellow, I cyan, P green (RGB565) */
const COLORS: [u16; 3] = [0xFFE0, 0x07FF, 0x07E0];

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
/* target characters; 0 never matches, so the first draw paints all */
static LAST: Racy<[[u8; COLS]; 3]> = Racy(core::cell::UnsafeCell::new([[0; COLS]; 3]));
/* bit (row*8+col) set = cell in LAST not yet drawn; cleared when its glyph
 * job is kicked (LAST holds the char that was kicked — a later draw() that
 * changes the cell re-marks it, so an in-flight glyph is never stale-final) */
static DIRTY: Racy<u32> = Racy(core::cell::UnsafeCell::new(0));

/// right-aligned decimal digits of `v` into `buf`, zero-padded
fn digits(buf: &mut [u8], mut v: u32) {
    for b in buf.iter_mut().rev() {
        *b = b'0' + (v % 10) as u8;
        v /= 10;
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

/// Format the screen from published values (units: 1e-4 V, 1e-5 A, 1e-4 W —
/// the getva integers) and mark changed cells dirty. Never touches the
/// hardware; `service`/`flush` do the drawing.
#[inline(never)]
pub fn draw(v_1e4: u32, i_1e5: u32, p_1e4: u32) {
    let mut lines = [[b' '; COLS]; 3];
    fmt2_4(&mut lines[0], v_1e4, b'V');
    fmt1_5(&mut lines[1], i_1e5, b'A');
    fmt2_4(&mut lines[2], p_1e4, b'W');

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

/// One pump pass: advance the tft engine; when it is idle, kick the next
/// dirty cell (at most one kick per call — the main loop stays responsive).
/// Ok(true) = engine idle and nothing left to draw. Err = a job failed and
/// was torn down (caller drops the always-on refresh, as before).
#[inline(never)]
pub fn service() -> Result<bool, u8> {
    if !tft::service()? {
        return Ok(false);
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
        COLORS[row],
        BG,
    )?;
    *dirty &= !(1 << n);
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
