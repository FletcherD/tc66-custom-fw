//! M3 main screen: voltage, current, power on three colored lines.
//!
//! Fixed layout of 8 monospace font cells per line (font.rs, 18x24). Cells
//! are always fully overdrawn by tft::draw_char, and only cells whose
//! character changed since the last draw are touched, so updates are
//! flicker-free and cheap (idle screen redraws nothing).

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
/* last drawn characters; 0 never matches, so the first draw paints all */
static LAST: Racy<[[u8; COLS]; 3]> = Racy(core::cell::UnsafeCell::new([[0; COLS]; 3]));

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

/// Draw/refresh the screen from published values (units: 1e-4 V, 1e-5 A,
/// 1e-4 W — the getva integers).
#[inline(never)]
pub fn draw(v_1e4: u32, i_1e5: u32, p_1e4: u32) -> Result<(), u8> {
    let mut lines = [[b' '; COLS]; 3];
    fmt2_4(&mut lines[0], v_1e4, b'V');
    fmt1_5(&mut lines[1], i_1e5, b'A');
    fmt2_4(&mut lines[2], p_1e4, b'W');

    let last = unsafe { &mut *LAST.0.get() };
    for (row, line) in lines.iter().enumerate() {
        let y = Y0 + row as u16 * Y_STEP;
        for (col, &ch) in line.iter().enumerate() {
            if last[row][col] != ch {
                tft::draw_char(X0 + col as u16 * font::ADV, y, ch, COLORS[row], BG)?;
                last[row][col] = ch;
            }
        }
    }
    Ok(())
}
