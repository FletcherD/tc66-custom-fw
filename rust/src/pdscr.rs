//! Screens 4 and 5: USB-PD info and charger detection (`triggers` feature;
//! ui.rs dispatches to it).
//!
//! The text lines of pdinfo.rs (PD) or detect.rs (detection) in the small font (font_small.rs, 7x13): a grid
//! of 22 columns x 6 rows, one page of 6 lines at a time, with a 3-px
//! scroll bar at the right edge (thumb = the shown page's share of the
//! whole). `next_page` (K2 short press, `pdinf 1`) advances and wraps.
//! One grid serves both screens: `enter(det)` picks the line source.
//!
//! Same incremental scheme as the big-font screens, per cell: `draw`
//! (2 Hz + on events) formats the shown page into `want` and flags work;
//! `service` (every main-loop pass) draws at most one changed glyph or
//! scroll-bar segment per call, so an unchanged screen costs nothing. A
//! row whose color changes is repainted whole. Entering and leaving clear
//! the panel (the big-font cells and this grid don't overlap cleanly).

use crate::detect;
use crate::font_small as font;
use crate::pdinfo;
use crate::text::COLS;
use crate::tft;

const ROWS: usize = 6;
const X0: u16 = 1;
const Y0: u16 = 1;
const BAR_X0: u16 = tft::WIDTH - 3;
const BAR_X1: u16 = tft::WIDTH - 1;
const H: u16 = tft::HEIGHT;
const C_TRACK: u16 = 0x2104;
const C_THUMB: u16 = 0xAD55;
const BG: u16 = 0x0000;
const _: () = assert!(X0 + COLS as u16 * font::ADV < BAR_X0);
const _: () = assert!(Y0 + ROWS as u16 * font::CELL_H <= H);

struct Scr {
    want: [[u8; COLS]; ROWS],
    shown: [[u8; COLS]; ROWS], /* 0 never matches: forces a repaint */
    want_c: [u16; ROWS],
    shown_c: [u16; ROWS],
    page: u8,
    bar_page: u8, /* (page, pages) the scroll bar shows; pages 0 = none */
    bar_pages: u8,
    bar_seg: u8, /* next scroll-bar segment to draw (0 = bar up to date) */
    active: bool,
    det: bool, /* line source: detect.rs, else pdinfo.rs */
    clear: bool,
    work: bool, /* something may differ from the panel */
}

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
/* .hibss (zeroed by main() before anything runs; all-zero = inactive) */
#[link_section = ".hibss"]
static SCR: Racy<Scr> = Racy(core::cell::UnsafeCell::new(Scr {
    want: [[0; COLS]; ROWS],
    shown: [[0; COLS]; ROWS],
    want_c: [0; ROWS],
    shown_c: [0; ROWS],
    page: 0,
    bar_page: 0,
    bar_pages: 0,
    bar_seg: 0,
    active: false,
    det: false,
    clear: false,
    work: false,
}));

#[inline(always)]
fn st() -> &'static mut Scr {
    unsafe { &mut *SCR.0.get() }
}

/// Screen selected (also re-selected after a rotation change): clear, back
/// to page 1, repaint everything on the next draw. `det` = the detection
/// screen.
pub fn enter(det: bool) {
    let s = st();
    s.active = true;
    s.det = det;
    s.clear = true;
    s.work = true;
    s.page = 0;
    s.shown = [[0; COLS]; ROWS];
    s.bar_pages = 0;
}

/// Another screen selected: clear the panel (no-op if not shown).
pub fn leave() {
    let s = st();
    if s.active {
        s.active = false;
        s.clear = true;
        s.work = true;
    }
}

fn lines(det: bool) -> usize {
    if det {
        detect::lines()
    } else {
        pdinfo::lines()
    }
}

fn line(det: bool, i: usize, out: &mut [u8; COLS]) -> u16 {
    if det {
        detect::line(i, out)
    } else {
        pdinfo::line(i, out)
    }
}

fn pages(det: bool) -> u8 {
    lines(det).div_ceil(ROWS).clamp(1, 255) as u8
}

/// A detection run starts: its screen (if shown) back to page 1.
pub fn top() {
    let s = st();
    if s.det {
        s.page = 0;
    }
}

/// Next page (wraps). The caller redraws.
pub fn next_page() {
    let s = st();
    s.page = if s.page as u32 + 1 >= pages(s.det) as u32 { 0 } else { s.page + 1 };
}

/// (page, pages), 0-based page.
pub fn page() -> (u8, u8) {
    let s = st();
    (s.page, pages(s.det))
}

/// Format the shown page from the line source. Never touches the hardware.
#[inline(never)]
pub fn draw() {
    let s = st();
    if !s.active {
        return;
    }
    let pages = pages(s.det);
    if s.page >= pages {
        s.page = 0;
    }
    for (r, (want, want_c)) in s.want.iter_mut().zip(s.want_c.iter_mut()).enumerate() {
        let mut l = [b' '; COLS];
        let c = line(s.det, s.page as usize * ROWS + r, &mut l);
        if l != *want || c != *want_c {
            *want = l;
            *want_c = c;
            s.work = true;
        }
    }
    if (s.page, pages) != (s.bar_page, s.bar_pages) {
        s.bar_page = s.page;
        s.bar_pages = pages;
        s.bar_seg = 1;
        s.work = true;
    }
}

/// Draw step; call only while the tft engine is idle. Ok(true) = nothing
/// to do; Ok(false) = a job was kicked (glyphs are pumped to completion
/// here, as in ui::service). The idle check is inlined into the fast-flash
/// caller (ui::service).
#[inline(always)]
pub fn service() -> Result<bool, u8> {
    if !st().work {
        return Ok(true);
    }
    service_work()
}

#[inline(never)]
#[link_section = ".fasttext.pdscr_service_work"]
fn service_work() -> Result<bool, u8> {
    let s = st();
    if s.clear {
        s.clear = false;
        tft::start_clear()?;
        return Ok(false);
    }
    if !s.active {
        s.work = false;
        return Ok(true);
    }
    if s.bar_seg != 0 {
        /* track above the thumb, the thumb, track below — empty ones skipped */
        let pages = s.bar_pages.max(1) as u16;
        let y0 = s.bar_page as u16 * H / pages;
        let y1 = (s.bar_page as u16 + 1) * H / pages; /* exclusive */
        let seg = s.bar_seg;
        s.bar_seg = if seg >= 3 { 0 } else { seg + 1 };
        let (a, b, c) = match seg {
            1 => (0, y0, C_TRACK),
            2 => (y0, y1, C_THUMB),
            _ => (y1, H, C_TRACK),
        };
        if a < b {
            tft::start_rect(BAR_X0, a, BAR_X1, b - 1, c)?;
            return Ok(false);
        }
        return Ok(false);
    }
    for r in 0..ROWS {
        let (Some(want), Some(shown), Some(&wc), Some(sc)) =
            (s.want.get(r), s.shown.get_mut(r), s.want_c.get(r), s.shown_c.get_mut(r))
        else {
            break;
        };
        if *sc != wc {
            *sc = wc;
            *shown = [0; COLS];
        }
        for (col, (&w, sh)) in want.iter().zip(shown.iter_mut()).enumerate() {
            if w != *sh {
                *sh = w;
                tft::start_char(
                    &font::FONT,
                    X0 + col as u16 * font::ADV,
                    Y0 + r as u16 * font::CELL_H,
                    w,
                    wc,
                    BG,
                )?;
                tft::pump_idle()?;
                return Ok(false);
            }
        }
    }
    s.work = false;
    Ok(true)
}
