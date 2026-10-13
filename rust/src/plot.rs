//! Screen 3: full-screen scrolling V/I plot (ui.rs dispatches to it).
//!
//! No sample buffer and no redraw: the panel's GRAM is the history. Each
//! column period one new 1×80 column is written at the oldest GRAM line and
//! the ST7735S rolling scroll (tft.rs `scroll_*`) moves it to the right
//! edge, so the trace scrolls right-to-left at ~0.1 ms of SPI per column.
//! The history lives only while this screen is shown: entering clears the
//! panel and starts empty, leaving exits scroll mode and clears again.
//!
//! A column is drawn as a few vertical fill runs (tft engine, no staging
//! RAM): V and I each as a segment from the previous column's value to the
//! current one (connected trace; V on top), dim gridlines at 1/4, 1/2, 3/4
//! of full scale and every GRID_X columns, black elsewhere. One run kicks
//! per service() call; the scroll command follows once the engine is idle.

use crate::ina;
use crate::tft;

/* Column period and the traces' full scale (top edge) are settings
 * (cfg.rs: plot_period_ms, default 500 ms -> 160 columns = 80 s;
 * plot_v_full_mv / plot_i_full_ma, default 20 V / 4 A — gridlines at 1/4
 * steps). */
const GRID_X: u32 = 20; /* a vertical gridline every 20 columns */

/// Column period in 4 ms sampler ticks (≥ 5: the setting's floor is 20 ms).
fn period_ticks() -> u32 {
    (crate::cfg::get(crate::cfg::ID_PLOT_PERIOD) / 4).max(1)
}
/// Full scale in wire units (1e-4 V, 1e-5 A).
fn v_full() -> u32 {
    crate::cfg::get(crate::cfg::ID_PLOT_V_FULL).max(1) * 10
}
fn i_full() -> u32 {
    crate::cfg::get(crate::cfg::ID_PLOT_I_FULL).max(1) * 100
}

const W: u16 = tft::WIDTH;
const H: u16 = tft::HEIGHT;
const GRID_Y: u16 = H / 4;

const C_V: u16 = 0x07E0; /* green, as on screen 0 */
const C_I: u16 = 0xFFE0; /* yellow, as on screen 0 */
const C_GRID: u16 = 0x2104;
const C_BG: u16 = 0x0000;

const ST_OFF: u8 = 0;
const ST_ENTER: u8 = 1;
const ST_RUN: u8 = 2;
const ST_EXIT: u8 = 3;

struct Plot {
    stage: u8,
    /* screen x the next column is written at (the oldest one) */
    x: u16,
    /* columns drawn since entering (vertical grid phase) */
    n: u32,
    last_tick: u32,
    /* a column is being drawn: next run starts at y (H = runs done,
     * scroll pending) */
    pend: bool,
    y: u16,
    v: (u16, u16),
    i: (u16, u16),
    /* the previous column's trace rows (None right after entering) */
    prev: Option<(u16, u16)>,
}

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
static PLOT: Racy<Plot> = Racy(core::cell::UnsafeCell::new(Plot {
    stage: ST_OFF,
    x: 0,
    n: 0,
    last_tick: 0,
    pend: false,
    y: 0,
    v: (0, 0),
    i: (0, 0),
    prev: None,
}));

#[inline(always)]
fn st() -> &'static mut Plot {
    unsafe { &mut *PLOT.0.get() }
}

/// Screen selected (also re-selected, e.g. after a rotation change):
/// restart with an empty plot.
pub fn enter() {
    st().stage = ST_ENTER;
}

/// A scale setting changed: restart the plot if it is showing (the old
/// history is at the old scale).
pub fn rescale() {
    if st().stage != ST_OFF {
        enter();
    }
}

/// Another screen selected: exit scroll mode + clear (no-op if not shown).
pub fn leave() {
    let p = st();
    if p.stage != ST_OFF {
        p.stage = ST_EXIT;
    }
}

/// Trace row of `val` at full scale `full` (0 = bottom edge, clamped top).
fn row(val: u32, full: u32) -> u16 {
    let s = (val.min(full) * (H as u32 - 1)).checked_div(full).unwrap_or(0);
    H - 1 - s as u16
}

fn span(cur: u16, prev: u16) -> (u16, u16) {
    (cur.min(prev), cur.max(prev))
}

fn color(p: &Plot, y: u16) -> u16 {
    if y >= p.v.0 && y <= p.v.1 {
        C_V
    } else if y >= p.i.0 && y <= p.i.1 {
        C_I
    } else if p.n % GRID_X == 0 || (y % GRID_Y == 0 && y != 0) {
        C_GRID
    } else {
        C_BG
    }
}

/// Advance the plot one step; call only while the tft engine is idle.
/// Ok(true) = nothing to do (screen not shown, or waiting for the next
/// column); Ok(false) = a job was kicked or a command sent. The off check
/// is inlined into the fast-flash caller (ui::service) — a cold slow-flash
/// call costs ~90 µs.
#[inline(always)]
pub fn service() -> Result<bool, u8> {
    if st().stage == ST_OFF {
        return Ok(true);
    }
    service_shown()
}

#[inline(never)]
fn service_shown() -> Result<bool, u8> {
    let p = st();
    match p.stage {
        ST_EXIT => {
            tft::scroll_off()?;
            tft::start_clear()?;
            p.stage = ST_OFF;
            Ok(false)
        }
        ST_ENTER => {
            tft::scroll_on()?;
            tft::start_clear()?;
            p.x = 0;
            p.n = 0;
            p.pend = false;
            p.prev = None;
            p.last_tick = ina::ticks().wrapping_sub(period_ticks());
            p.stage = ST_RUN;
            Ok(false)
        }
        ST_RUN => {
            if !p.pend {
                let now = ina::ticks();
                if now.wrapping_sub(p.last_tick) < period_ticks() {
                    return Ok(true);
                }
                p.last_tick = now;
                let (v, i) = ina::snapshot_vi();
                let (vy, iy) = (row(v, v_full()), row(i, i_full()));
                let (pv, pi) = p.prev.unwrap_or((vy, iy));
                p.v = span(vy, pv);
                p.i = span(iy, pi);
                p.prev = Some((vy, iy));
                p.y = 0;
                p.pend = true;
            }
            if p.y < H {
                let c = color(p, p.y);
                let mut e = p.y;
                while e + 1 < H && color(p, e + 1) == c {
                    e += 1;
                }
                tft::start_vline(p.x, p.y, e, c)?;
                p.y = e + 1;
            } else {
                tft::scroll_newest(p.x)?;
                p.x = (p.x + 1) % W;
                p.n = p.n.wrapping_add(1);
                p.pend = false;
            }
            Ok(false)
        }
        _ => Ok(true),
    }
}
