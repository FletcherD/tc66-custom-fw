//! Screen 5: on-device charger protocol detection (`triggers` feature) — the
//! equivalent of tools/detect.py, never raising VBUS above 5 V.
//!
//! A run is a state machine stepped from the main loop (`service`, one step
//! per pass when its tick deadline is due), so the waits between steps cost
//! nothing and the longest blocking chunk is the AFC exchange (~0.1-0.25 s):
//!
//!   start     refuse if a PD sink session is armed, the sniffer sees a
//!             sink on the other port, or the load exceeds LOAD_MAX_MA;
//!             PD engine off, D+/D− released, settle
//!   passive   VBUS, load, D+/D− (Apple/Samsung divider signatures; the
//!             load check again); begin the QC handshake (D+ 0.6 V, D−
//!             released)
//!   qc        D+/D− at 0.1 s, D− at 1.5 s -> class (qc::hvdcp_class);
//!             release, settle
//!   afc       the AFC entry in its two timed phases, then `afc(0x08)` (a
//!             5 V request — a probe); release, settle
//!   fcp       (`fcp` feature only) the same entry, then reads of regs
//!             0x21/0x22; release, settle
//!   pd        arm the sink (it takes PDO 1 = 5 V), wait ≤ PD_WAIT for the
//!             contract, record Rp (Status0 BC_LVL), the spec rev and the
//!             source PDOs; PD engine off
//!
//! End state: PD engine off, D+/D− released (a sniff that ran before is not
//! restored — re-entering the PD screen starts it again). The D+/D− steps
//! run before PD: a PD contract can disable a charger's legacy modes.
//!
//! Text: `lines`/`line` (pdinfo.rs's writer, shown by pdscr.rs). K2 short
//! press (`k2`): pages while a later page exists, on the last page (or
//! with no results) starts a run; ignored while running.
//!
//! Thread-exclusive: main loop (service, K2) and verb dispatch only.

use crate::ina;
use crate::pd;
use crate::pdinfo::{self, W, COLS};
use crate::qc;

/// Load above which a run is refused: a device on the other port would take
/// part in the handshakes. The meter's own supply shows ~10-13 mA when it
/// is powered from the measured rail (charger on OUT, bench).
const LOAD_MAX_MA: u32 = 30;
const T_SETTLE: u32 = 375; /* 1.5 s in 4-ms ticks: charger falls back */
const T_QC_EARLY: u32 = 25; /* 0.1 s */
const T_QC_LATE: u32 = 350; /* +1.4 s = 1.5 s */
const T_ACCP_REL: u32 = 50; /* 200 ms all released */
const T_ACCP_HOLD: u32 = 513; /* 2050 ms D+ 0.6 V */
const T_PD_WAIT: u32 = 1000; /* 4 s for caps + 5 V contract */

const S_IDLE: u8 = 0;
const S_START: u8 = 1;
const S_PASSIVE: u8 = 2;
const S_QC_EARLY: u8 = 3;
const S_QC_LATE: u8 = 4;
const S_AFC_REL: u8 = 5;
const S_AFC_HOLD: u8 = 6;
const S_AFC: u8 = 7;
#[cfg(feature = "fcp")]
const S_FCP_REL: u8 = 8;
#[cfg(feature = "fcp")]
const S_FCP_HOLD: u8 = 9;
#[cfg(feature = "fcp")]
const S_FCP: u8 = 10;
const S_PD_ARM: u8 = 11;
const S_PD_WAIT: u8 = 12;

/* `done` bits: which results are valid */
const D_PASSIVE: u8 = 1;
const D_QC: u8 = 2;
const D_AFC: u8 = 4;
const D_FCP: u8 = 8;
const D_PD: u8 = 16;

/* refusal / `detct` ER codes */
pub const E_BUSY: u8 = 0x01;
const E_SINK_ARMED: u8 = 0x02;
const E_SINK_SEEN: u8 = 0x03;
const E_LOAD: u8 = 0x04;

struct Det {
    step: u8,
    due: u32,
    done: u8,
    refused: u8,
    ran: bool,
    afc: bool,
    v_mv: u16,
    ma: u16,
    dp_mv: u16,
    dm_mv: u16,
    qc: [u16; 4], /* class, D+, D− early, D− late (mV) */
    fcp: [u8; 2], /* N raised voltages, max power (0.5 W); fcp_err 0 = valid */
    fcp_err: u8,
    pd_state: u8, /* engine state at the end of the wait (pd::status) */
    cc: u8,
    bc_lvl: u8,
    rev: u8,
    pd_err: u8, /* arm I²C error stage, 0 = none */
    caps_n: u8,
    caps: [u8; 28], /* source PDOs, LE (pd::caps) */
}

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
/* .hibss (zeroed by main() before anything runs; all-zero = never run) */
#[link_section = ".hibss"]
static DET: Racy<Det> = Racy(core::cell::UnsafeCell::new(Det {
    step: 0,
    due: 0,
    done: 0,
    refused: 0,
    ran: false,
    afc: false,
    v_mv: 0,
    ma: 0,
    dp_mv: 0,
    dm_mv: 0,
    qc: [0; 4],
    fcp: [0; 2],
    fcp_err: 0,
    pd_state: 0,
    cc: 0,
    bc_lvl: 0,
    rev: 0,
    pd_err: 0,
    caps_n: 0,
    caps: [0; 28],
}));

#[inline(always)]
fn st() -> &'static mut Det {
    unsafe { &mut *DET.0.get() }
}

pub fn busy() -> bool {
    st().step != S_IDLE
}

fn load_ma() -> u32 {
    ina::snapshot_vi().1 / 100 /* 1e-5 A -> mA */
}

/// Start a run (K2, `detct 1`). Err = refused (the reason is also shown).
#[inline(never)]
pub fn start() -> Result<(), u8> {
    let s = st();
    if s.step != S_IDLE {
        return Err(E_BUSY);
    }
    s.done = 0;
    s.ran = true;
    let l = pd::link();
    s.refused = if l.mode == pd::MODE_SINK {
        E_SINK_ARMED
    } else if l.mode == pd::MODE_SNIFF && l.snk {
        E_SINK_SEEN
    } else if load_ma() > LOAD_MAX_MA {
        E_LOAD
    } else {
        0
    };
    if s.refused != 0 {
        s.ma = load_ma() as u16;
        return Err(s.refused);
    }
    s.step = S_START;
    s.due = ina::ticks();
    crate::pdscr::top();
    Ok(())
}

fn next(s: &mut Det, step: u8, ticks: u32) {
    s.step = step;
    s.due = ina::ticks().wrapping_add(ticks);
}

/// Release D+/D− and wait for the charger to fall back, then `step`.
#[inline(never)]
fn settle(s: &mut Det, step: u8) {
    let _ = qc::off();
    next(s, step, T_SETTLE);
}

/// One main-loop pass: nothing unless a run's next step is due (the PD
/// wait polls the engine state every pass until its deadline).
#[inline(always)]
pub fn service() {
    let s = st();
    if s.step != S_IDLE && (s.step == S_PD_WAIT || ina::ticks().wrapping_sub(s.due) as i32 >= 0) {
        step(s);
    }
}

#[inline(never)]
fn step(s: &mut Det) {
    match s.step {
        S_START => {
            if !pd::is_off() {
                let _ = pd::off();
            }
            settle(s, S_PASSIVE);
        }
        S_PASSIVE => {
            let m = ina::snapshot();
            s.v_mv = (m.v_1e4 / 10).min(0xFFFF) as u16;
            s.ma = (m.i_1e5 / 100).min(0xFFFF) as u16;
            s.dp_mv = (m.dp_1e2 * 10).min(0xFFFF) as u16;
            s.dm_mv = (m.dm_1e2 * 10).min(0xFFFF) as u16;
            if s.ma as u32 > LOAD_MAX_MA {
                s.refused = E_LOAD;
                s.step = S_IDLE;
                return;
            }
            s.done |= D_PASSIVE;
            qc::hvdcp_begin();
            next(s, S_QC_EARLY, T_QC_EARLY);
        }
        S_QC_EARLY => {
            s.qc[1] = qc::line_mv(1);
            s.qc[2] = qc::line_mv(9);
            next(s, S_QC_LATE, T_QC_LATE);
        }
        S_QC_LATE => {
            s.qc[3] = qc::line_mv(9);
            s.qc[0] = qc::hvdcp_class(s.qc[2], s.qc[3]);
            s.done |= D_QC;
            settle(s, S_AFC_REL);
        }
        S_AFC_REL => {
            qc::accp_release();
            next(s, S_AFC_HOLD, T_ACCP_REL);
        }
        S_AFC_HOLD => {
            qc::accp_hold();
            next(s, S_AFC, T_ACCP_HOLD);
        }
        S_AFC => {
            qc::accp_entered();
            s.afc = qc::afc(0x08, 0).is_ok();
            s.done |= D_AFC;
            #[cfg(feature = "fcp")]
            settle(s, S_FCP_REL);
            #[cfg(not(feature = "fcp"))]
            settle(s, S_PD_ARM);
        }
        #[cfg(feature = "fcp")]
        S_FCP_REL => {
            qc::accp_release();
            next(s, S_FCP_HOLD, T_ACCP_REL);
        }
        #[cfg(feature = "fcp")]
        S_FCP_HOLD => {
            qc::accp_hold();
            next(s, S_FCP, T_ACCP_HOLD);
        }
        #[cfg(feature = "fcp")]
        S_FCP => {
            qc::accp_entered();
            s.fcp_err = qc::fcp_read(0x21, &mut s.fcp).err().unwrap_or(0);
            s.done |= D_FCP;
            settle(s, S_PD_ARM);
        }
        S_PD_ARM => {
            s.caps_n = 0;
            s.pd_state = 0;
            s.bc_lvl = 0;
            match pd::arm() {
                Ok(()) => next(s, S_PD_WAIT, T_PD_WAIT),
                Err(e) => {
                    s.pd_err = e;
                    pd_done(s);
                }
            }
        }
        S_PD_WAIT => {
            let mut p = [0u8; 20];
            pd::status(&mut p);
            /* Rp level: BC_LVL from a poll without CC activity while
             * attached — one taken during BMC traffic reads low (bench:
             * the Status0 read at the contract itself, Rp 3 A as 1.5 A) */
            if (2..=5).contains(&p[0]) && p[7] & 0x40 == 0 {
                s.bc_lvl = p[7] & 3;
            }
            /* caps stored + contract on PDO 1, or the deadline */
            if (p[0] == 5 && p[2] != 0) || ina::ticks().wrapping_sub(s.due) as i32 >= 0 {
                s.pd_state = p[0];
                s.cc = p[1];
                s.rev = p[13];
                s.pd_err = 0;
                s.caps_n = pd::caps(&mut s.caps);
                pd_done(s);
            }
        }
        _ => s.step = S_IDLE,
    }
}

fn pd_done(s: &mut Det) {
    let _ = pd::off();
    s.done |= D_PD;
    s.step = S_IDLE;
}

/// K2 short press on this screen.
pub fn k2() {
    if busy() {
        return;
    }
    let (page, pages) = crate::pdscr::page();
    if page + 1 < pages {
        crate::pdscr::next_page();
    } else {
        let _ = start();
    }
}

/* ---- text ---- */

const L_HEAD: u8 = 0;
const L_FOUND: u8 = 1;
const L_VBUS: u8 = 2;
const L_DPM: u8 = 3;
const L_DIV: u8 = 4;
const L_QC: u8 = 5;
const L_AFC: u8 = 6;
const L_FCP: u8 = 7;
const L_PD: u8 = 8;
const L_AGAIN: u8 = 9;
const L_PDO: u8 = 16; /* + PDO index */

/// The line kinds currently shown, in order; returns the count.
fn layout(out: &mut [u8; 18]) -> usize {
    let s = st();
    let mut n = 0;
    let mut push = |k: u8| {
        if let Some(x) = out.get_mut(n) {
            *x = k;
            n += 1;
        }
    };
    push(L_HEAD);
    if s.refused == 0 {
        if s.ran && s.step == S_IDLE {
            push(L_FOUND);
        }
        if s.done & D_PASSIVE != 0 {
            push(L_VBUS);
            push(L_DPM);
            push(L_DIV);
        }
        if s.done & D_QC != 0 {
            push(L_QC);
        }
        if s.done & D_AFC != 0 {
            push(L_AFC);
        }
        if s.done & D_FCP != 0 {
            push(L_FCP);
        }
        if s.done & D_PD != 0 {
            push(L_PD);
            for k in 0..s.caps_n {
                push(L_PDO + k);
            }
        }
    }
    if s.ran && s.step == S_IDLE {
        push(L_AGAIN);
    }
    n
}

pub fn lines() -> usize {
    layout(&mut [0; 18])
}

/// Divider signature: 1-4 = Apple 0.5/1/2.1/2.4 A, 5 = Samsung, 6 = D+ = D−
/// tied/biased, 0 = none (tools/detect.py `divider`).
fn divider(dp: u16, dm: u16) -> u8 {
    let near = |v: u16, t: u16| v.abs_diff(t) < 250;
    for (i, (p, m)) in [(2000, 2000), (2000, 2700), (2700, 2000), (2700, 2700)].iter().enumerate() {
        if near(dp, *p) && near(dm, *m) {
            return 1 + i as u8;
        }
    }
    if near(dp, 1200) && near(dm, 1200) {
        5
    } else if dp > 300 && dp.abs_diff(dm) < 50 {
        6
    } else {
        0
    }
}

/// Format line `i` into `out`; returns its color (0 past the end).
pub fn line(i: usize, out: &mut [u8; COLS]) -> u16 {
    let mut kinds = [0u8; 18];
    let n = layout(&mut kinds);
    let s = st();
    let mut w = W::new();
    let Some(&k) = kinds.get(i).filter(|_| i < n) else {
        *out = w.b;
        return 0;
    };
    let color = match k {
        L_HEAD => {
            if s.refused != 0 {
                w.s(b"refused: ");
                w.s(match s.refused {
                    E_SINK_ARMED => b"PD sink on" as &[u8],
                    E_SINK_SEEN => b"sink on port",
                    _ => b"load ",
                });
                if s.refused == E_LOAD {
                    w.uint(s.ma as u32, 0);
                    w.s(b"mA");
                }
                pdinfo::C_ORANGE
            } else if s.step != S_IDLE {
                w.s(b"detecting: ");
                w.s(match s.step {
                    S_START | S_PASSIVE => b"D+/D-" as &[u8],
                    S_QC_EARLY | S_QC_LATE => b"QC",
                    S_AFC_REL | S_AFC_HOLD | S_AFC => b"AFC",
                    S_PD_ARM | S_PD_WAIT => b"PD",
                    _ => b"FCP",
                });
                pdinfo::C_YELLOW
            } else if s.ran {
                w.s(b"charger detect done");
                pdinfo::C_WHITE
            } else {
                w.s(b"K2: detect charger");
                pdinfo::C_WHITE
            }
        }
        L_FOUND => {
            let div = divider(s.dp_mv, s.dm_mv);
            let qc = if div == 0 || div == 6 { s.qc[0] } else { 0 };
            let found = [
                (matches!(div, 1..=4), b"Apple " as &[u8]),
                (div == 5, b"Samsung "),
                (qc == 2, b"QC "),
                (qc == 1, b"DCP "),
                (s.afc, b"AFC "),
                (s.done & D_FCP != 0 && s.fcp_err == 0, b"FCP "),
                (s.caps_n != 0, b"PD"),
            ];
            let mut any = false;
            for (f, t) in found {
                if f {
                    w.s(t);
                    any = true;
                }
            }
            if !any {
                w.s(b"plain USB only");
                pdinfo::C_GREY
            } else {
                pdinfo::C_GREEN
            }
        }
        L_VBUS => {
            w.s(b"VBUS ");
            w.volts(s.v_mv as u32, 0);
            w.s(b" load ");
            w.uint(s.ma as u32, 0);
            w.s(b"mA");
            pdinfo::C_YELLOW
        }
        L_DPM => {
            w.s(b"D+ ");
            w.volts(s.dp_mv as u32, 0);
            w.s(b"  D- ");
            w.volts(s.dm_mv as u32, 0);
            pdinfo::C_BLUE
        }
        L_DIV => match divider(s.dp_mv, s.dm_mv) {
            0 => {
                w.s(b"no D+/D- divider");
                pdinfo::C_GREY
            }
            5 => {
                w.s(b"Samsung divider 2A");
                pdinfo::C_GREEN
            }
            6 => {
                w.s(b"D+=D- tied, biased");
                pdinfo::C_WHITE
            }
            d => {
                w.s(b"Apple divider ");
                w.s([b"0.5A" as &[u8], b"1A", b"2.1A", b"2.4A"].get((d as usize).wrapping_sub(1)).copied().unwrap_or(b""));
                pdinfo::C_GREEN
            }
        },
        L_QC => {
            if !matches!(divider(s.dp_mv, s.dm_mv), 0 | 6) {
                w.s(b"QC n/a (divider)");
                pdinfo::C_GREY
            } else {
                w.s(match s.qc[0] {
                    0 => b"no DCP " as &[u8],
                    1 => b"DCP    ",
                    _ => b"QC     ",
                });
                w.s(b"D- ");
                w.uint(s.qc[2] as u32, 0);
                w.ch(b'>');
                w.uint(s.qc[3] as u32, 0);
                w.s(b"mV");
                if s.qc[0] == 2 {
                    pdinfo::C_GREEN
                } else {
                    pdinfo::C_WHITE
                }
            }
        }
        L_AFC => {
            if s.afc {
                w.s(b"AFC answered");
                pdinfo::C_GREEN
            } else {
                w.s(b"AFC no answer");
                pdinfo::C_GREY
            }
        }
        L_FCP => {
            if s.fcp_err == 0 {
                w.s(b"FCP ");
                w.uint(s.fcp[0] as u32, 0);
                w.s(b" raised, ");
                w.fixed(s.fcp[1] as u32 * 5, 1, 0);
                w.ch(b'W');
                pdinfo::C_GREEN
            } else {
                w.s(if s.fcp_err & 0xF0 == 0x40 { b"FCP no reply" as &[u8] } else { b"FCP NACK" });
                pdinfo::C_GREY
            }
        }
        L_PD => {
            if s.pd_err != 0 {
                w.s(b"PD error ");
                w.uint(s.pd_err as u32, 0);
                pdinfo::C_ORANGE
            } else if s.pd_state <= 1 {
                w.s(b"no Type-C source");
                pdinfo::C_GREY
            } else {
                w.s(b"CC");
                w.uint(s.cc as u32, 0);
                w.s(match s.bc_lvl {
                    1 => b" Rp USB  " as &[u8],
                    2 => b" Rp 1.5A ",
                    3 => b" Rp 3.0A ",
                    _ => b" Rp ?    ",
                });
                if s.caps_n != 0 {
                    w.s(pdinfo::rev_str(s.rev));
                    pdinfo::C_GREEN
                } else {
                    w.s(b"no PD");
                    pdinfo::C_WHITE
                }
            }
        }
        L_AGAIN => {
            w.s(b"K2: run again");
            pdinfo::C_GREY
        }
        _ => {
            let k = k.wrapping_sub(L_PDO) as usize;
            w.ch(b' ');
            w.uint(k as u32 + 1, 0);
            w.ch(b' ');
            let p = s.caps.get(4 * k..4 * k + 4).and_then(|b| b.try_into().ok());
            pdinfo::pdo(&mut w, p.map_or(0, u32::from_le_bytes));
            pdinfo::C_WHITE
        }
    };
    *out = w.b;
    color
}
