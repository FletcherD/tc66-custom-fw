//! PD state parser + the PD screen's text (`triggers` feature).
//!
//! Fed by pd.rs at its capture points — every frame the engine receives or
//! sends (`cap_msg`: sniff and sink modes alike) and its connect/disconnect/
//! bad-CRC/hard-reset events (`cap_conn`) — so it sees exactly what the
//! KM003C capture sees, independent of whether a host drains that ring.
//! Keeps a compact latest state: source caps (≤ 7 PDOs + the caps' spec
//! rev), the contract, the cable e-marker, counters.
//!
//! The contract is the last sink Request confirmed by the source's PS_RDY.
//! Accept is never needed: the FUSB302 drops frames in the dense Request →
//! GoodCRC → Accept → GoodCRC burst in about half the negotiations, while
//! PS_RDY (tens of ms later) is rarely missed. A Request without its PS_RDY
//! (yet) is shown as pending ('?'); a Reject drops it. The contract keeps a
//! copy of the requested PDO, so a later Source_Cap (Apple re-advertises
//! after cable discovery) doesn't reinterpret it. Hard reset clears caps +
//! contract (the source re-advertises), a sink detach the contract, the
//! source going away everything.
//!
//! Thread-exclusive like pd.rs: the hooks run inside pd's service()/verbs,
//! the formatter in ui::draw and the `pdinf` verb — all main-loop context.

use crate::ina;
use crate::pd;
use crate::text::*;
/// Fixed status lines ahead of the PDO list.
const HEAD: usize = 6;

pub const C_WHITE: u16 = 0xFFFF;
pub const C_GREEN: u16 = 0x07E0;
pub const C_YELLOW: u16 = 0xFFE0;
pub const C_ORANGE: u16 = 0xFD20;
pub const C_BLUE: u16 = 0xAFFF;
pub const C_PINK: u16 = 0xFBEA;
pub const C_GREY: u16 = 0x8410;

struct Info {
    caps: [u32; 7],
    caps_n: u8,
    rev: u8, /* the caps header's spec rev field: 0 = 1.0, 1 = 2.0, 2 = 3.x */
    /* last sink Request not (yet) confirmed by a PS_RDY; rdo 0 = none (a
     * valid RDO always has object position >= 1) */
    req_rdo: u32,
    req_pdo: u32, /* the PDO it requested (0 if not among the stored caps) */
    rdo: u32,     /* the contract (0 = none) */
    pdo: u32,
    cable_vdo: u32, /* passive/active cable VDO from a SOP' Discover Identity ACK */
    cable_vid: u16,
    cable: u8, /* 0 = none seen, 1 = PD 2.0 VDO layout, 2 = PD 3.x */
    msgs: u16,
    bad_crc: u16,
    hard_resets: u16,
}

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
/* .hibss (zeroed by main() before anything runs; all-zero = nothing seen) */
#[link_section = ".hibss"]
static INFO: Racy<Info> = Racy(core::cell::UnsafeCell::new(Info {
    caps: [0; 7],
    caps_n: 0,
    rev: 0,
    req_rdo: 0,
    req_pdo: 0,
    rdo: 0,
    pdo: 0,
    cable_vdo: 0,
    cable_vid: 0,
    cable: 0,
    msgs: 0,
    bad_crc: 0,
    hard_resets: 0,
}));

#[inline(always)]
fn st() -> &'static mut Info {
    unsafe { &mut *INFO.0.get() }
}

/* ---- parser hooks (pd.rs) ---- */

/// New engine session (`pdctl 1`/`3`): forget everything.
pub fn reset() {
    let s = st();
    *s = Info {
        caps: [0; 7],
        caps_n: 0,
        rev: 0,
        req_rdo: 0,
        req_pdo: 0,
        rdo: 0,
        pdo: 0,
        cable_vdo: 0,
        cable_vid: 0,
        cable: 0,
        msgs: 0,
        bad_crc: 0,
        hard_resets: 0,
    };
}

fn drop_contract(s: &mut Info) {
    s.req_rdo = 0;
    s.rdo = 0;
}

fn drop_caps(s: &mut Info) {
    s.caps_n = 0;
    drop_contract(s);
}

/// The source went away (sniff: Rp gone from the locked CC; sink: detach).
pub fn source_gone() {
    let s = st();
    drop_caps(s);
    s.cable = 0;
}

/// pd.rs capture event code (0x21 connect, 0x22 disconnect, 0x32 bad CRC,
/// 0x33 hard reset; others ignored).
pub fn on_event(code: u8) {
    let s = st();
    match code {
        0x22 => drop_contract(s),
        0x32 => s.bad_crc = s.bad_crc.wrapping_add(1),
        0x33 => {
            s.hard_resets = s.hard_resets.wrapping_add(1);
            drop_caps(s);
        }
        _ => {}
    }
}

/// Data object `i` (LE) of a message, 0 if absent.
fn obj(objs: &[u8], i: usize) -> u32 {
    objs.get(4 * i..4 * i + 4)
        .and_then(|b| b.try_into().ok())
        .map_or(0, u32::from_le_bytes)
}

/// One PD message (header + data objects, no CRC); `sop` 0 = SOP,
/// 1 = SOP', 2 = SOP''.
pub fn on_msg(hdr: u16, objs: &[u8], sop: u8) {
    let s = st();
    s.msgs = s.msgs.wrapping_add(1);
    if hdr & 0x8000 != 0 {
        return; /* extended message (chunked; EPR caps etc.) */
    }
    let n = (hdr >> 12 & 7) as usize;
    let mtype = hdr & 0x1F;
    /* SOP: port power role (1 = source); SOP': cable plug (1 = from the cable) */
    let from_src = hdr & 0x100 != 0;
    match (sop, n, mtype) {
        (0, 0, 6) if from_src => {
            /* PS_RDY: the pending Request is now the contract */
            if s.req_rdo != 0 {
                s.rdo = s.req_rdo;
                s.pdo = s.req_pdo;
                s.req_rdo = 0;
            }
        }
        (0, 0, 4) if from_src => s.req_rdo = 0, /* Reject */
        (0, 1.., 1) if from_src => {
            /* Source_Capabilities (retries just overwrite) */
            for (i, c) in s.caps.iter_mut().enumerate() {
                *c = obj(objs, i);
            }
            s.caps_n = n as u8;
            s.rev = (hdr >> 6 & 3) as u8;
        }
        (0, 1, 2) if !from_src => {
            /* Request */
            let r = obj(objs, 0);
            let pos = (r >> 28) as usize;
            s.req_rdo = r;
            s.req_pdo = if (1..=s.caps_n as usize).contains(&pos) {
                s.caps.get(pos - 1).copied().unwrap_or(0)
            } else {
                0
            };
        }
        (1, 5.., 15) if from_src => {
            /* the cable's Discover Identity ACK: structured VDM header,
             * command 1, type ACK; ID Header product type passive (3) or
             * active (4) cable; the cable VDO is object 4 */
            let vdm = obj(objs, 0);
            let idh = obj(objs, 1);
            if vdm & 0x8000 != 0 && vdm & 0x1F == 1 && vdm >> 6 & 3 == 1 && matches!(idh >> 27 & 7, 3 | 4) {
                s.cable_vid = idh as u16;
                s.cable_vdo = obj(objs, 4);
                s.cable = if hdr >> 6 & 3 >= 2 { 2 } else { 1 };
            }
        }
        _ => {}
    }
}

/* ---- text ---- */

/// Number of text lines (the status block + one per source PDO, or one
/// "no source caps" line).
pub fn lines() -> usize {
    HEAD + (st().caps_n as usize).max(1)
}

/// Format line `i` into `out` (space-padded); returns its color. Lines past
/// `lines()` come back blank.
pub fn line(i: usize, out: &mut [u8; COLS]) -> u16 {
    let s = st();
    let mut w = W::new();
    let color = match i {
        0 => {
            let l = pd::link();
            match l.mode {
                pd::MODE_OFF => w.t(PD_OFF),
                m => {
                    w.t(if m == pd::MODE_SNIFF { SNIFF } else { SINK });
                    if s.caps_n > 0 {
                        w.rev(s.rev);
                    } else {
                        w.t(NO_REV);
                    }
                    if l.src {
                        w.t(SRC);
                    }
                    if l.snk && m == pd::MODE_SNIFF {
                        w.t(SNK);
                    }
                }
            }
            C_WHITE
        }
        1 => {
            if s.req_rdo != 0 {
                contract(&mut w, b'?', s.req_rdo, s.req_pdo);
                C_ORANGE
            } else if s.rdo != 0 {
                contract(&mut w, b'>', s.rdo, s.pdo);
                C_GREEN
            } else {
                w.t(NO_CONTRACT);
                C_GREY
            }
        }
        2 => {
            let (v, a) = ina::snapshot_vi(); /* 1e-4 V, 1e-5 A */
            w.t(VBUS);
            w.fixed(v / 10, 3, 0);
            w.t(V_SP);
            w.fixed(a / 100, 3, 0);
            w.ch(b'A');
            C_YELLOW
        }
        3 => {
            let l = pd::link();
            if l.cc == 0 {
                w.t(CC_NO_SOURCE);
            } else {
                w.t(CC);
                w.uint(l.cc as u32, 0);
                w.ch(b' ');
                w.volts(l.cc_mv as u32, 0);
                /* Rp's advertised current, from the CC voltage across a
                 * sink's Rd (Type-C vRd ranges: default 0.25-0.61 V,
                 * 1.5 A 0.70-1.16 V, 3 A 1.31-2.04 V); without a sink's Rd
                 * the line just shows the open-circuit level */
                if l.snk {
                    w.t(match l.cc_mv {
                        0..=659 => RP_USB,
                        660..=1229 => RP_1A5,
                        _ => RP_3A,
                    });
                } else {
                    w.t(OPEN);
                }
            }
            C_BLUE
        }
        4 => {
            if s.cable == 0 {
                w.t(NO_EMARKER);
                C_GREY
            } else {
                let v = s.cable_vdo;
                w.t(CBL);
                w.t(match v >> 5 & 3 {
                    1 => CBL_3A,
                    2 => CBL_5A,
                    _ => CBL_NO_A,
                });
                if s.cable == 2 {
                    w.ch(b' ');
                    w.uint(20 + 10 * (v >> 9 & 3), 0);
                    w.ch(b'V');
                }
                w.t(USB2 + (v & 7).min(5) as u8);
                w.ch(b' ');
                w.hex16(s.cable_vid);
                C_PINK
            }
        }
        5 => {
            w.t(MSG);
            w.uint((s.msgs as u32).min(9999), 0);
            w.t(CRC);
            w.uint((s.bad_crc as u32).min(999), 0);
            w.t(HR);
            w.uint((s.hard_resets as u32).min(99), 0);
            C_GREY
        }
        _ => {
            let k = i - HEAD;
            if s.caps_n == 0 && k == 0 {
                w.t(NO_CAPS);
                C_GREY
            } else if let Some(&p) = s.caps.get(k).filter(|_| k < s.caps_n as usize) {
                let pos = k as u32 + 1;
                let active = s.rdo != 0 && s.rdo >> 28 == pos && s.pdo == p;
                w.ch(if active { b'>' } else { b' ' });
                w.uint(pos, 0);
                w.ch(b' ');
                pdo(&mut w, p);
                if active {
                    C_GREEN
                } else {
                    C_WHITE
                }
            } else {
                0
            }
        }
    };
    *out = w.b;
    color
}

/// One source PDO.
pub fn pdo(w: &mut W, p: u32) {
    let v_lo = (p >> 10 & 0x3FF) * 50; /* fixed/variable/battery, mV */
    let v_hi = (p >> 20 & 0x3FF) * 50;
    let range = |w: &mut W, lo_mv: u32, hi_mv: u32| {
        w.fixed(lo_mv / 100, 1, 0);
        w.ch(b'-');
        w.fixed(hi_mv / 100, 1, 0);
        w.t(V_SP);
    };
    match p >> 30 {
        0 => {
            w.volts(v_lo, 5);
            w.ch(b' ');
            w.amps((p & 0x3FF) * 10);
        }
        1 => {
            w.t(VAR);
            range(w, v_lo, v_hi);
            w.amps((p & 0x3FF) * 10);
        }
        2 => {
            w.t(BAT);
            range(w, v_lo, v_hi);
            w.uint((p & 0x3FF) / 4, 0);
            w.ch(b'W');
        }
        _ => match p >> 28 & 3 {
            0 => {
                w.t(PPS);
                range(w, (p >> 8 & 0xFF) * 100, (p >> 17 & 0xFF) * 100);
                w.amps((p & 0x7F) * 50);
            }
            2 => {
                /* SPR AVS: 9-15 V current shown (bits 19:10), 15-20 V's in 9:0 */
                w.t(AVS_SPR);
                w.amps((p >> 10 & 0x3FF) * 10);
            }
            _ => {
                w.t(APDO);
                w.hex16((p >> 16) as u16);
                w.hex16(p as u16);
            }
        },
    }
}

/// The contract line: mark, object position, then what was granted — fixed:
/// the PDO's voltage + the RDO's operating current; PPS/AVS: the RDO's
/// setpoint; variable: current; battery: power.
fn contract(w: &mut W, mark: u8, rdo: u32, p: u32) {
    w.ch(mark);
    w.ch(b'#');
    w.uint(rdo >> 28, 0);
    w.ch(b' ');
    let op10 = rdo >> 10 & 0x3FF;
    if p == 0 {
        /* PDO not among the stored caps: fixed-supply RDO reading */
        w.t(UNKNOWN_PDO);
        w.amps(op10 * 10);
        return;
    }
    match p >> 30 {
        0 => {
            w.volts((p >> 10 & 0x3FF) * 50, 0);
            w.ch(b' ');
            w.amps(op10 * 10);
        }
        1 => {
            w.t(VAR);
            w.amps(op10 * 10);
        }
        2 => {
            w.t(BAT);
            w.uint(op10 / 4, 0);
            w.ch(b'W');
        }
        _ => {
            let avs = p >> 28 & 3 == 2;
            w.t(if avs { AVS } else { PPS });
            /* output voltage bits 20:9 — 20 mV (PPS) / 25 mV (AVS) units */
            w.volts((rdo >> 9 & 0xFFF) * if avs { 25 } else { 20 }, 0);
            w.ch(b' ');
            w.amps((rdo & 0x7F) * 50);
        }
    }
}
