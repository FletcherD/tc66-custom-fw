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
use crate::pdspec::*;
use crate::pdsta::{CAP_BAD_CRC, CAP_DISCONNECT, CAP_HARD_RESET, SOP, SOP1};
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
    /* that Discover Identity ACK as received (header + objects, LE), for
     * `pdcbl`; cable_id_len 0 = none */
    cable_id: [u8; MAX_MSG_LEN],
    cable_id_len: u8,
    cable: u8, /* CABLE_NONE / CABLE_LAYOUT_PD2 / CABLE_LAYOUT_PD3 */
    msgs: u16,
    bad_crc: u16,
    hard_resets: u16,
}

/* Info.cable: which cable VDO layout was seen (PD 3.x adds max VBUS) */
const CABLE_NONE: u8 = 0;
const CABLE_LAYOUT_PD2: u8 = 1;
const CABLE_LAYOUT_PD3: u8 = 2;

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
    cable_id: [0; MAX_MSG_LEN],
    cable_id_len: 0,
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
        cable_id: [0; MAX_MSG_LEN],
        cable_id_len: 0,
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
    s.cable = CABLE_NONE;
    s.cable_id_len = 0;
}

/// pd.rs capture event code (pdsta.rs CAP_*: disconnect, bad CRC and hard
/// reset count; others ignored).
pub fn on_event(code: u8) {
    let s = st();
    match code {
        CAP_DISCONNECT => drop_contract(s),
        CAP_BAD_CRC => s.bad_crc = s.bad_crc.wrapping_add(1),
        CAP_HARD_RESET => {
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
    let h = u32::from(hdr);
    if HDR_EXTENDED.get(h) != 0 {
        return; /* extended message (chunked; EPR caps etc.) */
    }
    let n = HDR_NUM_OBJECTS.get(h) as usize;
    let mtype = HDR_MSG_TYPE.get(h);
    let from_src = HDR_POWER_ROLE.get(h) != 0; /* SOP */
    let from_cable = HDR_CABLE_PLUG.get(h) != 0; /* SOP'/SOP'' */
    match (sop, n, mtype) {
        (SOP, 0, CTRL_PS_RDY) if from_src => {
            /* PS_RDY: the pending Request is now the contract */
            if s.req_rdo != 0 {
                s.rdo = s.req_rdo;
                s.pdo = s.req_pdo;
                s.req_rdo = 0;
            }
        }
        (SOP, 0, CTRL_REJECT) if from_src => s.req_rdo = 0,
        (SOP, 1.., DATA_SOURCE_CAP) if from_src => {
            /* Source_Capabilities (retries just overwrite) */
            for (i, c) in s.caps.iter_mut().enumerate() {
                *c = obj(objs, i);
            }
            s.caps_n = n as u8;
            s.rev = HDR_SPEC_REV.get(h) as u8;
        }
        (SOP, 1, DATA_REQUEST) if !from_src => {
            let r = obj(objs, 0);
            let pos = RDO_OBJECT_POSITION.get(r) as usize;
            s.req_rdo = r;
            s.req_pdo = if (1..=s.caps_n as usize).contains(&pos) {
                s.caps.get(pos - 1).copied().unwrap_or(0)
            } else {
                0
            };
        }
        (SOP1, DISCID_CABLE_MIN_OBJECTS.., DATA_VENDOR_DEFINED) if from_cable => {
            /* the cable's Discover Identity ACK (cable plug sender):
             * structured VDM, ID Header product type passive or active
             * cable, then its cable VDO */
            let vdm = obj(objs, DISCID_VDM_HEADER);
            let idh = obj(objs, DISCID_ID_HEADER);
            if VDM_STRUCTURED.get(vdm) != 0
                && VDM_COMMAND.get(vdm) == CMD_DISCOVER_IDENTITY
                && VDM_COMMAND_TYPE.get(vdm) == VDM_ACK
                && matches!(IDH_PRODUCT_TYPE.get(idh), CABLE_PASSIVE | CABLE_ACTIVE)
            {
                s.cable_vid = IDH_VID.get(idh) as u16;
                s.cable_vdo = obj(objs, DISCID_PRODUCT_TYPE_VDO);
                let n = (2 + objs.len()).min(MAX_MSG_LEN);
                s.cable_id[..2].copy_from_slice(&hdr.to_le_bytes());
                s.cable_id[2..n].copy_from_slice(&objs[..n - 2]);
                s.cable_id_len = n as u8;
                s.cable = if HDR_SPEC_REV.get(h) >= REV_3_X { CABLE_LAYOUT_PD3 } else { CABLE_LAYOUT_PD2 };
            }
        }
        _ => {}
    }
}

/// `pdcbl`: copy the cable's Discover Identity ACK frame; returns its length
/// (0 = no e-marker seen since the engine was armed / the source left).
pub fn cable_frame(out: &mut [u8; MAX_MSG_LEN]) -> u8 {
    let s = st();
    out.copy_from_slice(&s.cable_id);
    s.cable_id_len
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
            if s.cable == CABLE_NONE {
                w.t(NO_EMARKER);
                C_GREY
            } else {
                let v = s.cable_vdo;
                w.t(CBL);
                w.t(match CABLE_VBUS_CURRENT.get(v) {
                    CABLE_CURRENT_3A => CBL_3A,
                    CABLE_CURRENT_5A => CBL_5A,
                    _ => CBL_NO_A,
                });
                if s.cable == CABLE_LAYOUT_PD3 {
                    w.ch(b' ');
                    w.uint(CABLE_MAX_VBUS_BASE_V + CABLE_MAX_VBUS_STEP_V * CABLE_MAX_VBUS.get(v), 0);
                    w.ch(b'V');
                }
                /* speed strings in field order, then SPEED_UNKNOWN */
                w.t(USB2 + CABLE_USB_SPEED.get(v).min((SPEED_UNKNOWN - USB2) as u32) as u8);
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
                let active = s.rdo != 0 && RDO_OBJECT_POSITION.get(s.rdo) == pos && s.pdo == p;
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
    let v_lo = RANGE_MIN_VOLTAGE.val(p); /* = FIXED_VOLTAGE for a fixed PDO */
    let v_hi = RANGE_MAX_VOLTAGE.val(p);
    let range = |w: &mut W, lo_mv: u32, hi_mv: u32| {
        w.fixed(lo_mv / 100, 1, 0);
        w.ch(b'-');
        w.fixed(hi_mv / 100, 1, 0);
        w.t(V_SP);
    };
    match PDO_TYPE.get(p) {
        PDO_FIXED => {
            w.volts(FIXED_VOLTAGE.val(p), 5);
            w.ch(b' ');
            w.amps(FIXED_MAX_CURRENT.val(p));
        }
        PDO_BATTERY => {
            w.t(BAT);
            range(w, v_lo, v_hi);
            w.uint(BATTERY_MAX_POWER.val(p) / 1000, 0);
            w.ch(b'W');
        }
        PDO_VARIABLE => {
            w.t(VAR);
            range(w, v_lo, v_hi);
            w.amps(VARIABLE_MAX_CURRENT.val(p));
        }
        _ => match APDO_TYPE.get(p) {
            APDO_PPS => {
                w.t(PPS);
                range(w, PPS_MIN_VOLTAGE.val(p), PPS_MAX_VOLTAGE.val(p));
                w.amps(PPS_MAX_CURRENT.val(p));
            }
            APDO_SPR_AVS => {
                /* SPR AVS: the 9-15 V current shown */
                w.t(AVS_SPR);
                w.amps(SPR_AVS_MAX_CURRENT_15V.val(p));
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
    w.uint(RDO_OBJECT_POSITION.get(rdo), 0);
    w.ch(b' ');
    if p == 0 {
        /* PDO not among the stored caps: fixed-supply RDO reading */
        w.t(UNKNOWN_PDO);
        w.amps(RDO_OPERATING_CURRENT.val(rdo));
        return;
    }
    match PDO_TYPE.get(p) {
        PDO_FIXED => {
            w.volts(FIXED_VOLTAGE.val(p), 0);
            w.ch(b' ');
            w.amps(RDO_OPERATING_CURRENT.val(rdo));
        }
        PDO_BATTERY => {
            w.t(BAT);
            w.uint(RDO_OPERATING_POWER.val(rdo) / 1000, 0);
            w.ch(b'W');
        }
        PDO_VARIABLE => {
            w.t(VAR);
            w.amps(RDO_OPERATING_CURRENT.val(rdo));
        }
        _ => {
            let avs = APDO_TYPE.get(p) == APDO_SPR_AVS;
            w.t(if avs { AVS } else { PPS });
            w.volts(if avs { RDO_AVS_VOLTAGE } else { RDO_PPS_VOLTAGE }.val(rdo), 0);
            w.ch(b' ');
            w.amps(RDO_APDO_CURRENT.val(rdo));
        }
    }
}
