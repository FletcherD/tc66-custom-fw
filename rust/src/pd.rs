//! FUSB302 USB-PD sink engine (M4 bring-up). Host-armed; main.rs starts the sniffer on USB enumeration.
//!
//! The FUSB302 (I²C 0x22, shared bus with the INA226/EEPROM) is the Type-C
//! CC PHY: it handles BMC signaling, CRC and GoodCRC replies (AUTO_CRC) in
//! hardware; this module is the protocol/policy half — a minimal PD 2.0
//! sink. Flow: arm (`pdctl 1`) → present Rd, wait for a source attach
//! (CC comparator poll) → enable BMC on the active CC → collect
//! Source_Capabilities (asking with Get_Source_Cap if the source already
//! gave up before we armed) → immediately Request the 5 V PDO (a sink that
//! GoodCRCs caps but never Requests gets hard-reset ~30 ms later) →
//! Accept → PS_RDY = contract. `pdreq <n>` then renegotiates to any other
//! fixed PDO. **A granted Request raises the OUT-port voltage** — bench
//! rule: nothing attached downstream when triggering.
//!
//! Everything runs in thread mode from the main loop (`service()`) and the
//! verb handlers — same context, so the `PD` static is thread-exclusive (no
//! ISR ever touches it). All I²C goes through `ina::pause` per the bus rule.
//! The FUSB302's INT_N (PA15, active-low, internal pull-up) is read as a
//! plain GPIO in the main loop — not an MCU interrupt: while attached or
//! sniffing, a low INT_N runs a poll at once (CC activity edges, GoodCRC
//! sent, TX done/failed, resets — the MASK_* constants), else a backup poll
//! every BACKUP_POLL ticks (48 ms) covers timeouts, detach and the CC sweep.
//! The tight timing (GoodCRC within 195 µs) is AUTO_CRC hardware.
//!
//! Poll cost: one 7-byte status burst ≈ 1 ms of sampler pause per poll —
//! per wire event plus one per 48 ms while armed; attach polling (no source
//! yet) every 48 ms. Idle (state 0, the default): zero cost.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::i2c;
use gd32f1::gd32f150 as pac;
use crate::ina;
use crate::pdspec::*;
use crate::pdsta::{self, *};

const ADDR: u8 = 0x22;

/* registers */
const R_SWITCHES0: u8 = 0x02;
const R_SWITCHES1: u8 = 0x03;
const R_MEASURE: u8 = 0x04; /* bit6 MEAS_VBUS, bits5:0 MDAC */
const R_SLICE: u8 = 0x05; /* BMC slicer: bits7:6 hysteresis, 5:0 SDAC */
const R_CONTROL0: u8 = 0x06;
const R_CONTROL1: u8 = 0x07;
const R_CONTROL3: u8 = 0x09;
const R_MASK: u8 = 0x0A;
const R_POWER: u8 = 0x0B;
const R_RESET: u8 = 0x0C;
const R_MASKA: u8 = 0x0E;
const R_MASKB: u8 = 0x0F;
const R_STATUS0A: u8 = 0x3C; /* burst 0x3C..0x42: s0a s1a inta intb s0 s1 int */
const R_STATUS0: u8 = 0x40;
const R_FIFOS: u8 = 0x43;

/* Switches0 */
const SW0_NONE: u8 = 0x00; /* no pulldowns, no CC measure */
const SW0_PDWN: u8 = 0x03; /* Rd on both CC */
const SW0_MEAS_CC1: u8 = 0x04;
const SW0_MEAS_CC2: u8 = 0x08;
/* Switches1: SPECREV in bits 6:5 (01 = PD 2.0, 10 = PD 3.0 — also sets the
 * hardware GoodCRC's rev field) + AUTO_CRC + TXCC1/2 */
const SW1_AUTO_CRC: u8 = 0x04;
const SW1_TXCC1: u8 = 0x01;
const SW1_TXCC2: u8 = 0x02;
const SW1_SPECREV_SHIFT: u8 = 5;
/* header / Switches1 rev codes (Message Header Specification Revision) */
const REV20: u8 = REV_2_0 as u8;
const REV30: u8 = REV_3_X as u8;
fn sw1_rev(rev: u8) -> u8 {
    rev << SW1_SPECREV_SHIFT
}
/* Control0/1 */
const C0_TX_FLUSH: u8 = 0x40; /* INT_MASK (0x20) clear: INT_N live */
/* interrupt masks (1 = masked). INT_N (PA15) wakes service(): CC activity
 * edges (start/end of every frame on the wire) in both modes; the sink
 * also on GoodCRC-sent (a frame received), TX done/failed and resets. */
const MASK_ACTIVITY_ONLY: u8 = 0xBF;
const MASK_SNIFF: u8 = 0xAF; /* ACTIVITY + CRC_CHK (each packet checked) */
const MASKA_SINK: u8 = 0xE0; /* RETRYFAIL HARDSENT TXSENT SOFTRST HARDRST */
const MASKA_SNIFF: u8 = 0xF6; /* HARDSENT HARDRST */
const MASKB_SINK: u8 = 0x00; /* GCRCSENT */
const MASKB_SNIFF: u8 = 0x01;
/* backup poll while attached/sniffing (ticks): timeouts, detach, CC sweep */
const BACKUP_POLL: u32 = 12;
const C1_RX_FLUSH: u8 = 0x04;
/* also receive SOP' / SOP'' (cable traffic) and their _DEBUG variants */
const C1_ENSOP12: u8 = 0x63;
/* Control3: AUTO_RETRY + N_RETRIES=3; SEND_HARD_RESET (self-clearing) */
const C3_RETRIES: u8 = 0x07;
const C3_SEND_HARD_RESET: u8 = 0x40;
/* Power: PWR[3:0] all on — bandgap/wake, receiver + current refs,
 * measure block, internal oscillator */
const POWER_ALL: u8 = 0x0F;
/* Reset */
const RST_SW: u8 = 0x01;
const RST_PD: u8 = 0x02;
/* Interrupta */
const IA_RETRYFAIL: u8 = 0x10;
const IA_HARDSENT: u8 = 0x08;
const IA_TXSENT: u8 = 0x04;
const IA_HARDRST: u8 = 0x01;
/* Status0 bits: pdsta.rs (S0_*, reported raw via pdsta); Status1 */
const IR_CRC_CHK: u8 = 0x10; /* Interrupt: a packet's CRC was checked */
const S1_RX_EMPTY: u8 = 0x20;
const S1_RX_FULL: u8 = 0x10;

/* TX FIFO tokens (datasheet Table 41; SOP1/SOP2 there = Sync-1/Sync-2) */
const TK_TXON: u8 = 0xA1;
const TK_SYNC1: u8 = 0x12;
const TK_SYNC2: u8 = 0x13;
const TK_PACKSYM: u8 = 0x80;
const TK_JAM_CRC: u8 = 0xFF;
const TK_EOP: u8 = 0x14;
const TK_TXOFF: u8 = 0xFE;

/* RX FIFO token class, bits 7:5 of a packet's first byte (Table 42) */
const RX_TOKEN_CLASS_SHIFT: u8 = 5;
const RX_TOKEN_SOP: u8 = 7;
const RX_TOKEN_SOP1: u8 = 6;
const RX_TOKEN_SOP2: u8 = 5;
const RX_TOKEN_SOP1_DEBUG: u8 = 4;
const RX_TOKEN_SOP2_DEBUG: u8 = 3;

/* INT_N on PA15 */
const INT_N_PIN: u32 = 15;
/* pdask / pdsnd type byte: message type, this bit set for an extended message */
const PDSND_EXTENDED: u8 = 0x80;
/* the Sink_Capabilities we answer Get_Sink_Cap with: fixed 5 V, 100 mA */
const SINK_PDO_MV: u32 = 5000;
const SINK_PDO_MA: u32 = 100;

/* err codes carried in Pd.err (i2c stages are 1..0x0F | 0x80) */
const E_RETRYFAIL: u8 = 0xE1;
const E_REJECT: u8 = 0xE2;
const E_ACCEPT_TIMEOUT: u8 = 0xE3;
const E_PSRDY_TIMEOUT: u8 = 0xE4;
const E_RX_FULL: u8 = 0xE5; /* sniff: the RX FIFO filled between polls */
/* sniff: MDAC threshold ~2.69 V on the locked CC. A source's Rp alone pulls
 * CC above it (open); a sink's Rd holds it at 0.4-2.04 V (Type-C vRd). */
const SNIFF_OPEN_MDAC: u8 = 0x3F;
/* BMC slicer threshold SDAC 0x18 (reset 0x20), 85 mV hysteresis, both modes.
 * Bench (Apple charger -> meter -> Moto at 9 V / 2.8 A): at the reset value
 * the charger's first VDM to the phone was lost 3/3 (never even CRC-
 * checked); SDAC 0x10-0x1C caught it 12/12; 0x08 and >= 0x28 lost others. */
const SLICE: u8 = 0x58;
/* verb errors (ER codes) */
const E_BAD_PPS_SETPOINT: u8 = 0x0D; /* missing / outside the APDO's range */
const E_PDO_KIND: u8 = 0x0E; /* unsupported PDO kind, or PPS at rev 2.0 */
const E_BAD_ARGS: u8 = 0x0F; /* bad args / bad state (project-wide code) */

pub struct Pd {
    state: u8,
    cc: u8, /* active CC line: 0 none, 1, 2 */
    msg_id: u8,
    caps_n: u8,
    caps: [u32; 7],
    req_obj: u8,   /* outstanding Request object position (1-based) */
    contract: u8,  /* granted object position, 0 none */
    err: u8,       /* last error, sticky until next arm/request */
    events: u8,    /* accumulated Interrupta bits */
    status0: u8,   /* last Status0/Status1 (pdsta reports these) */
    status1: u8,
    detach: u8,    /* consecutive quiet BC_LVL=0 polls */
    rx_msgs: u16,
    rx_hdr: [u16; 3], /* last 3 received SOP headers, newest first (pdsta) */
    vcc1_mv: u16, /* CC pin voltages from the MDAC sweep (powerz Vcc1/Vcc2) */
    vcc2_mv: u16,
    rev: u8,       /* negotiated header rev code: REV20 / REV30 (from caps) */
    tx_pend: bool, /* a TX is in flight (no TXSENT/RETRYFAIL yet) — gates
                    * non-essential replies so bursts (Apple VDM spam) can't
                    * duplicate msg ids or flush a message mid-flight */
    last_rdo: u32, /* last Request object sent (PPS keepalive re-sends it) */
    sink: bool,    /* sniff: a sink's Rd loads the locked CC (pdsta byte 3) */
    rx_full: bool, /* sniff: RX FIFO full at the last poll (edge → event) */
    sop1: bool,    /* sink: listen-only phase after attach / hard reset (no
                    * AUTO_CRC, SOP'/SOP'' received) until the first caps */
    asked: bool,   /* sink: a pdsnd request is out — its answer is not
                    * Not_Supported'ed */
    err_reg: u8,   /* register of the last failed I2C transaction (pdsta) */
    poll_tick: u32,
    req_tick: u32,
    cap_ask_tick: u32,
    vcc_tick: u32,
    pps_tick: u32,
}

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}

/// Thread-exclusive (main loop service + verb dispatch — one context;
/// neither ISR touches this module).
static PD: Racy<Pd> = Racy(core::cell::UnsafeCell::new(Pd {
    state: ST_OFF,
    cc: 0,
    msg_id: 0,
    caps_n: 0,
    caps: [0; 7],
    req_obj: 0,
    contract: 0,
    err: 0,
    events: 0,
    status0: 0,
    status1: 0,
    detach: 0,
    rx_msgs: 0,
    rx_hdr: [0; 3],
    vcc1_mv: 0,
    vcc2_mv: 0,
    rev: REV20,
    tx_pend: false,
    last_rdo: 0,
    sink: false,
    rx_full: false,
    sop1: false,
    asked: false,
    err_reg: 0,
    poll_tick: 0,
    req_tick: 0,
    cap_ask_tick: 0,
    vcc_tick: 0,
    pps_tick: 0,
}));

#[inline(always)]
fn pd() -> &'static mut Pd {
    unsafe { &mut *PD.0.get() }
}

/* ---- PD capture (KM003C PdPacket attribute 0x0010) ----
 *
 * Every PD message the engine sends/receives, plus connect/disconnect, is
 * formatted into a KM003C PdPacket EVENT and pushed to a ring. The powerz
 * ISR drains it into GetData(ATT_PD) replies (powerz.rs). SPSC: the producer
 * is thread mode (rx_frame/send_frame/attach/detach all run in service() or a
 * verb — one context), the consumer is the USBD_LP ISR (drain_pd_packet).
 * head advances only in the producer, tail only in the consumer, and a slot
 * is written before head publishes it and read only while tail < head, so the
 * two never touch the same slot. Ring full = drop newest (capture is
 * best-effort).
 *
 * Event wire format (KM003C, github.com/okhsunrog/km003c-protocol-research):
 *   connect/disconnect (6 B): 0x45, ts LE24, 0x00, code(0x21/0x22)
 *   wrapped PD msg (6+N B): size_flag=0x80|(N+5), ts LE32, sop=0, N wire bytes
 * byte0 bit7 distinguishes them (0x45 clear, size_flag set). N ≤ 30
 * (2-byte header + up to 7 data objects). */
const CAP_SLOTS: usize = 8;
const CAP_SLOT_LEN: usize = 40; /* 6 header + 30 wire, rounded */
struct CapRing {
    bufs: core::cell::UnsafeCell<[[u8; CAP_SLOT_LEN]; CAP_SLOTS]>,
    lens: core::cell::UnsafeCell<[u8; CAP_SLOTS]>,
    head: AtomicU32,
    tail: AtomicU32,
}
unsafe impl Sync for CapRing {}
/* .hibss: high-RAM, manually zeroed at main() start (memory.x) — the ring
 * relies on head/tail starting 0, and nothing runs pd code before that. */
#[link_section = ".hibss"]
static CAP: CapRing = CapRing {
    bufs: core::cell::UnsafeCell::new([[0; CAP_SLOT_LEN]; CAP_SLOTS]),
    lens: core::cell::UnsafeCell::new([0; CAP_SLOTS]),
    head: AtomicU32::new(0),
    tail: AtomicU32::new(0),
};

/* Preamble fields (KM003C PD Measurement Block), published by service() in
 * thread mode and read by the ISR — plain atomics, staleness harmless. */
static PD_TS_MS: AtomicU32 = AtomicU32::new(0);
static PD_VBUS_MV: AtomicU32 = AtomicU32::new(0);
static PD_IBUS_MA: AtomicU32 = AtomicU32::new(0); /* i16 stored as u32 */
static PD_CC1_MV: AtomicU32 = AtomicU32::new(0);
static PD_CC2_MV: AtomicU32 = AtomicU32::new(0);

fn now_ms() -> u32 {
    ina::ticks().wrapping_mul(4) /* 4 ms per sampler tick */
}

/// Push a pre-formatted event of `len` bytes into the capture ring (thread
/// mode). Full ring drops the event.
fn cap_push(bytes: &[u8]) {
    let h = CAP.head.load(Ordering::Relaxed);
    let t = CAP.tail.load(Ordering::Acquire);
    if h.wrapping_sub(t) >= CAP_SLOTS as u32 {
        return; /* full: drop newest */
    }
    let i = h as usize % CAP_SLOTS;
    let n = bytes.len().min(CAP_SLOT_LEN);
    unsafe {
        let slot = &mut (*CAP.bufs.get())[i];
        slot[..n].copy_from_slice(&bytes[..n]);
        (*CAP.lens.get())[i] = n as u8;
    }
    CAP.head.store(h.wrapping_add(1), Ordering::Release);
}

/// Capture one PD wire message (header + data objects) as a wrapped event;
/// `sop` 0 = SOP, 1 = SOP', 2 = SOP''.
fn cap_msg(hdr: u16, objs: &[u8], sop: u8) {
    crate::pdinfo::on_msg(hdr, objs, sop);
    let wire = 2 + objs.len();
    let mut e = [0u8; CAP_SLOT_LEN];
    e[0] = EV_WRAPPED | (wire as u8 + EV_WRAPPED_SIZE_BIAS);
    e[EV_WRAPPED_TS..EV_WRAPPED_TS + 4].copy_from_slice(&now_ms().to_le_bytes());
    e[EV_WRAPPED_SOP] = sop;
    e[EV_WRAPPED_WIRE..EV_WRAPPED_WIRE + 2].copy_from_slice(&hdr.to_le_bytes());
    e[EV_WRAPPED_WIRE + 2..EV_WRAPPED_WIRE + 2 + objs.len()].copy_from_slice(objs);
    cap_push(&e[..EV_WRAPPED_WIRE + wire]);
}

/// Capture a connect (0x21) / disconnect (0x22) / CAP_RX_* event.
fn cap_conn(code: u8) {
    crate::pdinfo::on_event(code);
    let ms = now_ms();
    let t = ms.to_le_bytes(); /* LE24 timestamp */
    let e: [u8; EV_CONNECTION_LEN] = [EV_CONNECTION, t[0], t[1], t[2], 0, code];
    cap_push(&e);
}

/// Build a KM003C PdPacket payload into `out` (ISR, single consumer): the
/// 12-byte measurement preamble, then as many whole events as fit. Returns
/// the payload length. Drains the events it emits.
pub fn drain_pd_packet(out: &mut [u8]) -> usize {
    if out.len() < PRE_LEN {
        return 0;
    }
    out[PRE_TS..PRE_TS + 4].copy_from_slice(&PD_TS_MS.load(Ordering::Relaxed).to_le_bytes());
    out[PRE_VBUS..PRE_VBUS + 2].copy_from_slice(&(PD_VBUS_MV.load(Ordering::Relaxed) as u16).to_le_bytes());
    out[PRE_IBUS..PRE_IBUS + 2].copy_from_slice(&(PD_IBUS_MA.load(Ordering::Relaxed) as u16).to_le_bytes());
    out[PRE_CC1..PRE_CC1 + 2].copy_from_slice(&(PD_CC1_MV.load(Ordering::Relaxed) as u16).to_le_bytes());
    out[PRE_CC2..PRE_CC2 + 2].copy_from_slice(&(PD_CC2_MV.load(Ordering::Relaxed) as u16).to_le_bytes());
    let mut len = PRE_LEN;
    loop {
        let t = CAP.tail.load(Ordering::Relaxed);
        let h = CAP.head.load(Ordering::Acquire);
        if h == t {
            break;
        }
        let i = t as usize % CAP_SLOTS;
        let n = unsafe { (*CAP.lens.get())[i] } as usize;
        if len + n > out.len() {
            break; /* next event doesn't fit this packet; leave it queued */
        }
        let slot = unsafe { &(*CAP.bufs.get())[i] };
        out[len..len + n].copy_from_slice(&slot[..n]);
        len += n;
        CAP.tail.store(t.wrapping_add(1), Ordering::Release);
    }
    len
}

/* ---- I²C helpers (bus rule: every transaction inside ina::pause) ---- */

fn stage(e: i2c::I2cErr) -> u8 {
    e.stage
}

fn failed(reg: u8, e: i2c::I2cErr) -> u8 {
    pd().err_reg = reg;
    stage(e)
}

fn wr(reg: u8, val: u8) -> Result<(), u8> {
    ina::pause(|| i2c::transact(ADDR, &[reg, val], &mut [])).map_err(|e| failed(reg, e))
}

fn wr_buf(buf: &[u8]) -> Result<(), u8> {
    ina::pause(|| i2c::transact(ADDR, buf, &mut [])).map_err(|e| failed(buf[0], e))
}

fn rd(reg: u8, buf: &mut [u8]) -> Result<(), u8> {
    ina::pause(|| i2c::transact(ADDR, &[reg], buf)).map_err(|e| failed(reg, e))
}

/* ---- verb entry points (thread mode) ---- */

/// `pdctl 1`: reset + configure the FUSB302 as a presenting sink and start
/// the attach poll. Safe with or without a source already plugged.
#[inline(never)]
pub fn arm() -> Result<(), u8> {
    arm_as(ST_RD_CYCLE)
}

/// `pdctl 3`: listen-only sniffer for a source and a sink negotiating
/// THROUGH the meter (IN → OUT): no Rd, no hardware GoodCRC, never
/// transmits. The receiver follows whichever CC carries the Rp/Rd divider;
/// every SOP frame from either side lands in the KM003C capture ring.
#[inline(never)]
pub fn sniff() -> Result<(), u8> {
    arm_as(ST_SNIFF)
}

fn arm_as(state: u8) -> Result<(), u8> {
    let p = pd();
    p.state = ST_OFF; /* stop service() touching the chip mid-config */
    wr(R_RESET, RST_SW)?;
    crate::board::delay_ms(2);
    wr(R_POWER, POWER_ALL)?;
    wr(R_CONTROL3, C3_RETRIES)?;
    let sniff = state == ST_SNIFF;
    wr(R_MASK, if sniff { MASK_SNIFF } else { MASK_ACTIVITY_ONLY })?;
    wr(R_MASKA, if sniff { MASKA_SNIFF } else { MASKA_SINK })?;
    wr(R_MASKB, if sniff { MASKB_SNIFF } else { MASKB_SINK })?;
    wr(R_CONTROL0, C0_TX_FLUSH)?;
    wr(R_SLICE, SLICE)?;
    int_n_init();
    wr(R_CONTROL1, C1_RX_FLUSH)?;
    /* Drop Rd first (service restores it after ~600 ms): a source plugged
     * before we armed has exhausted its Source_Capabilities retries against
     * the chip's dead-Rd defaults and stopped listening for PD — only a
     * detach/re-attach makes it start over (bench-hit with an Apple 20 W).
     * The cycle costs nothing when no source is attached yet. */
    wr(R_SWITCHES0, SW0_NONE)?;
    wr(R_SWITCHES1, sw1_rev(REV20))?;
    crate::pdinfo::reset();
    *p = Pd {
        state,
        cc: 0,
        msg_id: 0,
        caps_n: 0,
        caps: [0; 7],
        req_obj: 0,
        contract: 0,
        err: 0,
        events: 0,
        status0: 0,
        status1: 0,
        detach: 0,
        rx_msgs: 0,
        rx_hdr: [0; 3],
        vcc1_mv: 0,
        vcc2_mv: 0,
        rev: REV20,
        tx_pend: false,
        last_rdo: 0,
        sink: false,
        rx_full: false,
        sop1: false,
        asked: false,
        err_reg: 0,
        poll_tick: ina::ticks(),
        req_tick: ina::ticks(), /* ST_RD_CYCLE start */
        cap_ask_tick: 0,
        vcc_tick: 0,
        pps_tick: 0,
    };
    Ok(())
}

/// `pdctl 0`: SW_RES and stop. (The chip's reset default still presents Rd
/// on both CC — same as before any firmware ran.)
#[inline(never)]
pub fn off() -> Result<(), u8> {
    pd().state = ST_OFF;
    wr(R_RESET, RST_SW)
}

/// `pdsta` payload: 22 bytes of engine state.
#[inline(never)]
pub fn status(out: &mut [u8; pdsta::LEN]) {
    let p = pd();
    out[pdsta::STATE] = p.state;
    out[pdsta::CC] = p.cc;
    out[pdsta::CAPS] = p.caps_n;
    out[pdsta::CONTRACT] = if p.state == ST_SNIFF { p.sink as u8 } else { p.contract };
    out[pdsta::REQ_OBJ] = p.req_obj;
    out[pdsta::ERR] = p.err;
    out[pdsta::EVENTS] = p.events;
    out[pdsta::STATUS0] = p.status0;
    out[pdsta::STATUS1] = p.status1;
    out[pdsta::MSG_ID] = p.msg_id;
    out[pdsta::RX_COUNT..pdsta::RX_COUNT + 2].copy_from_slice(&p.rx_msgs.to_le_bytes());
    out[pdsta::DETACH] = p.detach;
    out[pdsta::REV] = p.rev;
    for (i, h) in p.rx_hdr.iter().enumerate() {
        out[pdsta::RX_HDRS + 2 * i..pdsta::RX_HDRS + 2 * i + 2].copy_from_slice(&h.to_le_bytes());
    }
    out[pdsta::FLAGS] = if p.tx_pend { FLAG_TX_PEND } else { 0 }
        | if p.sop1 { FLAG_CABLE_LISTEN } else { 0 }
        | if p.asked { FLAG_ASKED } else { 0 };
    out[pdsta::ERR_REG] = p.err_reg;
}

pub const MODE_OFF: u8 = 0;
pub const MODE_SINK: u8 = 1;
pub const MODE_SNIFF: u8 = 2;

/// What the PD screen shows about the link (pdinfo.rs).
pub struct Link {
    pub mode: u8,   /* MODE_* */
    pub src: bool,  /* a source's Rp is seen (sniff: CC locked; sink: attached) */
    pub snk: bool,  /* an Rd loads the CC (sniff: a sink attached; sink: ours) */
    pub cc: u8,     /* active CC, 0 none */
    pub cc_mv: u16, /* its voltage from the ~500 ms MDAC sweep */
}

pub fn link() -> Link {
    let p = pd();
    let mode = match p.state {
        ST_OFF => MODE_OFF,
        ST_SNIFF => MODE_SNIFF,
        _ => MODE_SINK,
    };
    let src = match mode {
        MODE_SNIFF => p.cc != 0,
        _ => p.state >= ST_LISTEN && p.state != ST_RD_CYCLE,
    };
    Link {
        mode,
        src,
        snk: if mode == MODE_SNIFF { p.sink } else { src },
        cc: if src { p.cc } else { 0 },
        cc_mv: if p.cc == 2 { p.vcc2_mv } else { p.vcc1_mv },
    }
}

/// Engine off (`pdctl 0` / never armed).
pub fn is_off() -> bool {
    pd().state == ST_OFF
}

/// `pdcap`: copy the stored source PDOs; returns count.
#[inline(never)]
pub fn caps(out: &mut [u8; 28]) -> u8 {
    let p = pd();
    for i in 0..p.caps_n as usize {
        out[4 * i..4 * i + 4].copy_from_slice(&p.caps[i].to_le_bytes());
    }
    p.caps_n
}

/// `pdreq <n> [mv] [ma]`: request source PDO n (1-based). Fixed-supply PDOs
/// ignore the setpoint; a PPS APDO requires `mv` (and takes `ma`, default
/// its max) and a PD 3.0 source. mv/ma == 0 means "not given".
#[inline(never)]
pub fn request(idx: u8, mv: u16, ma: u16) -> Result<(), u8> {
    let p = pd();
    if p.state < ST_LISTEN || idx == 0 || idx > p.caps_n {
        return Err(E_BAD_ARGS);
    }
    let pdo = p.caps[idx as usize - 1];
    p.err = 0;
    match PDO_TYPE.get(pdo) {
        PDO_FIXED => send_rdo(p, idx, rdo_fixed(idx, pdo)),
        PDO_APDO if APDO_TYPE.get(pdo) == APDO_PPS => {
            /* PPS APDO (rev 3.0 only) */
            if p.rev < REV30 {
                return Err(E_PDO_KIND);
            }
            let min_mv = PPS_MIN_VOLTAGE.val(pdo);
            let max_mv = PPS_MAX_VOLTAGE.val(pdo);
            let max_ma = PPS_MAX_CURRENT.val(pdo);
            let ma = if ma == 0 { max_ma as u16 } else { ma };
            if (mv as u32) < min_mv || mv as u32 > max_mv || ma as u32 > max_ma {
                return Err(E_BAD_PPS_SETPOINT);
            }
            send_rdo(
                p,
                idx,
                RDO_OBJECT_POSITION.put(idx as u32)
                    | RDO_NO_USB_SUSPEND.put(1)
                    | RDO_PPS_VOLTAGE.put(mv as u32 / RDO_PPS_VOLTAGE.unit)
                    | RDO_APDO_CURRENT.put(ma as u32 / RDO_APDO_CURRENT.unit),
            )
        }
        _ => Err(E_PDO_KIND), /* variable / battery / AVS: not supported */
    }
}

/// A raw PD message (header + data objects, LE) kept for the host.
struct Frame {
    len: u8,
    b: [u8; MAX_MSG_LEN],
}

/// The source's answer to the last pdask / pdsnd request (`pdans`).
/// Thread-exclusive like PD.
static ANSWER: Racy<Frame> = Racy(core::cell::UnsafeCell::new(Frame { len: 0, b: [0; MAX_MSG_LEN] }));

#[inline(always)]
fn answer_buf() -> &'static mut Frame {
    unsafe { &mut *ANSWER.0.get() }
}

/// `pdsnd` (dev-pd): send one host-built SOP message from a contracted sink.
/// `t` = message type (bit 7: extended), `objs` = its data objects (LE, ≤ 7;
/// an extended message's first object starts with its extended header);
/// the header's rev, roles and message id are ours. The source's next
/// message is its answer (kept for `pdans`, captured, never
/// Not_Supported'ed).
#[cfg(feature = "dev-pd")]
#[inline(never)]
pub fn send_raw(t: u8, objs: &[u8]) -> Result<(), u8> {
    send_request(t, objs)
}

/// `pdask`: like `pdsnd`, limited to read-only information requests —
/// Get_Sink_Cap and the PD 3.x Get_* control messages, Get_Manufacturer_Info,
/// a Discover Identity REQ. Never a Request: it can't change the contract.
#[inline(never)]
pub fn ask(t: u8, objs: &[u8]) -> Result<(), u8> {
    let ty = HDR_MSG_TYPE.get(t as u32);
    let ok = if t & PDSND_EXTENDED != 0 {
        ty == EXT_GET_MANUFACTURER_INFO && objs.len() == 4
    } else if objs.is_empty() {
        matches!(
            ty,
            CTRL_GET_SINK_CAP
                | CTRL_GET_SOURCE_CAP_EXT
                | CTRL_GET_STATUS
                | CTRL_GET_PPS_STATUS
                | CTRL_GET_COUNTRY_CODES
                | CTRL_GET_SINK_CAP_EXT
                | CTRL_GET_SOURCE_INFO
                | CTRL_GET_REVISION
        )
    } else {
        let h = objs.get(..4).map_or(0, |b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]));
        ty == DATA_VENDOR_DEFINED
            && objs.len() == 4
            && VDM_SVID.get(h) == SVID_PD
            && VDM_STRUCTURED.get(h) != 0
            && VDM_COMMAND_TYPE.get(h) == VDM_REQ
            && VDM_COMMAND.get(h) == CMD_DISCOVER_IDENTITY
    };
    if !ok {
        return Err(E_BAD_ARGS);
    }
    send_request(t, objs)
}

fn send_request(t: u8, objs: &[u8]) -> Result<(), u8> {
    let p = pd();
    if p.state != ST_CONTRACT || p.tx_pend || objs.len() % 4 != 0 || objs.len() > 4 * MAX_DATA_OBJECTS {
        return Err(E_BAD_ARGS);
    }
    p.asked = true;
    p.events = 0; /* pdsta then shows what this exchange raised */
    answer_buf().len = 0;
    send_frame(p, HDR_MSG_TYPE.get(t as u32), t & PDSND_EXTENDED != 0, objs)
}

/// `pdans`: copy the stored answer frame; returns its length (0 = none yet).
#[inline(never)]
pub fn answer_frame(out: &mut [u8; MAX_MSG_LEN]) -> u8 {
    let a = answer_buf();
    out.copy_from_slice(&a.b);
    a.len
}

/// `pdctl 2`: send a PD Hard Reset (Control3 SEND_HARD_RESET); while
/// sniffing, a third-party one that makes the pair renegotiate. The source
/// drops VBUS, forgets the negotiated contract AND spec rev, and restarts
/// from Source_Capabilities at its highest rev — also the way to unpark a
/// source stuck at rev 2.0 from an earlier failed exchange.
#[inline(never)]
pub fn hard_reset() -> Result<(), u8> {
    let p = pd();
    if p.state == ST_SNIFF {
        /* a third party may send Hard Reset (an ordered set — no message
         * id): both ends reset, the source cycles VBUS and renegotiates with
         * the sink — a re-plug without unplugging. TX driver on the locked
         * CC only until I_HARDSENT (sniff_poll turns it off again). */
        if p.cc == 0 {
            return Err(E_BAD_ARGS);
        }
        let txcc = txcc(p);
        wr(R_SWITCHES1, sw1_rev(REV20) | txcc)?;
        /* the transmitter only drives after a PD-logic reset with TXCC set
         * (bench: without it HARDSENT fires but nothing reaches the wire) */
        wr(R_RESET, RST_PD)?;
        wr(R_CONTROL3, C3_RETRIES | C3_SEND_HARD_RESET)?;
        p.tx_pend = true;
        p.req_tick = ina::ticks();
        return Ok(());
    }
    if p.state < ST_LISTEN {
        return Err(E_BAD_ARGS);
    }
    wr(R_CONTROL3, C3_RETRIES | C3_SEND_HARD_RESET)?;
    p.contract = 0;
    p.req_obj = 0;
    p.state = ST_LISTEN;
    /* IA_HARDSENT lands on the next poll and runs the full PD-reset path */
    Ok(())
}

/// Latest CC pin voltages in mV (0 when the engine is off / nothing
/// measured yet) — feeds the powerz Vcc1/Vcc2 fields. Thread mode only.
pub fn cc_mv() -> (u16, u16) {
    let p = pd();
    (p.vcc1_mv, p.vcc2_mv)
}

/// Binary-search the 6-bit MDAC comparator against the currently selected
/// CC pin. Returns the highest MDAC code whose threshold ((code+1)·42 mV)
/// the pin still exceeds; 0 covers everything below 84 mV (reported as 0 V —
/// fine for a floating CC).
fn mdac_search() -> Result<u8, u8> {
    let mut code: u8 = 0;
    for bit in (0..6).rev() {
        let t = code | 1 << bit;
        wr(R_MEASURE, t)?;
        cortex_m::asm::delay(5_000); /* ~100 us comparator settle */
        let mut s = [0u8; 1];
        rd(R_STATUS0, &mut s)?;
        if s[0] & S0_COMP != 0 {
            code = t;
        }
    }
    Ok(code)
}

/// Measure the CC pin voltage(s) via the MDAC. The MEAS_CC switch ALSO
/// feeds the BMC receiver, so while a link is up (cc known) ONLY the
/// already-selected active CC is measured — flipping the switch to the
/// idle pin eats any message arriving meanwhile (bench-hit: periodic
/// contract drops every few sweeps). Both pins are swept only pre-attach.
/// VCONN above the DAC range clamps to ~2.7 V.
fn vcc_sweep(p: &mut Pd) -> Result<(), u8> {
    let mv = |m: u8| -> u16 {
        if m == 0 {
            0
        } else {
            (m as u16 + 1) * 42 + 21 /* midpoint of the found step */
        }
    };
    if p.cc != 0 {
        let m = mdac_search()?;
        if p.cc == 1 {
            p.vcc1_mv = mv(m);
            p.vcc2_mv = 0;
        } else {
            p.vcc2_mv = mv(m);
            p.vcc1_mv = 0;
        }
        return Ok(());
    }
    let pdwn = if p.state == ST_SNIFF { 0 } else { SW0_PDWN };
    wr(R_SWITCHES0, pdwn | SW0_MEAS_CC1)?;
    let m1 = mdac_search()?;
    wr(R_SWITCHES0, pdwn | SW0_MEAS_CC2)?;
    let m2 = mdac_search()?;
    wr(R_SWITCHES0, pdwn)?;
    p.vcc1_mv = mv(m1);
    p.vcc2_mv = mv(m2);
    Ok(())
}

/* ---- the engine (called from the main loop every pass) ---- */

/// FUSB302 INT_N on PA15: open-drain, active-low — input with pull-up.
fn int_n_init() {
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    gpioa.ctl().modify(|_, w| w.ctl15().input());
    gpioa.pud().modify(|_, w| w.pud15().pull_up());
}

#[inline(always)]
fn int_n_low() -> bool {
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    gpioa.istat().read().bits() & (1 << INT_N_PIN) == 0
}

/// The idle checks are inlined into the (fast-flash) main loop: a cold call
/// into slow flash costs ~90 µs even when it returns at once. The engine
/// body runs only when a poll is due: at once when INT_N (PA15) is asserted
/// while attached/sniffing, else on the backup interval.
#[inline(always)]
pub fn service() {
    let p = pd();
    if p.state == ST_OFF {
        return;
    }
    let attached = p.state >= ST_LISTEN && p.state != ST_RD_CYCLE;
    if attached && int_n_low() {
        service_due();
        return;
    }
    let interval = if attached { BACKUP_POLL } else { 12 };
    if ina::ticks().wrapping_sub(p.poll_tick) < interval {
        return;
    }
    service_due();
}

#[inline(never)]
fn service_due() {
    let p = pd();
    let now = ina::ticks();
    p.poll_tick = now;
    let r = match p.state {
        ST_RD_CYCLE => {
            /* req_tick = when the cycle started (arm) */
            if now.wrapping_sub(p.req_tick) >= 150 {
                let r = wr(R_SWITCHES0, SW0_PDWN);
                if r.is_ok() {
                    p.state = ST_WAIT_ATTACH;
                }
                r
            } else {
                Ok(())
            }
        }
        ST_WAIT_ATTACH => attach_poll(p),
        ST_SNIFF => sniff_poll(p),
        _ => active_poll(p, now),
    };
    if let Err(e) = r {
        p.err = e; /* sticky I²C diagnosis; keep polling — self-heals */
    }
    /* CC-voltage sweep for powerz Vcc1/Vcc2, every ~500 ms while armed
     * (never during the Rd-cycle — the sweep re-presents the pulldowns) */
    if p.state != ST_OFF && p.state != ST_RD_CYCLE && now.wrapping_sub(p.vcc_tick) >= 125 {
        p.vcc_tick = now;
        if let Err(e) = vcc_sweep(p) {
            p.err = e;
        }
        /* the sweep moved the MDAC: park it back on the sniff threshold */
        if p.state == ST_SNIFF && p.cc != 0 {
            if let Err(e) = wr(R_MEASURE, SNIFF_OPEN_MDAC) {
                p.err = e;
            }
        }
    }
    /* publish the KM003C PD-capture preamble (measurement block) */
    let m = ina::snapshot();
    PD_TS_MS.store(now_ms(), Ordering::Relaxed);
    PD_VBUS_MV.store(m.v_1e4 / 10, Ordering::Relaxed); /* 1e-4 V -> mV */
    PD_IBUS_MA.store((m.i_1e5 / 100) as u16 as u32, Ordering::Relaxed); /* 1e-5 A -> mA */
    PD_CC1_MV.store(p.vcc1_mv as u32, Ordering::Relaxed);
    PD_CC2_MV.store(p.vcc2_mv as u32, Ordering::Relaxed);

    /* PPS keepalive: a PPS contract dies (source hard-reset) unless the sink
     * re-Requests within ~10 s — re-send the standing RDO every 8 s */
    if p.state == ST_CONTRACT
        && p.contract > 0
        && PDO_TYPE.get(p.caps[p.contract as usize - 1]) == PDO_APDO
        && now.wrapping_sub(p.pps_tick) >= 2000
    {
        let (idx, rdo) = (p.req_obj, p.last_rdo);
        if let Err(e) = send_rdo(p, idx, rdo) {
            p.err = e;
        }
    }
}

/// Measure CC1 then CC2 through the comparator; on Rp detection lock the
/// orientation and bring BMC up.
#[inline(never)]
fn attach_poll(p: &mut Pd) -> Result<(), u8> {
    wr(R_SWITCHES0, SW0_PDWN | SW0_MEAS_CC1)?;
    crate::board::delay_ms(1); /* comparator settle */
    let mut s = [0u8; 1];
    rd(R_STATUS0, &mut s)?;
    let bc1 = s[0] & S0_BC_LVL;
    wr(R_SWITCHES0, SW0_PDWN | SW0_MEAS_CC2)?;
    crate::board::delay_ms(1);
    rd(R_STATUS0, &mut s)?;
    let bc2 = s[0] & S0_BC_LVL;
    p.status0 = s[0];
    if bc1 == 0 && bc2 == 0 {
        wr(R_SWITCHES0, SW0_PDWN)?;
        return Ok(());
    }
    let cc1 = bc1 >= bc2;
    p.cc = if cc1 { 1 } else { 2 };
    wr(
        R_SWITCHES0,
        SW0_PDWN | if cc1 { SW0_MEAS_CC1 } else { SW0_MEAS_CC2 },
    )?;
    /* Lead with rev 2.0 in the hardware GoodCRC: every source accepts it,
     * while a 3.0-stamped GoodCRC to a 2.0 message makes some sources
     * (bench-hit: Apple 96 W, which talks 2.0 even after a hard reset)
     * soft/hard-reset the link. If a source's caps arrive at 3.0 the
     * handler upgrades Switches1 + our headers before the Request. */
    p.rev = REV20;
    sop1_start(p)?;
    wr(R_RESET, RST_PD)?;
    p.msg_id = 0;
    p.detach = 0;
    p.state = ST_LISTEN;
    p.cap_ask_tick = ina::ticks();
    cap_conn(CAP_CONNECT);
    Ok(())
}

/// Sniffer poll. Unlocked (cc 0): the ~500 ms sweep in service_due measures
/// both CC pins; the higher one above ~0.2 V (a source's Rp) gets the
/// receiver, and the MDAC is parked at SNIFF_OPEN_MDAC. Locked: Status0.COMP
/// says whether a sink's Rd loads the CC — two agreeing quiet polls (no BMC
/// activity) emit the capture's CONNECT / DISCONNECT; every frame goes to the
/// capture ring; an RX-FIFO-full edge emits a CAP_RX_FULL event; a hard reset
/// re-arms the PD logic; the CC falling below ~0.2 V (source gone) unlocks.
#[inline(never)]
fn sniff_poll(p: &mut Pd) -> Result<(), u8> {
    if p.cc == 0 {
        let (v1, v2) = (p.vcc1_mv, p.vcc2_mv);
        if v1.max(v2) < 200 {
            return Ok(());
        }
        let cc1 = v1 >= v2;
        p.cc = if cc1 { 1 } else { 2 };
        wr(R_SWITCHES0, if cc1 { SW0_MEAS_CC1 } else { SW0_MEAS_CC2 })?;
        wr(R_SWITCHES1, sw1_rev(REV20))?; /* no AUTO_CRC, no TXCC */
        wr(R_MEASURE, SNIFF_OPEN_MDAC)?;
        rx_flush(p)?;
        wr(R_RESET, RST_PD)?;
        p.sink = false;
        p.detach = 0;
        return Ok(());
    }
    if (if p.cc == 1 { p.vcc1_mv } else { p.vcc2_mv }) < 200 {
        if p.sink {
            cap_conn(CAP_DISCONNECT);
        }
        crate::pdinfo::source_gone();
        p.cc = 0;
        p.sink = false;
        return Ok(());
    }
    let mut st = [0u8; 7];
    rd(R_STATUS0A, &mut st)?;
    let (inta, s0, s1, irq) = (st[2], st[4], st[5], st[6]);
    p.events |= inta;
    p.status0 = s0;
    p.status1 = s1;
    /* a packet was CRC-checked and failed (the FIFO only ever gets good
     * ones). Coalesced checks between polls report only the last packet. */
    if irq & IR_CRC_CHK != 0 && s0 & S0_CRC_CHK == 0 {
        cap_conn(CAP_BAD_CRC);
    }
    /* our own hard reset (hard_reset) went out: transmitter off again */
    if p.tx_pend && (inta & IA_HARDSENT != 0 || ina::ticks().wrapping_sub(p.req_tick) > 25) {
        p.tx_pend = false;
        wr(R_SWITCHES1, sw1_rev(REV20))?;
        rx_flush(p)?;
        wr(R_RESET, RST_PD)?;
        if inta & IA_HARDSENT != 0 {
            cap_conn(CAP_HARD_RESET);
        }
    }
    if s0 & S0_ACTIVITY == 0 {
        let loaded = s0 & S0_COMP == 0;
        if loaded != p.sink {
            p.detach += 1;
            if p.detach >= 2 {
                p.sink = loaded;
                p.detach = 0;
                cap_conn(if loaded { CAP_CONNECT } else { CAP_DISCONNECT });
            }
        } else {
            p.detach = 0;
        }
    }
    let full = s1 & S1_RX_FULL != 0;
    if full {
        p.err = E_RX_FULL;
        if !p.rx_full {
            cap_conn(CAP_RX_FULL);
        }
    }
    p.rx_full = full;
    if inta & IA_HARDRST != 0 {
        cap_conn(CAP_HARD_RESET);
        rx_flush(p)?;
        wr(R_RESET, RST_PD)?;
        return Ok(());
    }
    for _ in 0..8 {
        wait_cc_idle()?;
        if !rx_ready()? {
            break;
        }
        rx_capture(p)?;
    }
    Ok(())
}

/// Sniff: let the frame on the wire finish before touching the FIFO — up to
/// ~2 ms of Status0 polls (≈ 100 µs each at 400 kHz) for ACTIVITY to clear,
/// then go on regardless (a retry storm keeps the line busy; the FIFO must
/// still drain).
fn wait_cc_idle() -> Result<(), u8> {
    for _ in 0..20 {
        let mut s = [0u8; 1];
        rd(R_STATUS0, &mut s)?;
        if s[0] & S0_ACTIVITY == 0 {
            break;
        }
    }
    Ok(())
}

/// Whether the RX FIFO holds a frame. (Waiting for Status0.ACTIVITY to
/// clear as well was tried: under dense traffic — Source_Cap retry storms —
/// it held off draining until the FIFO overflowed and dropped more frames.)
fn rx_ready() -> Result<bool, u8> {
    let mut s = [0u8; 1];
    rd(R_STATUS0 + 1, &mut s)?; /* Status1 */
    Ok(s[0] & S1_RX_EMPTY == 0)
}

/// Flush the RX FIFO, keeping SOP'/SOP'' reception on while sniffing or in
/// the sink's listen-only phase.
fn rx_flush(p: &Pd) -> Result<(), u8> {
    let ensop = if p.state == ST_SNIFF || p.sop1 { C1_ENSOP12 } else { 0 };
    wr(R_CONTROL1, C1_RX_FLUSH | ensop)
}

fn txcc(p: &Pd) -> u8 {
    if p.cc == 1 {
        SW1_TXCC1
    } else {
        SW1_TXCC2
    }
}

/// Sink attach / hard reset: listen-only until the source's first
/// Source_Capabilities. A source probes the cable (SOP' Discover Identity)
/// before its first caps; with SOP' reception on, AUTO_CRC would GoodCRC
/// that probe too ("back to the relevant SOP*" — datasheet) and collide with
/// the e-marker's own GoodCRC, so the hardware acknowledges nothing until
/// the cable exchange is over.
fn sop1_start(p: &mut Pd) -> Result<(), u8> {
    wr(R_SWITCHES1, sw1_rev(p.rev) | txcc(p))?;
    p.sop1 = true;
    rx_flush(p)
}

/// End the listen-only phase: AUTO_CRC on, SOP' off. AUTO_CRC goes on
/// BEFORE the flush, so the queued (never acknowledged) caps are dropped and
/// the source's next, acknowledged, caps start the negotiation.
fn sop1_end(p: &mut Pd) -> Result<(), u8> {
    wr(R_SWITCHES1, sw1_rev(p.rev) | SW1_AUTO_CRC | txcc(p))?;
    p.sop1 = false;
    rx_flush(p)
}

/// Read one RX FIFO frame (token, header, objects, CRC) into the capture
/// ring and the pdsta counters. Returns (header, SOP kind, objects + CRC);
/// an unknown token flushes the FIFO (resync) and returns header 0.
fn rx_capture(p: &mut Pd) -> Result<(u16, u8, [u8; 32]), u8> {
    let mut head = [0u8; 3];
    let mut body = [0u8; 32]; /* 7·4 objects + 4 CRC max */
    rd(R_FIFOS, &mut head)?;
    /* token class (Table 42): 111 SOP, 110 SOP', 101 SOP'' */
    let sop = match head[0] >> RX_TOKEN_CLASS_SHIFT {
        RX_TOKEN_SOP => SOP,
        RX_TOKEN_SOP1 => SOP1,
        RX_TOKEN_SOP2 => SOP2,
        RX_TOKEN_SOP1_DEBUG => SOP1_DEBUG,
        RX_TOKEN_SOP2_DEBUG => SOP2_DEBUG,
        _ => {
            rx_flush(p)?;
            cap_conn(CAP_RX_RESYNC);
            return Ok((0, 0, body));
        }
    };
    let hdr = u16::from_le_bytes([head[1], head[2]]);
    let nobj = HDR_NUM_OBJECTS.get(hdr.into()) as usize;
    rd(R_FIFOS, &mut body[..4 * nobj + 4])?; /* objects + CRC */
    p.rx_msgs = p.rx_msgs.wrapping_add(1);
    p.rx_hdr = [hdr, p.rx_hdr[0], p.rx_hdr[1]];
    cap_msg(hdr, &body[..4 * nobj], sop); /* KM003C PD capture */
    Ok((hdr, sop, body))
}

/// One poll while attached: status burst, event handling, RX drain,
/// timeouts, detach debounce.
#[inline(never)]
fn active_poll(p: &mut Pd, now: u32) -> Result<(), u8> {
    /* burst 0x3C..0x42: Status0a Status1a Interrupta Interruptb Status0
     * Status1 Interrupt (the Interrupt* reads clear the chip's latches —
     * this poll is their single consumer) */
    let mut st = [0u8; 7];
    rd(R_STATUS0A, &mut st)?;
    let (inta, s0, s1) = (st[2], st[4], st[5]);
    p.events |= inta;
    p.status0 = s0;
    p.status1 = s1;

    if inta & IA_TXSENT != 0 {
        tx_done(p);
    }
    if inta & IA_RETRYFAIL != 0 {
        p.tx_pend = false;
    }
    if inta & (IA_HARDRST | IA_HARDSENT) != 0 {
        /* hard reset on the wire: back to listening, source re-sends caps
         * (and re-probes the cable first: listen-only phase again) */
        sop1_start(p)?;
        wr(R_RESET, RST_PD)?;
        p.msg_id = 0;
        p.tx_pend = false;
        p.contract = 0;
        p.req_obj = 0;
        p.state = ST_LISTEN;
        p.cap_ask_tick = now;
        crate::pdinfo::on_event(CAP_HARD_RESET);
        return Ok(());
    }
    if inta & IA_RETRYFAIL != 0 && p.state == ST_WAIT_ACCEPT {
        p.err = E_RETRYFAIL;
        p.state = ST_LISTEN;
    }

    /* detach debounce: Rp gone from the measured CC (ignore polls where BMC
     * activity pulls the comparator around) */
    if s0 & (S0_ACTIVITY | S0_BC_LVL) == 0 {
        p.detach += 1;
        if p.detach >= 10 {
            p.cc = 0;
            p.caps_n = 0;
            p.contract = 0;
            p.req_obj = 0;
            p.state = ST_WAIT_ATTACH;
            wr(R_SWITCHES1, sw1_rev(REV20))?;
            cap_conn(CAP_DISCONNECT);
            crate::pdinfo::source_gone();
            return Ok(());
        }
    } else {
        p.detach = 0;
    }

    /* drain RX (bounded: at most 8 frames per poll — the INT_N wake only
     * comes per CC activity edge, so leave nothing queued) */
    for _ in 0..8 {
        if !rx_ready()? {
            break;
        }
        rx_frame(p, now)?;
    }

    /* response timeouts (ticks are 4 ms). Fresh ticks read, NOT the poll-entry
     * `now`: send_request stamps req_tick mid-poll (after ms of I2C), so
     * now - req_tick can wrap negative and fire the timeout instantly —
     * build 68 dropped a perfectly good Accept/PS_RDY exchange exactly so. */
    let t = ina::ticks();
    if p.state == ST_WAIT_ACCEPT && t.wrapping_sub(p.req_tick) > 25 {
        p.err = E_ACCEPT_TIMEOUT;
        p.state = ST_LISTEN;
    }
    if p.state == ST_WAIT_PSRDY && t.wrapping_sub(p.req_tick) > 150 {
        p.err = E_PSRDY_TIMEOUT;
        p.state = ST_LISTEN;
    }

    /* listen-only phase: no caps within ~1 s of attach / hard reset (the
     * source gave up before we armed) — become a normal sink */
    if p.sop1 && now.wrapping_sub(p.cap_ask_tick) > 250 {
        sop1_end(p)?;
    }
    /* no caps yet: the source may have exhausted its caps retries before we
     * armed — ask every ~500 ms */
    if p.state == ST_LISTEN && p.caps_n == 0 && !p.sop1 && now.wrapping_sub(p.cap_ask_tick) > 125 {
        p.cap_ask_tick = now;
        send(p, CTRL_GET_SOURCE_CAP, &[])?;
    }
    Ok(())
}

/// Read + dispatch one RX FIFO frame (token, 2-byte header, 4·n objects,
/// 4 CRC bytes — CRC already checked by hardware).
#[inline(never)]
fn rx_frame(p: &mut Pd, now: u32) -> Result<(), u8> {
    let (hdr, sop, body) = rx_capture(p)?;
    if hdr == 0 || sop != SOP {
        return Ok(()); /* FIFO skew (flushed) or not for us */
    }
    let h = u32::from(hdr);
    let nobj = HDR_NUM_OBJECTS.get(h) as usize;
    let mtype = HDR_MSG_TYPE.get(h); /* 5-bit in rev 3.0; bit 4 is 0 in 2.0 */
    let extended = HDR_EXTENDED.get(h) != 0;
    if p.sop1 {
        /* listen-only: nothing was acknowledged; the first caps end it */
        if !extended && nobj > 0 && mtype == DATA_SOURCE_CAP {
            sop1_end(p)?;
        }
        return Ok(());
    }
    /* the first message after a pdsnd request is its answer */
    let answer = p.asked && !(nobj == 0 && mtype == CTRL_GOODCRC);
    if answer {
        p.asked = false;
        let a = answer_buf();
        let n = 2 + 4 * nobj;
        a.b[..2].copy_from_slice(&hdr.to_le_bytes());
        a.b[2..n].copy_from_slice(&body[..4 * nobj]);
        a.len = n as u8;
    }

    if nobj == 0 {
        match mtype {
            CTRL_GOODCRC => {
                if HDR_MSG_ID.get(h) == p.msg_id as u32 {
                    tx_done(p);
                }
            }
            CTRL_ACCEPT => {
                if p.state == ST_WAIT_ACCEPT {
                    p.state = ST_WAIT_PSRDY;
                    p.req_tick = now;
                }
            }
            CTRL_REJECT | CTRL_WAIT => {
                if p.state == ST_WAIT_ACCEPT {
                    p.err = E_REJECT;
                    p.state = ST_LISTEN;
                }
            }
            CTRL_PS_RDY => {
                if p.state == ST_WAIT_PSRDY {
                    p.contract = p.req_obj;
                    p.state = ST_CONTRACT;
                }
            }
            CTRL_GET_SINK_CAP => {
                /* one fixed sink PDO: 5 V, 100 mA */
                if !p.tx_pend {
                    let pdo = PDO_TYPE.put(PDO_FIXED)
                        | FIXED_VOLTAGE.put(SINK_PDO_MV / FIXED_VOLTAGE.unit)
                        | FIXED_MAX_CURRENT.put(SINK_PDO_MA / FIXED_MAX_CURRENT.unit);
                    send_data(p, DATA_SINK_CAP, pdo)?;
                }
            }
            CTRL_SOFT_RESET => {
                p.msg_id = 0;
                send(p, CTRL_ACCEPT, &[])?;
                p.state = ST_LISTEN;
                p.cap_ask_tick = now;
            }
            CTRL_PING | CTRL_NOT_SUPPORTED => {}
            _ => {
                /* swaps, rev-3.0 extensions: a PD 3.0 port must answer
                 * Not_Supported; 2.0 lets GoodCRC (hardware) stand alone.
                 * tx_pend gate: one reply at a time — a burst (Apple sends
                 * ~7 VDMs) must not duplicate msg ids / flush TX mid-flight */
                if p.rev >= REV30 && !p.tx_pend && !answer {
                    send(p, CTRL_NOT_SUPPORTED, &[])?;
                }
            }
        }
        return Ok(());
    }

    if extended {
        /* extended message: its type numbers are a separate space (type 1
         * is Source_Capabilities_Extended, not Source_Capabilities) */
        if p.rev >= REV30 && !p.tx_pend && !answer {
            send(p, CTRL_NOT_SUPPORTED, &[])?;
        }
    } else if mtype == DATA_SOURCE_CAP {
        p.caps_n = nobj as u8;
        for i in 0..nobj {
            p.caps[i] = u32::from_le_bytes([
                body[4 * i],
                body[4 * i + 1],
                body[4 * i + 2],
                body[4 * i + 3],
            ]);
        }
        /* negotiate the spec rev down from the source's caps header (this
         * also retargets the hardware GoodCRC rev field) */
        let rev = (HDR_SPEC_REV.get(h) as u8).clamp(REV20, REV30);
        if rev != p.rev {
            p.rev = rev;
            wr(R_SWITCHES1, sw1_rev(rev) | SW1_AUTO_CRC | txcc(p))?;
        }
        /* a sink that GoodCRCs caps MUST Request within tSenderResponse or
         * the source hard-resets: immediately take PDO 1 (vSafe5V — no
         * output-voltage change). `pdreq` renegotiates from there. */
        send_rdo(p, 1, rdo_fixed(1, p.caps[0]))?;
    } else if p.rev >= REV30 && !p.tx_pend && !answer {
        /* VDMs, Alert, … — PD 3.0 requires Not_Supported (tx_pend-gated) */
        send(p, CTRL_NOT_SUPPORTED, &[])?;
    }
    Ok(())
}

/* ---- TX ---- */

/// Our message was acknowledged — on I_TXSENT or on the partner's GoodCRC
/// with our message id in the RX FIFO, whichever is seen first. The FUSB302
/// (a PD 2.0 part) reads the message type as 4 bits: it sends control type
/// 17 (Get_Source_Cap_Ext) as type 1, GoodCRC, which expects no
/// acknowledge — I_TXSENT never fires for it (bench: 4/4), though the
/// partner's GoodCRC arrives.
fn tx_done(p: &mut Pd) {
    if p.tx_pend {
        p.tx_pend = false;
        p.msg_id = (p.msg_id + 1) & 7;
    }
}

/// Send a Request carrying `rdo` for object position `idx` and enter
/// WAIT_ACCEPT. The RDO is remembered for the PPS keepalive.
#[inline(never)]
fn send_rdo(p: &mut Pd, idx: u8, rdo: u32) -> Result<(), u8> {
    send_data(p, DATA_REQUEST, rdo)?;
    p.req_obj = idx;
    p.last_rdo = rdo;
    p.req_tick = ina::ticks();
    p.pps_tick = p.req_tick;
    p.state = ST_WAIT_ACCEPT;
    Ok(())
}

/// Fixed-supply RDO for object position `idx`: operating = max current =
/// the PDO's limit (both in the same 10 mA units), no USB suspend.
fn rdo_fixed(idx: u8, pdo: u32) -> u32 {
    let i = FIXED_MAX_CURRENT.get(pdo);
    RDO_OBJECT_POSITION.put(idx as u32)
        | RDO_NO_USB_SUSPEND.put(1)
        | RDO_OPERATING_CURRENT.put(i)
        | RDO_MAX_CURRENT.put(i)
}

fn send_data(p: &mut Pd, mtype: u32, obj: u32) -> Result<(), u8> {
    let o = obj.to_le_bytes();
    send_frame(p, mtype, false, &o)
}

fn send(p: &mut Pd, mtype: u32, objs: &[u8]) -> Result<(), u8> {
    send_frame(p, mtype, false, objs)
}

/// Write one SOP message into the TX FIFO and fire it. Header: sink/UFP,
/// the negotiated spec rev, our rolling message id (incremented on I_TXSENT).
#[inline(never)]
fn send_frame(p: &mut Pd, mtype: u32, extended: bool, objs: &[u8]) -> Result<(), u8> {
    wr(R_CONTROL0, C0_TX_FLUSH)?; /* flush any stale TX (power/data role 0: sink/UFP) */
    let hdr = (HDR_MSG_TYPE.put(mtype)
        | HDR_EXTENDED.put(extended as u32)
        | HDR_SPEC_REV.put(p.rev as u32)
        | HDR_MSG_ID.put(p.msg_id as u32)
        | HDR_NUM_OBJECTS.put((objs.len() / 4) as u32)) as u16;
    let mut buf = [0u8; 40]; /* 6 tokens + header + 7 objects + 4 tail */
    buf[0] = R_FIFOS;
    buf[1..5].copy_from_slice(&[TK_SYNC1, TK_SYNC1, TK_SYNC1, TK_SYNC2]); /* SOP ordered set */
    let n = 2 + objs.len();
    buf[5] = TK_PACKSYM | n as u8;
    buf[6..8].copy_from_slice(&hdr.to_le_bytes());
    buf[8..8 + objs.len()].copy_from_slice(objs);
    let tail = 8 + objs.len();
    buf[tail..tail + 4].copy_from_slice(&[TK_JAM_CRC, TK_EOP, TK_TXOFF, TK_TXON]);
    wr_buf(&buf[..tail + 4])?;
    p.tx_pend = true;
    cap_msg(hdr, objs, 0); /* KM003C PD capture (our TX) */
    Ok(())
}
