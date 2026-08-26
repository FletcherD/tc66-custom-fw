//! FUSB302 USB-PD sink engine (M4 bring-up). Host-armed, nothing at boot.
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
//! Everything runs in thread mode from the main loop (`service()`, ~every
//! 8 ms while active) and the verb handlers — same context, so the `PD`
//! static is thread-exclusive (no ISR ever touches it). All I²C goes
//! through `ina::pause` per the bus rule. The INT_N pin wiring is unknown
//! (and unneeded): the FUSB302 interrupt/status registers are polled; the
//! tight timing (GoodCRC within 195 µs) is AUTO_CRC hardware.
//!
//! Poll cost while armed: one 7-byte status burst per 2 ticks ≈ 1 ms of
//! sampler pause per 8 ms — the measurement window stretches slightly
//! while a PD session is active. Idle (state 0, the default): zero cost.

use crate::i2c;
use crate::ina;

const ADDR: u8 = 0x22;

/* registers */
const R_SWITCHES0: u8 = 0x02;
const R_SWITCHES1: u8 = 0x03;
const R_MEASURE: u8 = 0x04; /* bit6 MEAS_VBUS, bits5:0 MDAC */
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
const R_STATUS1: u8 = 0x41;
const R_FIFOS: u8 = 0x43;

/* Switches0 */
const SW0_PDWN: u8 = 0x03; /* Rd on both CC */
const SW0_MEAS_CC1: u8 = 0x04;
const SW0_MEAS_CC2: u8 = 0x08;
/* Switches1: SPECREV=01 (PD 2.0) + AUTO_CRC + TXCC1/2 */
const SW1_REV20: u8 = 0x20;
const SW1_AUTO_CRC: u8 = 0x04;
/* Control0/1 */
const C0_INT_MASK_TXFLUSH: u8 = 0x60;
const C1_RX_FLUSH: u8 = 0x04;
/* Control3: AUTO_RETRY + N_RETRIES=3 */
const C3_RETRIES: u8 = 0x07;
/* Reset */
const RST_SW: u8 = 0x01;
const RST_PD: u8 = 0x02;
/* Interrupta */
const IA_RETRYFAIL: u8 = 0x10;
const IA_HARDSENT: u8 = 0x08;
const IA_TXSENT: u8 = 0x04;
const IA_HARDRST: u8 = 0x01;
/* Status0 / Status1 (VBUSOK is Status0 bit7, reported raw via pdsta) */
const S0_ACTIVITY: u8 = 0x40;
const S0_COMP: u8 = 0x20; /* CC above the MDAC threshold */
const S0_BC_LVL: u8 = 0x03;
const S1_RX_EMPTY: u8 = 0x20;

/* TX FIFO tokens */
const TK_TXON: u8 = 0xA1;
const TK_SOP1: u8 = 0x12;
const TK_SOP2: u8 = 0x13;
const TK_PACKSYM: u8 = 0x80;
const TK_JAM_CRC: u8 = 0xFF;
const TK_EOP: u8 = 0x14;
const TK_TXOFF: u8 = 0xFE;

/* PD message types (rev 2.0) */
const CTL_GOODCRC: u8 = 1;
const CTL_ACCEPT: u8 = 3;
const CTL_REJECT: u8 = 4;
const CTL_PS_RDY: u8 = 6;
const CTL_GET_SRC_CAP: u8 = 7;
const CTL_GET_SNK_CAP: u8 = 8;
const CTL_WAIT: u8 = 12;
const CTL_SOFT_RESET: u8 = 13;
const DAT_SOURCE_CAP: u8 = 1;
const DAT_REQUEST: u8 = 2;
const DAT_SINK_CAP: u8 = 4;

/* states (pdsta byte 0; tools/pd.py names them) */
const ST_OFF: u8 = 0;
const ST_WAIT_ATTACH: u8 = 1;
const ST_LISTEN: u8 = 2; /* attached, BMC up; caps_n says if caps seen */
const ST_WAIT_ACCEPT: u8 = 3;
const ST_WAIT_PSRDY: u8 = 4;
const ST_CONTRACT: u8 = 5;
const ST_RD_CYCLE: u8 = 6; /* Rd dropped to force the source to re-attach */

/* err codes carried in Pd.err (i2c stages are 1..0x0F | 0x80) */
const E_RETRYFAIL: u8 = 0xE1;
const E_REJECT: u8 = 0xE2;
const E_ACCEPT_TIMEOUT: u8 = 0xE3;
const E_PSRDY_TIMEOUT: u8 = 0xE4;

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
    poll_tick: u32,
    req_tick: u32,
    cap_ask_tick: u32,
    vcc_tick: u32,
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
    poll_tick: 0,
    req_tick: 0,
    cap_ask_tick: 0,
    vcc_tick: 0,
}));

fn pd() -> &'static mut Pd {
    unsafe { &mut *PD.0.get() }
}

/* ---- I²C helpers (bus rule: every transaction inside ina::pause) ---- */

fn stage(e: i2c::I2cErr) -> u8 {
    e.stage
}

fn wr(reg: u8, val: u8) -> Result<(), u8> {
    ina::pause(|| i2c::transact(ADDR, &[reg, val], &mut [])).map_err(stage)
}

fn wr_buf(buf: &[u8]) -> Result<(), u8> {
    ina::pause(|| i2c::transact(ADDR, buf, &mut [])).map_err(stage)
}

fn rd(reg: u8, buf: &mut [u8]) -> Result<(), u8> {
    ina::pause(|| i2c::transact(ADDR, &[reg], buf)).map_err(stage)
}

/* ---- verb entry points (thread mode) ---- */

/// `pdctl 1`: reset + configure the FUSB302 as a presenting sink and start
/// the attach poll. Safe with or without a source already plugged.
#[inline(never)]
pub fn arm() -> Result<(), u8> {
    let p = pd();
    p.state = ST_OFF; /* stop service() touching the chip mid-config */
    wr(R_RESET, RST_SW)?;
    crate::board::delay_ms(2);
    wr(R_POWER, 0x0F)?;
    wr(R_CONTROL3, C3_RETRIES)?;
    /* INT_N pin unused: mask everything, we poll */
    wr(R_MASK, 0xFF)?;
    wr(R_MASKA, 0xFF)?;
    wr(R_MASKB, 0x01)?;
    wr(R_CONTROL0, C0_INT_MASK_TXFLUSH)?;
    wr(R_CONTROL1, C1_RX_FLUSH)?;
    /* Drop Rd first (service restores it after ~600 ms): a source plugged
     * before we armed has exhausted its Source_Capabilities retries against
     * the chip's dead-Rd defaults and stopped listening for PD — only a
     * detach/re-attach makes it start over (bench-hit with an Apple 20 W).
     * The cycle costs nothing when no source is attached yet. */
    wr(R_SWITCHES0, 0x00)?;
    wr(R_SWITCHES1, SW1_REV20)?;
    *p = Pd {
        state: ST_RD_CYCLE,
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
        poll_tick: ina::ticks(),
        req_tick: ina::ticks(), /* ST_RD_CYCLE start */
        cap_ask_tick: 0,
        vcc_tick: 0,
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

/// `pdsta` payload: 20 bytes of engine state.
#[inline(never)]
pub fn status(out: &mut [u8; 20]) {
    let p = pd();
    out[0] = p.state;
    out[1] = p.cc;
    out[2] = p.caps_n;
    out[3] = p.contract;
    out[4] = p.req_obj;
    out[5] = p.err;
    out[6] = p.events;
    out[7] = p.status0;
    out[8] = p.status1;
    out[9] = p.msg_id;
    out[10..12].copy_from_slice(&p.rx_msgs.to_le_bytes());
    out[12] = p.detach;
    out[13] = 0;
    for i in 0..3 {
        out[14 + 2 * i..16 + 2 * i].copy_from_slice(&p.rx_hdr[i].to_le_bytes());
    }
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

/// `pdreq <n>`: request source PDO n (1-based). Fixed-supply PDOs only for
/// now (APDO/PPS needs the rev-3.0 header + 20 mV RDO — later).
#[inline(never)]
pub fn request(idx: u8) -> Result<(), u8> {
    let p = pd();
    if p.state < ST_LISTEN || idx == 0 || idx > p.caps_n {
        return Err(0x0F);
    }
    let pdo = p.caps[idx as usize - 1];
    if pdo >> 30 != 0 {
        return Err(0x0E); /* not a fixed-supply PDO */
    }
    p.err = 0;
    send_request(p, idx, pdo)
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

/// Measure both CC pins via the MDAC (~5 ms of paused I2C), then restore
/// the MEAS switch for the active CC. VCONN above the DAC range clamps to
/// ~2.7 V. Runs every ~500 ms while armed (not during the Rd-cycle — it
/// must not re-present the pulldowns early).
fn vcc_sweep(p: &mut Pd) -> Result<(), u8> {
    wr(R_SWITCHES0, SW0_PDWN | SW0_MEAS_CC1)?;
    let m1 = mdac_search()?;
    wr(R_SWITCHES0, SW0_PDWN | SW0_MEAS_CC2)?;
    let m2 = mdac_search()?;
    let restore = match p.cc {
        1 => SW0_MEAS_CC1,
        2 => SW0_MEAS_CC2,
        _ => 0,
    };
    wr(R_SWITCHES0, SW0_PDWN | restore)?;
    let mv = |m: u8| -> u16 {
        if m == 0 {
            0
        } else {
            (m as u16 + 1) * 42 + 21 /* midpoint of the found step */
        }
    };
    p.vcc1_mv = mv(m1);
    p.vcc2_mv = mv(m2);
    Ok(())
}

/* ---- the engine (called from the main loop every pass) ---- */

#[inline(never)]
pub fn service() {
    let p = pd();
    if p.state == ST_OFF {
        return;
    }
    let now = ina::ticks();
    let interval = if p.state >= ST_LISTEN { 2 } else { 12 };
    if now.wrapping_sub(p.poll_tick) < interval {
        return;
    }
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
    wr(
        R_SWITCHES1,
        SW1_REV20 | SW1_AUTO_CRC | if cc1 { 0x01 } else { 0x02 },
    )?;
    wr(R_CONTROL1, C1_RX_FLUSH)?;
    wr(R_RESET, RST_PD)?;
    p.msg_id = 0;
    p.detach = 0;
    p.state = ST_LISTEN;
    p.cap_ask_tick = ina::ticks();
    Ok(())
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
        p.msg_id = (p.msg_id + 1) & 7;
    }
    if inta & (IA_HARDRST | IA_HARDSENT) != 0 {
        /* hard reset on the wire: back to listening, source re-sends caps */
        wr(R_CONTROL1, C1_RX_FLUSH)?;
        wr(R_RESET, RST_PD)?;
        p.msg_id = 0;
        p.contract = 0;
        p.req_obj = 0;
        p.state = ST_LISTEN;
        p.cap_ask_tick = now;
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
            wr(R_SWITCHES1, SW1_REV20)?;
            return Ok(());
        }
    } else {
        p.detach = 0;
    }

    /* drain RX (bounded: at most 4 frames per poll) */
    for _ in 0..4 {
        let mut s = [0u8; 1];
        rd(R_STATUS1, &mut s)?;
        if s[0] & S1_RX_EMPTY != 0 {
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

    /* no caps yet: the source may have exhausted its caps retries before we
     * armed — ask every ~500 ms */
    if p.state == ST_LISTEN && p.caps_n == 0 && now.wrapping_sub(p.cap_ask_tick) > 125 {
        p.cap_ask_tick = now;
        send(p, CTL_GET_SRC_CAP, &[])?;
    }
    Ok(())
}

/// Read + dispatch one RX FIFO frame (token, 2-byte header, 4·n objects,
/// 4 CRC bytes — CRC already checked by hardware).
#[inline(never)]
fn rx_frame(p: &mut Pd, now: u32) -> Result<(), u8> {
    let mut head = [0u8; 3];
    rd(R_FIFOS, &mut head)?;
    if head[0] & 0xE0 != 0xE0 {
        /* not SOP (or FIFO skew): flush and resync */
        wr(R_CONTROL1, C1_RX_FLUSH)?;
        return Ok(());
    }
    let hdr = u16::from_le_bytes([head[1], head[2]]);
    let nobj = ((hdr >> 12) & 7) as usize;
    let mut body = [0u8; 32]; /* 7·4 objects + 4 CRC max */
    rd(R_FIFOS, &mut body[..4 * nobj + 4])?;
    let mtype = (hdr & 0x0F) as u8; /* rev-2.0 type field */
    p.rx_msgs = p.rx_msgs.wrapping_add(1);
    p.rx_hdr = [hdr, p.rx_hdr[0], p.rx_hdr[1]];

    if nobj == 0 {
        match mtype {
            CTL_GOODCRC => {}
            CTL_ACCEPT => {
                if p.state == ST_WAIT_ACCEPT {
                    p.state = ST_WAIT_PSRDY;
                    p.req_tick = now;
                }
            }
            CTL_REJECT | CTL_WAIT => {
                if p.state == ST_WAIT_ACCEPT {
                    p.err = E_REJECT;
                    p.state = ST_LISTEN;
                }
            }
            CTL_PS_RDY => {
                if p.state == ST_WAIT_PSRDY {
                    p.contract = p.req_obj;
                    p.state = ST_CONTRACT;
                }
            }
            CTL_GET_SNK_CAP => {
                /* one fixed sink PDO: 5 V, 100 mA */
                send_data(p, DAT_SINK_CAP, (100u32 << 10) | 10)?;
            }
            CTL_SOFT_RESET => {
                p.msg_id = 0;
                send(p, CTL_ACCEPT, &[])?;
                p.state = ST_LISTEN;
                p.cap_ask_tick = now;
            }
            _ => {} /* Ping, swaps, … — GoodCRC (hardware) is answer enough */
        }
        return Ok(());
    }

    if mtype == DAT_SOURCE_CAP {
        p.caps_n = nobj as u8;
        for i in 0..nobj {
            p.caps[i] = u32::from_le_bytes([
                body[4 * i],
                body[4 * i + 1],
                body[4 * i + 2],
                body[4 * i + 3],
            ]);
        }
        /* a sink that GoodCRCs caps MUST Request within tSenderResponse or
         * the source hard-resets: immediately take PDO 1 (vSafe5V — no
         * output-voltage change). `pdreq` renegotiates from there. */
        send_request(p, 1, p.caps[0])?;
    }
    /* other data messages (VDMs …): ignored — rev 2.0 allows it */
    Ok(())
}

/* ---- TX ---- */

/// Fixed-supply RDO for PDO `idx`: operating = max = the PDO's current
/// limit, no-USB-suspend set.
#[inline(never)]
fn send_request(p: &mut Pd, idx: u8, pdo: u32) -> Result<(), u8> {
    let ma10 = pdo & 0x3FF;
    let rdo = (idx as u32) << 28 | 1 << 24 | ma10 << 10 | ma10;
    send_data(p, DAT_REQUEST, rdo)?;
    p.req_obj = idx;
    p.req_tick = ina::ticks();
    p.state = ST_WAIT_ACCEPT;
    Ok(())
}

fn send_data(p: &mut Pd, mtype: u8, obj: u32) -> Result<(), u8> {
    let o = obj.to_le_bytes();
    send_frame(p, mtype, 1, &o)
}

fn send(p: &mut Pd, mtype: u8, objs: &[u8]) -> Result<(), u8> {
    send_frame(p, mtype, 0, objs)
}

/// Write one SOP message into the TX FIFO and fire it. Header: sink/UFP,
/// spec rev 2.0, our rolling message id (incremented on I_TXSENT).
#[inline(never)]
fn send_frame(p: &mut Pd, mtype: u8, nobj: u8, objs: &[u8]) -> Result<(), u8> {
    wr(R_CONTROL0, C0_INT_MASK_TXFLUSH)?; /* flush any stale TX */
    let hdr = (mtype as u16) | 0x0040 | ((p.msg_id as u16) << 9) | ((nobj as u16) << 12);
    let mut buf = [0u8; 16];
    buf[0] = R_FIFOS;
    buf[1..5].copy_from_slice(&[TK_SOP1, TK_SOP1, TK_SOP1, TK_SOP2]);
    let n = 2 + objs.len();
    buf[5] = TK_PACKSYM | n as u8;
    buf[6..8].copy_from_slice(&hdr.to_le_bytes());
    buf[8..8 + objs.len()].copy_from_slice(objs);
    let tail = 8 + objs.len();
    buf[tail..tail + 4].copy_from_slice(&[TK_JAM_CRC, TK_EOP, TK_TXOFF, TK_TXON]);
    wr_buf(&buf[..tail + 4])
}
