//! Layout of the PD engine status (`pd::status`, the `pdsta` verb payload)
//! and the values it reports — our format, shared by pd.rs (producer),
//! detect.rs and the host tools (tools/pd.py mirrors it).

/* engine states (byte STATE; tools/pd.py names them) */
pub const ST_OFF: u8 = 0;
pub const ST_WAIT_ATTACH: u8 = 1;
pub const ST_LISTEN: u8 = 2; /* attached, BMC up; caps_n says if caps seen */
pub const ST_WAIT_ACCEPT: u8 = 3;
pub const ST_WAIT_PSRDY: u8 = 4;
pub const ST_CONTRACT: u8 = 5;
pub const ST_RD_CYCLE: u8 = 6; /* Rd dropped to force the source to re-attach */
pub const ST_SNIFF: u8 = 7; /* listen-only: no Rd, no GoodCRC, no TX (pdctl 3) */

/* byte offsets */
pub const LEN: usize = 22;
pub const STATE: usize = 0;
pub const CC: usize = 1; /* active CC line: 0 none, 1, 2 */
pub const CAPS: usize = 2; /* stored source PDO count */
pub const CONTRACT: usize = 3; /* granted object position; sniff: a sink's Rd seen */
pub const REQ_OBJ: usize = 4;
pub const ERR: usize = 5;
pub const EVENTS: usize = 6; /* accumulated FUSB302 Interrupta */
pub const STATUS0: usize = 7; /* FUSB302 Status0 */
pub const STATUS1: usize = 8;
pub const MSG_ID: usize = 9;
pub const RX_COUNT: usize = 10; /* u16 LE */
pub const DETACH: usize = 12;
pub const REV: usize = 13; /* negotiated header rev code (pdspec REV_*) */
pub const RX_HDRS: usize = 14; /* last 3 received SOP headers, u16 LE, newest first */
pub const FLAGS: usize = 20;
pub const ERR_REG: usize = 21; /* FUSB302 register of the last failed I2C transaction */

/* FLAGS bits */
pub const FLAG_TX_PEND: u8 = 0x01;
pub const FLAG_CABLE_LISTEN: u8 = 0x02;
pub const FLAG_ASKED: u8 = 0x04;

/* FUSB302 Status0 bits (datasheet Table 26) */
pub const S0_ACTIVITY: u8 = 0x40;
pub const S0_COMP: u8 = 0x20; /* CC above the MDAC threshold */
pub const S0_CRC_CHK: u8 = 0x10; /* last received packet's CRC was good */
pub const S0_BC_LVL: u8 = 0x03;

/* ---- KM003C PD capture (PdPacket attribute 0x0010; the ring in pd.rs,
 * drained by powerz.rs; format per github.com/okhsunrog/km003c-protocol-
 * research, decoded by tools/pdcap.py) ---- */
/* event codes: KM003C's connect / disconnect, then ours */
pub const CAP_CONNECT: u8 = 0x21;
pub const CAP_DISCONNECT: u8 = 0x22;
pub const CAP_RX_FULL: u8 = 0x30; /* RX FIFO went full */
pub const CAP_RX_RESYNC: u8 = 0x31; /* a non-SOP token: the queued frames were flushed */
pub const CAP_BAD_CRC: u8 = 0x32; /* sniff: a packet arrived with a bad CRC */
pub const CAP_HARD_RESET: u8 = 0x33; /* hard reset on the wire */
/* payload preamble (measurement block) */
pub const PRE_TS: usize = 0; /* u32 ms */
pub const PRE_VBUS: usize = 4; /* u16 mV */
pub const PRE_IBUS: usize = 6; /* i16 mA */
pub const PRE_CC1: usize = 8; /* u16 mV */
pub const PRE_CC2: usize = 10; /* u16 mV */
pub const PRE_LEN: usize = 12;
/* connection event: tag, ts LE24, 0, code */
pub const EV_CONNECTION: u8 = 0x45;
pub const EV_CONNECTION_LEN: usize = 6;
/* wrapped PD message: size flag (0x80 | N + 5), ts LE32, sop, N wire bytes */
pub const EV_WRAPPED: u8 = 0x80;
pub const EV_WRAPPED_SIZE_BIAS: u8 = 5;
pub const EV_WRAPPED_TS: usize = 1;
pub const EV_WRAPPED_SOP: usize = 5;
pub const EV_WRAPPED_WIRE: usize = 6;
/* the wrapped message's sop byte (KM003C: 0-2; the _DEBUG kinds are ours) */
pub const SOP: u8 = 0;
pub const SOP1: u8 = 1; /* SOP' */
pub const SOP2: u8 = 2; /* SOP'' */
pub const SOP1_DEBUG: u8 = 3;
pub const SOP2_DEBUG: u8 = 4;
