"""Host side of the meter's PD engine interface (rust/src/pd.rs, pdsta.rs):
engine states, the `pdsta` / `pdcap` reply layouts, the FUSB302 status bits
and error codes it reports, and PDO / header formatting for the CLI tools
(tools/pd.py, tools/detect.py).
"""
import pd_decode
from pd_spec import *  # noqa: F403 — the PD spec's named fields and values
from pd_spec import F, bit, get, u16, u32, val  # noqa: F401

# engine states (rust/src/pd.rs ST_*)
ST_OFF, ST_WAIT_ATTACH, ST_LISTEN, ST_WAIT_ACCEPT, ST_WAIT_PSRDY = 0, 1, 2, 3, 4
ST_CONTRACT, ST_RD_CYCLE, ST_SNIFF = 5, 6, 7
STATES = {
    ST_OFF: "off",
    ST_WAIT_ATTACH: "wait-attach",
    ST_LISTEN: "listen (attached)",
    ST_WAIT_ACCEPT: "wait-accept",
    ST_WAIT_PSRDY: "wait-ps-rdy",
    ST_CONTRACT: "CONTRACT",
    ST_RD_CYCLE: "rd-cycle (forcing re-attach)",
    ST_SNIFF: "SNIFF (listen-only)",
}
# pdsta reply: "OK" + PDSTA_LEN bytes, layout rust/src/pd.rs `status()`
PDSTA_LEN = 22
PDSTA_STATE, PDSTA_CC, PDSTA_CAPS, PDSTA_CONTRACT, PDSTA_REQ_OBJ, PDSTA_ERR = 0, 1, 2, 3, 4, 5
PDSTA_SNIFF_SINK = PDSTA_CONTRACT  # while sniffing: a sink's Rd on the locked CC
PDSTA_EVENTS, PDSTA_STATUS0, PDSTA_STATUS1, PDSTA_MSG_ID, PDSTA_RX_COUNT = 6, 7, 8, 9, 10  # rx: u16
PDSTA_DETACH, PDSTA_REV, PDSTA_RX_HDRS, PDSTA_FLAGS, PDSTA_ERR_REG = 12, 13, 14, 20, 21  # hdrs: 3 x u16
PDSTA_FLAG_NAMES = {bit(0): "tx_pend", bit(1): "cable-listen", bit(2): "asked"}
ENGINE_REVS = {REV_2_0: "2.0", REV_3_X: "3.0"}  # negotiated header rev
# FUSB302 Status0 / Interrupta (FUSB302B datasheet, Tables 26 / 29)
STATUS0_VBUSOK = bit(7)
STATUS0_ACTIVITY = bit(6)
STATUS0_BC_LVL = F(0, 2)
INTERRUPTA_NAMES = {bit(4): "RETRYFAIL", bit(3): "HARDSENT", bit(2): "TXSENT", bit(1): "SOFTRST",
                    bit(0): "HARDRST", bit(6): "TOGDONE", bit(5): "SOFTFAIL"}
# error byte: firmware codes below, else an I2C stage (rust/src/i2c.rs) + NACK flag
I2C_STAGE = F(0, 7)
I2C_NACK = bit(7)
ERRS = {
    0x0D: "PPS setpoint missing or out of the APDO's range",
    0x0E: "unsupported PDO kind (or PPS on a PD 2.0 source)",
    0x0F: "bad args / bad state",
    0xE1: "tx retries exhausted (no GoodCRC)",
    0xE2: "source rejected the request",
    0xE3: "Accept timeout",
    0xE4: "PS_RDY timeout",
    0xE5: "sniff: RX FIFO overflowed between polls (frames lost)",
}


# pdcap reply: "OK" + count + count x 4-byte LE source PDOs (up to 7)
PDCAP_COUNT, PDCAP_PDOS, PDCAP_LEN = 2, 3, 3 + 7 * 4


def parse_caps(r):
    """pdcap reply -> PDOs, or None if garbled."""
    if r[:2] != b"OK" or len(r) <= PDCAP_COUNT:
        return None
    return [u32(r, PDCAP_PDOS + 4 * i) for i in range(r[PDCAP_COUNT])]


def pdo_max_volts(pdo):
    """Highest voltage a PDO can put on VBUS (fixed: its voltage; ranges: top)."""
    kind = get(pdo, PDO_TYPE)
    if kind == PDO_FIXED:
        return val(pdo, FIXED_VOLTAGE)
    if kind == PDO_BATTERY:
        return val(pdo, BATTERY_MAX_VOLTAGE)
    if kind == PDO_VARIABLE:
        return val(pdo, VARIABLE_MAX_VOLTAGE)
    apdo = get(pdo, APDO_TYPE)
    if apdo == APDO_PPS:
        return val(pdo, PPS_MAX_VOLTAGE)
    if apdo == APDO_EPR_AVS:
        return val(pdo, EPR_AVS_MAX_VOLTAGE)
    return 20.0  # SPR AVS: 9-20 V


def decode_pdo(i, pdo):
    """One source PDO, column-aligned for the caps listing."""
    kind = get(pdo, PDO_TYPE)
    if kind == PDO_FIXED:
        return "PDO %d: fixed  %5.2f V  %.2f A" % (i, val(pdo, FIXED_VOLTAGE), val(pdo, FIXED_MAX_CURRENT))
    if kind == PDO_APDO and get(pdo, APDO_TYPE) == APDO_PPS:
        return "PDO %d: PPS    %5.2f-%.2f V %.2f A" % (i, val(pdo, PPS_MIN_VOLTAGE), val(pdo, PPS_MAX_VOLTAGE),
                                                    val(pdo, PPS_MAX_CURRENT))
    return "PDO %d: %s" % (i, pd_decode.pdo(pdo))


def hdr_name(h):
    if h == 0:
        return "-"
    t = get(h, HDR_MSG_TYPE)
    if get(h, HDR_EXTENDED):
        name = EXT_MSGS.get(t, "ext%d" % t)
    elif get(h, HDR_NUM_OBJECTS):
        name = DATA_MSGS.get(t, "data%d" % t)
    else:
        name = CTRL_MSGS.get(t, "type%d" % t)
    return "%s(0x%04x)" % (name, h)
