"""Host side of the meter's PD engine interface (rust/src/pd.rs, pdsta.rs):
engine states, the `pdsta` / `pdcap` reply layouts, the FUSB302 status bits
and error codes it reports, and PDO / header formatting for the CLI tools
(tools/pd.py, tools/detect.py), and the serial helpers that read and drive it
(tools/pd.py, tools/pdcap.py).
"""
import sys

import pd_decode
from pd_spec import *  # noqa: F403 — the PD spec's named fields and values
from pd_spec import F, bit, get, pack, u16, u32, val  # noqa: F401

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


def xfer(s, pkt, rlen):
    s.reset_input_buffer()
    s.write(pkt)
    return s.read(rlen)


def ok_er(r, what):
    if r[:2] == b"OK":
        print("%s: OK" % what)
        return True
    if r[:2] == b"ER":
        e = r[2] if len(r) > 2 else -1
        print("%s: ER 0x%02x (%s)" % (what, e, ERRS.get(e, "I2C stage %d%s" % (
            get(e, I2C_STAGE), " NACK/buserr" if get(e, I2C_NACK) else ""))))
        return False
    sys.exit("garbled reply: %r" % r)


def read_status(s):
    """pdsta -> the PDSTA_LEN-byte engine state."""
    r = xfer(s, b"pdsta", 2 + PDSTA_LEN)
    if r[:2] != b"OK" or len(r) < 2 + PDSTA_LEN:
        sys.exit("garbled pdsta reply: %r" % r)
    return r[2:]


# ---- information requests from the sink (pdask / pdsnd, answers via pdans) ----

PDSND_EXTENDED = 0x80  # pdask / pdsnd type byte: message type, this bit = extended
FRAME_MAX = 2 + 4 * 7  # header + 7 data objects (pdans / pdcbl payload)


def ext_first_object(size, data):
    """First data object of a one-chunk extended message: Extended Message
    Header (chunked, `size` data bytes) + up to 2 data bytes, LE."""
    eh = pack((EXTHDR_CHUNKED, 1), (EXTHDR_DATA_SIZE, size))
    return eh | int.from_bytes(bytes(data).ljust(2, b"\0"), "little") << 16


def discover_identity_req(rev3):
    """SOP Discover Identity REQ VDM header (SVDM 2.0 at PD 3.x, else 1.0)."""
    return pack((VDM_SVID, SVID_PD), (VDM_STRUCTURED, 1), (VDM_VERSION_MAJOR, 1 if rev3 else 0),
                (VDM_COMMAND_TYPE, VDM_REQ), (VDM_COMMAND, CMD_DISCOVER_IDENTITY))


# read-only information requests the firmware's `pdask` accepts (pd.rs `ask`):
# name -> (type byte, data objects); None = built per call (discid)
INFO_PD2 = {"snkcap"}  # valid from a PD 2.0 sink; the rest are PD 3.x
INFO_REQUESTS = {
    "srccapext": (CTRL_GET_SOURCE_CAP_EXT, []),  # PD 3.x
    "mfrinfo": (PDSND_EXTENDED | EXT_GET_MANUFACTURER_INFO,  # Table 6.56: target port, ref 0
                [ext_first_object(2, [MANUFACTURER_TARGET_PORT, 0])]),
    "discid": None,                                # Discover Identity (UFP-initiated: PD 3.x)
    "status": (CTRL_GET_STATUS, []),
    "ppsstatus": (CTRL_GET_PPS_STATUS, []),
    "srcinfo": (CTRL_GET_SOURCE_INFO, []),
    "revision": (CTRL_GET_REVISION, []),
    "countries": (CTRL_GET_COUNTRY_CODES, []),
    "snkcap": (CTRL_GET_SINK_CAP, []),            # dual-role sources only
    "snkcapext": (CTRL_GET_SINK_CAP_EXT, []),
}


# the answer each request expects: (extended?, message type) — Tables 6.4-6.47
INFO_ANSWERS = {
    "srccapext": (True, EXT_SOURCE_CAP_EXT), "mfrinfo": (True, EXT_MANUFACTURER_INFO),
    "discid": (False, DATA_VENDOR_DEFINED), "status": (True, EXT_STATUS),
    "ppsstatus": (True, EXT_PPS_STATUS), "srcinfo": (False, DATA_SOURCE_INFO),
    "revision": (False, DATA_REVISION), "countries": (True, EXT_COUNTRY_CODES),
    "snkcap": (False, DATA_SINK_CAP), "snkcapext": (True, EXT_SINK_CAP_EXT),
}


def answer_kind(name, wire):
    """'answer' (the expected response; a Discover Identity NAK/BUSY counts
    as 'unsupported'), 'unsupported' (Not_Supported / Reject) or
    'unexpected'."""
    h = u16(wire, 0)
    ext, typ, nobj = get(h, HDR_EXTENDED), get(h, HDR_MSG_TYPE), get(h, HDR_NUM_OBJECTS)
    if not ext and not nobj and typ in (CTRL_NOT_SUPPORTED, CTRL_REJECT):
        return "unsupported"
    if (bool(ext), typ) != INFO_ANSWERS[name] or (not ext and not nobj):
        return "unexpected"
    if typ == DATA_VENDOR_DEFINED and not ext and get(u32(wire, 2), VDM_COMMAND_TYPE) != VDM_ACK:
        return "unsupported"
    return "answer"


def request_bytes(name, rev3):
    """(type byte, data objects) of an INFO_REQUESTS entry."""
    if name == "discid":
        return DATA_VENDOR_DEFINED, [discover_identity_req(rev3)]
    return INFO_REQUESTS[name]


def read_frame(s, verb):
    """pdans / pdcbl -> the frame (header + objects) or None if none yet."""
    r = xfer(s, verb, 3 + FRAME_MAX)
    if r[:2] != b"OK" or len(r) < 3 or len(r) < 3 + r[2]:
        return None
    return r[3:3 + r[2]] or None


def _unsolicited(wire):
    """The source's own VDM request (e.g. its Discover Identity to us) that
    arrived first instead of the answer."""
    h = u16(wire, 0)
    return (get(h, HDR_NUM_OBJECTS) and get(h, HDR_MSG_TYPE) == DATA_VENDOR_DEFINED
            and not get(h, HDR_EXTENDED) and get(u32(wire, 2), VDM_COMMAND_TYPE) == VDM_REQ)


def ask_info(s, name, rev3, verb=b"pdask", timeout=0.5, tries=3):
    """Send an information request from the contracted sink and wait for the
    source's answer -> its frame, or an error string."""
    import time
    t, objs = request_bytes(name, rev3) if name in INFO_REQUESTS else name
    pkt = verb + bytes([t]) + b"".join(o.to_bytes(4, "little") for o in objs)
    for _ in range(tries):
        sent = False
        for _ in range(10):  # the engine may be busy with its own TX
            r = xfer(s, pkt, 3)
            if r[:2] == b"OK":
                sent = True
                break
            time.sleep(0.05)
        if not sent:
            return "refused (%r)" % r
        end = time.time() + timeout
        while time.time() < end:
            wire = read_frame(s, b"pdans")
            if wire:
                if _unsolicited(wire) and t != DATA_VENDOR_DEFINED:
                    break  # not our answer: ask again
                return wire
            time.sleep(0.02)
        else:
            return "no answer"
    return "no answer"
