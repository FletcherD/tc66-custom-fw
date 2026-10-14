#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Host side of the firmware's PD sink verbs (FUSB302).

  pd.py arm            arm the sink engine (pdctl 1) — then plug the source in
  pd.py off            FUSB302 SW_RES + engine off (pdctl 0)
  pd.py hardreset      send a PD hard reset: source re-caps at its top rev (pdctl 2);
                       while sniffing, makes the charger and the device
                       renegotiate from scratch (a re-plug without unplugging)
  pd.py sniff          listen-only: capture a source and a sink negotiating
                       through the meter (pdctl 3; read with tools/pdcap.py)
  pd.py status         one engine-state snapshot (pdsta)
  pd.py caps           decode the stored Source_Capabilities PDOs (pdcap)
  pd.py req <n> [volts [amps]]   request source PDO n (pdreq); volts (+ amps,
                       default the APDO max) required for a PPS APDO
  pd.py watch [secs]   poll status at 2 Hz (default 20 s)
  pd.py info [next]    the meter's PD screen text (pdinf): parsed source caps,
                       contract, cable, counters; `next` first advances the
                       screen's page as K2 does
  pd.py ask <what>     (dev-pd builds, sink in contract) send a request to the
                       source and let tools/pdcap.py capture its answer:
                       srccap, snkcap, srccapext, status, ppsstatus, countries,
                       srcinfo, revision, mfrinfo, discid, or
                       raw:<type>[:<hex object>...] (type | 0x80 = extended)

DANGER: a granted request RAISES THE OUT-PORT VOLTAGE (9/12/15/20 V).
Nothing should be attached downstream of the meter when triggering. `req`
refuses PDOs above --max-v and asks for confirmation unless --yes
(tools/trigger_guard.py). The firmware's trigger verbs are in every build
(cargo feature `triggers`, on by default).
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402
import trigger_guard  # noqa: E402
import pd_decode  # noqa: E402
from pd_spec import *  # noqa: E402,F403 — the PD spec's named fields and values
from pd_spec import bit  # noqa: E402

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


def open_meter():
    ports = meter_candidates()
    if not ports:
        sys.exit("no meter found")
    s = serial.Serial(ports[0], 115200, timeout=1.5)
    s.reset_input_buffer()
    return s


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


# pdcap reply: "OK" + count + count x 4-byte LE source PDOs (up to 7)
PDCAP_COUNT, PDCAP_PDOS, PDCAP_LEN = 2, 3, 3 + 7 * 4


def parse_caps(r):
    """pdcap reply -> PDOs, or None if garbled."""
    if r[:2] != b"OK" or len(r) <= PDCAP_COUNT:
        return None
    return [u32(r, PDCAP_PDOS + 4 * i) for i in range(r[PDCAP_COUNT])]


def read_caps(s):
    caps = parse_caps(xfer(s, b"pdcap", PDCAP_LEN))
    if caps is None:
        sys.exit("garbled pdcap reply")
    return caps


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


def show_status(s):
    st = read_status(s)
    ev = ",".join(n for f, n in INTERRUPTA_NAMES.items() if get(st[PDSTA_EVENTS], f)) or "-"
    hdrs = [u16(st, PDSTA_RX_HDRS + 2 * i) for i in range(3)]
    s0, err = st[PDSTA_STATUS0], st[PDSTA_ERR]
    print(
        "state=%s cc=%s rev=%s caps=%d contract=PDO%d req=PDO%d err=%s\n"
        "events=%s status0=0x%02x (vbus %s, bc_lvl %d) status1=0x%02x msgid=%d rx=%d\n"
        "last rx: %s"
        % (
            STATES.get(st[PDSTA_STATE], st[PDSTA_STATE]),
            st[PDSTA_CC] or "-",
            ENGINE_REVS.get(st[PDSTA_REV], st[PDSTA_REV]),
            st[PDSTA_CAPS],
            st[PDSTA_CONTRACT],
            st[PDSTA_REQ_OBJ],
            ("0x%02x (%s)" % (err, ERRS.get(err, "I2C stage"))) if err else "-",
            ev,
            s0,
            "OK" if get(s0, STATUS0_VBUSOK) else "low",
            get(s0, STATUS0_BC_LVL),
            st[PDSTA_STATUS1],
            st[PDSTA_MSG_ID],
            u16(st, PDSTA_RX_COUNT),
            "  ".join(hdr_name(h) for h in hdrs),
        )
    )
    fl = [n for f, n in PDSTA_FLAG_NAMES.items() if get(st[PDSTA_FLAGS], f)]
    print("flags: %s%s" % (",".join(fl) or "-",
                           "  last I2C error at reg 0x%02x" % st[PDSTA_ERR_REG] if err else ""))
    if st[PDSTA_STATE] == ST_SNIFF:
        print("sniff: sink %s" % ("ATTACHED" if st[PDSTA_SNIFF_SINK] else "not attached"))
    return st


def read_status(s):
    """pdsta -> the PDSTA_LEN-byte engine state."""
    r = xfer(s, b"pdsta", 2 + PDSTA_LEN)
    if r[:2] != b"OK" or len(r) < 2 + PDSTA_LEN:
        sys.exit("garbled pdsta reply: %r" % r)
    return r[2:]


COLS = 22  # pdinfo::COLS


def show_info(s, advance=False):
    s.reset_input_buffer()
    s.write(b"pdinf" + (b"\x01" if advance else b""))
    h = s.read(5)
    if h[:2] != b"OK" or len(h) < 5:
        sys.exit("garbled pdinf reply: %r" % h)
    page, pages, n = h[2], h[3], h[4]
    body = s.read(n * COLS)
    if len(body) != n * COLS:
        sys.exit("short pdinf reply: %d of %d bytes" % (len(body), n * COLS))
    print("screen page %d/%d (6 lines each)" % (page + 1, pages))
    for i in range(n):
        mark = "*" if i // 6 == page else " "
        print("%s %2d | %s" % (mark, i, body[i * COLS:(i + 1) * COLS].decode("ascii", "replace")))


# pdsnd: type byte = message type, PDSND_EXTENDED set for an extended message
PDSND_EXTENDED = 0x80


def ext_first_object(size, data):
    """First data object of a one-chunk extended message: Extended Message
    Header (chunked, `size` data bytes) + up to 2 data bytes, LE."""
    eh = pack((EXTHDR_CHUNKED, 1), (EXTHDR_DATA_SIZE, size))
    return eh | int.from_bytes(bytes(data).ljust(2, b"\0"), "little") << 16


# pdsnd presets: (type byte, data objects)
ASK = {
    "srccap": (CTRL_GET_SOURCE_CAP, []),
    "snkcap": (CTRL_GET_SINK_CAP, []),        # dual-role sources only
    "srccapext": (CTRL_GET_SOURCE_CAP_EXT, []),  # PD 3.x from here on
    "status": (CTRL_GET_STATUS, []),
    "ppsstatus": (CTRL_GET_PPS_STATUS, []),
    "countries": (CTRL_GET_COUNTRY_CODES, []),
    "srcinfo": (CTRL_GET_SOURCE_INFO, []),
    "revision": (CTRL_GET_REVISION, []),
    "mfrinfo": (PDSND_EXTENDED | EXT_GET_MANUFACTURER_INFO,  # Table 6.56: target port, ref 0
                [ext_first_object(2, [MANUFACTURER_TARGET_PORT, 0])]),
}


def ask(s, what):
    if what.startswith("raw:"):
        f = what.split(":")[1:]
        t, objs = int(f[0], 0), [int(x, 16) for x in f[1:]]
    elif what == "discid":  # SOP Discover Identity REQ, SVDM version per the negotiated rev
        rev3 = read_status(s)[PDSTA_REV] >= REV_3_X
        t, objs = DATA_VENDOR_DEFINED, [pack((VDM_SVID, SVID_PD), (VDM_STRUCTURED, 1),
                                             (VDM_VERSION_MAJOR, 1 if rev3 else 0),
                                             (VDM_COMMAND_TYPE, VDM_REQ), (VDM_COMMAND, CMD_DISCOVER_IDENTITY))]
    elif what in ASK:
        t, objs = ASK[what]
    else:
        sys.exit("unknown request %r (pd.py --help)" % what)
    pkt = b"pdsnd" + bytes([t]) + b"".join(o.to_bytes(4, "little") for o in objs)
    ok_er(xfer(s, pkt, 3), "ask %s (type 0x%02x, %d objects)" % (what, t, len(objs)))


def main():
    guard, argv = trigger_guard.parse(__doc__)
    cmd = argv[0]
    s = open_meter()
    if cmd == "arm":
        ok_er(xfer(s, b"pdctl\x01", 3), "arm")
    elif cmd == "off":
        ok_er(xfer(s, b"pdctl\x00", 3), "off")
    elif cmd == "hardreset":
        ok_er(xfer(s, b"pdctl\x02", 3), "hard reset")
    elif cmd == "sniff":
        ok_er(xfer(s, b"pdctl\x03", 3), "sniff")
    elif cmd == "status":
        show_status(s)
    elif cmd == "caps":
        pdos = read_caps(s)
        if not pdos:
            print("no source capabilities stored (source attached? PD at all?)")
        for i, pdo in enumerate(pdos):
            print(decode_pdo(i + 1, pdo), " raw=0x%08x" % pdo)
    elif cmd == "req":
        n = int(argv[1], 0)
        mv = int(float(argv[2]) * 1000) if len(argv) > 2 else 0
        ma = int(float(argv[3]) * 1000) if len(argv) > 3 else 0
        pdos = read_caps(s)
        if not 1 <= n <= len(pdos):
            sys.exit("no PDO %d among the %d stored source caps (pd.py caps)" % (n, len(pdos)))
        what = "request PDO %d" % n + (" @ %.2f V" % (mv / 1000) if mv else "")
        trigger_guard.confirm(guard, mv / 1000 if mv else pdo_max_volts(pdos[n - 1]), what)
        pkt = b"pdreq" + bytes([n]) + mv.to_bytes(2, "little") + ma.to_bytes(2, "little")
        if ok_er(xfer(s, pkt, 3), what):
            time.sleep(0.7)
            show_status(s)
    elif cmd == "info":
        show_info(s, len(argv) > 1 and argv[1] == "next")
    elif cmd == "ask":
        ask(s, argv[1])
    elif cmd == "watch":
        secs = float(argv[1]) if len(argv) > 1 else 20
        end = time.time() + secs
        while time.time() < end:
            show_status(s)
            time.sleep(0.5)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
