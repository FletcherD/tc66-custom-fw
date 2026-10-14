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
(tools/lib/trigger_guard.py). The firmware's trigger verbs are in every build
(cargo feature `triggers`, on by default).
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0] + "/lib")
from tc66lib import open_meter  # noqa: E402
import trigger_guard  # noqa: E402
from pd_engine import *  # noqa: E402,F403 — engine layouts, PD spec names, formatting


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


def read_caps(s):
    caps = parse_caps(xfer(s, b"pdcap", PDCAP_LEN))
    if caps is None:
        sys.exit("garbled pdcap reply")
    return caps


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
