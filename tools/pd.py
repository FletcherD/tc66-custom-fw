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

STATES = {
    0: "off",
    1: "wait-attach",
    2: "listen (attached)",
    3: "wait-accept",
    4: "wait-ps-rdy",
    5: "CONTRACT",
    6: "rd-cycle (forcing re-attach)",
    7: "SNIFF (listen-only)",
}
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
EVENTS = [  # Interrupta accumulator bits
    (0x10, "RETRYFAIL"),
    (0x08, "HARDSENT"),
    (0x04, "TXSENT"),
    (0x02, "SOFTRST"),
    (0x01, "HARDRST"),
    (0x40, "TOGDONE"),
    (0x20, "SOFTFAIL"),
]


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
        print("%s: ER 0x%02x (%s)" % (what, e, ERRS.get(e, "I2C stage %d%s" % (e & 0x7F, " NACK/buserr" if e & 0x80 else ""))))
        return False
    sys.exit("garbled reply: %r" % r)


def read_caps(s):
    r = xfer(s, b"pdcap", 31)
    if r[:2] != b"OK":
        sys.exit("garbled pdcap reply: %r" % r)
    return [int.from_bytes(r[3 + 4 * i : 7 + 4 * i], "little") for i in range(r[2])]


def pdo_max_mv(pdo):
    """Highest voltage a PDO can put on VBUS (fixed: its voltage; ranges: top)."""
    if pdo >> 30 == 0:
        return ((pdo >> 10) & 0x3FF) * 50
    if pdo >> 30 == 3:
        return ((pdo >> 17) & 0xFF) * 100
    return ((pdo >> 20) & 0x3FF) * 50


def decode_pdo(i, pdo):
    kind = pdo >> 30
    if kind == 0:
        mv, ma = ((pdo >> 10) & 0x3FF) * 50, (pdo & 0x3FF) * 10
        return "PDO %d: fixed  %5.2f V  %.2f A" % (i, mv / 1000, ma / 1000)
    if kind == 1:  # PD Table 6.6: 01b battery, 10b variable
        lo, hi = ((pdo >> 10) & 0x3FF) * 50, ((pdo >> 20) & 0x3FF) * 50
        return "PDO %d: battery %.2f-%.2f V %d W" % (i, lo / 1000, hi / 1000, (pdo & 0x3FF) // 4)
    if kind == 2:
        lo, hi = ((pdo >> 10) & 0x3FF) * 50, ((pdo >> 20) & 0x3FF) * 50
        return "PDO %d: variable %.2f-%.2f V %.2f A" % (i, lo / 1000, hi / 1000, (pdo & 0x3FF) / 100)
    lo, hi = ((pdo >> 8) & 0xFF) * 100, ((pdo >> 17) & 0xFF) * 100
    return "PDO %d: PPS    %5.2f-%.2f V %.2f A" % (i, lo / 1000, hi / 1000, (pdo & 0x7F) / 20)


CTL_NAMES = {1: "GoodCRC", 3: "Accept", 4: "Reject", 5: "Ping", 6: "PS_RDY",
             7: "GetSrcCap", 8: "GetSnkCap", 12: "Wait", 13: "SoftReset"}
DAT_NAMES = {1: "SourceCap", 2: "Request", 4: "SinkCap"}


def hdr_name(h):
    if h == 0:
        return "-"
    nobj = (h >> 12) & 7
    t = h & 0x1F
    name = (DAT_NAMES if nobj else CTL_NAMES).get(t & 0x0F, "type%d" % t)
    return "%s(0x%04x)" % (name, h)


def show_status(s):
    r = xfer(s, b"pdsta", 24)
    if r[:2] != b"OK" or len(r) < 24:
        sys.exit("garbled pdsta reply: %r" % r)
    st = r[2:]
    ev = ",".join(n for b, n in EVENTS if st[6] & b) or "-"
    hdrs = [st[14] | (st[15] << 8), st[16] | (st[17] << 8), st[18] | (st[19] << 8)]
    print(
        "state=%s cc=%s rev=%s caps=%d contract=PDO%d req=PDO%d err=%s\n"
        "events=%s status0=0x%02x (vbus %s, bc_lvl %d) status1=0x%02x msgid=%d rx=%d\n"
        "last rx: %s"
        % (
            STATES.get(st[0], st[0]),
            st[1] or "-",
            {1: "2.0", 2: "3.0"}.get(st[13], st[13]),
            st[2],
            st[3],
            st[4],
            ("0x%02x (%s)" % (st[5], ERRS.get(st[5], "I2C stage"))) if st[5] else "-",
            ev,
            st[7],
            "OK" if st[7] & 0x80 else "low",
            st[7] & 3,
            st[8],
            st[9],
            st[10] | (st[11] << 8),
            "  ".join(hdr_name(h) for h in hdrs),
        )
    )
    fl = [n for b, n in ((1, "tx_pend"), (2, "cable-listen"), (4, "asked")) if st[20] & b]
    print("flags: %s%s" % (",".join(fl) or "-", "  last I2C error at reg 0x%02x" % st[21] if st[5] else ""))
    if st[0] == 7:  # sniff: byte 3 = a sink's Rd on the locked CC
        print("sniff: sink %s" % ("ATTACHED" if st[3] else "not attached"))
    return st


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


# pdsnd presets: (message type, bit 7 = extended; data objects)
ASK = {
    "srccap": (7, []),       # Get_Source_Cap
    "snkcap": (8, []),       # Get_Sink_Cap (dual-role sources only)
    "srccapext": (17, []),   # Get_Source_Cap_Extended (PD 3.x)
    "status": (18, []),      # Get_Status
    "ppsstatus": (20, []),   # Get_PPS_Status
    "countries": (21, []),   # Get_Country_Codes
    "srcinfo": (23, []),     # Get_Source_Info
    "revision": (24, []),    # Get_Revision
    # Get_Manufacturer_Info: extended header (chunked, 2 data bytes) + target 0 (port), ref 0
    "mfrinfo": (0x80 | 6, [0x00008002]),
}


def ask(s, what):
    if what.startswith("raw:"):
        f = what.split(":")[1:]
        t, objs = int(f[0], 0), [int(x, 16) for x in f[1:]]
    elif what == "discid":  # SOP Discover Identity REQ, SVDM version per the negotiated rev
        st = xfer(s, b"pdsta", 24)
        t, objs = 15, [0xFF00A001 if st[2 + 13] >= 2 else 0xFF008001]
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
        trigger_guard.confirm(guard, (mv or pdo_max_mv(pdos[n - 1])) / 1000, what)
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
