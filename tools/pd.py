#!/usr/bin/env python3
"""Host side of the firmware's PD sink verbs (M4 bring-up, FUSB302).

  pd.py arm            arm the sink engine (pdctl 1) — then plug the source in
  pd.py off            FUSB302 SW_RES + engine off (pdctl 0)
  pd.py status         one engine-state snapshot (pdsta)
  pd.py caps           decode the stored Source_Capabilities PDOs (pdcap)
  pd.py req <n>        request source PDO n (1-based, fixed-supply) (pdreq)
  pd.py watch [secs]   poll status at 2 Hz (default 20 s)

DANGER: a granted request RAISES THE OUT-PORT VOLTAGE (9/12/15/20 V).
Nothing should be attached downstream of the meter when triggering.
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

STATES = {
    0: "off",
    1: "wait-attach",
    2: "listen (attached)",
    3: "wait-accept",
    4: "wait-ps-rdy",
    5: "CONTRACT",
}
ERRS = {
    0x0E: "not a fixed-supply PDO",
    0x0F: "bad args / bad state",
    0xE1: "tx retries exhausted (no GoodCRC)",
    0xE2: "source rejected the request",
    0xE3: "Accept timeout",
    0xE4: "PS_RDY timeout",
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


def decode_pdo(i, pdo):
    kind = pdo >> 30
    if kind == 0:
        mv, ma = ((pdo >> 10) & 0x3FF) * 50, (pdo & 0x3FF) * 10
        return "PDO %d: fixed  %5.2f V  %.2f A" % (i, mv / 1000, ma / 1000)
    if kind == 1:
        lo, hi = ((pdo >> 10) & 0x3FF) * 50, ((pdo >> 20) & 0x3FF) * 50
        return "PDO %d: variable %.2f-%.2f V %.2f A" % (i, lo / 1000, hi / 1000, (pdo & 0x3FF) / 100)
    if kind == 2:
        lo, hi = ((pdo >> 10) & 0x3FF) * 50, ((pdo >> 20) & 0x3FF) * 50
        return "PDO %d: battery %.2f-%.2f V %d W" % (i, lo / 1000, hi / 1000, (pdo & 0x3FF) // 4)
    lo, hi = ((pdo >> 8) & 0xFF) * 100, ((pdo >> 17) & 0xFF) * 100
    return "PDO %d: PPS    %5.2f-%.2f V %.2f A (pdreq: not yet)" % (i, lo / 1000, hi / 1000, (pdo & 0x7F) / 20)


def show_status(s):
    r = xfer(s, b"pdsta", 16)
    if r[:2] != b"OK" or len(r) < 16:
        sys.exit("garbled pdsta reply: %r" % r)
    st = r[2:]
    ev = ",".join(n for b, n in EVENTS if st[6] & b) or "-"
    print(
        "state=%s cc=%s caps=%d contract=PDO%d req=PDO%d err=%s\n"
        "events=%s status0=0x%02x (vbus %s, bc_lvl %d) status1=0x%02x msgid=%d rx=%d"
        % (
            STATES.get(st[0], st[0]),
            st[1] or "-",
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
        )
    )
    return st


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    cmd = sys.argv[1]
    s = open_meter()
    if cmd == "arm":
        ok_er(xfer(s, b"pdctl\x01", 3), "arm")
    elif cmd == "off":
        ok_er(xfer(s, b"pdctl\x00", 3), "off")
    elif cmd == "status":
        show_status(s)
    elif cmd == "caps":
        r = xfer(s, b"pdcap", 31)
        if r[:2] != b"OK":
            sys.exit("garbled pdcap reply: %r" % r)
        n = r[2]
        if not n:
            print("no source capabilities stored (source attached? PD at all?)")
        for i in range(n):
            pdo = int.from_bytes(r[3 + 4 * i : 7 + 4 * i], "little")
            print(decode_pdo(i + 1, pdo), " raw=0x%08x" % pdo)
    elif cmd == "req":
        n = int(sys.argv[2], 0)
        if ok_er(xfer(s, b"pdreq" + bytes([n]), 3), "request PDO %d" % n):
            time.sleep(0.7)
            show_status(s)
    elif cmd == "watch":
        secs = float(sys.argv[2]) if len(sys.argv) > 2 else 20
        end = time.time() + secs
        while time.time() < end:
            show_status(s)
            time.sleep(0.5)
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
