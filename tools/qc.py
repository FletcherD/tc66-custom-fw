#!/usr/bin/env python3
"""Host side of the QC2.0 D+/D- trigger dev verbs (qctrg/qcoff, pd.rs qc.rs).

  qc.py <5|9|12|20>    request that voltage via QC2.0 (first call blocks ~1.5 s
                       for QC entry) — then reads the meter's own bus voltage
  qc.py off            release D+/D- (charger reverts to 5 V), restores SWD
  qc.py sweep          5 -> 9 -> 12 -> 20 -> 5, reading bus at each step

DANGER: a QC trigger RAISES THE OUT-PORT VOLTAGE. Nothing downstream.
Only works with a QC2.0-capable charger on IN (non-PD; a PD source ignores
D+/D-). PA13/PA14 are SWDIO/SWCLK — SWD is dead until `qc.py off`.
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

MODES = {5: 0, 9: 1, 12: 2, 20: 3}


def open_meter():
    ports = meter_candidates()
    if not ports:
        sys.exit("no meter found")
    s = serial.Serial(ports[0], 115200, timeout=3)
    s.reset_input_buffer()
    return s


def xfer(s, pkt):
    s.reset_input_buffer()
    s.write(pkt)
    return s.read(3)


def ok(r, what):
    if r[:2] == b"OK":
        print("%s: OK" % what)
        return True
    e = r[2] if len(r) > 2 and r[:2] == b"ER" else -1
    print("%s: %r" % (what, r))
    return False


def bus(s):
    s.reset_input_buffer()
    s.write(b"rawva")
    r = s.read(46)
    if len(r) >= 16 and r[:4] == b"raw1":
        return int.from_bytes(r[8:12], "little") / 1e4
    return None


def trig(s, volts):
    ok(xfer(s, b"qctrg" + bytes([MODES[volts]])), "QC %dV" % volts)
    time.sleep(0.6)
    print("   bus: %.3f V" % (bus(s) or 0))


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    s = open_meter()
    cmd = sys.argv[1]
    if cmd == "off":
        ok(xfer(s, b"qcoff"), "off")
    elif cmd == "sweep":
        for v in (5, 9, 12, 20, 5):
            trig(s, v)
    else:
        v = int(cmd)
        if v not in MODES:
            sys.exit("voltage must be one of 5 9 12 20")
        trig(s, v)


if __name__ == "__main__":
    main()
