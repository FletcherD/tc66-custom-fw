#!/usr/bin/env python3
"""Bench test of the firmware watchdog (wdg.rs).

  wdgtest.py            read the boot reset cause (rstrd), hang the main
                        loop (wdtst), time the reboot, read the cause again
  wdgtest.py --cause    only print the reset cause

RCU_RSTSCK bits: 26 POR, 27 NRST pin, 28 software (AIRCR), 29 FWDGT,
30 WWDGT, 31 low-power.
"""
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

BITS = {26: "POR", 27: "NRST", 28: "SW", 29: "FWDGT", 30: "WWDGT", 31: "LPWR"}


def open_meter(timeout=1.5):
    ports = meter_candidates()
    if not ports:
        sys.exit("no meter found")
    s = serial.Serial(ports[0], 115200, timeout=timeout)
    s.reset_input_buffer()
    return s


def cause(s):
    s.reset_input_buffer()
    s.write(b"rstrd")
    r = s.read(6)
    if r[:2] != b"OK" or len(r) != 6:
        raise IOError("rstrd failed: %r" % r)
    v = int.from_bytes(r[2:], "little")
    names = [n for b, n in BITS.items() if v >> b & 1]
    return v, names


def main():
    s = open_meter()
    v, names = cause(s)
    print("reset cause: 0x%08x %s" % (v, names))
    if "--cause" in sys.argv:
        return
    s.write(b"wdtst")
    print("wdtst ->", s.read(2))
    t0 = time.time()
    s.close()
    # the port disappears on reset and comes back after re-enumeration
    gone = None
    while time.time() - t0 < 40:
        ports = meter_candidates()
        if not ports and gone is None:
            gone = time.time()
            print("port gone after %.1f s" % (gone - t0))
        elif ports and gone is not None:
            print("port back after %.1f s" % (time.time() - t0))
            break
        time.sleep(0.2)
    else:
        sys.exit("no reboot observed within 40 s")
    time.sleep(2.0)
    s = open_meter()
    s.write(b"query")
    print("query ->", s.read(4))
    v, names = cause(s)
    print("reset cause: 0x%08x %s" % (v, names))
    if "FWDGT" not in names:
        sys.exit("FAIL: watchdog reset flag not set")
    print("PASS")


if __name__ == "__main__":
    main()
