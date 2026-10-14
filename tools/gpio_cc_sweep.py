#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Does any MCU GPIO move a CC line? For every pin `gpiot` allows, set it to
output-low, output-high, input+pull-down and input+pull-up, measure the CC
pin's voltage with the FUSB302 MDAC comparator (no pull-downs: Switches0 =
MEAS_CCx only, as in sniff mode), and restore the pin. Prints pins whose
setting moves the reading by more than one MDAC step. Needs a dev build with
`dev-board`; leave the PD engine off (a reboot leaves it off).
"""
import argparse
import glob
import sys
import time

import serial

PORTS = {0: "A", 1: "B", 2: "F"}
CFGS = {0x01: "out-low", 0x11: "out-high", 0x08: "pull-down", 0x04: "pull-up"}


class Meter:
    def __init__(self):
        ports = glob.glob("/dev/ttyACM*")
        if not ports:
            sys.exit("no meter")
        self.s = serial.Serial(ports[0], 115200, timeout=1)

    def x(self, pkt, n):
        self.s.reset_input_buffer()
        self.s.write(pkt)
        return self.s.read(n)

    def wr(self, reg, val):
        if self.x(b"i2crd" + bytes([0x22, 2, reg, val, 0]), 2) != b"OK":
            sys.exit("i2c write failed")

    def rd(self, reg):
        r = self.x(b"i2crd" + bytes([0x22, 1, reg, 0, 1]), 3)
        if r[:2] != b"OK":
            sys.exit("i2c read failed")
        return r[2]

    def cc_mv(self, settle):
        """MDAC binary search on the selected CC pin -> mV (42 mV steps)."""
        code = 0
        for bit in range(5, -1, -1):
            t = code | 1 << bit
            self.wr(0x04, t)
            time.sleep(settle)
            if self.rd(0x40) & 0x20:
                code = t
        return (code + 1) * 42 if code else 0

    def gpiot(self, port, pin, cfg):
        r = self.x(b"gpiot" + bytes([port, pin, cfg]), 3)
        return r[2] if r[:2] == b"OK" else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--cc", type=int, default=2, choices=(1, 2), help="CC pin to watch")
    ap.add_argument("--settle", type=float, default=0.03, help="seconds after each change")
    a = ap.parse_args()

    m = Meter()
    sw0, meas = m.rd(0x02), m.rd(0x04)
    m.wr(0x0B, 0x0F)  # all FUSB302 blocks on
    m.wr(0x02, 0x04 if a.cc == 1 else 0x08)  # MEAS_CCx, no pull-downs
    base = [m.cc_mv(0.002) for _ in range(5)]
    print(f"CC{a.cc} baseline: {base} mV")
    lo, hi = min(base) - 42, max(base) + 42
    hits = []
    for port, name in PORTS.items():
        for pin in range(16 if port < 2 else 2):
            old = m.gpiot(port, pin, 0x80)
            if old is None:
                continue  # refused (USB / I2C / TFT reset)
            for cfg, label in CFGS.items():
                m.gpiot(port, pin, cfg)
                time.sleep(a.settle)
                v = m.cc_mv(0.002)
                m.gpiot(port, pin, old)
                if not lo <= v <= hi:
                    hits.append((f"P{name}{pin}", label, v))
                    print(f"  P{name}{pin:<2} {label:9s} -> CC{a.cc} {v} mV")
    m.wr(0x02, sw0)
    m.wr(0x04, meas)
    print("no pin moves CC%d" % a.cc if not hits else "%d effect(s)" % len(hits))


if __name__ == "__main__":
    main()
