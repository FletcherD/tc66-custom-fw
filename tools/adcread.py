#!/usr/bin/env python3
"""Host side of the firmware's `adcrd` dev verb.

  adcread.py <channel>      sum of 16 conversions of one channel
  adcread.py --scan         channels 0..17 (16 = internal temp, 17 = Vrefint)
"""
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402


def open_meter():
    ports = meter_candidates()
    if not ports:
        sys.exit("no meter found")
    s = serial.Serial(ports[0], 115200, timeout=1.5)
    s.reset_input_buffer()
    return s


def adcrd(s, ch):
    s.reset_input_buffer()
    s.write(b"adcrd" + bytes([ch]))
    r = s.read(6)
    if r[:2] == b"OK" and len(r) == 6:
        return int.from_bytes(r[2:6], "little")
    stages = {1: "RSTCLB stuck", 2: "CLB stuck", 3: "EOC timeout", 0x0F: "bad args"}
    if r[:2] == b"ER" and len(r) >= 3:
        raise IOError("adcrd ch%d: stage %d (%s)" % (ch, r[2], stages.get(r[2], "?")))
    raise IOError("adcrd ch%d failed: %r" % (ch, r))


def show(ch, sum16):
    volts = sum16 * 3.3 / 65520  # assuming 3.3 V Vref, 12-bit, sum of 16
    print("ch %2d: sum16=%5d  avg=%6.1f  ~%.3f V" % (ch, sum16, sum16 / 16, volts))


def main():
    s = open_meter()
    if len(sys.argv) >= 2 and sys.argv[1] == "--scan":
        for ch in range(18):
            try:
                show(ch, adcrd(s, ch))
            except IOError as e:
                print("ch %2d: %s" % (ch, e))
        return
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    ch = int(sys.argv[1], 0)
    show(ch, adcrd(s, ch))


if __name__ == "__main__":
    main()
