#!/usr/bin/env python3
"""Host side of the firmware's `tftfl`/`tftbl` dev verbs.

  tft.py pattern            white border + R/G/B bands test pattern
  tft.py fill <rgb565>      solid fill, e.g. 0xF800 (red) 0x07E0 (green)
  tft.py bl <0..999>        backlight PWM compare (0 = dark, 999 = full)
  tft.py ui                 draw/refresh the main screen (V/I/P lines)
"""
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

STAGES = {1: "SPI TBE timeout", 2: "SPI shift-idle timeout", 0x0F: "bad args"}


def open_meter():
    ports = meter_candidates()
    if not ports:
        sys.exit("no meter found")
    s = serial.Serial(ports[0], 115200, timeout=3)
    s.reset_input_buffer()
    return s


def verb(s, cmd):
    s.reset_input_buffer()
    s.write(cmd)
    r = s.read(2)
    if r == b"OK":
        return
    if r == b"ER":
        stage = s.read(1)
        st = stage[0] if stage else -1
        raise IOError("stage %d (%s)" % (st, STAGES.get(st, "?")))
    raise IOError("bad reply: %r" % r)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    s = open_meter()
    op = sys.argv[1]
    if op == "pattern":
        verb(s, b"tftfl")
    elif op == "fill":
        c = int(sys.argv[2], 0)
        verb(s, b"tftfl" + c.to_bytes(2, "little"))
    elif op == "ui":
        verb(s, b"tftui")
    elif op == "bl":
        v = int(sys.argv[2], 0)
        verb(s, b"tftbl" + v.to_bytes(2, "little"))
    else:
        sys.exit(__doc__)
    print("OK")


if __name__ == "__main__":
    main()
