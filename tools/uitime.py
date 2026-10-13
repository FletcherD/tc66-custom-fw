#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial"]
# ///
"""Time full-screen redraws via the `uiscr` dev verb (select screen n, draw
to completion, then reply "OK"+n). Cycles screens 0..2 and prints the
round-trip per switch: a blocking redraw of all 24 cells, so it tracks
display-path speed across builds."""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--rounds", type=int, default=2, help="passes over screens 0..2")
    a = ap.parse_args()
    s = None
    for c in meter_candidates():
        s = serial.Serial(c, 115200, timeout=1.0)
        s.reset_input_buffer()
        s.write(b"query")
        if s.read(4) == b"firm":
            break
        s.close()
        s = None
    if s is None:
        sys.exit("meter not found")
    s.timeout = 5.0
    for _ in range(a.rounds):
        for n in (0, 1, 2):
            t = time.monotonic()
            s.write(b"uiscr" + bytes([n]))
            r = s.read(3)
            print(f"screen {n}: {r!r} {1e3 * (time.monotonic() - t):7.1f} ms")


if __name__ == "__main__":
    main()
