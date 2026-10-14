#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial"]
# ///
"""Dump the meter's datalog with the stock `gtrec` verb: N x 8-B records
(V u32 LE 1e-4 V, I u32 LE 1e-5 A) then "reck". Recording is controlled by
the `log_on` / `log_interval_s` settings (tools/cfg.py); `cfg.py set
log_clear 1` empties the log.

  gtrec.py              print the records
  gtrec.py --csv f.csv  also write them as CSV (index, V, A)
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402


def gtrec(s, cap=30.0):
    s.reset_input_buffer()
    s.write(b"gtrec")
    buf = b""
    end = time.time() + cap
    while time.time() < end:
        buf += s.read(max(1, s.in_waiting))
        if buf.endswith(b"reck") and len(buf) % 8 == 4:
            body = buf[:-4]
            return [struct.unpack_from("<II", body, k) for k in range(0, len(body), 8)]
    raise RuntimeError(f"no 'reck' trailer after {cap}s ({len(buf)} B)")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    ap.add_argument("--csv", help="write records to this CSV file")
    a = ap.parse_args()
    for c in meter_candidates():
        try:
            s = serial.Serial(c, 115200, timeout=0.2)
        except Exception:
            continue
        recs = gtrec(s)
        s.close()
        print(f"{len(recs)} records")
        for n, (v, i) in enumerate(recs):
            print(f"{n:5d}  {v / 1e4:8.4f} V  {i / 1e5:8.5f} A")
        if a.csv:
            with open(a.csv, "w") as f:
                f.write("index,V,A\n")
                for n, (v, i) in enumerate(recs):
                    f.write(f"{n},{v / 1e4},{i / 1e5}\n")
        return
    sys.exit("no meter")


if __name__ == "__main__":
    main()
