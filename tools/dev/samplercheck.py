#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial", "tqdm"]
# ///
"""Sampler health + noise over `rawva`: published window rate (seq advance
per second of host time — expect ~10.4 Hz), missed windows, and V / I mean,
standard deviation and the window-to-window difference spread (sd of
consecutive differences / sqrt 2: insensitive to slow load drift, so it
compares builds' noise under a real, wandering load)."""
import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../lib"))
import serial  # noqa: E402
from tqdm import tqdm  # noqa: E402
from rawva_proto import rawva  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402


def spread(xs):
    d = [b - a for a, b in zip(xs, xs[1:])]
    return statistics.pstdev(d) / 2**0.5


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("seconds", type=float, nargs="?", default=30.0, help="capture length")
    ap.add_argument("--poll-hz", type=float, default=40.0, help="rawva poll rate")
    args = ap.parse_args()

    s = serial.Serial(meter_candidates()[0], 115200, timeout=1.0)
    s.reset_input_buffer()
    v, i, seqs = [], [], []
    last = None
    t_start = time.time()
    with tqdm(total=args.seconds, unit="s", bar_format="{l_bar}{bar}| {n:.0f}/{total:.0f} s") as bar:
        while (now := time.time()) - t_start < args.seconds:
            smp = rawva(s)
            if smp["seq"] != last:
                last = smp["seq"]
                seqs.append((now, smp["seq"]))
                v.append(smp["V"])
                i.append(smp["A"])
            bar.n = min(now - t_start, args.seconds)
            bar.refresh()
            time.sleep(max(0.0, 1 / args.poll_hz - (time.time() - now)))
    s.close()

    (t0, q0), (t1, q1) = seqs[0], seqs[-1]
    missed = sum(b - a - 1 for (_, a), (_, b) in zip(seqs, seqs[1:]) if b - a > 1)
    print(f"windows: {len(seqs)}  rate {(q1 - q0) / (t1 - t0):.3f} Hz  missed {missed}")
    print(f"V: mean {statistics.mean(v):.5f} V  sd {statistics.pstdev(v) * 1e3:.3f} mV  "
          f"w2w {spread(v) * 1e3:.3f} mV")
    print(f"I: mean {statistics.mean(i) * 1e3:.3f} mA  sd {statistics.pstdev(i) * 1e3:.3f} mA  "
          f"w2w {spread(i) * 1e3:.3f} mA")


if __name__ == "__main__":
    main()
