#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial", "tqdm"]
# ///
"""Round-trip latency of CDC verbs (write verb, read the full reply): median /
p10 / p90 / max over N runs per verb. Compares reply-path speed across builds
(getva = AES compat packet, rawva = plaintext, query = minimal)."""
import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial  # noqa: E402
from tqdm import tqdm  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

REPLY_LEN = {"getva": 192, "rawva": 46, "query": 4}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("verbs", nargs="*", default=["query", "rawva", "getva"],
                    help=f"verbs to time (known reply lengths: {REPLY_LEN})")
    ap.add_argument("-n", type=int, default=300, help="runs per verb")
    args = ap.parse_args()
    ports = meter_candidates()
    if not ports:
        sys.exit("no meter found")
    s = serial.Serial(ports[0], 115200, timeout=1.0)
    for verb in args.verbs:
        if verb not in REPLY_LEN:
            sys.exit(f"unknown reply length for {verb!r}")
        n = REPLY_LEN[verb]
        s.reset_input_buffer()
        ts = []
        for _ in tqdm(range(args.n), desc=verb, leave=False):
            t0 = time.perf_counter()
            s.write(verb.encode())
            r = s.read(n)
            ts.append((time.perf_counter() - t0) * 1e3)
            if len(r) != n:
                sys.exit(f"{verb}: short reply {len(r)}/{n} B")
        ts.sort()
        q = lambda f: ts[min(len(ts) - 1, int(f * len(ts)))]
        print(f"{verb:6} n={len(ts)}  median {statistics.median(ts):6.2f} ms  "
              f"p10 {q(0.1):6.2f}  p90 {q(0.9):6.2f}  max {ts[-1]:6.2f}")


if __name__ == "__main__":
    main()
