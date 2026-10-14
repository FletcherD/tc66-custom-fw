# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy", "pandas"]
# ///
"""Decode Huawei FCP/SCP (HiSilicon ACCP) D- frames from a scope CSV capture.

Prints the D- timeline: Mping/Sping as [H x.xx ms], each frame as its decoded
bytes with parity/CRC/SYNC checks against the rule qc.rs's accp_frame emits:
before each byte the SYNC quarters (!P, P, and !P again if the byte's MSB
equals the previous level P), after the last byte (!P, P[, !P]) toward the
high Mping. CSV format: idx,ch1_V,ch2_V (the stock captures fcp.csv/scp.csv,
D- on ch1, 100 kS/s).
"""
import argparse

import numpy as np
import pandas as pd


def crc8(bs):
    c = 0
    for x in bs:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x39) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def sync_rule(vals):
    out, p = [], 0
    for x in vals:
        out.append([1 - p, p] + ([1 - p] if (x >> 7) == p else []))
        p = 1 if bin(x).count("1") % 2 == 0 else 0
    out.append([1 - p, p] + ([1 - p] if p == 1 else []))
    return out


def decode(seg, ui, quarter_max):
    """seg: [(level, us)] starting at the frame's first edge -> (bytes, sync groups)."""
    syms = []
    for lvl, dur in seg:
        if dur <= quarter_max:
            syms.append(("q", lvl))
        else:
            syms += [("b", lvl)] * round(dur / ui)
    byts, cur, syncs, q = [], [], [], []
    for t, lvl in syms:
        if t == "q":
            if cur:
                byts.append(cur)
                cur = []
            q.append(lvl)
        else:
            if q or not syncs:
                syncs.append(q)
                q = []
            cur.append(lvl)
    if cur:
        byts.append(cur)
    syncs.append(q)
    return byts, syncs


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--us", type=float, default=10.0, help="sample period, µs")
    ap.add_argument("--ch", type=int, default=1, help="CSV column holding D-")
    ap.add_argument("--thresh", type=float, default=1.0, help="logic threshold, V")
    ap.add_argument("--ui", type=float, default=155.0, help="bit cell, µs")
    ap.add_argument("--quarter-max", type=float, default=80.0,
                    help="runs up to this long (µs) are SYNC quarters")
    a = ap.parse_args()

    v = pd.read_csv(a.csv).to_numpy()[:, a.ch] > a.thresh
    edges = np.flatnonzero(np.diff(v.astype(np.int8))) + 1
    b = np.concatenate([[0], edges, [len(v)]])
    runs = [(int(v[b[i]]), (b[i + 1] - b[i]) * a.us) for i in range(len(b) - 1)]

    def flush(seg):
        while seg and seg[0][0] == 0:
            seg = seg[1:]
        if not seg:
            return
        byts, syncs = decode(seg, a.ui, a.quarter_max)
        if any(len(x) != 9 for x in byts) or not byts:
            print("   ?", " ".join(f"{'HL'[1 - l]}{d:.0f}" for l, d in seg))
            return
        vals = [int("".join(map(str, x[:8])), 2) for x in byts]
        par = all(sum(x) % 2 == 1 for x in byts)
        crc = len(vals) > 1 and crc8(vals[:-1]) == vals[-1]
        rule = sync_rule(vals)
        sync = syncs[:-1] == rule[:-1] and syncs[-1][:2] == rule[-1][:2]
        print("  ", " ".join(f"{x:02x}" for x in vals),
              "parity-ok" if par else "PARITY-BAD",
              "crc-ok" if crc else "no-crc",
              "sync-ok" if sync else f"SYNC {syncs} expected {rule}")

    seg = []
    for lvl, dur in runs:
        if dur >= (1800 if lvl else 1500):  # pings / gaps; a byte's runs are shorter
            flush(seg)
            seg = []
            print(f"[{'HL'[1 - lvl]} {dur / 1000:.2f} ms]")
        else:
            seg.append((lvl, dur))
    flush(seg)


if __name__ == "__main__":
    main()
