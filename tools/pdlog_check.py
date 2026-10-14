#!/usr/bin/env python3
"""Summarize a tools/pdcap.py sniff log per hard reset: split at each
`*** HARD RESET ***` (pd.py hardreset re-triggers a negotiation), then per
segment count SOP / SOP' frames and bad-CRC events, and pair every SOP
message with the other side's GoodCRC of the same message id (within 8 ms).
`ack without msg` = a message the sniffer missed; `msg without ack` = a
missed GoodCRC (or an unanswered message). Unanswered Source_Caps (retries)
are not counted.

    pdlog_check.py capture.txt
"""
import argparse
import re


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help="pdcap.py output")
    a = ap.parse_args()

    ev = [(int(m.group(1)), m.group(2)) for line in open(a.log)
          for m in [re.match(r"\s*\[\s*(\d+) ms\] (.*)", line)] if m]
    segs = []
    for t, d in ev:
        if "HARD RESET" in d:
            segs.append((t, []))
        elif segs:
            segs[-1][1].append((t, d))
    for t0, es in segs:
        sop = [(t, d) for t, d in es if not d.startswith(("SOP'", "!!!", "***"))]
        n_sopp = sum(1 for _, d in es if d.startswith("SOP'"))
        bad = [t - t0 for t, d in es if "bad-CRC" in d]
        msgs, acks = [], []
        for t, d in sop:
            m = re.match(r"(\S+)\s+\[(SRC|SNK) r\S+ id(\d)\]", d)
            if m:
                (acks if m.group(1) == "GoodCRC" else msgs).append(
                    (t, m.group(2), int(m.group(3)), m.group(1)))
        used, unacked = set(), 0
        for t, r, k, name in msgs:
            hit = [j for j, (ta, ra, ka, _) in enumerate(acks)
                   if ra != r and ka == k and 0 <= ta - t <= 8 and j not in used]
            if hit:
                used.add(hit[0])
            elif name != "Source_Cap":
                unacked += 1
        print(f"reset @{t0} ms: SOP {len(sop)}, SOP' {n_sopp}, bad-CRC {len(bad)}; "
              f"msg without ack {unacked}, ack without msg {len(acks) - len(used)}")


if __name__ == "__main__":
    main()
