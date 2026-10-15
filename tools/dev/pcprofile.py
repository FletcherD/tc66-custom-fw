#!/usr/bin/env python3
"""Statistical PC profiler over CDC (no SWD, no wedge risk).

The TIMER5_DAC entry veneer (ina.rs) samples the interrupted context's PC
at every sampler tick (~446 Hz) into a SRAM ring; the `pcprf` verb dumps it (u32 LE counter +
128 u32 LE PCs). This polls the ring, dedups by counter, symbolizes with
nm, and histograms — split by fast (<0x08008000) vs slow flash.

Sampling notes: thread-mode PCs only (USBD_LP time is attributed to the
interrupted thread PC on tail-chains; the TIMER5 tick's own work is
invisible); tick-cadence thread work is under-sampled.

Usage: pcprofile.py <elf> [seconds] [--load VERB[:REPLYLEN]]
  --load getva:192   interleave a getva between polls (default: none)
"""
import os
import bisect
import struct
import subprocess
import sys
import time

FAST_END = 0x0800_8000
RING = 128


def main():
    elf = sys.argv[1]
    secs = float(sys.argv[2]) if len(sys.argv) > 2 else 30
    load = None
    if "--load" in sys.argv:
        spec = sys.argv[sys.argv.index("--load") + 1]
        verb, _, rlen = spec.partition(":")
        load = (verb.encode(), int(rlen) if rlen else 0)

    nm = subprocess.run(["arm-none-eabi-nm", "-S", elf],
                        capture_output=True, text=True, check=True).stdout
    syms = []
    for line in nm.splitlines():
        p = line.split()
        if len(p) == 4 and p[2].lower() in ("t", "w"):
            a, s = int(p[0], 16) & ~1, int(p[1], 16)
            syms.append((a, a + s, p[3]))
    syms.sort()
    starts = [s[0] for s in syms]

    def sym_for(pc):
        i = bisect.bisect_right(starts, pc) - 1
        if i >= 0 and syms[i][0] <= pc < syms[i][1]:
            return syms[i][2]
        return f"?{pc:#010x}"

    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../lib"))
    import serial
    import tc66lib as t
    s = serial.Serial(t.meter_candidates()[0], 115200, timeout=2)

    # resync: a previously aborted read may have left a stale reply queued
    for _ in range(3):
        s.reset_input_buffer()
        s.write(b"query")
        if s.read(4) == b"firm":
            break
        time.sleep(0.5)
    else:
        sys.exit("pcprofile: no query->firm, device not in sync")

    def pcprf():
        s.write(b"pcprf")
        raw = s.read(4 + 4 * RING)
        assert len(raw) == 4 + 4 * RING, f"short pcprf reply {len(raw)}"
        words = struct.unpack(f"<{1 + RING}I", raw)
        return words[0], words[1:]

    pcs = []
    last, _ = pcprf()
    end = time.time() + secs
    polls = 0
    while time.time() < end:
        if load:
            s.write(load[0])
            if load[1]:
                s.read(load[1])
        cnt, ring = pcprf()
        polls += 1
        fresh = min(cnt - last, RING - 8)  # guard: ring keeps filling mid-copy
        for k in range(1, fresh + 1):
            pcs.append(ring[(cnt - k) % RING])
        last = cnt
    s.close()

    from collections import Counter
    hist = Counter(sym_for(pc) for pc in pcs)
    total = len(pcs)
    fast = sum(1 for pc in pcs if pc < FAST_END)
    print(f"{total} samples over {secs:.0f}s ({polls} polls"
          f"{', load=' + load[0].decode() if load else ''}) | "
          f"fast {fast/total:.1%} | slow {(total-fast)/total:.1%}")
    addr_of = {nn: st for st, en, nn in syms}
    for name, c in hist.most_common(20):
        region = "fast" if addr_of.get(name, 1 << 32) < FAST_END else "SLOW"
        print(f"{c:6d} {c/total:6.1%}  [{region}] {name}")


if __name__ == "__main__":
    main()
