#!/usr/bin/env python3
"""Continuous 2-channel scope capture — run it, trigger all the protocols you
want, then Ctrl-C to stop. Saves every sample to a CSV for later analysis.

  scope_capture.py [out.csv]        default: /tmp/scope_all.csv
  rate/range are fixed for the D+/D- serial work (100 kS/s, 5 V range, x1).

Uses the OWON VDS1022 gapless roll-mode read (read_iter). Prints a live
one-line status (sample count + D-/D+ levels) so you can see it's alive.
"""
import sys
import time

sys.path.insert(0, "/home/fletcher/.claude/skills/owon-vds1022")
from vds1022.vds1022 import VDS1022, CH1, CH2, DC  # noqa: E402

RATE = 100e3           # 100 kS/s = 10 us/sample (~15 samples per 152 us bit)
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/scope_all.csv"


def main():
    dev = VDS1022()
    dev.set_sampling(RATE, roll=True)
    dev.set_channel(CH1, range=5, offset=0.5, probe=1, coupling=DC)
    dev.set_channel(CH2, range=5, offset=0.5, probe=1, coupling=DC)

    f = open(OUT, "w")
    f.write("idx,ch1_V,ch2_V\n")
    n = 0
    t0 = time.time()
    last_print = 0.0
    print("Capturing to %s at %d kS/s. Trigger your protocols. Ctrl-C to stop.\n"
          % (OUT, RATE / 1e3))
    try:
        for frames in dev.read_iter():
            y1 = frames[CH1].y()
            y2 = frames[CH2].y()
            m = min(len(y1), len(y2))
            for i in range(m):
                f.write("%d,%.4f,%.4f\n" % (n, y1[i], y2[i]))
                n += 1
            now = time.time()
            if now - last_print > 0.25 and m:
                last_print = now
                sys.stdout.write(
                    "\r  %6.1fs  %8d samples   CH1(D-)=%+.2fV  CH2(D+)=%+.2fV   "
                    % (now - t0, n, y1[-1], y2[-1])
                )
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            dev.stop()
        except Exception:
            pass
        try:
            dev.dispose()
        except Exception:
            pass
        f.close()
        print("\n\nStopped. Wrote %d samples (%.1fs) to %s" % (n, n / RATE, OUT))


if __name__ == "__main__":
    main()
