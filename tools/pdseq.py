#!/usr/bin/env python3
"""Run a PD capture (tools/pdcap.py under sudo) while driving the engine
through a sequence of tools/pd.py commands, then print the decoded capture.

    pdseq.py [--secs N] [--save cap.hex] STEP...

A STEP is either `wait:<seconds>` or a pd.py command line with words joined
by `:` (e.g. `arm`, `status`, `ask:srccapext`, `hardreset`, `off`). Steps
run in order, starting 1 s after the capture; trigger requests (`req:...`)
need `--yes` passed through as their own step word (`req:2:--yes`).

    pdseq.py --secs 12 arm wait:5 status info ask:srccapext wait:1 status off

Needs passwordless sudo for pdcap.py (USB vendor interface).
"""
import argparse
import os
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--secs", type=float, default=15, help="capture duration (default: %(default)s)")
    ap.add_argument("--save", metavar="FILE", help="also keep the raw capture for pdcap.py --replay (default: none)")
    ap.add_argument("steps", nargs="+", help="pd.py commands (words joined by ':') or wait:<s>")
    a = ap.parse_args()

    py = subprocess.run(["uv", "run", "-q", "--script", os.path.join(HERE, "pdcap.py"), "--python-path"],
                        capture_output=True, text=True, check=True).stdout.strip()
    cmd = ["sudo", py, os.path.join(HERE, "pdcap.py"), str(a.secs)] + (["--save", a.save] if a.save else [])
    cap = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    t0 = time.time()
    time.sleep(1)
    for step in a.steps:
        if step.startswith("wait:"):
            time.sleep(float(step[5:]))
            continue
        r = subprocess.run(["uv", "run", "-q", os.path.join(HERE, "pd.py")] + step.split(":"),
                           capture_output=True, text=True)
        print("[%5.1f s] pd.py %s\n%s" % (time.time() - t0, step.replace(":", " "),
                                          (r.stdout + r.stderr).rstrip()))
    out, _ = cap.communicate()
    print("---- capture ----")
    print(out.rstrip())


if __name__ == "__main__":
    main()
