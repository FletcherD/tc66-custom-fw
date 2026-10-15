#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial"]
# ///
"""Enumeration reliability after reboots (docs/QUIRKS.md §6): reboot the meter
N times via `updat` (`make flash` in the rust dir, any extra make args passed
through), and per boot report how long until it answers `versn` and how many
configuration-descriptor reads the kernel failed (journalctl -k). If the host
gives up, cycle the meter's hub port to make it retry — with the meter powered
from IN that does not reboot it (powered from USB it is a cold boot)."""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../lib"))
import serial  # noqa: E402
from tc66lib import HUB_TOOL, find_meter_hub_port, meter_candidates  # noqa: E402


def alive():
    c = meter_candidates()
    if not c:
        return None
    try:
        s = serial.Serial(c[0], 115200, timeout=1)
        s.reset_input_buffer()
        s.write(b"versn")
        r = s.read(8)
        s.close()
        return r.decode(errors="replace") if r.startswith(b"OK") else None
    except (serial.SerialException, OSError):
        return None


def kernel_failures(since):
    out = subprocess.run(["journalctl", "-k", "--no-pager", "--since", since],
                         capture_output=True, text=True).stdout
    return out.count("can't read configurations")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("n", type=int, nargs="?", default=10, help="reboots")
    ap.add_argument("--rust", default=os.path.join(HERE, "../../rust"),
                    help="rust dir to run `make flash` in")
    ap.add_argument("--make-args", default="", help="extra make args, e.g. FEATURES=grace")
    ap.add_argument("--kick-after", type=float, default=60.0,
                    help="seconds without an answer before cycling the hub port")
    ap.add_argument("--give-up", type=float, default=150.0, help="seconds before stopping")
    args = ap.parse_args()

    port = find_meter_hub_port()
    if not port:
        sys.exit("meter not on the hub")
    totals = {"boots": 0, "failed": 0, "kicked": 0}
    for k in range(args.n):
        since = time.strftime("%Y-%m-%d %H:%M:%S")
        r = subprocess.run(["make", "flash", *args.make_args.split()], cwd=args.rust,
                           capture_output=True, text=True)
        if "COMPLETE" not in r.stdout:
            sys.exit(f"boot {k}: flash failed\n{r.stdout[-500:]}{r.stderr[-500:]}")
        t0 = time.time()
        kicked = False
        v = None
        while time.time() - t0 < args.give_up:
            time.sleep(0.5)
            v = alive()
            if v:
                break
            if not kicked and time.time() - t0 > args.kick_after:
                subprocess.run(["python3", HUB_TOOL, "cycle", port], capture_output=True)
                kicked = True
        fails = kernel_failures(since)
        totals["boots"] += 1
        totals["failed"] += fails > 0
        totals["kicked"] += kicked
        print(f"boot {k}: {v} after {time.time() - t0:5.1f} s, failed config reads {fails}"
              f"{' (hub kick)' if kicked else ''}", flush=True)
        if not v:
            sys.exit("meter did not come back")
        time.sleep(3)
    print(f"{totals['failed']}/{totals['boots']} boots with failed config reads, "
          f"{totals['kicked']} needed a hub kick")


if __name__ == "__main__":
    main()
