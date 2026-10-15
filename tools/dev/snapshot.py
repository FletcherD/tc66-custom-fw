#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Snapshot the meter's observable hardware state and diff two snapshots —
for finding which pin/ADC/register reflects a physical change (a switch, a
cable). Needs a dev build with `dev-board` (`make flash FEATURES=dev-board`).

    snapshot.py save before.json        # read everything, write JSON
    snapshot.py diff before.json after.json

A snapshot holds: GPIOA/B/F input levels (gpird) and every pin's
mode/pull/output config (gpiot read-only), ADC channels 0-17 sampled
--passes times (min/max band), FUSB302 config + status registers (not the
read-to-clear interrupt registers), INA226 registers, rawva, pdsta, clkrd,
rstrd. Read-only except gpird, which (re)sets PA15 to input + pull-up —
already its state in every build (FUSB302 INT_N).
"""
import argparse, json, os, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/../lib")
from tc66lib import open_meter  # noqa: E402
from rawva_proto import rawva  # noqa: E402

PORTS = {"A": 0, "B": 1, "F": 2}
FUSB = 0x22
FUSB_REGS = list(range(0x01, 0x0F)) + [0x3C, 0x3D, 0x40, 0x41]  # skip 0x3E/0x3F/0x42 (clear on read)
INA = 0x4A
INA_REGS = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0xFE, 0xFF]


def cmd(s, pkt, n):
    s.reset_input_buffer()
    s.write(pkt)
    r = s.read(n)
    if r[:2] != b"OK":
        raise IOError("%r -> %r" % (pkt, r))
    return r[2:]


def i2c(s, addr, reg, n):
    return cmd(s, b"i2crd" + bytes([addr, 1, reg, 0, n]), 2 + n)


def adc(s, ch):
    return int.from_bytes(cmd(s, b"adcrd" + bytes([ch]), 6), "little") / 16


def take(s, passes):
    snap = {"time": time.strftime("%Y-%m-%d %H:%M:%S")}
    snap["versn"] = cmd(s, b"versn", 5).decode(errors="replace")
    r = cmd(s, b"gpird", 8)
    snap["gpio_in"] = {p: int.from_bytes(r[2 * i:2 * i + 2], "little") for i, p in enumerate("ABF")}
    cfg = {}
    for p, n in PORTS.items():
        for pin in range(16):
            s.reset_input_buffer()
            s.write(b"gpiot" + bytes([n, pin, 0x80]))
            r = s.read(3)
            if r[:2] == b"OK":
                cfg["P%s%d" % (p, pin)] = r[2]
    snap["gpio_cfg"] = cfg
    bands = {}
    for _ in range(passes):
        for ch in range(18):
            v = adc(s, ch)
            lo, hi = bands.get(ch, (v, v))
            bands[ch] = (min(lo, v), max(hi, v))
    snap["adc"] = {str(ch): list(b) for ch, b in bands.items()}
    snap["fusb"] = {"%02x" % r: i2c(s, FUSB, r, 1)[0] for r in FUSB_REGS}
    snap["ina"] = {"%02x" % r: int.from_bytes(i2c(s, INA, r, 2), "big") for r in INA_REGS}
    snap["rawva"] = {k: v for k, v in rawva(s).items() if k not in ("seq", "mAh", "mWh")}
    snap["pdsta"] = cmd(s, b"pdsta", 24).hex()
    snap["clkrd"] = cmd(s, b"clkrd", 10).hex()
    snap["rstrd"] = cmd(s, b"rstrd", 6).hex()
    return snap


def cfg_str(c):
    mode = ["in", "out", "af", "an"][c & 3]
    pud = ["", "+pu", "+pd", "+?"][(c >> 2) & 3]
    return "%s%s%s" % (mode, pud, " hi" if (mode == "out" and c & 0x10) else "")


def diff(a, b, adc_margin):
    found = 0
    for p in "ABF":
        x = a["gpio_in"][p] ^ b["gpio_in"][p]
        for pin in range(16):
            if x >> pin & 1:
                name = "P%s%d" % (p, pin)
                c = a["gpio_cfg"].get(name)
                print("GPIO %-5s %d -> %d   (%s)" % (name, a["gpio_in"][p] >> pin & 1, b["gpio_in"][p] >> pin & 1,
                                                      cfg_str(c) if c is not None else "?"))
                found += 1
    for k in sorted(set(a["gpio_cfg"]) | set(b["gpio_cfg"])):
        if a["gpio_cfg"].get(k) != b["gpio_cfg"].get(k):
            print("CFG  %-5s %s -> %s" % (k, cfg_str(a["gpio_cfg"][k]), cfg_str(b["gpio_cfg"][k])))
            found += 1
    for ch in sorted(a["adc"], key=int):
        (alo, ahi), (blo, bhi) = a["adc"][ch], b["adc"][ch]
        if blo > ahi + adc_margin or bhi < alo - adc_margin:
            print("ADC  ch%-3s %6.0f..%-6.0f -> %6.0f..%-6.0f  (~%.3f -> %.3f V)"
                  % (ch, alo, ahi, blo, bhi, (alo + ahi) / 2 * 3.3 / 4095, (blo + bhi) / 2 * 3.3 / 4095))
            found += 1
    for grp in ("fusb", "ina"):
        for r in a[grp]:
            if a[grp][r] != b[grp].get(r):
                print("%-4s reg %s  0x%x -> 0x%x" % (grp.upper(), r, a[grp][r], b[grp][r]))
                found += 1
    for k in a["rawva"]:
        if a["rawva"][k] != b["rawva"].get(k):
            print("RAWVA %-8s %s -> %s" % (k, a["rawva"][k], b["rawva"][k]))
    for k in ("pdsta", "clkrd", "rstrd", "versn"):
        if a[k] != b[k]:
            print("%-5s %s\n   -> %s" % (k.upper(), a[k], b[k]))
    print("%d GPIO/ADC/register differences (rawva/pdsta lines are informational)" % found)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sv = sub.add_parser("save", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    sv.add_argument("out")
    sv.add_argument("--passes", type=int, default=5, help="ADC scans per snapshot (noise band)")
    df = sub.add_parser("diff", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    df.add_argument("a")
    df.add_argument("b")
    df.add_argument("--adc-margin", type=float, default=20, help="ADC counts beyond the noise bands")
    args = ap.parse_args()
    if args.cmd == "save":
        s = open_meter()
        snap = take(s, args.passes)
        with open(args.out, "w") as f:
            json.dump(snap, f, indent=1)
        print("saved %s (build %s): GPIOA=%04x B=%04x F=%04x, %d pin configs"
              % (args.out, snap["versn"], snap["gpio_in"]["A"], snap["gpio_in"]["B"], snap["gpio_in"]["F"],
                 len(snap["gpio_cfg"])))
    else:
        with open(args.a) as f:
            a = json.load(f)
        with open(args.b) as f:
            b = json.load(f)
        diff(a, b, args.adc_margin)


if __name__ == "__main__":
    main()
