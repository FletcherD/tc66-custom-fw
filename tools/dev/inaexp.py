#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial", "tqdm"]
# ///
"""INA226 conversion-time / noise experiment via the `inaxp` dev verb
(`make flash FEATURES=dev-ina`).

For each CONFIG value the meter waits for conversion-ready (CVRF, polled over
I2C), timestamps it with SysTick and reads the raw shunt + bus registers. This
reports, per config: the measured conversion cycle (continuous modes: mean
interval between CVRFs, poll jitter averages out; triggered modes: trigger ->
CVRF, biased high by up to one ~0.13 ms poll) against the datasheet nominal,
and the raw shunt / bus mean, standard deviation and distinct-code count.
Calls rotate through the configs (32 samples each) so load drift during the
run hits every config equally — compare configs by their mean differences.

CONFIG = 0x4000 | AVG<<9 | VBUSCT<<6 | VSHCT<<3 | MODE; CT codes 0..7 =
140 204 332 588 1100 2116 4156 8244 us; AVG codes 0..7 = 1 4 16 64 128 256
512 1024; MODE 7 = continuous shunt+bus, 3 = triggered shunt+bus.
The sampler's CONFIG is restored after every call.
"""
import argparse
import csv
import os
import statistics
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../lib"))
import serial  # noqa: E402
from tqdm import tqdm  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

CT_US = [140, 204, 332, 588, 1100, 2116, 4156, 8244]
AVG_N = [1, 4, 16, 64, 128, 256, 512, 1024]
SHUNT_UV_LSB = 2.5
BUS_MV_LSB = 1.25
SHUNT_MOHM = 15.0
CHUNK = 32


def cfg_word(vsh, vbus, avg=0, mode=7):
    return 0x4000 | avg << 9 | vbus << 6 | vsh << 3 | mode


def describe(cfg):
    avg, vbus, vsh, mode = (cfg >> 9) & 7, (cfg >> 6) & 7, (cfg >> 3) & 7, cfg & 7
    kind = {7: "cont", 3: "trig"}.get(mode, f"mode{mode}")
    return f"0x{cfg:04X} sh {CT_US[vsh]}us bus {CT_US[vbus]}us avg {AVG_N[avg]} {kind}"


def nominal_cycle_us(cfg):
    avg, vbus, vsh = (cfg >> 9) & 7, (cfg >> 6) & 7, (cfg >> 3) & 7
    return AVG_N[avg] * (CT_US[vsh] + CT_US[vbus])


def inaxp(s, cfg, n):
    s.reset_input_buffer()
    s.write(b"inaxp" + struct.pack(">HB", cfg, n))
    want = 2 + 8 + 8 * n
    r = s.read(want)
    if r[:2] == b"ER":
        sys.exit(f"inaxp 0x{cfg:04X}: ER stage 0x{r[2]:02X}" if len(r) > 2 else "inaxp: short ER")
    if len(r) != want or r[:2] != b"OK":
        sys.exit(f"inaxp 0x{cfg:04X}: bad reply ({len(r)} B of {want}): {r[:16].hex()}")
    csr, rvr = struct.unpack_from("<II", r, 2)
    recs = [struct.unpack_from("<Ihh", r, 10 + 8 * i) for i in range(n)]
    return csr, rvr, recs


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--cfg", type=lambda x: int(x, 0), nargs="*",
                    help="CONFIG words to test (default: continuous sh=bus CT 588..8244 us, "
                         "stock 0x41A7, triggered 1100/4156 us)")
    ap.add_argument("-n", type=int, default=256,
                    help="samples per config (sent in calls of %d) (default: %%(default)s)" % CHUNK)
    ap.add_argument("--csv", help="write every raw sample to this CSV")
    ap.add_argument("--port", help="serial port (default: first meter found)")
    args = ap.parse_args()

    cfgs = args.cfg or (
        [cfg_word(c, c) for c in range(3, 8)]
        + [0x41A7]
        + [cfg_word(4, 4, mode=3), cfg_word(6, 6, mode=3), cfg_word(6, 4, mode=3)]
    )
    port = args.port or (meter_candidates() or [None])[0]
    if not port:
        sys.exit("no meter found")
    s = serial.Serial(port, 115200, timeout=3.0)

    # round-robin over the configs so a drifting load affects them all alike
    calls = [(cfg, min(CHUNK, args.n - k)) for k in range(0, args.n, CHUNK) for cfg in cfgs]
    weights = [n * nominal_cycle_us(cfg) for cfg, n in calls]
    data = {cfg: [] for cfg in cfgs}
    clk_hz = None
    with tqdm(total=sum(weights), unit="us", unit_scale=True, desc="sampling") as bar:
        for (cfg, n), w in zip(calls, weights):
            csr, rvr, recs = inaxp(s, cfg, n)
            if rvr != 0xFFFFFF:
                sys.exit(f"SysTick reload is 0x{rvr:X}, not 0xFFFFFF: dt values invalid")
            clk_hz = 48e6 if csr & 4 else 6e6
            data[cfg] += recs
            bar.update(w)
            time.sleep(0.02)
    s.close()

    print(f"SysTick: {'HCLK' if clk_hz == 48e6 else 'HCLK/8'} = {clk_hz / 1e6:g} MHz")
    print(f"{'config':44} {'cycle us':>9} {'nom':>6} {'ratio':>6} | "
          f"{'shunt mean':>10} {'sd LSB':>6} {'sd mA':>6} {'codes':>5} | "
          f"{'bus mV':>8} {'sd LSB':>6} {'codes':>5}")
    for cfg in cfgs:
        recs = data[cfg]
        dts = [r[0] / clk_hz * 1e6 for r in recs]
        sh = [r[1] for r in recs]
        bus = [r[2] for r in recs]
        cyc = statistics.mean(dts) if (cfg & 7) >= 5 else min(dts)
        nom = nominal_cycle_us(cfg)
        sd_sh = statistics.pstdev(sh)
        print(f"{describe(cfg):44} {cyc:9.1f} {nom:6d} {cyc / nom:6.3f} | "
              f"{statistics.mean(sh):10.2f} {sd_sh:6.2f} "
              f"{sd_sh * SHUNT_UV_LSB / SHUNT_MOHM:6.3f} {len(set(sh)):5d} | "
              f"{statistics.mean(bus) * BUS_MV_LSB:8.1f} {statistics.pstdev(bus):6.2f} "
              f"{len(set(bus)):5d}")
        if (cfg & 7) < 5:
            print(f"{'':44} trig dt min/med/max {min(dts):.0f}/{statistics.median(dts):.0f}/"
                  f"{max(dts):.0f} us")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["cfg", "dt_us", "shunt_raw", "bus_raw"])
            for cfg in cfgs:
                for dt, sh, bus in data[cfg]:
                    w.writerow([f"0x{cfg:04X}", f"{dt / clk_hz * 1e6:.2f}", sh, bus])
        print(f"raw samples -> {args.csv}")


if __name__ == "__main__":
    main()
