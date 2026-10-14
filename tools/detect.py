#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Detect which charge protocols the charger on the meter supports (either
port: the probed lines pass straight through), without ever raising VBUS
above 5 V.

Steps (D+/D- protocols first: a PD contract can disable a charger's legacy
modes):
  1. passive  rawva: VBUS, load current, D+/D- voltages (Apple / Samsung
              divider signatures)
  2. QC       qcdet: D+ 0.6 V, D- released; a dedicated charger's D+/D- short
              makes D- follow D+, a QC charger opens it after ~1.25 s
  3. AFC      afctr 0x08: a closed-loop Samsung AFC request for 5 V
  4. FCP      fcprd: Huawei FCP register reads (0x21 = number of raised
              voltages, 0x22 max power, 0x30+k voltages) — needs a firmware
              built with the `fcp` feature; untested against a real charger
  5. PD       pdctl 1: the sink takes the 5 V PDO; pdsta (Type-C Rp level),
              pdcap (Source_Capabilities); then pdctl 0
Between steps `qcoff` releases D+/D- and the charger gets time to fall back.

--device runs the same detection ON the meter instead (the `detct` verb —
the detection screen's state machine, also started by K2 on that screen) and
prints the screen's text lines; the meter refuses (ER) while a PD sink
session is armed, the sniffer sees a sink, or the load exceeds 30 mA.

Nothing should be plugged into the other port: the probes drive D+/D- and
present Rd on CC, and a device there would take part in (and disturb) the
handshakes. The current reads unsigned: with the charger on OUT and the
meter powered from its measured rail, its own supply current shows as a
small load.
"""
import argparse
import struct
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402
from pd import decode_pdo  # noqa: E402
from rawva import rawva  # noqa: E402

QC_CLASS = {0: "no D+/D- short (not a dedicated charger)",
            1: "dedicated charger (BC1.2 DCP), no QC",
            2: "QC (HVDCP) handshake answered"}
BC_LVL = {0: "none", 1: "Default USB", 2: "1.5 A", 3: "3.0 A"}


def open_meter():
    for c in meter_candidates():
        try:
            s = serial.Serial(c, 115200, timeout=1.0)
            s.reset_input_buffer()
            s.write(b"query")
            if s.read(4) == b"firm":
                return s
            s.close()
        except serial.SerialException:
            pass
    sys.exit("no meter found")


def cmd(s, pkt, n, timeout):
    s.reset_input_buffer()
    s.timeout = timeout
    s.write(pkt)
    return s.read(n)


def release(s, settle):
    cmd(s, b"qcoff", 2, 1.0)
    time.sleep(settle)


def divider(dp, dm):
    """Apple / Samsung D+/D- resistor-divider signatures (volts)."""
    def near(v, t):
        return abs(v - t) < 0.25
    apple = {(2.0, 2.0): "0.5 A", (2.0, 2.7): "1 A", (2.7, 2.0): "2.1 A", (2.7, 2.7): "2.4 A"}
    for (p, m), amps in apple.items():
        if near(dp, p) and near(dm, m):
            return f"Apple divider ({amps})"
    if near(dp, 1.2) and near(dm, 1.2):
        return "Samsung divider (2 A)"
    if dp > 0.3 and abs(dp - dm) < 0.05:
        return f"D+ = D- = {dp:.2f} V (lines tied and biased by the charger)"
    return None


def fcp_read(s, reg, n):
    """fcprd -> list of n register bytes, or an error string."""
    r = cmd(s, b"fcprd" + bytes([reg, n]), 2 + n, 15.0)
    if r[:2] == b"OK" and len(r) == 2 + n:
        return list(r[2:])
    if r[:2] == b"ER" and len(r) == 3:
        e = r[2]
        return ("no reply" if e & 0xF0 == 0x40 else "NACK") + f" (ER 0x{e:02x})"
    return "no answer (firmware without the `fcp` feature?)"


DETCT_ER = {1: "a run is already in progress", 2: "PD sink session armed (pdctl 0 first)",
            3: "the sniffer sees a sink on the other port", 4: "load current above 30 mA"}


def device_run(s, timeout):
    """Start the on-device detection (detct 1), wait for it, print its lines."""
    r = cmd(s, b"detct\x01", 3, 1.0)
    if r[:2] == b"ER" and len(r) == 3:
        print("refused:", DETCT_ER.get(r[2], f"ER 0x{r[2]:02x}"))
    elif r != b"OK":
        sys.exit(f"detct: bad reply {r!r} (firmware without detection?)")
    t_end = time.time() + timeout
    while True:
        r = cmd(s, b"detct", 4, 1.0)
        if r[:2] != b"OK" or len(r) != 4:
            sys.exit(f"detct: bad reply {r!r}")
        busy, n = r[2], r[3]
        lines = s.read(n * 22)
        if not busy or time.time() > t_end:
            break
        time.sleep(0.5)
    for i in range(n):
        print(lines[22 * i:22 * i + 22].decode(errors="replace").rstrip())
    if busy:
        sys.exit("timed out while the run was still in progress")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--settle", type=float, default=1.5,
                    help="seconds between steps for the charger to fall back (default: %(default)s)")
    ap.add_argument("--pd-wait", type=float, default=4.0,
                    help="seconds to wait for PD Source_Capabilities (default: %(default)s)")
    ap.add_argument("--no-fcp", action="store_true", help="skip the FCP step (default: off)")
    ap.add_argument("--no-pd", action="store_true", help="skip the PD step (default: off)")
    ap.add_argument("--device", action="store_true",
                    help="run the meter's own detection (detct) and print its result (default: off)")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="--device: seconds to wait for the run (default: %(default)s)")
    a = ap.parse_args()

    s = open_meter()
    print("firmware:", cmd(s, b"versn", 32, 1.0).decode(errors="replace"))
    if a.device:
        device_run(s, a.timeout)
        return
    cmd(s, b"pdctl\x00", 2, 1.0)
    release(s, a.settle)

    m = rawva(s)
    print(f"\n[passive] VBUS {m['V']:.2f} V, load {m['A'] * 1000:.0f} mA, "
          f"D+ {m['dp_V']:.2f} V, D- {m['dm_V']:.2f} V")
    if m["A"] > 0.005:
        print("  WARNING: current is flowing — unplug any device on the other port; "
              "results below may be wrong")
    div = divider(m["dp_V"], m["dm_V"])
    print("  divider signature:", div or "none")

    r = cmd(s, b"qcdet", 9, 4.0)
    if r[:2] != b"OK" or len(r) != 9:
        sys.exit(f"qcdet: bad reply {r!r}")
    cls = r[2]
    dp, dm0, dm1 = struct.unpack("<3H", r[3:9])
    print(f"\n[QC] D+ {dp} mV, D- {dm0} mV at 0.1 s -> {dm1} mV at 1.5 s")
    print("  " + (QC_CLASS.get(cls, f"class {cls}") if not div or div.startswith("D+ = D-")
                  else "(meaningless with a divider charger)"))
    release(s, a.settle)

    r = cmd(s, b"afctr\x08", 3, 6.0)
    afc = r[:2] == b"OK"
    print("\n[AFC]", "answered (Samsung AFC)" if afc else f"no answer ({r!r})")
    release(s, a.settle)

    fcp = None
    if not a.no_fcp:
        caps = fcp_read(s, 0x21, 2)
        if isinstance(caps, list):
            n, pwr = caps
            volts = fcp_read(s, 0x30, min(n + 1, 8))
            fcp = (n, pwr, volts)
            vs = ", ".join(f"{v / 10:.1f} V" for v in volts) if isinstance(volts, list) else volts
            print(f"\n[FCP] answered: {n} raised voltage(s), max {pwr / 2:.1f} W; outputs: {vs}")
        else:
            print("\n[FCP]", caps)
        release(s, a.settle)

    pd_caps = []
    if not a.no_pd:
        cmd(s, b"pdctl\x01", 2, 1.0)
        st = None
        bc = 0
        t_end = time.time() + a.pd_wait
        while time.time() < t_end:
            r = cmd(s, b"pdsta", 24, 1.0)
            if r[:2] == b"OK" and len(r) == 24:
                st = r[2:]
                # Rp: BC_LVL only from a poll without CC activity (one taken
                # during BMC traffic reads low)
                if 2 <= st[0] <= 5 and not st[7] & 0x40:
                    bc = st[7] & 3
                if st[2] and st[0] == 5:  # caps stored, contract (5 V PDO 1)
                    break
            time.sleep(0.25)
        if st is None:
            print("\n[PD] no pdsta reply")
        else:
            r = cmd(s, b"pdcap", 31, 1.0)
            if r[:2] == b"OK":
                pd_caps = [int.from_bytes(r[3 + 4 * i:7 + 4 * i], "little") for i in range(r[2])]
            if st[0] <= 1:  # off / wait-attach: no Rp seen on either CC
                print("\n[Type-C] no Type-C source on CC (Type-A charger or A-to-C cable)")
            else:
                print(f"\n[Type-C] CC{st[1]}, Rp advertises {BC_LVL[bc]}")
                rev = {1: "PD 2.0", 2: "PD 3.0"}.get(st[13], "?")
                print(f"[PD] {rev}, {len(pd_caps)} PDO(s):" if pd_caps else "[PD] no Source_Capabilities")
                for i, p in enumerate(pd_caps, 1):
                    print("  " + decode_pdo(i, p))
        cmd(s, b"pdctl\x00", 2, 1.0)

    print("\nsummary:", ", ".join(x for x in [
        div if div and not div.startswith("D+ = D-") else None,
        *(["QC" if cls == 2 else ("DCP" if cls == 1 else None)]
          if not div or div.startswith("D+ = D-") else []),
        "AFC" if afc else None,
        "FCP" if fcp else None,
        "USB PD" if pd_caps else None,
    ] if x) or "nothing beyond plain USB")


if __name__ == "__main__":
    main()
