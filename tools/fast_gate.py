#!/usr/bin/env python3
"""Fast-flash placement gate (make fast-gate).

The GD32F150 fetches zero-wait only below FAST_END (0x0800_8000); above it
every cold fetch costs tens of µs (QUIRKS.md §4, FIRMWARE.md "Fast-flash
placement"). fast.x pins the hot path (.fasttext) right after the vector
table. This gate fails the build if .fasttext itself crosses FAST_END
(nothing may spill silently — decide what to demote), or if any CRITICAL
symbol ends above FAST_END (catches a fast.x pattern that stopped matching,
e.g. after a rename), and reports the region budget.

Usage: fast_gate.py <elf>
"""
import subprocess
import sys

FAST_END = 0x0800_8000

# Substrings of mangled names that MUST live entirely below FAST_END.
# These are the ISR chains and the AES/getva reply path — the measured wins.
CRITICAL = [
    "USBD_LP",
    "TIMER5_DAC",
    "12drain_serial",
    "7tc66_fw3i2c",
    "7tc66_fw3ina",
    "10usb_device6device",  # UsbDevice::poll (+ get_descriptor rides outside)
    "14aes256_encrypt",
    "9sub_bytes",
    "11xor_columns",
    "13mix_columns_",
    "5getva9build_raw",  # rawva streaming; getva5build is deliberately slow (fast.x)
    "11u64_div_rem",
    "3mem6memcpy",
]
# ...except these (matched by a CRITICAL substring but deliberately slow —
# see fast.x's .slowtext — or a linker-generated veneer that lives wherever
# its slow caller does):
EXEMPT = ["get_descriptor", "control_out", "set_error", "handle_out",
          "key_schedule", "try_cal", "Thumbv7ABSLongThunk"]


def main():
    elf = sys.argv[1]
    nm = subprocess.run(["arm-none-eabi-nm", "-S", elf],
                        capture_output=True, text=True, check=True).stdout
    syms = []  # (addr, size, name) for defined text-ish symbols with size
    for line in nm.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[2].lower() in ("t", "w"):
            syms.append((int(parts[0], 16), int(parts[1], 16), parts[3]))

    efast = None
    for line in nm.splitlines():
        parts = line.split()
        if parts and parts[-1] == "__efasttext":
            efast = int(parts[0], 16)
    if efast is None:
        sys.exit("fast-gate: __efasttext not found — fast.x not linked in?")

    bad = []
    for addr, size, name in syms:
        if addr + size <= FAST_END:
            continue
        if any(c in name for c in CRITICAL) and not any(e in name for e in EXEMPT):
            bad.append((addr, name))

    fast_used = efast - 0x0800_2C00
    fast_cap = FAST_END - 0x0800_2C00
    # how much ordinary .text also fits below the boundary (bonus fast code)
    bonus = FAST_END - efast
    print(f"fast-gate: .fasttext ends {efast:#x} — {fast_used}/{fast_cap} B of "
          f"the fast region ({bonus} B of .text also below the boundary)")
    if efast > FAST_END:
        sys.exit(f"fast-gate FAIL: .fasttext crosses {FAST_END:#x} by "
                 f"{efast - FAST_END} B — demote something from fast.x / "
                 f"#[link_section = \".fasttext.*\"] or shrink code")
    if bad:
        for addr, name in sorted(bad):
            print(f"  SLOW: {addr:#x} {name}")
        sys.exit(f"fast-gate FAIL: {len(bad)} critical symbol(s) above "
                 f"{FAST_END:#x} — fix its fast.x pattern or annotation")
    print("fast-gate ok")


if __name__ == "__main__":
    main()
