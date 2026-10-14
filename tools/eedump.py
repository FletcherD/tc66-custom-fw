#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial", "tqdm"]
# ///
"""Dump or decode the meter's 32 KB EEPROM (BL24C256) via the custom
firmware's read-only `eeprd` verb (32 B per transaction; every build).

  eedump.py dump out.bin [--start A] [--len N]   raw image (default: all 32 KB)
  eedump.py show <image.bin>                     decode config, energy ring, datalog
  eedump.py show --live                          dump the persistence regions, then decode

Layout: docs/HARDWARE.md "EEPROM layout" (stock-compatible).
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))
from tqdm import tqdm  # noqa: E402
from tc66lib import open_meter  # noqa: E402

SIZE = 0x8000
CHUNK = 32
TAG = 0xC0DE
CFG_NAMES = {
    0x2004: "gainV (cal)", 0x2008: "gainCp (cal)", 0x200C: "gainCn (cal)",
    0x2010: "group", 0x2014: "rotation", 0x2018: "disp opt (stock)",
    0x201C: "brightness", 0x2020: "disp opt2 (stock)", 0x2024: "screen",
    0x202C: "runcount", 0x2034: "timeout (stock)", 0x2038: "usb_cmd_off",
    0x203C: "ble_cmd_off", 0x2040: "reset-screen opt", 0x2044: "offV (cal)",
    0x2048: "cal extra1", 0x204C: "cal extra2", 0x2050: "panel variant",
    0x2054: "first-boot latch", 0x2058: "ref (cal)", 0x2060: "our magic",
    0x2064: "idle_off_s",
}


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def eeprd(s, addr, n):
    s.reset_input_buffer()
    s.write(b"eeprd" + struct.pack("<HB", addr, n))
    r = s.read(2 + n)
    if r[:2] == b"OK" and len(r) == 2 + n:
        return r[2:]
    raise IOError("eeprd %#06x: %r" % (addr, r))


def read_range(s, start, length):
    out = bytearray()
    with tqdm(total=length, unit="B", unit_scale=True, desc="eeprom") as bar:
        a = start
        while a < start + length:
            n = min(CHUNK, start + length - a)
            for attempt in range(3):
                try:
                    out += eeprd(s, a, n)
                    break
                except IOError:
                    if attempt == 2:
                        raise
            a += n
            bar.update(n)
    return bytes(out)


def show(img, base=0):
    def u32(a):
        return struct.unpack_from("<I", img, a - base)[0]

    print("== config 0x2000..0x2068")
    for a in range(0x2000, 0x2068, 4):
        if a in CFG_NAMES:
            print(f"  {a:#06x} {CFG_NAMES[a]:18s} {u32(a):#010x} ({u32(a)})")
    print("== factory cal master 0x2200..0x2218")
    print("  " + " ".join(f"{u32(a):#x}" for a in range(0x2204, 0x2218, 4)))

    print("== energy ring")
    slots = []
    for n in range(256):
        b = img[n * 32 - base : n * 32 - base + 32]
        w = struct.unpack("<5I", b[:20])
        tag, crc = struct.unpack_from("<HH", b, 0x1C)
        slots.append((n, w, struct.unpack_from("<2I", b, 0x14), tag == TAG,
                      crc16_modbus(b[:0x1C]) == crc))
    live = [x for x in slots if x[1][4] != 0xFFFFFFFF]
    if not live:
        print("  (erased)")
    else:
        newest = max(live, key=lambda x: x[1][4])
        tagged = sum(1 for x in slots if x[3])
        n, w, secs, t, ok = newest
        print(f"  newest slot {n} seq {w[4]}: g0 {w[0]} mAh {w[1]} mWh, "
              f"g1 {w[2]} mAh {w[3]} mWh" + (f", run {secs[0]}/{secs[1]} s" if t else "")
              + (f", {'CRC ok' if ok else 'CRC BAD'}" if t else ", untagged (stock)"))
        print(f"  {len(live)} written slots, {tagged} carry our tag")

    print("== datalog 0x2400..")
    recs = []
    for n in range(1440):
        a = 0x2400 + 16 * n
        if a + 16 - base > len(img):
            break
        recs.append(struct.unpack_from("<3I", img, a - base))
    chain = 0
    while chain < len(recs) and recs[chain][2] not in (0, 0xFFFFFFFF):
        chain += 1
    print(f"  {chain} records in the seq chain from record 0")
    for v, i, q in recs[: min(chain, 3)]:
        print(f"    {v / 1e4:.4f} V {i / 1e5:.5f} A seq {q}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("dump", formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    d.add_argument("out")
    d.add_argument("--start", type=lambda v: int(v, 0), default=0)
    d.add_argument("--len", type=lambda v: int(v, 0), default=SIZE)
    sh = sub.add_parser("show")
    sh.add_argument("image", nargs="?")
    sh.add_argument("--live", action="store_true", help="read the meter instead of a file")
    a = ap.parse_args()

    if a.cmd == "dump":
        img = read_range(open_meter(), a.start, a.len)
        open(a.out, "wb").write(img)
        print(f"wrote {len(img)} B to {a.out}")
    elif a.live:
        img = read_range(open_meter(), 0, 0x2400 + 16 * 1440)
        show(img)
    elif a.image:
        show(open(a.image, "rb").read())
    else:
        sys.exit("show needs an image file or --live")


if __name__ == "__main__":
    main()
