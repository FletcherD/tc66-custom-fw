#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial"]
# ///
"""Read and change the meter's persisted settings (custom firmware `cfgrd` /
`cfgwr` verbs, USB CDC).

  cfg.py                     list every setting (name, id, value, range)
  cfg.py get <name|id>       print one value
  cfg.py set <name|id> <v>   change a setting (applies at once; the firmware
                             writes it to EEPROM ~2 s after the last change)

Settings: group (active energy group 0/1), rotation (stock 0/2 = 180°),
brightness (0..9), screen (0..3, 3 = plot), runcount (boots, read-only),
log_interval_s (datalog period 1..60), usb_cmd_off / ble_cmd_off (ignore
that link's verbs except query/updat/cfg*), idle_off_s (screen-off after
N s without a reading change, 0 = never), idle_dv_mv / idle_di_ma (a 1 s
V / I change above this keeps the screen on; 0 disables that trigger, both
0 = always time out), log_on (datalog recording), log_clear (write 1 =
empty the datalog), plot_period_ms (plot column period; 160 columns span
the screen), plot_v_full_mv / plot_i_full_ma (plot full scale = top edge;
changing either restarts the plot).

Wire format (rust/src/cfg.rs):
  cfgrd [start]     -> "OK" + total u8 + start u8 + n u8 + n x (id u8,
                       value u32 LE, min u32 LE, max u32 LE, flags u8) +
                       CRC16/MODBUS (LE) over all preceding bytes; at most
                       13 entries per reply (one BLE buffer) — page until
                       start + n == total
  cfgwr <id> <val>  (ASCII decimal args) -> "OK" | "ER" + code
A setting's id is its EEPROM word index: address = 0x2000 + 4 * id (the
stock firmware's config block, docs/HARDWARE.md "EEPROM layout").
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

FLAG_RO = 1

NAMES = {
    4: "group",
    5: "rotation",
    7: "brightness",
    9: "screen",
    11: "runcount",
    13: "log_interval_s",
    14: "usb_cmd_off",
    15: "ble_cmd_off",
    25: "idle_off_s",
    26: "log_on",
    27: "log_clear",
    28: "idle_dv_mv",
    29: "idle_di_ma",
    30: "plot_period_ms",
    31: "plot_v_full_mv",
    32: "plot_i_full_ma",
}
IDS = {v: k for k, v in NAMES.items()}

ERRORS = {
    0x0F: "bad arguments",
    0x10: "unknown setting id",
    0x11: "value out of range",
    0x12: "read-only setting",
}


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def open_meter():
    for c in meter_candidates():
        try:
            s = serial.Serial(c, 115200, timeout=1.0)
            s.reset_input_buffer()
            s.write(b"query")
            if s.read(4) == b"firm":
                return s
            s.close()
        except Exception:
            pass
    sys.exit("no meter in app mode")


def read_reply(s, n_min, cap=2.0):
    buf = b""
    end = time.time() + cap
    while time.time() < end and len(buf) < n_min:
        buf += s.read(max(1, s.in_waiting))
    return buf


def cfgrd_page(s, start):
    s.reset_input_buffer()
    s.write(b"cfgrd" if start == 0 else f"cfgrd {start}".encode())
    head = read_reply(s, 5)
    if head[:2] != b"OK":
        raise RuntimeError(f"cfgrd: {head!r}")
    total, first, n = head[2], head[3], head[4]
    size = 5 + 14 * n + 2
    buf = head + read_reply(s, size - len(head))
    if len(buf) < size:
        raise RuntimeError(f"cfgrd: short reply {len(buf)}/{size} B")
    if crc16_modbus(buf[: size - 2]) != struct.unpack_from("<H", buf, size - 2)[0]:
        raise RuntimeError("cfgrd: CRC mismatch")
    rows = [struct.unpack_from("<BIIIB", buf, 5 + 14 * k) for k in range(n)]
    return total, first, rows


def cfgrd(s):
    """Every setting as (id, value, min, max, flags), paging the reply."""
    out, start = [], 0
    while True:
        total, first, rows = cfgrd_page(s, start)
        out += rows
        start = first + len(rows)
        if not rows or start >= total:
            return out


def cfgwr(s, sid, value):
    s.reset_input_buffer()
    s.write(f"cfgwr {sid} {value}".encode())
    r = read_reply(s, 2)
    if r[:2] == b"OK":
        return
    if r[:2] == b"ER":
        r += read_reply(s, 1, cap=0.3)
        code = r[2] if len(r) > 2 else None
        raise RuntimeError(f"cfgwr: {ERRORS.get(code, hex(code) if code is not None else '?')}")
    raise RuntimeError(f"cfgwr: no reply ({r!r})")


def resolve(key):
    if key in IDS:
        return IDS[key]
    try:
        return int(key, 0)
    except ValueError:
        sys.exit(f"unknown setting {key!r} (known: {', '.join(IDS)})")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd")
    g = sub.add_parser("get")
    g.add_argument("key")
    st = sub.add_parser("set")
    st.add_argument("key")
    st.add_argument("value", type=lambda v: int(v, 0))
    a = ap.parse_args()

    s = open_meter()
    try:
        if a.cmd == "set":
            cfgwr(s, resolve(a.key), a.value)
            print("OK")
        elif a.cmd == "get":
            sid = resolve(a.key)
            for row in cfgrd(s):
                if row[0] == sid:
                    print(row[1])
                    return
            sys.exit(f"setting {sid} not reported by the firmware")
        else:
            for sid, val, lo, hi, flags in cfgrd(s):
                name = NAMES.get(sid, f"id{sid}")
                ro = "  (read-only)" if flags & FLAG_RO else ""
                print(f"{name:12s} id {sid:2d} = {val:<10d} [{lo}..{hi}]{ro}")
    finally:
        s.close()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as e:
        sys.exit(str(e))
