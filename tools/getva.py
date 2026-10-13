#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial", "cryptography"]
# ///
"""Poll `getva` over USB CDC once and print every decoded field (format:
HARDWARE.md "getva reply"), including the per-block checksums.

  getva.py
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial  # noqa: E402
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

KEY = bytes.fromhex("5821fa5601b2f02687ff1204622a4fb086f40260816f9a0ba7f106619ab87288")


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def decode(raw):
    d = Cipher(algorithms.AES(KEY), modes.ECB()).decryptor()
    p = d.update(raw) + d.finalize()
    u = lambda o: struct.unpack_from("<I", p, o)[0]  # noqa: E731
    crc_ok = all(crc16_modbus(p[b * 64 : b * 64 + 60]) == u(b * 64 + 60) for b in range(3))
    return {
        "markers": p[0:4] + p[64:68] + p[128:132],
        "model": p[4:8].decode("latin1"),
        "fw": p[8:12].decode("latin1"),
        "serial": u(12),
        "runcount": u(44),
        "V": u(48) / 1e4,
        "A": u(52) / 1e5,
        "W": u(56) / 1e4,
        "ohm": u(68) / 10,
        "g0_mAh": u(72), "g0_mWh": u(76), "g1_mAh": u(80), "g1_mWh": u(84),
        "degC": -u(92) if u(88) else u(92),
        "D+": u(96) / 100, "D-": u(100) / 100,
        "seq": u(132),
        "crc_ok": crc_ok,
    }


def main():
    for c in meter_candidates():
        try:
            s = serial.Serial(c, 115200, timeout=1.0)
            s.reset_input_buffer()
            s.write(b"getva")
            raw = s.read(192)
            s.close()
        except Exception:
            continue
        if len(raw) == 192:
            for k, v in decode(raw).items():
                print(f"{k:9s} {v}")
            return
    sys.exit("no getva reply")


if __name__ == "__main__":
    main()
