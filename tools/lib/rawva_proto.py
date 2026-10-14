"""The custom firmware's plaintext `rawva` verb: reply layout (46 B,
rust/src/getva.rs): "raw1" magic, then LE u32 seq, v_1e4, i_1e5, p_1e4,
r_1e1, mah, mwh, temp_c (i32), dp_1e2, dm_1e2, CRC16/MODBUS of bytes 0..44
(LE u16 at 44). Used by tools/rawva.py and tools/detect.py.
"""
import struct

RAW_LEN = 46


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def parse(pkt):
    """Return a dict of decoded fields; raises on bad magic/CRC/length."""
    if len(pkt) != RAW_LEN:
        raise ValueError(f"short rawva reply: {len(pkt)} B")
    if pkt[:4] != b"raw1":
        raise ValueError(f"bad magic {pkt[:4]!r}")
    if struct.unpack_from("<H", pkt, 44)[0] != crc16_modbus(pkt[:44]):
        raise ValueError("CRC mismatch")
    seq, v, i, p, r, mah, mwh, temp, dp, dm = struct.unpack_from("<7Ii2I", pkt, 4)
    return {"seq": seq, "V": v / 1e4, "A": i / 1e5, "W": p / 1e4,
            "ohm": r / 10, "mAh": mah, "mWh": mwh, "degC": temp,
            "dp_V": dp / 100, "dm_V": dm / 100}


def rawva(s):
    s.reset_input_buffer()
    s.write(b"rawva")
    buf = b""
    while len(buf) < RAW_LEN:
        chunk = s.read(RAW_LEN - len(buf))
        if not chunk:
            raise RuntimeError(f"timeout, got {len(buf)} B")
        buf += chunk
    return parse(buf)
