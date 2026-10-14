#!/usr/bin/env python3
"""Regression vectors for tools/pd_decode.py: real frames from bench sniffs
(Apple 96 W charger, Apple e-marked cable, Pixel/Moto) and synthetic frames
built from the USB PD R3.2 tables (incl. the spec's own examples). Each case
checks that the expected substrings appear in the decoded output; also
round-trips a PdPacket through pdcap.py's --replay path.

    python3 tools/pd_decode_test.py        (-v prints every decode)
"""
import os
import struct
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pd_decode import Decoder  # noqa: E402


def hdr(typ, nobj=0, src=True, dfp=True, rev=1, mid=0, ext=False, cable=None):
    h = typ | rev << 6 | mid << 9 | nobj << 12 | ext << 15
    if cable is None:
        h |= src << 8 | dfp << 5
    else:
        h |= cable << 8
    return struct.pack("<H", h)


def data(typ, objs, **kw):
    return hdr(typ, len(objs), **kw) + b"".join(struct.pack("<I", o) for o in objs)


def ext(typ, block, chunk=0, req=False, **kw):
    """One chunk (chunked=1) of an extended message carrying `block`."""
    part = b"" if req else block[26 * chunk:26 * chunk + 26]
    eh = struct.pack("<H", len(block) * (not req) | req << 10 | chunk << 11 | 1 << 15)
    body = eh + part
    body += b"\0" * (-len(body) % 4)
    return hdr(typ, len(body) // 4, ext=True, rev=2, **kw) + body


SCEDB = struct.pack("<HHI", 0x05AC, 0x1234, 2598) + bytes([7, 3, 0x05, 20, 0x01, 0x03]) + \
    struct.pack("<HHH", 15 | 5 << 5 | 2 << 11, 0, 0) + bytes([1, 0x03, 0x00, 96, 0])
SKEDB = struct.pack("<HHI", 0x18D1, 0x4EE1, 0) + bytes([1, 2, 1, 0]) + struct.pack("<H", 0) + \
    bytes([0, 1, 0x01, 0x09, 15, 27, 45, 0, 0, 0])
PPS = 3 << 30 | 210 << 17 | 33 << 8 | 60                 # PPS 3.3-21 V 3 A (Table 6.13)
EPR_AVS = 3 << 30 | 1 << 28 | 280 << 17 | 150 << 8 | 140  # EPR AVS 15-28 V 140 W (Table 6.16)
# Active Cable VDO1/VDO2 (Tables 6.43 / 6.44)
ACTIVE1 = 1 << 28 | 2 << 24 | 3 << 21 | 2 << 18 | 1 << 13 | 3 << 11 | 1 << 8 | 2 << 5 | 1 << 4 | 2
ACTIVE2 = 90 << 24 | 75 << 16 | 1 << 9 | 1 << 3 | 1
EPR_CAPS = [0x0881912C, 0x0002D12C, 0x0004B12C, 0x0006412C, 0, 0, 0, 0x0008C12C, EPR_AVS]

CASES = [
    # --- real frames (bench logs) ---
    ("Apple Source_Cap", 0, data(1, [0x0801912C, 0x0002D12C, 0x0004B12C, 0x000641D6], mid=0),
     ["Source_Cap", "[SRC DFP r2.0 id0]", "PDO 1: fixed 5.00 V 3.00 A", "unconstrained power",
      "PDO 4: fixed 20.00 V 4.70 A"]),
    ("Pixel Request", 0, [data(1, [0x0801912C]), data(2, [0x1304B12C], src=False, dfp=False)],
     ["[SNK UFP", "PDO 1 (fixed 5.00 V 3.00 A)", "operating 3.00 A", "USB comms", "no USB suspend"]),
    ("Apple cable identity (PD 2.0)", 1,
     data(15, [0xFF008041, 0x1C0005AC, 0x00000A26, 0x17810100, 0xA1084050], cable=1),
     ["[CABLE r2.0", "Discover Identity ACK", "passive cable", "0x05AC", "product 0x1781",
      "XID 2598", "5 A VBUS", "USB 2.0 only", "to Type-C", "~2 m", "HW 10 FW 1", "VBUS through cable"]),
    ("phone NAK", 0, data(15, [0xFF008081], src=False, dfp=False), ["Discover Identity NAK"]),
    ("GoodCRC", 0, hdr(1, src=False, dfp=False, mid=3), ["GoodCRC", "[SNK UFP r2.0 id3]"]),
    # --- PDO type order: 01b battery, 10b variable (Table 6.6) ---
    ("battery PDO", 0, data(4, [0x1001912C, 1 << 30 | 300 << 20 | 100 << 10 | 60], src=False, dfp=False),
     ["PDO 2: battery 5.00-15.00 V 15.00 W", "higher capability"]),
    ("variable PDO", 0, data(1, [0x0801912C, 2 << 30 | 240 << 20 | 100 << 10 | 200]),
     ["PDO 2: variable 5.00-12.00 V 2.00 A"]),
    ("PPS caps + request", 0, [data(1, [0x0801912C, PPS], rev=2),
                               data(2, [2 << 28 | 450 << 9 | 40], src=False, dfp=False, rev=2)],
     ["PDO 2: PPS 3.3-21.0 V 3.00 A", "PDO 2 (PPS 3.3-21.0): 9.000 V 2.00 A"]),
    # --- data messages from PD 3.x ---
    ("Revision (spec example)", 0, data(12, [0x32120000], rev=2), ["PD revision 3.2, version 1.2"]),
    ("Get_Country_Info (spec example)", 0, data(7, [0x434E0000], rev=2), ["country: CN"]),
    ("Alert OCP", 0, data(6, [1 << 26], rev=2), ["Alert", "alert: OCP"]),
    ("Battery_Status", 0, data(5, [452 << 16 | 1 << 9 | 1 << 10], src=False, dfp=False, rev=2),
     ["battery: 45.2 Wh, discharging"]),
    ("Source_Info", 0, data(11, [1 << 31 | 100 << 16 | 96 << 8 | 96, 200 << 9 | 120], rev=2),
     ["guaranteed port, max 100 W, present 96 W, reported 96 W", "max 100.0 W, guaranteed 60.0 W"]),
    ("EPR_Mode enter failed", 0, data(10, [4 << 24 | 1 << 16], rev=2), ["enter failed: cable not EPR capable"]),
    ("EPR_Request", 0, data(9, [8 << 28 | 500 << 10 | 500, 0x0008C12C], src=False, dfp=False, rev=2),
     ["PDO 8 (fixed 28.00 V 3.00 A): operating 5.00 A"]),
    ("Enter_USB", 0, data(8, [2 << 28 | 1 << 21 | 3 << 17 | 1 << 13], rev=2),
     ["enter USB4", "USB3 Gen1", "passive", "5 A", "host present"]),
    ("Get_Source_Cap_Ext (control)", 0, hdr(17, src=False, dfp=False, rev=2), ["Get_Source_Cap_Ext"]),
    # --- VDMs ---
    ("charger identity (PD 3.x, power brick)", 0,
     data(15, [0xFF00A841, 0x01C005AC | 3 << 23 | 2 << 21, 0, 0x7A0B0100, 0x40000001], rev=2),
     ["SVDM 2.1", "not a UFP / DFP power brick", "receptacle", "DFP: no USB host, port 1"]),
    ("phone identity (PD 3.x, peripheral)", 0,
     data(15, [0xFF00A041, 0x50000000 | 2 << 27 | 3 << 21 | 0x18D1, 1234, 0x4EE10440, 0x66000002],
          src=False, dfp=False, rev=2),
     ["peripheral", "vendor 0x18D1", "XID 1234", "plug", "UFP: USB3.2 device, USB2 device", "USB3/USB4 Gen2"]),
    ("Discover SVIDs", 0, data(15, [0xFF008042, 0xFF0105AC, 0x00000000], src=False, dfp=False, rev=2),
     ["Discover SVIDs ACK", "0xFF01 (DisplayPort)", "0x05AC"]),
    ("active cable PD 3.x", 1,
     data(15, [0xFF00A041, 4 << 27 | 0x2109, 0, 0x00010000, ACTIVE1, ACTIVE2], cable=1, rev=2),
     ["active cable: 5 A VBUS, USB3/USB4 Gen2, to Type-C, ~1 m, HW 1 FW 2, both ends active, max 20 V",
      "SBU not supported", "VBUS through cable", "VDO v3",
      "active cable 2: max 90 C, shutdown 75 C, copper, re-timer, USB4, USB2, USB3, two lanes, Gen2+"]),
    # --- extended messages ---
    ("Source_Cap_Ext", 0, ext(1, SCEDB),
     ["Source_Cap_Ext", "vendor 0x05AC", "product 0x1234", "XID 2598", "FW 7 HW 3",
      "load step 500 mA/us, 90 % IoC", "holdup 20 ms", "compliance LPS", "ground pin",
      "peak current 1: +150 % for 100 ms at 10 % duty", "IEC 62368-1 TS1",
      "external supply unconstrained", "SPR PDP 96 W"]),
    ("Sink_Cap_Ext", 0, ext(15, SKEDB, src=False, dfp=False),
     ["Sink_Cap_Ext", "SKEDB v1", "PPS charging, battery powered", "SPR PDP min 15 / operational 27 / max 45 W"]),
    ("Status", 0, ext(2, bytes([45, 0x06, 0, 0x10, 0x02, 0x02, 0])),
     ["temp 45 C, AC external power", "PPS CL mode", "temperature normal", "power limited by: cable current"]),
    ("cable Status", 1, ext(2, bytes([60, 1]), cable=1), ["cable plug temp 60 C, THERMAL SHUTDOWN"]),
    ("PPS_Status", 0, ext(12, struct.pack("<HBB", 450, 40, 0x0A)),
     ["PPS status: 9.00 V, 2.00 A, temperature normal, CL mode"]),
    ("Manufacturer_Info", 0, ext(7, struct.pack("<HH", 0x05AC, 0x1234) + b"96W USB-C\0"),
     ["vendor 0x05AC", "\"96W USB-C\""]),
    ("Get_Manufacturer_Info", 0, ext(6, bytes([0, 0]), src=False, dfp=False), ["target: port/cable plug"]),
    ("Country_Codes", 0, ext(14, bytes([2, 0]) + b"USCN"), ["countries: US CN"]),
    ("Extended_Control", 0, ext(16, bytes([3, 0]), src=False, dfp=False), ["EPR_KeepAlive"]),
]


def run(verbose):
    fails = 0
    for name, sop, wire, want in CASES:
        dec = Decoder()
        text = "\n".join("\n".join([ln] + det) for ln, det in
                         (dec.frame(w, sop) for w in (wire if isinstance(wire, list) else [wire])))
        miss = [w for w in want if w not in text]
        if verbose or miss:
            print("== %s\n%s" % (name, text))
        if miss:
            print("   MISSING: %s" % miss)
            fails += 1
    # chunked EPR_Source_Cap: 36 bytes -> chunk 0 (26), a chunk request, chunk 1 (10)
    block = b"".join(struct.pack("<I", p) for p in EPR_CAPS)
    dec = Decoder()
    outs = [dec.frame(w, 0) for w in (ext(17, block, 0), ext(17, block, 1, req=True, src=False, dfp=False),
                                      ext(17, block, 1))]
    text = "\n".join(line + "\n" + "\n".join(d) for line, d in outs)
    want = ["chunk 0: 26 of 36 bytes", "chunk 1 requested", "chunks 0-1 reassembled, 36 bytes",
            "PDO 8: fixed 28.00 V 3.00 A", "PDO 9: EPR AVS 15.0-28.0 V 140 W", "EPR capable"]
    miss = [w for w in want if w not in text]
    if verbose or miss:
        print("== chunked EPR_Source_Cap\n%s" % text)
    if miss:
        print("   MISSING: %s" % miss)
        fails += 1
    # pdcap.py --replay round trip: one PdPacket with an event and the Source_Cap frame
    wire = CASES[0][2]  # Apple Source_Cap
    pkt = bytes(12) + bytes([0x45, 10, 0, 0, 0, 0x21]) + bytes([0x80 | (len(wire) + 5)]) + \
        struct.pack("<I", 11) + bytes([0]) + wire
    with tempfile.NamedTemporaryFile("w", suffix=".hex", delete=False) as f:
        f.write(pkt.hex() + "\n")
    import subprocess
    r = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "pdcap.py"),
                        "--replay", f.name], capture_output=True, text=True)
    os.unlink(f.name)
    if verbose or "CONNECT" not in r.stdout or "PDO 4: fixed 20.00 V 4.70 A" not in r.stdout:
        print("== replay\n%s%s" % (r.stdout, r.stderr))
        if "CONNECT" not in r.stdout or "PDO 4" not in r.stdout:
            fails += 1
    print("%d/%d cases failed" % (fails, len(CASES) + 2))
    return fails


if __name__ == "__main__":
    sys.exit(1 if run("-v" in sys.argv) else 0)
