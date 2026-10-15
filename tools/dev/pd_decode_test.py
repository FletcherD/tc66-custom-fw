#!/usr/bin/env python3
"""Regression vectors for tools/lib/pd_decode.py: real frames from bench sniffs
(Apple 96 W charger, Apple e-marked cable, Pixel/Moto — kept as captured)
and synthetic frames built from the named fields of tools/lib/pd_spec.py
(incl. the spec's own examples). The captured values are also rebuilt from
named fields, which cross-checks pd_spec against real traffic. Each case
checks that the expected substrings appear in the decoded output; also
round-trips a PdPacket through pdcap.py's --replay path.

    python3 tools/dev/pd_decode_test.py        (-v prints every decode)
"""
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(TOOLS, "lib"))
from pd_decode import Decoder  # noqa: E402
from pd_spec import *  # noqa: E402,F403 — the spec's named fields and values
import km003c  # noqa: E402
from pd_engine import answer_kind  # noqa: E402


def hdr(typ, nobj=0, src=True, dfp=True, rev=REV_2_0, mid=0, ext=False, cable=None):
    """Message header (Table 6.2); `cable` set = SOP'/SOP'' (Cable Plug bit)."""
    fields = [(HDR_MSG_TYPE, typ), (HDR_SPEC_REV, rev), (HDR_MSG_ID, mid), (HDR_NUM_OBJECTS, nobj),
              (HDR_EXTENDED, ext)]
    fields += [(HDR_POWER_ROLE, src), (HDR_DATA_ROLE, dfp)] if cable is None else [(HDR_CABLE_PLUG, cable)]
    return struct.pack("<H", pack(*fields))


def data(typ, objs, **kw):
    return hdr(typ, len(objs), **kw) + b"".join(struct.pack("<I", o) for o in objs)


def ext(typ, block, chunk=0, req=False, **kw):
    """One chunk (chunked) of an extended message carrying `block`."""
    part = b"" if req else block[MAX_EXT_CHUNK_LEN * chunk:MAX_EXT_CHUNK_LEN * (chunk + 1)]
    eh = pack((EXTHDR_CHUNKED, 1), (EXTHDR_CHUNK_NUMBER, chunk), (EXTHDR_REQUEST_CHUNK, req),
              (EXTHDR_DATA_SIZE, 0 if req else len(block)))
    body = struct.pack("<H", eh) + part
    body += b"\0" * (-len(body) % 4)
    return hdr(typ, len(body) // 4, ext=True, rev=REV_3_X, **kw) + body


def blk(size, *fields):
    """A byte-oriented data block: (offset, struct format, value) fields."""
    b = bytearray(size)
    for off, fmt, v in fields:
        struct.pack_into("<" + fmt, b, off, *(v if isinstance(v, tuple) else (v,)))
    return bytes(b)


def fixed(volts, amps, *flags):
    return pack((PDO_TYPE, PDO_FIXED), (FIXED_VOLTAGE, volts), (FIXED_MAX_CURRENT, amps), *flags)


def vdm_hdr(cmd, ctype, svid=SVID_PD, major=0, minor=0, pos=0):
    return pack((VDM_SVID, svid), (VDM_STRUCTURED, 1), (VDM_VERSION_MAJOR, major), (VDM_VERSION_MINOR, minor),
                (VDM_OBJECT_POSITION, pos), (VDM_COMMAND_TYPE, ctype), (VDM_COMMAND, cmd))


# ---- real frames, as captured ----
APPLE_CAPS = [0x0801912C, 0x0002D12C, 0x0004B12C, 0x000641D6]
PIXEL_RDO = 0x1304B12C
APPLE_CABLE_ACK = [0xFF008041, 0x1C0005AC, 0x00000A26, 0x17810100, 0xA1084050]
PHONE_NAK = 0xFF008081
# ... and the same values from named fields
assert APPLE_CAPS == [fixed(5, 3, (SRC_UNCONSTRAINED, 1)), fixed(9, 3), fixed(15, 3), fixed(20, 4.7)]
assert PIXEL_RDO == pack((RDO_OBJECT_POSITION, 1), (RDO_USB_COMMS, 1), (RDO_NO_USB_SUSPEND, 1),
                         (RDO_OPERATING_CURRENT, 3), (RDO_MAX_CURRENT, 3))
assert APPLE_CABLE_ACK == [
    vdm_hdr(CMD_DISCOVER_IDENTITY, VDM_ACK),
    pack((IDH_UFP_TYPE, CABLE_PASSIVE), (IDH_MODAL, 1), (IDH_VID, 0x05AC)),
    2598,  # Cert Stat VDO: XID
    pack((PRODUCT_PID, 0x1781), (PRODUCT_BCD, 0x0100)),
    pack((CABLE_HW_VERSION, 10), (CABLE_FW_VERSION, 1), (CABLE_PLUG_TYPE, 2), (CABLE_LATENCY, 2),
         (CABLE_VBUS_CURRENT, 2), (CABLE_VBUS_THROUGH, 1))]  # PD 2.0 layout (PD 2.0 Table 6-28)
assert PHONE_NAK == vdm_hdr(CMD_DISCOVER_IDENTITY, VDM_NAK)

# ---- synthetic objects ----
PPS = pack((PDO_TYPE, PDO_APDO), (APDO_TYPE, APDO_PPS), (PPS_MAX_VOLTAGE, 21), (PPS_MIN_VOLTAGE, 3.3),
           (PPS_MAX_CURRENT, 3))
EPR_AVS = pack((PDO_TYPE, PDO_APDO), (APDO_TYPE, APDO_EPR_AVS), (EPR_AVS_MAX_VOLTAGE, 28),
               (EPR_AVS_MIN_VOLTAGE, 15), (EPR_AVS_PDP, 140))
BATTERY = pack((PDO_TYPE, PDO_BATTERY), (BATTERY_MAX_VOLTAGE, 15), (BATTERY_MIN_VOLTAGE, 5), (BATTERY_MAX_POWER, 15))
VARIABLE = pack((PDO_TYPE, PDO_VARIABLE), (VARIABLE_MAX_VOLTAGE, 12), (VARIABLE_MIN_VOLTAGE, 5),
                (VARIABLE_MAX_CURRENT, 2))
ACTIVE1 = pack((CABLE_HW_VERSION, 1), (CABLE_FW_VERSION, 2), (CABLE_VDO_VERSION, 3), (CABLE_PLUG_TYPE, 2),
               (CABLE_LATENCY, 1), (CABLE_TERMINATION, 3), (ACTIVE_SBU_NOT_SUPPORTED, 1), (CABLE_VBUS_CURRENT, 2),
               (CABLE_VBUS_THROUGH, 1), (CABLE_USB_SPEED, 2))
ACTIVE2 = pack((ACTIVE2_MAX_TEMP, 90), (ACTIVE2_SHUTDOWN_TEMP, 75), (ACTIVE2_RETIMER, 1), (ACTIVE2_TWO_LANES, 1),
               (ACTIVE2_GEN2, 1))
EPR_CAPS = [fixed(5, 3, (SRC_UNCONSTRAINED, 1), (SRC_EPR_CAPABLE, 1)), fixed(9, 3), fixed(15, 3), fixed(20, 3),
            0, 0, 0, fixed(28, 3), EPR_AVS]
assert len(EPR_CAPS) == EPR_SPR_SLOTS + 2

SCEDB = blk(SCEDB_LEN, (SCEDB_VID, "H", 0x05AC), (SCEDB_PID, "H", 0x1234), (SCEDB_XID, "I", 2598),
            (SCEDB_FW_VERSION, "B", 7), (SCEDB_HW_VERSION, "B", 3),
            (SCEDB_VOLTAGE_REGULATION, "B", pack((LOAD_STEP, 1), (VOLTAGE_REGULATION_IOC_90, 1))),
            (SCEDB_HOLDUP_TIME, "B", 20), (SCEDB_COMPLIANCE, "B", 0b001), (SCEDB_TOUCH_CURRENT, "B", 0b011),
            (SCEDB_PEAK_CURRENTS[0], "H", pack((PEAK_OVERLOAD, 150), (PEAK_PERIOD, 100), (PEAK_DUTY_CYCLE, 10))),
            (SCEDB_TOUCH_TEMP, "B", 1),
            (SCEDB_SOURCE_INPUTS, "B", pack((INPUT_EXTERNAL_SUPPLY, 1), (INPUT_EXTERNAL_UNCONSTRAINED, 1))),
            (SCEDB_SPR_PDP, "B", 96))
SKEDB = blk(SKEDB_LEN, (SKEDB_VID, "H", 0x18D1), (SKEDB_PID, "H", 0x4EE1), (SKEDB_FW_VERSION, "B", 1),
            (SKEDB_HW_VERSION, "B", 2), (SKEDB_VERSION, "B", 1), (SKEDB_TOUCH_TEMP, "B", 1),
            (SKEDB_BATTERY_INFO, "B", pack((BATTERIES_FIXED, 1))), (SKEDB_SINK_MODES, "B", 0b1001),
            (SKEDB_SPR_MIN_PDP, "B", 15), (SKEDB_SPR_OPERATIONAL_PDP, "B", 27), (SKEDB_SPR_MAX_PDP, "B", 45))
SDB = blk(SDB_POWER_STATE_CHANGE + 1, (SDB_INTERNAL_TEMP, "B", 45),
          (SDB_PRESENT_INPUT, "B", pack((INPUT_EXTERNAL_POWER, 3))),
          (SDB_EVENT_FLAGS, "B", pack((EVENT_PPS_CL_MODE, 1))),
          (SDB_TEMPERATURE_STATUS, "B", pack((TEMPERATURE_STATUS, 1))),
          (SDB_POWER_STATUS, "B", 0b10))  # bit 1: cable current
PPSSDB = blk(PPSSDB_LEN, (PPSSDB_OUTPUT_VOLTAGE, "H", 450), (PPSSDB_OUTPUT_CURRENT, "B", 40),
             (PPSSDB_REAL_TIME_FLAGS, "B", pack((PPS_TEMPERATURE_FLAG, 1), (PPS_CL_MODE, 1))))
MIDB = blk(MIDB_STRING, (MIDB_VID, "H", 0x05AC), (MIDB_PID, "H", 0x1234)) + b"96W USB-C\0"
BCDB = blk(BCDB_LEN, (BCDB_VID, "H", 0x05AC), (BCDB_PID, "H", 0x0001), (BCDB_DESIGN_CAPACITY, "H", 500),
           (BCDB_LAST_FULL_CAPACITY, "H", BATTERY_CAPACITY_UNKNOWN))

SNK = dict(src=False, dfp=False)
R3 = dict(rev=REV_3_X)
CASES = [
    # --- real frames (bench logs) ---
    ("Apple Source_Cap", 0, data(DATA_SOURCE_CAP, APPLE_CAPS),
     ["Source_Cap", "[SRC DFP r2.0 id0]", "PDO 1: fixed 5.00 V 3.00 A", "unconstrained power",
      "PDO 4: fixed 20.00 V 4.70 A"]),
    ("Pixel Request", 0, [data(DATA_SOURCE_CAP, APPLE_CAPS[:1]), data(DATA_REQUEST, [PIXEL_RDO], **SNK)],
     ["[SNK UFP", "PDO 1 (fixed 5.00 V 3.00 A)", "operating 3.00 A", "USB comms", "no USB suspend"]),
    ("Apple cable identity (PD 2.0)", 1, data(DATA_VENDOR_DEFINED, APPLE_CABLE_ACK, cable=1),
     ["[CABLE r2.0", "Discover Identity ACK", "passive cable", "0x05AC", "product 0x1781",
      "XID 2598", "5 A VBUS", "USB 2.0 only", "to Type-C", "~2 m", "HW 10 FW 1", "VBUS through cable"]),
    ("phone NAK", 0, data(DATA_VENDOR_DEFINED, [PHONE_NAK], **SNK), ["Discover Identity NAK"]),
    ("GoodCRC", 0, hdr(CTRL_GOODCRC, mid=3, **SNK), ["GoodCRC", "[SNK UFP r2.0 id3]"]),
    # --- PDO type order: 01b battery, 10b variable (Table 6.6) ---
    ("battery PDO", 0, data(DATA_SINK_CAP, [fixed(5, 3, (SNK_HIGHER_CAPABILITY, 1)), BATTERY], **SNK),
     ["PDO 2: battery 5.00-15.00 V 15.00 W", "higher capability"]),
    ("variable PDO", 0, data(DATA_SOURCE_CAP, [APPLE_CAPS[0], VARIABLE]), ["PDO 2: variable 5.00-12.00 V 2.00 A"]),
    ("PPS caps + request", 0,
     [data(DATA_SOURCE_CAP, [APPLE_CAPS[0], PPS], **R3),
      data(DATA_REQUEST, [pack((RDO_OBJECT_POSITION, 2), (RDO_PPS_VOLTAGE, 9), (RDO_APDO_CURRENT, 2))], **SNK, **R3)],
     ["PDO 2: PPS 3.3-21.0 V 3.00 A", "PDO 2 (PPS 3.3-21.0): 9.000 V 2.00 A"]),
    # --- data messages from PD 3.x ---
    ("Revision (spec example)", 0, data(DATA_REVISION, [0x32120000], **R3), ["PD revision 3.2, version 1.2"]),
    ("Get_Country_Info (spec example)", 0, data(DATA_GET_COUNTRY_INFO, [0x434E0000], **R3), ["country: CN"]),
    ("Alert OCP", 0, data(DATA_ALERT, [pack((bit(26), 1))], **R3), ["Alert", "alert: OCP"]),
    ("Battery_Status", 0,
     data(DATA_BATTERY_STATUS, [pack((BSDO_CAPACITY, 45.2), (BSDO_PRESENT, 1), (BSDO_CHARGING_STATUS, 1))],
          **SNK, **R3),
     ["battery: 45.2 Wh, discharging"]),
    ("Source_Info", 0,
     data(DATA_SOURCE_INFO, [pack((SIDO_PORT_TYPE, 1), (SIDO1_MAX_PDP, 100), (SIDO1_PRESENT_PDP, 96),
                                  (SIDO1_REPORTED_PDP, 96)),
                             pack((SIDO2_MAX_PDP, 100), (SIDO2_GUARANTEED_PDP, 60))], **R3),
     ["guaranteed port, max 100 W, present 96 W, reported 96 W", "max 100.0 W, guaranteed 60.0 W"]),
    ("EPR_Mode enter failed", 0, data(DATA_EPR_MODE, [pack((EPRMDO_ACTION, EPR_ENTER_FAILED), (EPRMDO_DATA, 1))], **R3),
     ["enter failed: cable not EPR capable"]),
    ("EPR_Mode enter", 0, data(DATA_EPR_MODE, [pack((EPRMDO_ACTION, EPR_ENTER), (EPRMDO_DATA, 140))], **SNK, **R3),
     ["enter EPR mode, sink EPR operational PDP 140 W"]),
    ("EPR_Request", 0,
     data(DATA_EPR_REQUEST, [pack((RDO_OBJECT_POSITION, 8), (RDO_OPERATING_CURRENT, 5), (RDO_MAX_CURRENT, 5)),
                             EPR_CAPS[7]], **SNK, **R3),
     ["PDO 8 (fixed 28.00 V 3.00 A): operating 5.00 A"]),
    ("Enter_USB", 0,
     data(DATA_ENTER_USB, [pack((EUDO_USB_MODE, 2), (EUDO_CABLE_SPEED, 1), (EUDO_CABLE_CURRENT, 3), (bit(13), 1))],
          **R3),
     ["enter USB4", "USB3 Gen1", "passive", "5 A", "host present"]),
    ("BIST", 0, data(DATA_BIST, [pack((BIST_MODE, 8))]), ["BIST mode: test data"]),
    ("Get_Source_Cap_Ext (control)", 0, hdr(CTRL_GET_SOURCE_CAP_EXT, **SNK, **R3), ["Get_Source_Cap_Ext"]),
    # --- VDMs ---
    ("charger identity (PD 3.x, power brick)", 0,
     data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_DISCOVER_IDENTITY, VDM_ACK, major=1, minor=1),
                                pack((IDH_DFP_TYPE, DFP_POWER_BRICK), (IDH_CONNECTOR, CONNECTOR_RECEPTACLE),
                                     (IDH_VID, 0x05AC)),
                                0, pack((PRODUCT_PID, 0x7A0B), (PRODUCT_BCD, 0x0100)),
                                pack((DFP_VDO_VERSION, 2), (DFP_PORT_NUMBER, 1))], **R3),
     ["SVDM 2.1", "not a UFP / DFP power brick", "receptacle", "DFP: no USB host, port 1"]),
    ("phone identity (PD 3.x, peripheral)", 0,
     data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_DISCOVER_IDENTITY, VDM_ACK, major=1),
                                pack((IDH_USB_DEVICE, 1), (IDH_UFP_TYPE, UFP_PERIPHERAL), (IDH_CONNECTOR, CONNECTOR_PLUG),
                                     (IDH_VID, 0x18D1)),
                                1234, pack((PRODUCT_PID, 0x4EE1), (PRODUCT_BCD, 0x0440)),
                                pack((UFP_VDO_VERSION, 3), (UFP_USB32_DEVICE, 1), (UFP_USB2_DEVICE, 2),
                                     (UFP_USB_SPEED, 2))], **SNK, **R3),
     ["peripheral", "vendor 0x18D1", "XID 1234", "plug", "UFP: USB3.2 device, USB2 device", "USB3/USB4 Gen2"]),
    ("AMA identity (PD 2.0)", 0,
     data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_DISCOVER_IDENTITY, VDM_ACK),
                                pack((IDH_UFP_TYPE, UFP_AMA), (IDH_MODAL, 1), (IDH_VID, 0x05AC)), 0, 0,
                                pack((AMA_HW_VERSION, 1), (AMA_VCONN_POWER, 1), (AMA_VCONN_REQUIRED, 1),
                                     (AMA_USB_SPEED, 3))], **SNK),
     ["AMA, vendor 0x05AC", "AMA: HW 1 FW 0, billboard only, VCONN required 1.5 W, VBUS not required"]),
    ("VPD identity", 1,
     data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_DISCOVER_IDENTITY, VDM_ACK, major=1),
                                pack((IDH_UFP_TYPE, CABLE_VPD), (IDH_VID, 0x05AC)), 0, 0,
                                pack((VPD_HW_VERSION, 2), (VPD_CHARGE_THROUGH, 1), (VPD_CT_CURRENT_5A, 1),
                                     (VPD_VBUS_IMPEDANCE, 10), (VPD_GND_IMPEDANCE, 5))], cable=1, **R3),
     ["VPD, vendor", "VPD: HW 2 FW 0, charge through supported, 5 A, VBUS 10 / GND 5"]),
    ("Discover SVIDs", 0,
     data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_DISCOVER_SVIDS, VDM_ACK), pack((SVIDS_PER_VDO[0], SVID_DISPLAYPORT),
                                                                           (SVIDS_PER_VDO[1], 0x05AC)), 0], **SNK, **R3),
     ["Discover SVIDs ACK", "0xFF01 (DisplayPort)", "0x05AC"]),
    ("Discover Modes", 0,
     data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_DISCOVER_MODES, VDM_ACK, svid=SVID_DISPLAYPORT), 0x00000C46], **SNK),
     ["Discover Modes ACK, SVID 0xFF01 (DisplayPort)", "mode 1: 0x00000c46"]),
    ("Exit Mode all", 0, data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_EXIT_MODE, VDM_REQ, pos=VDM_EXIT_ALL_MODES)]),
     ["Exit Mode REQ", "exit all modes"]),
    ("Attention", 0,
     data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_ATTENTION, VDM_REQ, svid=SVID_DISPLAYPORT, pos=1), 0x9A], **SNK),
     ["Attention REQ", "object 1", "attention VDO: 0000009a"]),
    ("unstructured VDM", 0, data(DATA_VENDOR_DEFINED, [pack((VDM_SVID, 0x05AC)), 0x12345678]),
     ["unstructured VDM, SVID 0x05AC", "12345678"]),
    ("active cable PD 3.x", 1,
     data(DATA_VENDOR_DEFINED, [vdm_hdr(CMD_DISCOVER_IDENTITY, VDM_ACK, major=1),
                                pack((IDH_UFP_TYPE, CABLE_ACTIVE), (IDH_VID, 0x2109)), 0,
                                pack((PRODUCT_PID, 1)), ACTIVE1, ACTIVE2], cable=1, **R3),
     ["active cable: 5 A VBUS, USB3/USB4 Gen2, to Type-C, ~1 m, HW 1 FW 2, both ends active, max 20 V",
      "SBU not supported", "VBUS through cable", "VDO v3",
      "active cable 2: max 90 C, shutdown 75 C, copper, re-timer, USB4, USB2, USB3, two lanes, Gen2+"]),
    # --- extended messages ---
    ("Source_Cap_Ext", 0, ext(EXT_SOURCE_CAP_EXT, SCEDB),
     ["Source_Cap_Ext", "vendor 0x05AC", "product 0x1234", "XID 2598", "FW 7 HW 3",
      "load step 500 mA/us, 90 % IoC", "holdup 20 ms", "compliance LPS", "ground pin",
      "peak current 1: +150 % for 100 ms at 10 % duty", "IEC 62368-1 TS1",
      "external supply unconstrained", "SPR PDP 96 W"]),
    ("Sink_Cap_Ext", 0, ext(EXT_SINK_CAP_EXT, SKEDB, **SNK),
     ["Sink_Cap_Ext", "SKEDB v1", "PPS charging, battery powered", "SPR PDP min 15 / operational 27 / max 45 W"]),
    ("Status", 0, ext(EXT_STATUS, SDB),
     ["temp 45 C, AC external power", "PPS CL mode", "temperature normal", "power limited by: cable current"]),
    ("cable Status", 1, ext(EXT_STATUS, blk(2, (CABLE_SDB_TEMP, "B", 60), (CABLE_SDB_FLAGS, "B", 1)), cable=1),
     ["cable plug temp 60 C, THERMAL SHUTDOWN"]),
    ("PPS_Status", 0, ext(EXT_PPS_STATUS, PPSSDB), ["PPS status: 9.00 V, 2.00 A, temperature normal, CL mode"]),
    ("Manufacturer_Info", 0, ext(EXT_MANUFACTURER_INFO, MIDB), ["vendor 0x05AC", "\"96W USB-C\""]),
    ("Get_Manufacturer_Info", 0,
     ext(EXT_GET_MANUFACTURER_INFO, blk(2, (GMIDB_TARGET, "B", MANUFACTURER_TARGET_PORT)), **SNK),
     ["target: port/cable plug"]),
    ("Get_Battery_Status", 0, ext(EXT_GET_BATTERY_STATUS, bytes([5]), **SNK), ["battery ref 5 (hot-swap 1)"]),
    ("Battery_Cap", 0, ext(EXT_BATTERY_CAP, BCDB), ["design 50.0 Wh, last full unknown"]),
    ("Country_Info", 0, ext(EXT_COUNTRY_INFO, blk(CIDB_DATA, (CIDB_COUNTRY_CODE, "2s", b"DE")) + b"\x01\x02"),
     ["country DE: 01 02"]),
    ("Country_Codes", 0, ext(EXT_COUNTRY_CODES, blk(CCDB_CODES, (CCDB_LENGTH, "B", 2)) + b"USCN"),
     ["countries: US CN"]),
    ("Extended_Control", 0, ext(EXT_EXTENDED_CONTROL, blk(2, (ECDB_TYPE, "B", 3)), **SNK), ["EPR_KeepAlive"]),
    ("Vendor_Defined_Extended", 0,
     ext(EXT_VENDOR_DEFINED, blk(VDEM_DATA, (VDEM_SVID, "H", 0x05AC), (VDEM_COMMAND, "H", 0x8005)) + b"\xAA"),
     ["vendor ext: SVID 0x05AC", "command 0x0005: aa"]),
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
    outs = [dec.frame(w, 0) for w in (ext(EXT_EPR_SOURCE_CAP, block, 0),
                                      ext(EXT_EPR_SOURCE_CAP, block, 1, req=True, **SNK),
                                      ext(EXT_EPR_SOURCE_CAP, block, 1))]
    text = "\n".join(line + "\n" + "\n".join(d) for line, d in outs)
    want = ["chunk 0: 26 of 36 bytes", "chunk 1 requested", "chunks 0-1 reassembled, 36 bytes",
            "PDO 8: fixed 28.00 V 3.00 A", "PDO 9: EPR AVS 15.0-28.0 V 140 W", "EPR capable"]
    miss = [w for w in want if w not in text]
    if verbose or miss:
        print("== chunked EPR_Source_Cap\n%s" % text)
    if miss:
        print("   MISSING: %s" % miss)
        fails += 1
    # pd_engine.answer_kind: what detect.py makes of a source's answer
    kinds = [
        ("ppsstatus", ext(EXT_PPS_STATUS, PPSSDB), "answer"),  # the PD 3.0 charger's reply
        ("srccapext", hdr(CTRL_NOT_SUPPORTED, rev=REV_3_X), "unsupported"),
        ("snkcap", hdr(CTRL_REJECT), "unsupported"),
        ("srcinfo", data(DATA_SOURCE_CAP, APPLE_CAPS), "unexpected"),  # Apple: type 23 read as 7
        ("discid", data(DATA_VENDOR_DEFINED, [PHONE_NAK]), "unsupported"),
        ("discid", data(DATA_VENDOR_DEFINED, APPLE_CABLE_ACK), "answer"),
        ("srccapext", ext(EXT_SOURCE_CAP_EXT, SCEDB), "answer"),
    ]
    for name, wire, want in kinds:
        got = answer_kind(name, wire)
        if verbose or got != want:
            print("== answer_kind %s: %s (want %s)" % (name, got, want))
        fails += got != want
    # pdcap.py --replay round trip: one PdPacket with a connect event and the Apple Source_Cap frame
    wire = CASES[0][2]
    pkt = bytes(km003c.PRE_LEN) + bytes([km003c.EV_CONNECTION, 10, 0, 0, 0, km003c.EV_CONNECT]) + \
        bytes([pack((km003c.EV_WRAPPED, 1), (km003c.EV_WRAPPED_SIZE, len(wire) + km003c.EV_WRAPPED_SIZE_BIAS))]) + \
        struct.pack("<IB", 11, 0) + wire
    with tempfile.NamedTemporaryFile("w", suffix=".hex", delete=False) as f:
        f.write(pkt.hex() + "\n")
    r = subprocess.run([sys.executable, os.path.join(TOOLS, "pdcap.py"), "--replay", f.name],
                       capture_output=True, text=True)
    os.unlink(f.name)
    ok = "CONNECT" in r.stdout and "PDO 4: fixed 20.00 V 4.70 A" in r.stdout
    if verbose or not ok:
        print("== replay\n%s%s" % (r.stdout, r.stderr))
    fails += not ok
    print("%d/%d cases failed" % (fails, len(CASES) + 2 + len(kinds)))
    return fails


if __name__ == "__main__":
    sys.exit(1 if run("-v" in sys.argv) else 0)
