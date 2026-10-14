"""USB-PD message decoders for the capture tools (PD 2.0 / 3.x).

`Decoder.frame(wire, sop)` turns one captured frame (16-bit header + data)
into a summary line and indented detail lines: message names for all
control / data / extended types, PDOs (with the first fixed PDO's flags),
RDOs, BIST/Battery_Status/Alert/Country/Enter_USB/EPR_Mode/Source_Info/
Revision objects, structured VDMs (Discover Identity with the ID Header,
UFP/DFP/cable/VPD/AMA product VDOs, Discover SVIDs/Modes) and extended
messages (Source/Sink_Capabilities_Extended, Status, PPS_Status,
Manufacturer_Info, Battery_Capabilities, Country_Info/Codes,
Extended_Control, EPR_Source/Sink_Capabilities, Vendor_Defined_Extended),
reassembling chunked extended messages.

Field layouts are taken from the USB PD specification, Revision 3.2
Version 1.2 (usb.org document library, free download — section numbers in
the comments refer to it) and Revision 2.0 Version 1.3 for the PD 2.0
VDOs (cable / AMA VDOs differ from 3.x); cross-checked against the Linux
kernel's include/linux/usb/pd.h / pd_vdo.h / pd_ext_sdb.h / pd_ado.h,
Chromium EC include/usb_pd.h and ST's stm32-mw-usbpd-core usbpd_def.h.
Vendor/product names come from the system usb.ids (PD uses the USB-IF
vendor IDs); a PD product ID is vendor-assigned and usually not listed.
"""
import os

USB_IDS = ("/usr/share/hwdata/usb.ids", "/usr/share/misc/usb.ids", "/var/lib/usbutils/usb.ids")
_ids = None


def _load_ids():
    """usb.ids -> {vid: (vendor name, {pid: product name})}; {} if absent."""
    global _ids
    if _ids is not None:
        return _ids
    _ids = {}
    path = next((p for p in USB_IDS if os.path.exists(p)), None)
    if path is None:
        return _ids
    cur = None
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("C "):  # device classes follow the vendor list
                break
            if line[:1] not in "0123456789abcdef\t" or line.startswith("#"):
                continue
            if line[0] == "\t":
                if cur is not None and line[1:2] != "\t":
                    pid, _, name = line[1:].partition("  ")
                    try:
                        cur[1][int(pid, 16)] = name.strip()
                    except ValueError:
                        pass
                continue
            vid, _, name = line.partition("  ")
            try:
                cur = _ids.setdefault(int(vid, 16), (name.strip(), {}))
            except ValueError:
                cur = None
    return _ids


# Tables 6.4 / 6.5 / 6.47
CTRL_MSGS = {1: "GoodCRC", 2: "GotoMin", 3: "Accept", 4: "Reject", 5: "Ping", 6: "PS_RDY",
             7: "Get_Source_Cap", 8: "Get_Sink_Cap", 9: "DR_Swap", 10: "PR_Swap",
             11: "VCONN_Swap", 12: "Wait", 13: "Soft_Reset", 14: "Data_Reset",
             15: "Data_Reset_Complete", 16: "Not_Supported", 17: "Get_Source_Cap_Ext",
             18: "Get_Status", 19: "FR_Swap", 20: "Get_PPS_Status", 21: "Get_Country_Codes",
             22: "Get_Sink_Cap_Ext", 23: "Get_Source_Info", 24: "Get_Revision"}
DATA_MSGS = {1: "Source_Cap", 2: "Request", 3: "BIST", 4: "Sink_Cap", 5: "Battery_Status",
             6: "Alert", 7: "Get_Country_Info", 8: "Enter_USB", 9: "EPR_Request",
             10: "EPR_Mode", 11: "Source_Info", 12: "Revision", 15: "Vendor"}
EXT_MSGS = {1: "Source_Cap_Ext", 2: "Status", 3: "Get_Battery_Cap", 4: "Get_Battery_Status",
            5: "Battery_Cap", 6: "Get_Manufacturer_Info", 7: "Manufacturer_Info",
            8: "Security_Request", 9: "Security_Response", 10: "FW_Update_Request",
            11: "FW_Update_Response", 12: "PPS_Status", 13: "Country_Info", 14: "Country_Codes",
            15: "Sink_Cap_Ext", 16: "Extended_Control", 17: "EPR_Source_Cap",
            18: "EPR_Sink_Cap", 30: "Vendor_Ext"}
VDM_CMDS = {1: "Discover Identity", 2: "Discover SVIDs", 3: "Discover Modes",
            4: "Enter Mode", 5: "Exit Mode", 6: "Attention"}
VDM_TYPES = {0: "REQ", 1: "ACK", 2: "NAK", 3: "BUSY"}
SIDS = {0xFF00: "PD SID", 0xFF01: "DisplayPort"}  # Table 6.33
USB_SPEED = {0: "USB 2.0 only", 1: "USB3 Gen1", 2: "USB3/USB4 Gen2", 3: "USB4 Gen3",
             4: "USB4 Gen4"}
USB_SPEED_20 = {0: "USB 2.0 only", 1: "USB 3.1 Gen1", 2: "USB 3.1 Gen1 and Gen2"}


def _bits(v, names):
    """Names of the set bits in `v` per {bit: name}."""
    return [n for b, n in sorted(names.items()) if v >> b & 1]


def _u16(d, i):
    return d[i] | d[i + 1] << 8 if i + 2 <= len(d) else None


def _words(d):
    return [int.from_bytes(d[i:i + 4], "little") for i in range(0, len(d) - 3, 4)]


def vendor(vid):
    v = _load_ids().get(vid)
    return "0x%04X%s" % (vid, " (%s)" % v[0] if v else "")


def product(vid, pid):
    v = _load_ids().get(vid)
    name = v[1].get(pid) if v else None
    return "0x%04X%s" % (pid, " (%s)" % name if name else "")


def svid_name(s):
    return "0x%04X (%s)" % (s, SIDS[s]) if s in SIDS else vendor(s)


# ---- power data objects (6.4.1.3) ----

PEAK = {1: "peak OL1 (150/125/110 %)", 2: "peak OL2 (200/150/125 %)",
        3: "peak OL3 (200/175/150 %)"}  # Table 6.18


def pdo(p, sink=False):
    """One power data object -> text."""
    kind = p >> 30
    if kind == 0:
        s = "fixed %.2f V %.2f A" % (((p >> 10) & 0x3FF) * 0.05, (p & 0x3FF) * 0.01)
        if not sink and (p >> 20) & 3:
            s += ", " + PEAK[(p >> 20) & 3]
        return s
    if kind == 1:  # 01b battery: power in 250 mW
        return "battery %.2f-%.2f V %.2f W" % (((p >> 10) & 0x3FF) * 0.05,
                                               ((p >> 20) & 0x3FF) * 0.05, (p & 0x3FF) * 0.25)
    if kind == 2:  # 10b variable
        return "variable %.2f-%.2f V %.2f A" % (((p >> 10) & 0x3FF) * 0.05,
                                                ((p >> 20) & 0x3FF) * 0.05, (p & 0x3FF) * 0.01)
    apdo = (p >> 28) & 3
    if apdo == 0:
        return "PPS %.1f-%.1f V %.2f A%s" % (((p >> 8) & 0xFF) * 0.1, ((p >> 17) & 0xFF) * 0.1,
                                            (p & 0x7F) * 0.05,
                                            ", power limited" if not sink and p >> 27 & 1 else "")
    peak = ", " + PEAK[(p >> 26) & 3] if not sink and (p >> 26) & 3 else ""
    if apdo == 1:
        return "EPR AVS %.1f-%.1f V %d W%s" % (((p >> 8) & 0xFF) * 0.1,
                                               ((p >> 17) & 0x1FF) * 0.1, p & 0xFF, peak)
    if apdo == 2:
        return "SPR AVS 9-15 V %.2f A, 15-20 V %.2f A%s" % (((p >> 10) & 0x3FF) * 0.01,
                                                           (p & 0x3FF) * 0.01, peak)
    return "APDO type %d (0x%08x)" % (apdo, p)


def pdo1_flags(p, sink):
    """Flags carried by the 5 V fixed PDO in position 1 (Tables 6.8 / 6.9)."""
    if p >> 30:
        return None
    if sink:
        f = _bits(p, {29: "dual-role power", 28: "higher capability", 27: "unconstrained power",
                      26: "USB comms", 25: "dual-role data"})
        frs = (p >> 23) & 3
        if frs:
            f.append("FRS current %s" % {1: "default", 2: "1.5 A", 3: "3.0 A"}[frs])
    else:
        f = _bits(p, {29: "dual-role power", 28: "USB suspend", 27: "unconstrained power",
                      26: "USB comms", 25: "dual-role data", 24: "unchunked ext msgs",
                      23: "EPR capable"})
    return "flags: " + (", ".join(f) if f else "none")


def rdo(r, src):
    """Request data object -> text; `src` = the PDO it selects (None if unknown)."""
    pos = (r >> 28) & 0xF
    what = "PDO %d" % pos
    flags = _bits(r, {26: "capability mismatch", 25: "USB comms", 24: "no USB suspend",
                      23: "unchunked ext msgs", 22: "EPR capable"})
    tail = "".join(", " + f for f in flags)
    if src is not None and src >> 30 == 3:
        avs = (src >> 28) & 3 in (1, 2)  # Table 6.21 PPS: 20 mV; 6.22 AVS: 25 mV
        return "%s (%s): %.3f V %.2f A%s" % (what, " ".join(pdo(src).split(" ")[:2]),
                                             ((r >> 9) & 0xFFF) * (0.025 if avs else 0.02),
                                             (r & 0x7F) * 0.05, tail)
    if src is not None:
        what += " (%s)" % pdo(src)
    if src is not None and src >> 30 == 1:  # battery RDO: 250 mW units
        return "%s: operating %.2f W, max %.2f W%s" % (what, ((r >> 10) & 0x3FF) * 0.25,
                                                      (r & 0x3FF) * 0.25, tail)
    return "%s: operating %.2f A, max %.2f A%s" % (what, ((r >> 10) & 0x3FF) * 0.01,
                                                  (r & 0x3FF) * 0.01, tail)


# ---- other data objects (6.4.3 - 6.4.11) ----

def bist(o):
    return "BIST mode: " + {5: "carrier", 8: "test data", 9: "shared test mode entry",
                            10: "shared test mode exit"}.get(o >> 28, "invalid %d" % (o >> 28))


def bsdo(o):
    cap = o >> 16
    s = "battery: %s" % ("SoC unknown" if cap == 0xFFFF else "%.1f Wh" % (cap * 0.1))
    if o >> 8 & 1:
        return s + ", invalid battery reference"
    if not o >> 9 & 1:
        return s + ", not present"
    return s + ", " + {0: "charging", 1: "discharging", 2: "idle"}.get((o >> 10) & 3, "?")


def ado(o):
    f = _bits(o, {30: "OVP", 29: "source input change", 28: "operating condition change",
                  27: "OTP", 26: "OCP", 25: "battery status change"})
    if o >> 25 & 1:
        f.append("batteries fixed %s / hot-swap %s" % (bin(o >> 20 & 0xF), bin(o >> 16 & 0xF)))
    if o >> 31:
        f.append({1: "power state change", 2: "power button press", 3: "power button release",
                  4: "controller initiated wake", 5: "source reducing capabilities"}.get(
            o & 0xF, "extended alert %d" % (o & 0xF)))
    return "alert: " + (", ".join(f) if f else "none")


def ccdo(o):
    return "country: %s" % (chr(o >> 24 & 0xFF) + chr(o >> 16 & 0xFF))


def eudo(o):
    mode = {0: "USB2.0", 1: "USB3.2", 2: "USB4"}.get((o >> 28) & 7, "USB4 (invalid %d)" % (o >> 28 & 7))
    f = _bits(o, {26: "USB4 DRD", 25: "USB3 DRD", 16: "PCIe", 15: "DP", 14: "TBT", 13: "host present"})
    return "enter %s; cable %s, %s, %s%s" % (
        mode, USB_SPEED.get((o >> 21) & 7, "speed %d" % (o >> 21 & 7)),
        {0: "passive", 1: "active re-timer", 2: "active re-driver", 3: "optically isolated"}[(o >> 19) & 3],
        {0: "no VBUS", 1: "no VBUS", 2: "3 A", 3: "5 A"}[(o >> 17) & 3],
        "".join(", " + x for x in f))


def eprmdo(o):
    act, data = o >> 24, (o >> 16) & 0xFF
    s = {1: "enter EPR mode", 2: "enter acknowledged", 3: "enter succeeded", 4: "enter failed",
         5: "exit EPR mode"}.get(act, "action 0x%02x" % act)
    if act == 1:
        s += ", sink EPR operational PDP %d W" % data
    elif act == 4:
        s += ": " + {0: "unknown", 1: "cable not EPR capable", 2: "source failed to become VCONN source",
                     3: "EPR capable not set in RDO", 4: "source unable to enter EPR now",
                     5: "EPR capable not set in PDO"}.get(data, "data 0x%02x" % data)
    return "EPR mode: " + s


def source_info(objs):
    o = objs[0]
    lines = ["source info: %s port, max %d W, present %d W, reported %d W" % (
        "guaranteed" if o >> 31 else "managed", o >> 16 & 0xFF, o >> 8 & 0xFF, o & 0xFF)]
    if len(objs) > 1:
        o = objs[1]
        lines.append("source info 2: max %.1f W, guaranteed %.1f W%s" % (
            (o >> 9 & 0x1FF) * 0.5, (o & 0x1FF) * 0.5, ", DPS port" if o >> 30 & 1 else ""))
    return lines


def rmdo(o):
    return "PD revision %d.%d, version %d.%d" % (o >> 28, o >> 24 & 0xF, o >> 20 & 0xF, o >> 16 & 0xF)


# ---- VDMs (6.4.12) ----

def cable_vdo(v, rev3, active):
    """Passive / active cable VDO (VDO1) -> text."""
    plug = {0: "Type-A", 1: "Type-B", 2: "Type-C", 3: "captive"}[(v >> 18) & 3]
    lat = (v >> 13) & 0xF
    if 1 <= lat <= 7:
        length = "~%d m" % lat
    elif lat == 8:
        length = "~100 m" if active else ">7 m"
    elif active and lat in (9, 10):
        length = "~%d m" % ((lat - 7) * 100)
    else:
        length = "latency %d" % lat
    cur = (v >> 5) & 3
    if rev3:
        cur = {1: "3 A", 2: "5 A"}.get(cur, "3 A (invalid %d)" % cur)
        speed = USB_SPEED.get(v & 7, "speed %d" % (v & 7))
    else:
        cur = {1: "3 A", 2: "5 A"}.get(cur, "reserved %d" % cur)
        speed = USB_SPEED_20.get(v & 7, "speed %d" % (v & 7))
    s = "%s cable: %s VBUS, %s, to %s, %s, HW %d FW %d" % (
        "active" if active else "passive", cur, speed, plug, length, v >> 28, (v >> 24) & 0xF)
    term = (v >> 11) & 3
    if active:
        s += ", %s" % {2: "one end active", 3: "both ends active"}.get(term, "termination %d" % term)
    elif term == 1:
        s += ", VCONN required"
    if rev3:
        mv = (v >> 9) & 3
        s += ", max %s" % {0: "20 V", 3: "50 V"}.get(mv, "%d V (deprecated)" % (20 + 10 * mv))
        f = _bits(v, {17: "EPR capable"})
        if active:
            f += ["SBU %s" % ("not supported" if v >> 8 & 1 else ("active" if v >> 7 & 1 else "passive"))]
            f += _bits(v, {4: "VBUS through cable", 3: "SOP'' controller"})
        s += "".join(", " + x for x in f) + ", VDO v%d" % ((v >> 21) & 7)
    else:
        s += "".join(", " + x for x in _bits(v, {4: "VBUS through cable"} if not active else
                                             {4: "VBUS through cable", 3: "SOP'' controller"}))
    return s


def active_vdo2(v):
    """Active Cable VDO2 (Table 6.44)."""
    f = ["max %d C, shutdown %d C" % (v >> 24, v >> 16 & 0xFF),
         "optical" if v >> 10 & 1 else "copper",
         "re-timer" if v >> 9 & 1 else "re-driver",
         "USB4" if not v >> 8 & 1 else "no USB4",
         "USB2" if not v >> 5 & 1 else "no USB2",
         "USB3" if not v >> 4 & 1 else "no USB3",
         "two lanes" if v >> 3 & 1 else "one lane",
         "Gen2+" if v & 1 else "Gen1"]
    f += _bits(v, {2: "optically isolated", 1: "USB4 asymmetric"})
    return "active cable 2: " + ", ".join(f)


def vpd_vdo(v):
    """VCONN Powered USB Device VDO (Table 6.45)."""
    return "VPD: HW %d FW %d, charge through %s%s, VBUS %d / GND %d (2 mOhm / 1 mOhm units)" % (
        v >> 28, v >> 24 & 0xF, "supported" if v & 1 else "not supported",
        (", " + ("5 A" if v >> 14 & 1 else "3 A")) if v & 1 else "", v >> 7 & 0x3F, v >> 1 & 0x3F)


def ufp_vdo(v):
    """UFP VDO (Table 6.40)."""
    f = []
    usb2 = (v >> 24) & 3
    f += _bits(v, {27: "USB4 device", 26: "USB3.2 device"})
    if usb2:
        f.append({1: "USB2 billboard only", 2: "USB2 device"}.get(usb2, "USB2 invalid (11b)"))
    f += _bits(v, {5: "alt modes without reconfig", 4: "non-TBT3 reconfig alt modes", 3: "TBT3 alt mode"})
    if v >> 7 & 1:
        f.append("needs VCONN %s" % ["1", "1.5", "2", "3", "4", "5", "6", "?"][v >> 8 & 7] + " W")
    return "UFP: %s, highest %s, VDO v1.%d" % (", ".join(f) or "no USB",
                                              USB_SPEED.get(v & 7, "speed %d" % (v & 7)), v >> 29)


def dfp_vdo(v):
    """DFP VDO (Table 6.41)."""
    f = _bits(v, {26: "USB4 host", 25: "USB3.2 host", 24: "USB2 host"})
    return "DFP: %s, port %d" % (", ".join(f) or "no USB host", v & 0x1F)


def ama_vdo(v):
    """PD 2.0 Alternate Mode Adapter VDO (PD 2.0 Table 6-30)."""
    return "AMA: HW %d FW %d, %s, VCONN %s, VBUS %s" % (
        v >> 28, v >> 24 & 0xF,
        {0: "USB 2.0 only", 1: "USB 3.1 Gen1", 2: "USB 3.1 Gen1/Gen2", 3: "billboard only"}.get(v & 7, "?"),
        ("required %s W" % ["1", "1.5", "2", "3", "4", "5", "6", "?"][v >> 5 & 7]) if v >> 4 & 1 else "not required",
        "required" if v >> 3 & 1 else "not required")


def discover_identity(objs, sop, rev3, svdm2):
    """Discover Identity ACK VDOs (after the VDM header) -> lines."""
    idh, cert, prod = objs[0], objs[1], objs[2]
    vid = idh & 0xFFFF
    ut = (idh >> 27) & 7
    if sop:
        ptype = {0: "not a cable", 3: "passive cable", 4: "active cable", 6: "VPD"}.get(ut, "type %d" % ut)
    else:
        ptype = {0: "not a UFP" if rev3 else "undefined", 1: "hub", 2: "peripheral", 3: "PSD",
                 5: "AMA"}.get(ut, "UFP type %d" % ut)
    dt = (idh >> 23) & 7 if rev3 and not sop else 0
    roles = ptype if not dt else "%s / DFP %s" % (
        ptype, {1: "hub", 2: "host", 3: "power brick", 4: "AMC"}.get(dt, "type %d" % dt))
    f = _bits(idh, {31: "USB host", 30: "USB device", 26: "modal"})
    if rev3 and (idh >> 21) & 3:
        f.append({2: "receptacle", 3: "plug"}.get((idh >> 21) & 3, "connector ?"))
    lines = ["identity: %s, vendor %s, product %s bcd %04X, XID %d%s" % (
        roles, vendor(vid), product(vid, prod >> 16), prod & 0xFFFF, cert,
        "".join(", " + x for x in f))]
    rest = objs[3:]
    if sop:
        if ut in (3, 4) and rest:
            lines.append(cable_vdo(rest[0], rev3, ut == 4))
            if ut == 4 and rev3 and len(rest) > 1:
                lines.append(active_vdo2(rest[1]))
        elif ut == 6 and rest:
            lines.append(vpd_vdo(rest[0]))
        return lines
    if not rev3:
        if ut == 5 and rest:
            lines.append(ama_vdo(rest[0]))
        return lines
    if not svdm2:  # SVDM 1.0 product VDOs used older layouts
        lines += ["product VDO: 0x%08x" % v for v in rest]
        return lines
    if ut in (1, 2) and rest:
        lines.append(ufp_vdo(rest[0]))
        rest = rest[2:] if dt in (1, 2, 3) else rest[1:]  # UFP VDO, pad, DFP VDO
    if dt in (1, 2, 3) and rest:
        lines.append(dfp_vdo(rest[0]))
    return lines


def vdm(objs, sop, rev3):
    """Vendor_Defined message objects -> list of text lines."""
    h = objs[0]
    svid = h >> 16
    if not h & (1 << 15):
        return ["unstructured VDM, SVID %s: %s" % (svid_name(svid), " ".join("%08x" % o for o in objs[1:]))]
    cmd, ctype = h & 0x1F, (h >> 6) & 3
    major = (h >> 13) & 3
    ver = "1.0" if major == 0 else "2.%d" % ((h >> 11) & 3)
    name = VDM_CMDS.get(cmd, "SVID cmd %d" % cmd if cmd >= 16 else "cmd %d" % cmd)
    pos = (h >> 8) & 7
    lines = ["VDM %s %s, SVID %s, SVDM %s%s" % (
        name, VDM_TYPES[ctype], svid_name(svid), ver,
        (", exit all modes" if pos == 7 and cmd == 5 else ", object %d" % pos) if pos else "")]
    body = objs[1:]
    if ctype != 1 or not body:
        if cmd == 6 and body:
            lines.append("attention VDO: " + " ".join("%08x" % o for o in body))
        return lines
    if cmd == 1 and len(body) >= 3:
        lines += discover_identity(body, sop, rev3, major >= 1)
    elif cmd == 2:
        svids = [w >> s & 0xFFFF for w in body for s in (16, 0)]
        lines.append("SVIDs: " + (", ".join(svid_name(s) for s in svids if s) or "none"))
    elif cmd == 3:
        lines += ["mode %d: 0x%08x" % (n, o) for n, o in enumerate(body, 1)]
    else:
        lines.append("VDOs: " + " ".join("%08x" % o for o in body))
    return lines


# ---- extended messages (6.5) ----

def _peak(v):
    """Peak Current 1..3 / Sink Load Characteristics field (Table 6.50)."""
    pct = min(v & 0x1F, 25) * 10
    if not pct:
        return "none"
    return "+%d %% for %d ms at %d %% duty%s" % (pct, (v >> 5 & 0x3F) * 20, (v >> 11 & 0xF) * 5,
                                                 ", VBUS may droop 5 %" if v >> 15 else "")


def _idline(d):
    if len(d) < 4:
        return "(%d bytes)" % len(d)
    vid, pid = _u16(d, 0), _u16(d, 2)
    xid = int.from_bytes(d[4:8], "little") if len(d) >= 8 else None
    s = "vendor %s, product %s" % (vendor(vid), product(vid, pid))
    if xid is not None:
        s += ", XID %d" % xid
    if len(d) >= 10:
        s += ", FW %d HW %d" % (d[8], d[9])
    return s


def scedb(d):
    """Source Capabilities Extended Data Block (Table 6.50)."""
    lines = ["source: " + _idline(d)]
    if len(d) < 25:
        return lines + ["(%d of 25 bytes)" % len(d)]
    lines.append("load step %s, %s IoC; holdup %s" % (
        "500 mA/us" if d[10] & 3 == 1 else "150 mA/us", "90 %" if d[10] >> 2 & 1 else "25 %",
        "%d ms" % d[11] if d[11] else "not supported"))
    comp = _bits(d[12], {0: "LPS", 1: "PS1", 2: "PS2"})
    touch = _bits(d[13], {0: "low touch current EPS", 1: "ground pin", 2: "ground pin is protective earth"})
    lines.append("compliance %s; touch current %s; touch temp %s" % (
        "/".join(comp) or "none", ", ".join(touch) or "-",
        {0: "IEC 60950-1", 1: "IEC 62368-1 TS1", 2: "IEC 62368-1 TS2"}.get(d[20], "IEC 60950-1")))
    for n in range(3):
        v = _u16(d, 14 + 2 * n)
        if v & 0x1F:
            lines.append("peak current %d: %s" % (n + 1, _peak(v)))
    inp = d[21]
    src = ["external supply %s" % ("unconstrained" if inp >> 1 & 1 else "constrained")] if inp & 1 \
        else ["no external supply"]
    if inp >> 2 & 1:
        src.append("internal battery")
    lines.append("inputs: %s; batteries %d fixed + %d hot-swap" % (", ".join(src), d[22] & 0xF, d[22] >> 4))
    lines.append("SPR PDP %d W%s" % (d[23], ", EPR PDP %d W" % d[24] if d[24] else ""))
    return lines


def skedb(d):
    """Sink Capabilities Extended Data Block (Table 6.61)."""
    lines = ["sink: " + _idline(d)]
    if len(d) < 24:
        return lines + ["(%d of 24 bytes)" % len(d)]
    lines.append("SKEDB v%d, load step %s, load %s" % (
        d[10], "500 mA/us" if d[11] & 3 == 1 else "150 mA/us", _peak(_u16(d, 12))))
    lines.append("compliance %s; touch temp %s; batteries %d fixed + %d hot-swap" % (
        "/".join(_bits(d[14], {0: "LPS", 1: "PS1", 2: "PS2"})) or "none",
        {0: "none", 1: "IEC 60950-1", 2: "IEC 62368-1 TS1", 3: "IEC 62368-1 TS2"}.get(d[15], "IEC 60950-1"),
        d[16] & 0xF, d[16] >> 4))
    lines.append("modes: " + (", ".join(_bits(d[17], {0: "PPS charging", 1: "VBUS powered",
                                                     2: "AC powered", 3: "battery powered",
                                                     4: "battery essentially unlimited",
                                                     5: "AVS"})) or "none"))
    lines.append("SPR PDP min %d / operational %d / max %d W" % (d[18], d[19], d[20]))
    if d[21] or d[22] or d[23]:
        lines.append("EPR PDP min %d / operational %d / max %d W" % (d[21], d[22], d[23]))
    return lines


def _temp(t):
    return {0: "n/a", 1: "<2 C"}.get(t, "%d C" % t)


def sdb(d, sop):
    """Status Data Block (Tables 6.51 / 6.52)."""
    if not d:
        return []
    if sop:
        return ["cable plug temp %s%s" % (_temp(d[0]), ", THERMAL SHUTDOWN" if len(d) > 1 and d[1] & 1 else "")]
    s = ["temp %s" % _temp(d[0])]
    if len(d) > 1:
        pi = d[1]
        s.append({0: "internally powered", 1: "DC external power", 3: "AC external power"}.get(pi >> 1 & 3, "input ?"))
        if pi >> 3 & 1:
            s.append("from battery (mask 0x%02x)" % (d[2] if len(d) > 2 else 0))
        if pi >> 4 & 1:
            s.append("internal non-battery source")
    lines = ["status: " + ", ".join(s)]
    if len(d) > 3:
        ev = _bits(d[3], {1: "OCP", 2: "OTP", 3: "OVP"})
        ev.append("PPS CL mode" if d[3] >> 4 & 1 else "PPS CV mode")
        lines.append("events: %s; temperature %s" % (
            ", ".join(ev), {0: "not supported", 1: "normal", 2: "warning", 3: "over-temperature"}[
                d[4] >> 1 & 3] if len(d) > 4 else "?"))
    if len(d) > 5 and d[5]:
        lines.append("power limited by: " + ", ".join(_bits(d[5], {
            1: "cable current", 2: "other ports", 3: "external power", 4: "event flags", 5: "temperature"})))
    if len(d) > 6 and d[6]:
        lines.append("power state %s, indicator %s" % (
            {1: "S0", 2: "modern standby", 3: "S3", 4: "S4", 5: "S5", 6: "G3"}.get(d[6] & 7, "n/a"),
            {0: "off", 1: "on", 2: "blinking", 3: "breathing"}.get(d[6] >> 3 & 7, "?")))
    return lines


def ppssdb(d):
    """PPS Status Data Block (Table 6.58)."""
    if len(d) < 4:
        return []
    v, i, f = _u16(d, 0), d[2], d[3]
    return ["PPS status: %s, %s, temperature %s, %s mode" % (
        "V n/a" if v == 0xFFFF else "%.2f V" % (v * 0.02), "I n/a" if i == 0xFF else "%.2f A" % (i * 0.05),
        {0: "n/a", 1: "normal", 2: "warning", 3: "over-temperature"}[f >> 1 & 3], "CL" if f >> 3 & 1 else "CV")]


def _wh(v):
    return {0: "not present", 0xFFFF: "unknown"}.get(v, "%.1f Wh" % (v * 0.1))


def _cstr(b):
    return bytes(b).split(b"\0")[0].decode("ascii", "replace")


def epr_caps(d, sink):
    """EPR_Source/Sink_Capabilities: positions 1..7 SPR (zero-filled), 8+ EPR."""
    out = []
    for n, p in enumerate(_words(d), 1):
        if p:
            out.append("PDO %d: %s" % (n, pdo(p, sink)))
            if n == 1:
                fl = pdo1_flags(p, sink)
                if fl:
                    out.append("       " + fl)
    return out


def ext_data(typ, d, sop):
    """Decoded lines for an extended message's (reassembled) data block."""
    if typ == 1:
        return scedb(d)
    if typ == 2:
        return sdb(d, sop)
    if typ in (3, 4) and d:
        return ["battery ref %d (%s)" % (d[0], "fixed %d" % d[0] if d[0] < 4 else "hot-swap %d" % (d[0] - 4))]
    if typ == 5 and len(d) >= 9:
        return ["battery: vendor %s, product 0x%04X, design %s, last full %s%s" % (
            vendor(_u16(d, 0)), _u16(d, 2), _wh(_u16(d, 4)), _wh(_u16(d, 6)),
            ", invalid reference" if d[8] & 1 else "")]
    if typ == 6 and len(d) >= 2:
        return ["target: " + ("port/cable plug" if d[0] == 0 else "battery %d" % d[1] if d[0] == 1
                              else "invalid %d" % d[0])]
    if typ == 7 and len(d) >= 4:
        return ["manufacturer: vendor %s, product %s, \"%s\"" % (
            vendor(_u16(d, 0)), product(_u16(d, 0), _u16(d, 2)), _cstr(d[4:]))]
    if typ == 12:
        return ppssdb(d)
    if typ == 13 and len(d) >= 4:
        return ["country %s: %s" % (chr(d[0]) + chr(d[1]), bytes(d[4:]).hex(" "))]
    if typ == 14 and d:
        codes = [chr(d[i]) + chr(d[i + 1]) for i in range(2, min(2 + 2 * d[0], len(d) - 1), 2)]
        return ["countries: " + " ".join(codes)]
    if typ == 15:
        return skedb(d)
    if typ == 16 and d:
        return [{1: "EPR_Get_Source_Cap", 2: "EPR_Get_Sink_Cap", 3: "EPR_KeepAlive",
                 4: "EPR_KeepAlive_Ack"}.get(d[0], "extended control type %d" % d[0])]
    if typ in (17, 18):
        return epr_caps(d, typ == 18)
    if typ == 30 and len(d) >= 4:
        return ["vendor ext: SVID %s, command 0x%04X: %s" % (svid_name(_u16(d, 0)), _u16(d, 2) & 0x7FFF,
                                                              bytes(d[4:]).hex(" "))]
    return ["data: " + bytes(d).hex(" ")] if d else []


class Decoder:
    """Stateful frame decoder: remembers the last Source_Caps (for Request
    decoding) and reassembles chunked extended messages per sender."""

    def __init__(self):
        self.src_caps = []
        self.chunks = {}  # (sop, role bit, type) -> bytearray

    def frame(self, wire, sop=0):
        """One captured frame -> (summary line, [detail lines])."""
        if len(wire) < 2:
            return "runt", []
        hdr = wire[0] | (wire[1] << 8)
        nobj = (hdr >> 12) & 7
        typ = hdr & 0x1F
        ext = hdr >> 15
        rev = (hdr >> 6) & 3
        if sop:  # SOP'/SOP'': bit 8 is Cable Plug, bit 5 reserved
            tag = "CABLE" if hdr & (1 << 8) else "PORT"
        else:
            tag = ("SRC" if hdr & (1 << 8) else "SNK") + (" DFP" if hdr & (1 << 5) else " UFP")
        if ext:
            name = EXT_MSGS.get(typ, "ext%d" % typ)
        elif nobj:
            name = DATA_MSGS.get(typ, "data%d" % typ)
        else:
            name = CTRL_MSGS.get(typ, "type%d" % typ)
        objs = [int.from_bytes(wire[2 + 4 * i:6 + 4 * i], "little") for i in range(nobj)
                if 6 + 4 * i <= len(wire)]
        line = "%-14s [%s r%s id%d]%s" % (name, tag, {0: "1.0", 1: "2.0", 2: "3.x"}.get(rev, "?"),
                                          (hdr >> 9) & 7, "".join(" %08x" % o for o in objs))
        if ext:
            return line, self._ext(wire, hdr, typ, sop)
        if len(objs) != nobj or not objs:
            return line, []
        return line, self._data(typ, objs, sop, rev >= 2)

    def _data(self, typ, objs, sop, rev3):
        if typ == 15:
            return vdm(objs, sop, rev3)
        if sop:
            return []
        if typ in (1, 4):
            sink = typ == 4
            if not sink:
                self.src_caps = objs
            out = ["PDO %d: %s" % (n, pdo(o, sink)) for n, o in enumerate(objs, 1)]
            fl = pdo1_flags(objs[0], sink)
            return out[:1] + (["       " + fl] if fl else []) + out[1:]
        if typ == 2:
            pos = (objs[0] >> 28) & 0xF
            return [rdo(objs[0], self.src_caps[pos - 1] if 0 < pos <= len(self.src_caps) else None)]
        if typ == 9:  # EPR_Request: RDO + a copy of the requested PDO
            return [rdo(objs[0], objs[1] if len(objs) > 1 else None)]
        one = {3: bist, 5: bsdo, 6: ado, 7: ccdo, 8: eudo, 10: eprmdo, 12: rmdo}.get(typ)
        if one:
            return [one(objs[0])]
        if typ == 11:
            return source_info(objs)
        return []

    def _ext(self, wire, hdr, typ, sop):
        if len(wire) < 4:
            return ["extended header missing"]
        eh = wire[2] | (wire[3] << 8)
        chunked, num, req, size = eh >> 15, (eh >> 11) & 0xF, (eh >> 10) & 1, eh & 0x1FF
        payload = bytearray(wire[4:])
        if not chunked:
            got = payload[:size]
            note = [] if len(got) >= size else ["unchunked, %d of %d bytes captured" % (len(got), size)]
            return note + ext_data(typ, got, sop)
        if req:
            return ["chunk %d requested" % num]
        key = (sop, hdr >> 8 & 1, typ)
        if num == 0:
            self.chunks[key] = bytearray()
        buf = self.chunks.get(key)
        if buf is None or len(buf) != 26 * num:
            self.chunks.pop(key, None)
            return ["chunk %d of a %d-byte block without its earlier chunks" % (num, size)]
        buf += payload[:min(26, size - len(buf))]
        if len(buf) < size:
            return ["chunk %d: %d of %d bytes" % (num, len(buf), size)]
        del self.chunks[key]
        return (["chunks 0-%d reassembled, %d bytes" % (num, size)] if num else []) + ext_data(typ, buf, sop)
