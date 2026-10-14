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

Field layouts are the named constants of tools/pd_spec.py, taken from the
USB PD specification, Revision 3.2 Version 1.2 (usb.org document library,
free download — table numbers in the comments refer to it) and Revision
2.0 Version 1.3 for the PD 2.0 VDOs (cable / AMA VDOs differ from 3.x);
cross-checked against the Linux
kernel's include/linux/usb/pd.h / pd_vdo.h / pd_ext_sdb.h / pd_ado.h,
Chromium EC include/usb_pd.h and ST's stm32-mw-usbpd-core usbpd_def.h.
Vendor/product names come from the system usb.ids (PD uses the USB-IF
vendor IDs); a PD product ID is vendor-assigned and usually not listed.
"""
import os

from pd_spec import *  # noqa: F403 — the spec's named fields and values

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


def _flags(v, names):
    """Names of the set flags in `v` per {field: name}, lowest bit first."""
    return [n for f, n in sorted(names.items(), key=lambda kv: kv[0].shift) if get(v, f)]


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
    return "0x%04X (%s)" % (s, SVID_NAMES[s]) if s in SVID_NAMES else vendor(s)


def _speed(names, v, f):
    s = get(v, f)
    return names.get(s, "speed %d" % s)


# ---- power data objects (6.4.1.3) ----

def _peak(p, f, sink):
    """", peak OLn" for a source PDO's Peak Current field (Table 6.18)."""
    return ", " + PEAK_NAMES[get(p, f)] if not sink and get(p, f) else ""


def pdo(p, sink=False):
    """One power data object -> text."""
    kind = get(p, PDO_TYPE)
    if kind == PDO_FIXED:
        return "fixed %.2f V %.2f A%s" % (val(p, FIXED_VOLTAGE), val(p, FIXED_MAX_CURRENT),
                                          _peak(p, FIXED_PEAK_CURRENT, sink))
    if kind == PDO_BATTERY:
        return "battery %.2f-%.2f V %.2f W" % (val(p, BATTERY_MIN_VOLTAGE), val(p, BATTERY_MAX_VOLTAGE),
                                               val(p, BATTERY_MAX_POWER))
    if kind == PDO_VARIABLE:
        return "variable %.2f-%.2f V %.2f A" % (val(p, VARIABLE_MIN_VOLTAGE), val(p, VARIABLE_MAX_VOLTAGE),
                                                val(p, VARIABLE_MAX_CURRENT))
    apdo = get(p, APDO_TYPE)
    if apdo == APDO_PPS:
        return "PPS %.1f-%.1f V %.2f A%s" % (val(p, PPS_MIN_VOLTAGE), val(p, PPS_MAX_VOLTAGE),
                                            val(p, PPS_MAX_CURRENT),
                                            ", power limited" if not sink and get(p, PPS_POWER_LIMITED) else "")
    if apdo == APDO_EPR_AVS:
        return "EPR AVS %.1f-%.1f V %d W%s" % (val(p, EPR_AVS_MIN_VOLTAGE), val(p, EPR_AVS_MAX_VOLTAGE),
                                               val(p, EPR_AVS_PDP), _peak(p, EPR_AVS_PEAK_CURRENT, sink))
    if apdo == APDO_SPR_AVS:
        return "SPR AVS 9-15 V %.2f A, 15-20 V %.2f A%s" % (val(p, SPR_AVS_MAX_CURRENT_15V),
                                                           val(p, SPR_AVS_MAX_CURRENT_20V),
                                                           _peak(p, SPR_AVS_PEAK_CURRENT, sink))
    return "APDO type %d (0x%08x)" % (apdo, p)


def pdo1_flags(p, sink):
    """Flags carried by the 5 V fixed PDO in position 1 (Tables 6.8 / 6.9)."""
    if get(p, PDO_TYPE) != PDO_FIXED:
        return None
    if sink:
        f = _flags(p, {SNK_DUAL_ROLE_POWER: "dual-role power", SNK_HIGHER_CAPABILITY: "higher capability",
                       SNK_UNCONSTRAINED: "unconstrained power", SNK_USB_COMMS: "USB comms",
                       SNK_DUAL_ROLE_DATA: "dual-role data"})
        if get(p, SNK_FRS_CURRENT):
            f.append("FRS current %s" % FRS_NAMES[get(p, SNK_FRS_CURRENT)])
    else:
        f = _flags(p, {SRC_DUAL_ROLE_POWER: "dual-role power", SRC_USB_SUSPEND: "USB suspend",
                       SRC_UNCONSTRAINED: "unconstrained power", SRC_USB_COMMS: "USB comms",
                       SRC_DUAL_ROLE_DATA: "dual-role data", SRC_UNCHUNKED_EXT: "unchunked ext msgs",
                       SRC_EPR_CAPABLE: "EPR capable"})
    return "flags: " + (", ".join(f) if f else "none")


def rdo(r, src):
    """Request data object -> text; `src` = the PDO it selects (None if unknown)."""
    what = "PDO %d" % get(r, RDO_OBJECT_POSITION)
    tail = "".join(", " + f for f in _flags(r, {
        RDO_CAPABILITY_MISMATCH: "capability mismatch", RDO_USB_COMMS: "USB comms",
        RDO_NO_USB_SUSPEND: "no USB suspend", RDO_UNCHUNKED_EXT: "unchunked ext msgs",
        RDO_EPR_CAPABLE: "EPR capable"}))
    if src is not None and get(src, PDO_TYPE) == PDO_APDO:
        avs = get(src, APDO_TYPE) in (APDO_EPR_AVS, APDO_SPR_AVS)
        return "%s (%s): %.3f V %.2f A%s" % (what, " ".join(pdo(src).split(" ")[:2]),
                                             val(r, RDO_AVS_VOLTAGE if avs else RDO_PPS_VOLTAGE),
                                             val(r, RDO_APDO_CURRENT), tail)
    if src is not None:
        what += " (%s)" % pdo(src)
    if src is not None and get(src, PDO_TYPE) == PDO_BATTERY:
        return "%s: operating %.2f W, max %.2f W%s" % (what, val(r, RDO_OPERATING_POWER),
                                                      val(r, RDO_MAX_POWER), tail)
    return "%s: operating %.2f A, max %.2f A%s" % (what, val(r, RDO_OPERATING_CURRENT),
                                                  val(r, RDO_MAX_CURRENT), tail)


# ---- other data objects (6.4.3 - 6.4.11) ----

def bist(o):
    m = get(o, BIST_MODE)
    return "BIST mode: " + BIST_MODES.get(m, "invalid %d" % m)


def bsdo(o):
    cap = get(o, BSDO_CAPACITY)
    s = "battery: %s" % ("SoC unknown" if cap == BSDO_CAPACITY_UNKNOWN else "%.1f Wh" % val(o, BSDO_CAPACITY))
    if get(o, BSDO_INVALID_REF):
        return s + ", invalid battery reference"
    if not get(o, BSDO_PRESENT):
        return s + ", not present"
    return s + ", " + BSDO_CHARGING_NAMES.get(get(o, BSDO_CHARGING_STATUS), "?")


def ado(o):
    f = _flags(o, ADO_EVENTS)
    if get(o, ADO_BATTERY_STATUS_CHANGE):
        f.append("batteries fixed %s / hot-swap %s" % (bin(get(o, ADO_FIXED_BATTERIES)),
                                                       bin(get(o, ADO_HOT_SWAP_BATTERIES))))
    if get(o, ADO_EXTENDED):
        e = get(o, ADO_EXTENDED_EVENT)
        f.append(ADO_EXTENDED_EVENTS.get(e, "extended alert %d" % e))
    return "alert: " + (", ".join(f) if f else "none")


def ccdo(o):
    return "country: %s" % (chr(get(o, CCDO_CHAR1)) + chr(get(o, CCDO_CHAR2)))


def eudo(o):
    m = get(o, EUDO_USB_MODE)
    return "enter %s; cable %s, %s, %s%s" % (
        EUDO_USB_MODES.get(m, "USB4 (invalid %d)" % m), _speed(USB_SPEEDS, o, EUDO_CABLE_SPEED),
        EUDO_CABLE_TYPES[get(o, EUDO_CABLE_TYPE)], EUDO_CABLE_CURRENTS[get(o, EUDO_CABLE_CURRENT)],
        "".join(", " + x for x in _flags(o, EUDO_FLAGS)))


def eprmdo(o):
    act, data = get(o, EPRMDO_ACTION), get(o, EPRMDO_DATA)
    s = EPRMDO_ACTIONS.get(act, "action 0x%02x" % act)
    if act == EPR_ENTER:
        s += ", sink EPR operational PDP %d W" % data
    elif act == EPR_ENTER_FAILED:
        s += ": " + EPR_FAIL_REASONS.get(data, "data 0x%02x" % data)
    return "EPR mode: " + s


def source_info(objs):
    o = objs[0]
    lines = ["source info: %s port, max %d W, present %d W, reported %d W" % (
        "guaranteed" if get(o, SIDO_PORT_TYPE) else "managed", get(o, SIDO1_MAX_PDP),
        get(o, SIDO1_PRESENT_PDP), get(o, SIDO1_REPORTED_PDP))]
    if len(objs) > 1:
        o = objs[1]
        lines.append("source info 2: max %.1f W, guaranteed %.1f W%s" % (
            val(o, SIDO2_MAX_PDP), val(o, SIDO2_GUARANTEED_PDP), ", DPS port" if get(o, SIDO2_DPS_PORT) else ""))
    return lines


def rmdo(o):
    return "PD revision %d.%d, version %d.%d" % (get(o, RMDO_REV_MAJOR), get(o, RMDO_REV_MINOR),
                                                 get(o, RMDO_VER_MAJOR), get(o, RMDO_VER_MINOR))


# ---- VDMs (6.4.12) ----

def cable_vdo(v, rev3, active):
    """Passive / active cable VDO (VDO1) -> text."""
    lat = get(v, CABLE_LATENCY)
    if 1 <= lat <= 7:
        length = "~%d m" % lat
    elif lat == 8:
        length = "~100 m" if active else ">7 m"
    elif active and lat in (9, 10):
        length = "~%d m" % ((lat - 7) * 100)
    else:
        length = "latency %d" % lat
    cur = get(v, CABLE_VBUS_CURRENT)
    if rev3:
        cur = CABLE_CURRENT_NAMES.get(cur, "3 A (invalid %d)" % cur)
        speed = _speed(USB_SPEEDS, v, CABLE_USB_SPEED)
    else:
        cur = CABLE_CURRENT_NAMES.get(cur, "reserved %d" % cur)
        speed = _speed(USB_SPEEDS_PD20, v, CABLE_USB_SPEED)
    s = "%s cable: %s VBUS, %s, to %s, %s, HW %d FW %d" % (
        "active" if active else "passive", cur, speed, CABLE_PLUG_NAMES[get(v, CABLE_PLUG_TYPE)], length,
        get(v, CABLE_HW_VERSION), get(v, CABLE_FW_VERSION))
    term = get(v, CABLE_TERMINATION)
    if active:
        s += ", %s" % {2: "one end active", 3: "both ends active"}.get(term, "termination %d" % term)
    elif term == 1:
        s += ", VCONN required"
    active_bits = {CABLE_VBUS_THROUGH: "VBUS through cable", ACTIVE_SOP2_CONTROLLER: "SOP'' controller"}
    if rev3:
        mv = get(v, CABLE_MAX_VBUS)
        s += ", max %s" % CABLE_MAX_VBUS_NAMES.get(mv, "%d V (deprecated)" % (20 + 10 * mv))
        f = _flags(v, {CABLE_EPR_CAPABLE: "EPR capable"})
        if active:
            f += ["SBU %s" % ("not supported" if get(v, ACTIVE_SBU_NOT_SUPPORTED)
                              else ("active" if get(v, ACTIVE_SBU_ACTIVE) else "passive"))]
            f += _flags(v, active_bits)
        s += "".join(", " + x for x in f) + ", VDO v%d" % get(v, CABLE_VDO_VERSION)
    else:
        s += "".join(", " + x for x in _flags(v, active_bits if active
                                              else {CABLE_VBUS_THROUGH: "VBUS through cable"}))
    return s


def active_vdo2(v):
    """Active Cable VDO2 (Table 6.44)."""
    f = ["max %d C, shutdown %d C" % (get(v, ACTIVE2_MAX_TEMP), get(v, ACTIVE2_SHUTDOWN_TEMP)),
         "optical" if get(v, ACTIVE2_OPTICAL) else "copper",
         "re-timer" if get(v, ACTIVE2_RETIMER) else "re-driver",
         "no USB4" if get(v, ACTIVE2_NO_USB4) else "USB4",
         "no USB2" if get(v, ACTIVE2_NO_USB2) else "USB2",
         "no USB3" if get(v, ACTIVE2_NO_USB3) else "USB3",
         "two lanes" if get(v, ACTIVE2_TWO_LANES) else "one lane",
         "Gen2+" if get(v, ACTIVE2_GEN2) else "Gen1"]
    f += _flags(v, {ACTIVE2_OPTICALLY_ISOLATED: "optically isolated", ACTIVE2_USB4_ASYMMETRIC: "USB4 asymmetric"})
    return "active cable 2: " + ", ".join(f)


def vpd_vdo(v):
    """VCONN Powered USB Device VDO (Table 6.45)."""
    ct = get(v, VPD_CHARGE_THROUGH)
    return "VPD: HW %d FW %d, charge through %s%s, VBUS %d / GND %d (2 mOhm / 1 mOhm units)" % (
        get(v, VPD_HW_VERSION), get(v, VPD_FW_VERSION), "supported" if ct else "not supported",
        (", " + ("5 A" if get(v, VPD_CT_CURRENT_5A) else "3 A")) if ct else "",
        get(v, VPD_VBUS_IMPEDANCE), get(v, VPD_GND_IMPEDANCE))


def ufp_vdo(v):
    """UFP VDO (Table 6.40)."""
    f = _flags(v, {UFP_USB4_DEVICE: "USB4 device", UFP_USB32_DEVICE: "USB3.2 device"})
    usb2 = get(v, UFP_USB2_DEVICE)
    if usb2:
        f.append(UFP_USB2_NAMES.get(usb2, "USB2 invalid (11b)"))
    f += _flags(v, UFP_ALT_MODES)
    if get(v, UFP_VCONN_REQUIRED):
        f.append("needs VCONN %s W" % VCONN_POWER_NAMES[get(v, UFP_VCONN_POWER)])
    return "UFP: %s, highest %s, VDO v1.%d" % (", ".join(f) or "no USB", _speed(USB_SPEEDS, v, UFP_USB_SPEED),
                                              get(v, UFP_VDO_VERSION))


def dfp_vdo(v):
    """DFP VDO (Table 6.41)."""
    return "DFP: %s, port %d" % (", ".join(_flags(v, DFP_HOSTS)) or "no USB host", get(v, DFP_PORT_NUMBER))


def ama_vdo(v):
    """PD 2.0 Alternate Mode Adapter VDO (PD 2.0 Table 6-30)."""
    return "AMA: HW %d FW %d, %s, VCONN %s, VBUS %s" % (
        get(v, AMA_HW_VERSION), get(v, AMA_FW_VERSION), AMA_USB_SPEEDS.get(get(v, AMA_USB_SPEED), "?"),
        ("required %s W" % VCONN_POWER_NAMES[get(v, AMA_VCONN_POWER)]) if get(v, AMA_VCONN_REQUIRED)
        else "not required",
        "required" if get(v, AMA_VBUS_REQUIRED) else "not required")


def discover_identity(objs, sop, rev3, svdm2):
    """Discover Identity ACK VDOs (after the VDM header) -> lines."""
    idh, cert, prod = objs[0], objs[1], objs[2]
    vid = get(idh, IDH_VID)
    ut = get(idh, IDH_UFP_TYPE)
    if sop:
        ptype = CABLE_TYPE_NAMES.get(ut, "type %d" % ut)
    else:
        ptype = {UFP_NOT_UFP: "not a UFP" if rev3 else "undefined", **UFP_TYPE_NAMES}.get(ut, "UFP type %d" % ut)
    dt = get(idh, IDH_DFP_TYPE) if rev3 and not sop else DFP_NOT_DFP
    roles = ptype if not dt else "%s / DFP %s" % (ptype, DFP_TYPE_NAMES.get(dt, "type %d" % dt))
    f = _flags(idh, {IDH_USB_HOST: "USB host", IDH_USB_DEVICE: "USB device", IDH_MODAL: "modal"})
    if rev3 and get(idh, IDH_CONNECTOR):
        f.append(CONNECTOR_NAMES.get(get(idh, IDH_CONNECTOR), "connector ?"))
    lines = ["identity: %s, vendor %s, product %s bcd %04X, XID %d%s" % (
        roles, vendor(vid), product(vid, get(prod, PRODUCT_PID)), get(prod, PRODUCT_BCD), cert,
        "".join(", " + x for x in f))]
    rest = objs[3:]  # product type VDOs
    if sop:
        if ut in (CABLE_PASSIVE, CABLE_ACTIVE) and rest:
            lines.append(cable_vdo(rest[0], rev3, ut == CABLE_ACTIVE))
            if ut == CABLE_ACTIVE and rev3 and len(rest) > 1:
                lines.append(active_vdo2(rest[1]))
        elif ut == CABLE_VPD and rest:
            lines.append(vpd_vdo(rest[0]))
        return lines
    if not rev3:
        if ut == UFP_AMA and rest:
            lines.append(ama_vdo(rest[0]))
        return lines
    if not svdm2:  # SVDM 1.0 product VDOs used older layouts
        lines += ["product VDO: 0x%08x" % v for v in rest]
        return lines
    has_dfp_vdo = dt in (DFP_HUB, DFP_HOST, DFP_POWER_BRICK)
    if ut in (UFP_HUB, UFP_PERIPHERAL) and rest:
        lines.append(ufp_vdo(rest[0]))
        rest = rest[2:] if has_dfp_vdo else rest[1:]  # UFP VDO, pad, DFP VDO (6.4.12.3)
    if has_dfp_vdo and rest:
        lines.append(dfp_vdo(rest[0]))
    return lines


def vdm(objs, sop, rev3):
    """Vendor_Defined message objects -> list of text lines."""
    h = objs[0]
    svid = get(h, VDM_SVID)
    if not get(h, VDM_STRUCTURED):
        return ["unstructured VDM, SVID %s: %s" % (svid_name(svid), " ".join("%08x" % o for o in objs[1:]))]
    cmd, ctype = get(h, VDM_COMMAND), get(h, VDM_COMMAND_TYPE)
    major = get(h, VDM_VERSION_MAJOR)
    ver = "1.0" if major == 0 else "2.%d" % get(h, VDM_VERSION_MINOR)
    name = VDM_CMDS.get(cmd, "SVID cmd %d" % cmd if cmd >= CMD_SVID_SPECIFIC else "cmd %d" % cmd)
    pos = get(h, VDM_OBJECT_POSITION)
    lines = ["VDM %s %s, SVID %s, SVDM %s%s" % (
        name, VDM_TYPES[ctype], svid_name(svid), ver,
        (", exit all modes" if pos == VDM_EXIT_ALL_MODES and cmd == CMD_EXIT_MODE else ", object %d" % pos)
        if pos else "")]
    body = objs[1:]
    if ctype != VDM_ACK or not body:
        if cmd == CMD_ATTENTION and body:
            lines.append("attention VDO: " + " ".join("%08x" % o for o in body))
        return lines
    if cmd == CMD_DISCOVER_IDENTITY and len(body) >= 3:
        lines += discover_identity(body, sop, rev3, major >= 1)
    elif cmd == CMD_DISCOVER_SVIDS:
        svids = [get(w, f) for w in body for f in SVIDS_PER_VDO]
        lines.append("SVIDs: " + (", ".join(svid_name(s) for s in svids if s) or "none"))
    elif cmd == CMD_DISCOVER_MODES:
        lines += ["mode %d: 0x%08x" % (n, o) for n, o in enumerate(body, 1)]
    else:
        lines.append("VDOs: " + " ".join("%08x" % o for o in body))
    return lines


# ---- extended messages (6.5) ----

def _peak_current(v):
    """Peak Current 1..3 / Sink Load Characteristics field (Tables 6.50 / 6.61)."""
    pct = min(get(v, PEAK_OVERLOAD), PEAK_OVERLOAD_MAX) * PEAK_OVERLOAD.unit
    if not pct:
        return "none"
    return "+%d %% for %d ms at %d %% duty%s" % (pct, val(v, PEAK_PERIOD), val(v, PEAK_DUTY_CYCLE),
                                                 ", VBUS may droop 5 %" if get(v, PEAK_VBUS_DROOP) else "")


def _idline(d, vid_at, pid_at, xid_at, fw_at, hw_at):
    if len(d) < pid_at + 2:
        return "(%d bytes)" % len(d)
    vid, pid, xid = u16(d, vid_at), u16(d, pid_at), u32(d, xid_at)
    s = "vendor %s, product %s" % (vendor(vid), product(vid, pid))
    if xid is not None:
        s += ", XID %d" % xid
    if len(d) > hw_at:
        s += ", FW %d HW %d" % (d[fw_at], d[hw_at])
    return s


def _load_step(b):
    return "500 mA/us" if get(b, LOAD_STEP) == 1 else "150 mA/us"


def _compliance(b):
    return "/".join(_flags(b, COMPLIANCE_BITS)) or "none"


def _batteries(b):
    return "batteries %d fixed + %d hot-swap" % (get(b, BATTERIES_FIXED), get(b, BATTERIES_HOT_SWAP))


def scedb(d):
    """Source Capabilities Extended Data Block (Table 6.50)."""
    lines = ["source: " + _idline(d, SCEDB_VID, SCEDB_PID, SCEDB_XID, SCEDB_FW_VERSION, SCEDB_HW_VERSION)]
    if len(d) < SCEDB_LEN:
        return lines + ["(%d of %d bytes)" % (len(d), SCEDB_LEN)]
    reg, holdup = d[SCEDB_VOLTAGE_REGULATION], d[SCEDB_HOLDUP_TIME]
    lines.append("load step %s, %s IoC; holdup %s" % (
        _load_step(reg), "90 %" if get(reg, VOLTAGE_REGULATION_IOC_90) else "25 %",
        "%d ms" % holdup if holdup else "not supported"))
    lines.append("compliance %s; touch current %s; touch temp %s" % (
        _compliance(d[SCEDB_COMPLIANCE]), ", ".join(_flags(d[SCEDB_TOUCH_CURRENT], TOUCH_CURRENT_BITS)) or "-",
        SCEDB_TOUCH_TEMPS.get(d[SCEDB_TOUCH_TEMP], SCEDB_TOUCH_TEMPS[0])))
    for n, off in enumerate(SCEDB_PEAK_CURRENTS, 1):
        v = u16(d, off)
        if get(v, PEAK_OVERLOAD):
            lines.append("peak current %d: %s" % (n, _peak_current(v)))
    inp = d[SCEDB_SOURCE_INPUTS]
    src = ["external supply %s" % ("unconstrained" if get(inp, INPUT_EXTERNAL_UNCONSTRAINED) else "constrained")] \
        if get(inp, INPUT_EXTERNAL_SUPPLY) else ["no external supply"]
    if get(inp, INPUT_INTERNAL_BATTERY):
        src.append("internal battery")
    lines.append("inputs: %s; %s" % (", ".join(src), _batteries(d[SCEDB_BATTERIES])))
    lines.append("SPR PDP %d W%s" % (d[SCEDB_SPR_PDP], ", EPR PDP %d W" % d[SCEDB_EPR_PDP] if d[SCEDB_EPR_PDP] else ""))
    return lines


def skedb(d):
    """Sink Capabilities Extended Data Block (Table 6.61)."""
    lines = ["sink: " + _idline(d, SKEDB_VID, SKEDB_PID, SKEDB_XID, SKEDB_FW_VERSION, SKEDB_HW_VERSION)]
    if len(d) < SKEDB_LEN:
        return lines + ["(%d of %d bytes)" % (len(d), SKEDB_LEN)]
    lines.append("SKEDB v%d, load step %s, load %s" % (
        d[SKEDB_VERSION], _load_step(d[SKEDB_LOAD_STEP]), _peak_current(u16(d, SKEDB_LOAD_CHARACTERISTICS))))
    lines.append("compliance %s; touch temp %s; %s" % (
        _compliance(d[SKEDB_COMPLIANCE]), SKEDB_TOUCH_TEMPS.get(d[SKEDB_TOUCH_TEMP], SKEDB_TOUCH_TEMPS[1]),
        _batteries(d[SKEDB_BATTERY_INFO])))
    lines.append("modes: " + (", ".join(_flags(d[SKEDB_SINK_MODES], SINK_MODE_BITS)) or "none"))
    lines.append("SPR PDP min %d / operational %d / max %d W" % (
        d[SKEDB_SPR_MIN_PDP], d[SKEDB_SPR_OPERATIONAL_PDP], d[SKEDB_SPR_MAX_PDP]))
    epr = (d[SKEDB_EPR_MIN_PDP], d[SKEDB_EPR_OPERATIONAL_PDP], d[SKEDB_EPR_MAX_PDP])
    if any(epr):
        lines.append("EPR PDP min %d / operational %d / max %d W" % epr)
    return lines


def _temp(t):
    return {TEMP_NOT_SUPPORTED: "n/a", TEMP_BELOW_2C: "<2 C"}.get(t, "%d C" % t)


def sdb(d, sop):
    """Status Data Block (Tables 6.51 / 6.52)."""
    if not d:
        return []
    if sop:
        shut = len(d) > CABLE_SDB_FLAGS and get(d[CABLE_SDB_FLAGS], CABLE_THERMAL_SHUTDOWN)
        return ["cable plug temp %s%s" % (_temp(d[CABLE_SDB_TEMP]), ", THERMAL SHUTDOWN" if shut else "")]
    s = ["temp %s" % _temp(d[SDB_INTERNAL_TEMP])]
    if len(d) > SDB_PRESENT_INPUT:
        pi = d[SDB_PRESENT_INPUT]
        s.append(INPUT_EXTERNAL_NAMES.get(get(pi, INPUT_EXTERNAL_POWER), "input ?"))
        if get(pi, INPUT_FROM_BATTERY):
            s.append("from battery (mask 0x%02x)" % (
                d[SDB_PRESENT_BATTERY_INPUT] if len(d) > SDB_PRESENT_BATTERY_INPUT else 0))
        if get(pi, INPUT_FROM_NON_BATTERY):
            s.append("internal non-battery source")
    lines = ["status: " + ", ".join(s)]
    if len(d) > SDB_EVENT_FLAGS:
        ev = d[SDB_EVENT_FLAGS]
        f = _flags(ev, EVENT_BITS) + ["PPS CL mode" if get(ev, EVENT_PPS_CL_MODE) else "PPS CV mode"]
        lines.append("events: %s; temperature %s" % (
            ", ".join(f), TEMPERATURE_STATUS_NAMES[get(d[SDB_TEMPERATURE_STATUS], TEMPERATURE_STATUS)]
            if len(d) > SDB_TEMPERATURE_STATUS else "?"))
    if len(d) > SDB_POWER_STATUS and d[SDB_POWER_STATUS]:
        lines.append("power limited by: " + ", ".join(_flags(d[SDB_POWER_STATUS], POWER_LIMIT_BITS)))
    if len(d) > SDB_POWER_STATE_CHANGE and d[SDB_POWER_STATE_CHANGE]:
        p = d[SDB_POWER_STATE_CHANGE]
        lines.append("power state %s, indicator %s" % (POWER_STATE_NAMES.get(get(p, POWER_STATE), "n/a"),
                                                       POWER_INDICATOR_NAMES.get(get(p, POWER_INDICATOR), "?")))
    return lines


def ppssdb(d):
    """PPS Status Data Block (Table 6.58)."""
    if len(d) < PPSSDB_LEN:
        return []
    v, i, f = u16(d, PPSSDB_OUTPUT_VOLTAGE), d[PPSSDB_OUTPUT_CURRENT], d[PPSSDB_REAL_TIME_FLAGS]
    ptf = get(f, PPS_TEMPERATURE_FLAG)
    return ["PPS status: %s, %s, temperature %s, %s mode" % (
        "V n/a" if v == PPS_VOLTAGE_NOT_SUPPORTED else "%.2f V" % (v * PPS_VOLTAGE_UNIT),
        "I n/a" if i == PPS_CURRENT_NOT_SUPPORTED else "%.2f A" % (i * PPS_CURRENT_UNIT),
        "n/a" if ptf == 0 else TEMPERATURE_STATUS_NAMES[ptf], "CL" if get(f, PPS_CL_MODE) else "CV")]


def _wh(v):
    return {BATTERY_NOT_PRESENT: "not present", BATTERY_CAPACITY_UNKNOWN: "unknown"}.get(
        v, "%.1f Wh" % (v * BATTERY_CAPACITY_UNIT))


def _cstr(b):
    return bytes(b).split(b"\0")[0].decode("ascii", "replace")


def epr_caps(d, sink):
    """EPR_Source/Sink_Capabilities: positions 1..EPR_SPR_SLOTS SPR (zero-filled), then EPR."""
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
    if typ == EXT_SOURCE_CAP_EXT:
        return scedb(d)
    if typ == EXT_STATUS:
        return sdb(d, sop)
    if typ in (EXT_GET_BATTERY_CAP, EXT_GET_BATTERY_STATUS) and d:
        ref = d[BATTERY_REF]
        return ["battery ref %d (%s)" % (ref, "fixed %d" % ref if ref < FIXED_BATTERY_SLOTS
                                         else "hot-swap %d" % (ref - FIXED_BATTERY_SLOTS))]
    if typ == EXT_BATTERY_CAP and len(d) >= BCDB_LEN:
        return ["battery: vendor %s, product 0x%04X, design %s, last full %s%s" % (
            vendor(u16(d, BCDB_VID)), u16(d, BCDB_PID), _wh(u16(d, BCDB_DESIGN_CAPACITY)),
            _wh(u16(d, BCDB_LAST_FULL_CAPACITY)),
            ", invalid reference" if get(d[BCDB_BATTERY_TYPE], BCDB_INVALID_REF) else "")]
    if typ == EXT_GET_MANUFACTURER_INFO and len(d) > GMIDB_REF:
        t = d[GMIDB_TARGET]
        return ["target: " + ("port/cable plug" if t == MANUFACTURER_TARGET_PORT
                              else "battery %d" % d[GMIDB_REF] if t == MANUFACTURER_TARGET_BATTERY
                              else "invalid %d" % t)]
    if typ == EXT_MANUFACTURER_INFO and len(d) >= MIDB_STRING:
        vid = u16(d, MIDB_VID)
        return ["manufacturer: vendor %s, product %s, \"%s\"" % (
            vendor(vid), product(vid, u16(d, MIDB_PID)), _cstr(d[MIDB_STRING:]))]
    if typ == EXT_PPS_STATUS:
        return ppssdb(d)
    if typ == EXT_COUNTRY_INFO and len(d) >= CIDB_DATA:
        code = chr(d[CIDB_COUNTRY_CODE]) + chr(d[CIDB_COUNTRY_CODE + 1])
        return ["country %s: %s" % (code, bytes(d[CIDB_DATA:]).hex(" "))]
    if typ == EXT_COUNTRY_CODES and d:
        end = min(CCDB_CODES + 2 * d[CCDB_LENGTH], len(d) - 1)
        return ["countries: " + " ".join(chr(d[i]) + chr(d[i + 1]) for i in range(CCDB_CODES, end, 2))]
    if typ == EXT_SINK_CAP_EXT:
        return skedb(d)
    if typ == EXT_EXTENDED_CONTROL and d:
        t = d[ECDB_TYPE]
        return [ECDB_TYPES.get(t, "extended control type %d" % t)]
    if typ in (EXT_EPR_SOURCE_CAP, EXT_EPR_SINK_CAP):
        return epr_caps(d, typ == EXT_EPR_SINK_CAP)
    if typ == EXT_VENDOR_DEFINED and len(d) >= VDEM_DATA:
        return ["vendor ext: SVID %s, command 0x%04X: %s" % (
            svid_name(u16(d, VDEM_SVID)), get(u16(d, VDEM_COMMAND), VDEM_COMMAND_SPACE), bytes(d[VDEM_DATA:]).hex(" "))]
    return ["data: " + bytes(d).hex(" ")] if d else []


class Decoder:
    """Stateful frame decoder: remembers the last Source_Caps (for Request
    decoding) and reassembles chunked extended messages per sender."""

    def __init__(self):
        self.src_caps = []
        self.chunks = {}  # (sop, role/cable-plug bit, type) -> bytearray

    def frame(self, wire, sop=0):
        """One captured frame (header + data objects, LE) -> (summary line,
        [detail lines]); `sop` 0 = SOP, else SOP'/SOP'' (and _DEBUG)."""
        if len(wire) < 2:
            return "runt", []
        hdr = u16(wire, 0)
        nobj, typ, rev = get(hdr, HDR_NUM_OBJECTS), get(hdr, HDR_MSG_TYPE), get(hdr, HDR_SPEC_REV)
        ext = get(hdr, HDR_EXTENDED)
        if sop:
            tag = "CABLE" if get(hdr, HDR_CABLE_PLUG) else "PORT"
        else:
            tag = ("SRC" if get(hdr, HDR_POWER_ROLE) else "SNK") + (" DFP" if get(hdr, HDR_DATA_ROLE) else " UFP")
        if ext:
            name = EXT_MSGS.get(typ, "ext%d" % typ)
        elif nobj:
            name = DATA_MSGS.get(typ, "data%d" % typ)
        else:
            name = CTRL_MSGS.get(typ, "type%d" % typ)
        objs = _words(wire[2:2 + 4 * nobj])
        line = "%-14s [%s r%s id%d]%s" % (name, tag, REV_NAMES.get(rev, "?"), get(hdr, HDR_MSG_ID),
                                          "".join(" %08x" % o for o in objs))
        if ext:
            return line, self._ext(wire, hdr, typ, sop)
        if len(objs) != nobj or not objs:
            return line, []
        return line, self._data(typ, objs, sop, rev >= REV_3_X)

    def _data(self, typ, objs, sop, rev3):
        if typ == DATA_VENDOR_DEFINED:
            return vdm(objs, sop, rev3)
        if sop:
            return []
        if typ in (DATA_SOURCE_CAP, DATA_SINK_CAP):
            sink = typ == DATA_SINK_CAP
            if not sink:
                self.src_caps = objs
            out = ["PDO %d: %s" % (n, pdo(o, sink)) for n, o in enumerate(objs, 1)]
            fl = pdo1_flags(objs[0], sink)
            return out[:1] + (["       " + fl] if fl else []) + out[1:]
        if typ == DATA_REQUEST:
            pos = get(objs[0], RDO_OBJECT_POSITION)
            return [rdo(objs[0], self.src_caps[pos - 1] if 0 < pos <= len(self.src_caps) else None)]
        if typ == DATA_EPR_REQUEST:  # RDO + a copy of the requested PDO
            return [rdo(objs[0], objs[1] if len(objs) > 1 else None)]
        one = {DATA_BIST: bist, DATA_BATTERY_STATUS: bsdo, DATA_ALERT: ado, DATA_GET_COUNTRY_INFO: ccdo,
               DATA_ENTER_USB: eudo, DATA_EPR_MODE: eprmdo, DATA_REVISION: rmdo}.get(typ)
        if one:
            return [one(objs[0])]
        if typ == DATA_SOURCE_INFO:
            return source_info(objs)
        return []

    def _ext(self, wire, hdr, typ, sop):
        if len(wire) < 4:
            return ["extended header missing"]
        eh = u16(wire, 2)
        num, size = get(eh, EXTHDR_CHUNK_NUMBER), get(eh, EXTHDR_DATA_SIZE)
        payload = bytearray(wire[4:])
        if not get(eh, EXTHDR_CHUNKED):
            got = payload[:size]
            note = [] if len(got) >= size else ["unchunked, %d of %d bytes captured" % (len(got), size)]
            return note + ext_data(typ, got, sop)
        if get(eh, EXTHDR_REQUEST_CHUNK):
            return ["chunk %d requested" % num]
        key = (sop, get(hdr, HDR_POWER_ROLE), typ)
        if num == 0:
            self.chunks[key] = bytearray()
        buf = self.chunks.get(key)
        if buf is None or len(buf) != MAX_EXT_CHUNK_LEN * num:
            self.chunks.pop(key, None)
            return ["chunk %d of a %d-byte block without its earlier chunks" % (num, size)]
        buf += payload[:min(MAX_EXT_CHUNK_LEN, size - len(buf))]
        if len(buf) < size:
            return ["chunk %d: %d of %d bytes" % (num, len(buf), size)]
        del self.chunks[key]
        return (["chunks 0-%d reassembled, %d bytes" % (num, size)] if num else []) + ext_data(typ, buf, sop)
