"""USB-PD data-object decoders for the capture tools (PD 2.0/3.x layouts):
PDOs (Source/Sink_Capabilities), Request RDOs, structured VDMs and the
Discover Identity response (ID Header, Cert Stat, Product and cable VDOs).
Layouts per the USB PD 3.1 spec, tables 6-x (PDO/RDO) and 6-2x (VDOs).
Vendor/product names come from the system usb.ids (PD uses the USB-IF vendor
IDs); a cable's PD product ID is vendor-assigned and usually not listed.
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
VDM_CMDS = {1: "Discover Identity", 2: "Discover SVIDs", 3: "Discover Modes",
            4: "Enter Mode", 5: "Exit Mode", 6: "Attention"}
VDM_TYPES = {0: "REQ", 1: "ACK", 2: "NAK", 3: "BUSY"}


def pdo(p):
    """One power data object -> text."""
    kind = p >> 30
    if kind == 0:
        return "fixed %.2f V %.2f A" % (((p >> 10) & 0x3FF) * 0.05, (p & 0x3FF) * 0.01)
    if kind == 1:
        return "variable %.2f-%.2f V %.2f A" % (((p >> 10) & 0x3FF) * 0.05,
                                                ((p >> 20) & 0x3FF) * 0.05, (p & 0x3FF) * 0.01)
    if kind == 2:
        return "battery %.2f-%.2f V %.2f W" % (((p >> 10) & 0x3FF) * 0.05,
                                               ((p >> 20) & 0x3FF) * 0.05, (p & 0x3FF) * 0.25)
    apdo = (p >> 28) & 3
    if apdo == 0:
        return "PPS %.1f-%.1f V %.2f A" % (((p >> 8) & 0xFF) * 0.1, ((p >> 17) & 0xFF) * 0.1,
                                          (p & 0x7F) * 0.05)
    if apdo == 1:
        return "EPR AVS %.1f-%.1f V %d W" % (((p >> 8) & 0xFF) * 0.1, ((p >> 17) & 0x1FF) * 0.1,
                                            p & 0xFF)
    return "APDO type %d (0x%08x)" % (apdo, p)


def rdo(r, caps):
    """Request data object -> text; `caps` = the last Source_Cap PDOs (for PPS)."""
    pos = (r >> 28) & 0xF
    src = caps[pos - 1] if 0 < pos <= len(caps) else None
    if src is not None and src >> 30 == 3:
        return "PDO %d (PPS): %.2f V %.2f A" % (pos, ((r >> 9) & 0xFFF) * 0.02, (r & 0x7F) * 0.05)
    what = "PDO %d" % pos + (" (%s)" % pdo(src) if src is not None else "")
    return "%s: operating %.2f A, max %.2f A%s" % (
        what, ((r >> 10) & 0x3FF) * 0.01, (r & 0x3FF) * 0.01,
        ", capability mismatch" if r & (1 << 26) else "")


def vendor(vid):
    v = _load_ids().get(vid)
    return "0x%04X%s" % (vid, " (%s)" % v[0] if v else "")


def product(vid, pid):
    v = _load_ids().get(vid)
    name = v[1].get(pid) if v else None
    return "0x%04X%s" % (pid, " (%s)" % name if name else "")


def cable_vdo(v, rev3):
    cur = {1: "3 A", 2: "5 A"}.get((v >> 5) & 3, "?")
    speed = {0: "USB 2.0 only", 1: "USB 3.2 Gen1", 2: "USB 3.2 Gen2" if rev3 else "USB 3.1 Gen1+2",
             3: "USB4 Gen3", 4: "USB4 Gen4"}.get(v & 7, "speed %d" % (v & 7))
    plug = {0: "Type-A", 1: "Type-B", 2: "Type-C", 3: "captive"}.get((v >> 18) & 3)
    s = "cable: %s VBUS, %s, to %s, HW %d FW %d" % (cur, speed, plug, v >> 28, (v >> 24) & 0xF)
    if rev3:
        s += ", max %s" % {0: "20 V", 1: "30 V", 2: "40 V", 3: "50 V"}[(v >> 9) & 3]
    return s


def vdm(objs, sop, rev3):
    """Vendor_Defined message objects -> list of text lines."""
    h = objs[0]
    svid = h >> 16
    if not h & (1 << 15):
        return ["unstructured VDM, SVID %s" % vendor(svid)]
    cmd, ctype = h & 0x1F, (h >> 6) & 3
    lines = ["VDM %s %s, SVID 0x%04X" % (VDM_CMDS.get(cmd, "cmd %d" % cmd),
                                        VDM_TYPES[ctype], svid)]
    if cmd == 1 and ctype == 1 and len(objs) >= 4:
        idh, cert, prod = objs[1], objs[2], objs[3]
        if sop:
            ptype = {3: "passive cable", 4: "active cable", 6: "VPD"}.get((idh >> 27) & 7, "?")
        else:
            ptype = {1: "hub", 2: "peripheral", 3: "PSD", 5: "AMA"}.get((idh >> 27) & 7,
                                                                    "UFP type %d" % ((idh >> 27) & 7))
        lines.append("identity: %s, vendor %s, product %s rev %04X, XID %d%s%s" % (
            ptype, vendor(idh & 0xFFFF), product(idh & 0xFFFF, prod >> 16), prod & 0xFFFF, cert,
            ", USB host" if idh & (1 << 31) else "", ", USB device" if idh & (1 << 30) else ""))
        if sop and (idh >> 27) & 7 in (3, 4) and len(objs) >= 5:
            lines.append(cable_vdo(objs[4], rev3))
    return lines
