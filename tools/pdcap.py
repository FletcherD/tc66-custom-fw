#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyusb"]
# ///
"""KM003C PD-capture reader: polls the vendor bulk endpoint for the PdPacket
attribute (0x0010) and decodes captured USB-PD traffic.

  pdcap.py [seconds]   stream captured PD events for `seconds` (default 20).
                       Start the engine first over CDC: `pd.py arm` (our own
                       sink's traffic) or `pd.py sniff` (a source and a sink
                       negotiating through the meter).

Needs root for the USB device node; run the uv environment's Python under sudo:
  sudo $(uv run --script tools/pdcap.py --python-path) tools/pdcap.py 30

Talks the POWER-Z KM003C protocol directly over libusb (VID:PID 5FC9:0063,
bulk 0x01 OUT / 0x81 IN), independent of the CDC verb channel. The firmware
may answer a GetData(PD) with the pre-armed ADC packet first; this filters
replies by the ext-header attribute and keeps only PdPacket (0x0010).
"""
import sys
import time

import usb.core
import usb.util

CTL = {1: "GoodCRC", 3: "Accept", 4: "Reject", 5: "Ping", 6: "PS_RDY",
       7: "Get_Source_Cap", 8: "Get_Sink_Cap", 12: "Wait", 13: "Soft_Reset",
       16: "Not_Supported"}
DAT = {1: "Source_Cap", 2: "Request", 3: "BIST", 4: "Sink_Cap", 15: "Vendor"}


def decode_pd(wire):
    if len(wire) < 2:
        return "runt"
    hdr = wire[0] | (wire[1] << 8)
    nobj = (hdr >> 12) & 7
    typ = hdr & 0x1F
    role = "SRC" if hdr & (1 << 8) else "SNK"
    rev = {0: "1.0", 1: "2.0", 2: "3.0"}.get((hdr >> 6) & 3, "?")
    name = (DAT if nobj else CTL).get(typ & 0x0F if nobj else typ, "type%d" % typ)
    objs = ["%08x" % (int.from_bytes(wire[2 + 4 * i:6 + 4 * i], "little")) for i in range(nobj)]
    return "%-14s [%s r%s id%d]%s" % (name, role, rev, (hdr >> 9) & 7,
                                      " " + " ".join(objs) if objs else "")


def parse_pdpacket(payload):
    if len(payload) < 12:
        return
    ts, vbus, ibus, cc1, cc2 = (
        int.from_bytes(payload[0:4], "little"),
        int.from_bytes(payload[4:6], "little"),
        int.from_bytes(payload[6:8], "little", signed=True),
        int.from_bytes(payload[8:10], "little"),
        int.from_bytes(payload[10:12], "little"),
    )
    i = 12
    while i < len(payload):
        b0 = payload[i]
        if b0 == 0x45:  # connection event
            if i + 6 > len(payload):
                break
            code = payload[i + 5]
            ets = payload[i + 1] | (payload[i + 2] << 8) | (payload[i + 3] << 16)
            print("  [%8d ms] %s" % (ets, {0x21: "*** CONNECT ***", 0x22: "*** DISCONNECT ***"}.get(code, "event %02x" % code)))
            i += 6
        elif b0 & 0x80:  # wrapped PD message
            wire_len = (b0 & 0x3F) - 5
            if wire_len < 0 or i + 6 + wire_len > len(payload):
                break
            ets = int.from_bytes(payload[i + 1:i + 5], "little")
            wire = payload[i + 6:i + 6 + wire_len]
            print("  [%8d ms] %s" % (ets, decode_pd(wire)))
            i += 6 + wire_len
        else:
            break
    return vbus, ibus, cc1, cc2


def main():
    if sys.argv[1:] == ["--python-path"]:
        print(sys.executable)
        return
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20
    dev = usb.core.find(idVendor=0x5FC9, idProduct=0x0063)
    if dev is None:
        sys.exit("KM003C (5FC9:0063) not found")
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (usb.core.USBError, NotImplementedError):
        pass

    tid = [0]

    def getdata(attr):
        tid[0] = (tid[0] + 1) & 0xFF
        dev.write(0x01, bytes([0x0C, tid[0], (attr << 1) & 0xFF, (attr << 1) >> 8]))
        return bytes(dev.read(0x81, 64, timeout=500))

    # arm the sink over the vendor Connect (pd.rs arms via the CDC pdctl verb;
    # here we just start polling — arm separately with tools/pd.py if needed)
    print("Polling PdPacket (0x0010) for %.0f s (start the engine with pd.py arm / sniff)" % secs)
    end = time.time() + secs
    seen_vbus = None
    while time.time() < end:
        try:
            r = getdata(0x0010)
        except usb.core.USBError:
            continue
        if len(r) < 8 or r[0] != 0x41:
            time.sleep(0.05)
            continue
        ext = int.from_bytes(r[4:8], "little")
        att = ext & 0x7FFF
        size = (ext >> 22) & 0x3FF
        if att != 0x0010:  # ADC reply arrived first — skip, poll again
            continue
        info = parse_pdpacket(r[8:8 + size])
        if info:
            seen_vbus = info[0]
        if size <= 12:  # preamble only: ring empty, idle a little
            time.sleep(0.02)  # else poll straight away — the 8-slot ring overflows in a burst
    if seen_vbus is not None:
        print("last: VBUS=%d mV" % seen_vbus)


if __name__ == "__main__":
    main()
