#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyusb"]
# ///
"""KM003C PD-capture reader: polls the vendor bulk endpoint for the PdPacket
attribute (0x0010) and decodes captured USB-PD traffic (tools/lib/pd_decode.py:
every control/data/extended message type, PDOs + flags, RDOs, VDMs incl.
identities, Source/Sink_Capabilities_Extended, Status, PPS_Status,
Manufacturer_Info, ...; chunked extended messages are reassembled).

  pdcap.py [seconds]           stream captured PD events for `seconds`. Start
                               the engine first over CDC: `pd.py arm` (our own
                               sink's traffic) or `pd.py sniff` (a source and a
                               sink negotiating through the meter).
  pdcap.py 60 --save cap.hex   also write every raw PdPacket payload (hex, one
                               per line) for later re-decoding.
  pdcap.py --replay cap.hex    decode a saved capture offline (no device).

Opening the USB device node needs the udev rule tools/udev/70-tc66.rules
(install instructions in it); without it only root can.

Talks the POWER-Z KM003C protocol directly over libusb (VID:PID 5FC9:0063,
bulk 0x01 OUT / 0x81 IN), independent of the CDC verb channel. The firmware
may answer a GetData(PD) with the pre-armed ADC packet first; this filters
replies by the ext-header attribute and keeps only PdPacket (0x0010).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))
from km003c import *  # noqa: E402,F403 — KM003C framing + parse_pdpacket
from pd_decode import Decoder  # noqa: E402
from pd_spec import get, pack, u32  # noqa: E402


def replay(path):
    dec = Decoder()
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                parse_pdpacket(bytes.fromhex(line), dec)


def capture(secs, save):
    import errno
    import usb.core
    import usb.util

    dev = usb.core.find(idVendor=KM_VID, idProduct=KM_PID)
    if dev is None:
        sys.exit("KM003C (%04X:%04X) not found" % (KM_VID, KM_PID))
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
        usb.util.claim_interface(dev, 0)
    except usb.core.USBError as e:
        if e.errno == errno.EACCES:
            sys.exit("No permission to open the meter's USB device (bus %03d device %03d).\n"
                     "Install the udev rule (steps in the file): %s"
                     % (dev.bus, dev.address,
                        os.path.join(os.path.dirname(os.path.abspath(__file__)), "udev", "70-tc66.rules")))
        raise

    tid = [0]

    def getdata(attr):
        tid[0] = (tid[0] + 1) & 0xFF
        dev.write(KM_EP_OUT, bytes([KM_CMD_GET_DATA, tid[0]]) +
                  pack((KM_REQ_ATTRIBUTE, attr)).to_bytes(2, "little"))
        return bytes(dev.read(KM_EP_IN, KM_REPLY_MAX, timeout=500))

    out = open(save, "w") if save else None
    dec = Decoder()
    print("Polling PdPacket (0x0010) for %.0f s (start the engine with pd.py arm / sniff)" % secs)
    end = time.time() + secs
    seen_vbus = None
    while time.time() < end:
        try:
            r = getdata(KM_ATT_PD)
        except usb.core.USBTimeoutError:
            continue
        if len(r) < KM_HDR_LEN or r[0] != KM_PUT_DATA:
            time.sleep(0.05)
            continue
        ext = u32(r, KM_EXT_HDR)
        size = get(ext, KM_EXT_SIZE)
        if get(ext, KM_EXT_ATTRIBUTE) != KM_ATT_PD:  # ADC reply arrived first — skip, poll again
            continue
        payload = r[KM_HDR_LEN:KM_HDR_LEN + size]
        if out and size > PRE_LEN:
            out.write(payload.hex() + "\n")
        info = parse_pdpacket(payload, dec)
        if info:
            seen_vbus = info[0]
        if size <= PRE_LEN:  # preamble only: ring empty, idle a little
            time.sleep(0.02)  # else poll straight away — the 8-slot ring overflows in a burst
    if out:
        out.close()
    if seen_vbus is not None:
        print("last: VBUS=%d mV" % seen_vbus)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seconds", nargs="?", type=float, default=20,
                    help="capture duration (default: %(default)s)")
    ap.add_argument("--save", metavar="FILE", help="write raw PdPacket payloads (hex) to FILE (default: none)")
    ap.add_argument("--replay", metavar="FILE", help="decode a --save file instead of capturing (default: none)")
    a = ap.parse_args()
    if a.replay:
        replay(a.replay)
    else:
        capture(a.seconds, a.save)


if __name__ == "__main__":
    main()
