#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyusb"]
# ///
"""KM003C PD-capture reader: polls the vendor bulk endpoint for the PdPacket
attribute (0x0010) and decodes captured USB-PD traffic (tools/pd_decode.py:
every control/data/extended message type, PDOs + flags, RDOs, VDMs incl.
identities, Source/Sink_Capabilities_Extended, Status, PPS_Status,
Manufacturer_Info, ...; chunked extended messages are reassembled).

  pdcap.py [seconds]           stream captured PD events for `seconds`. Start
                               the engine first over CDC: `pd.py arm` (our own
                               sink's traffic) or `pd.py sniff` (a source and a
                               sink negotiating through the meter).
  pdcap.py 60 --save cap.hex   also write every raw PdPacket payload (hex, one
                               per line) for later re-decoding.
  pdcap.py --replay cap.hex    decode a saved capture offline (no device, no root).

Needs root for the USB device node; run the uv environment's Python under sudo:
  sudo $(uv run --script tools/pdcap.py --python-path) tools/pdcap.py 30

Talks the POWER-Z KM003C protocol directly over libusb (VID:PID 5FC9:0063,
bulk 0x01 OUT / 0x81 IN), independent of the CDC verb channel. The firmware
may answer a GetData(PD) with the pre-armed ADC packet first; this filters
replies by the ext-header attribute and keeps only PdPacket (0x0010).
"""
import argparse
import os
import sys
import time

from pd_decode import Decoder

EVENTS = {0x21: "*** CONNECT ***", 0x22: "*** DISCONNECT ***",
          0x30: "!!! RX FIFO full: frames may be lost !!!",
          0x31: "!!! RX FIFO resync: queued frames flushed !!!",
          0x32: "!!! bad-CRC packet (not captured) !!!",
          0x33: "*** HARD RESET ***"}


def parse_pdpacket(payload, dec):
    """Print the events/messages of one PdPacket payload; returns
    (vbus, ibus, cc1, cc2) from its preamble, or None if too short."""
    if len(payload) < 12:
        return None
    vbus, ibus, cc1, cc2 = (
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
            print("  [%8d ms] %s" % (ets, EVENTS.get(code, "event %02x" % code)))
            i += 6
        elif b0 & 0x80:  # wrapped PD message
            wire_len = (b0 & 0x3F) - 5
            if wire_len < 0 or i + 6 + wire_len > len(payload):
                break
            ets = int.from_bytes(payload[i + 1:i + 5], "little")
            wire = payload[i + 6:i + 6 + wire_len]
            sop = payload[i + 5]
            line, details = dec.frame(wire, sop)
            print("  [%8d ms] %s%s" % (ets, {0: "", 1: "SOP' ", 2: "SOP'' ", 3: "SOP'_DBG ", 4: "SOP''_DBG "}.get(sop, "sop%d " % sop), line))
            for d in details:
                print("               %s" % d)
            i += 6 + wire_len
        else:
            break
    return vbus, ibus, cc1, cc2


def replay(path):
    dec = Decoder()
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                parse_pdpacket(bytes.fromhex(line), dec)


def capture(secs, save):
    import usb.core

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

    out = open(save, "w") if save else None
    dec = Decoder()
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
        payload = r[8:8 + size]
        if out and size > 12:
            out.write(payload.hex() + "\n")
        info = parse_pdpacket(payload, dec)
        if info:
            seen_vbus = info[0]
        if size <= 12:  # preamble only: ring empty, idle a little
            time.sleep(0.02)  # else poll straight away — the 8-slot ring overflows in a burst
    if out:
        out.close()
        if os.environ.get("SUDO_UID"):  # hand the file back to the invoking user
            os.chown(save, int(os.environ["SUDO_UID"]), int(os.environ["SUDO_GID"]))
    if seen_vbus is not None:
        print("last: VBUS=%d mV" % seen_vbus)


def main():
    if sys.argv[1:] == ["--python-path"]:
        print(sys.executable)
        return
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
