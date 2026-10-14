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
from pd_spec import F, get, pack, u16, u32

# KM003C protocol (github.com/okhsunrog/km003c-protocol-research; firmware
# side rust/src/powerz.rs + the capture ring format in rust/src/pd.rs)
KM_VID, KM_PID = 0x5FC9, 0x0063
KM_EP_OUT, KM_EP_IN = 0x01, 0x81
KM_CMD_GET_DATA = 0x0C          # request: cmd, tid, attribute << 1 (LE16)
KM_REQ_ATTRIBUTE = F(1, 15)
KM_PUT_DATA = 0x41              # reply: 4-byte header, 4-byte extended header, payload
KM_HDR_LEN = 8
KM_EXT_HDR = 4
KM_EXT_ATTRIBUTE = F(0, 15)
KM_EXT_SIZE = F(22, 10)
KM_ATT_PD = 0x0010              # PdPacket
KM_REPLY_MAX = 64
# PdPacket payload: measurement preamble, then events
PRE_TS, PRE_VBUS_MV, PRE_IBUS_MA, PRE_CC1_MV, PRE_CC2_MV, PRE_LEN = 0, 4, 6, 8, 10, 12
EV_CONNECTION = 0x45            # 6 B: 0x45, ts LE24, 0x00, code
EV_CONN_TS, EV_CONN_TS_LEN, EV_CONN_CODE, EV_CONN_LEN = 1, 3, 5, 6
EV_WRAPPED = F(7, 1)            # 6 + N B: size flag, ts LE32, sop, N wire bytes
EV_WRAPPED_SIZE = F(0, 6)       # N + 5
EV_WRAPPED_TS, EV_WRAPPED_SOP, EV_WRAPPED_WIRE = 1, 5, 6
EV_WRAPPED_SIZE_BIAS = 5
EV_CONNECT, EV_DISCONNECT = 0x21, 0x22
EV_RX_FULL, EV_RX_RESYNC, EV_BAD_CRC, EV_HARD_RESET = 0x30, 0x31, 0x32, 0x33  # ours (pd.rs CAP_*)
EVENTS = {EV_CONNECT: "*** CONNECT ***", EV_DISCONNECT: "*** DISCONNECT ***",
          EV_RX_FULL: "!!! RX FIFO full: frames may be lost !!!",
          EV_RX_RESYNC: "!!! RX FIFO resync: queued frames flushed !!!",
          EV_BAD_CRC: "!!! bad-CRC packet (not captured) !!!",
          EV_HARD_RESET: "*** HARD RESET ***"}
SOP_NAMES = {0: "", 1: "SOP' ", 2: "SOP'' ", 3: "SOP'_DBG ", 4: "SOP''_DBG "}  # sop byte


def parse_pdpacket(payload, dec):
    """Print the events/messages of one PdPacket payload; returns
    (vbus, ibus, cc1, cc2) from its preamble, or None if too short."""
    if len(payload) < PRE_LEN:
        return None
    ibus = int.from_bytes(payload[PRE_IBUS_MA:PRE_IBUS_MA + 2], "little", signed=True)
    vbus, cc1, cc2 = u16(payload, PRE_VBUS_MV), u16(payload, PRE_CC1_MV), u16(payload, PRE_CC2_MV)
    i = PRE_LEN
    while i < len(payload):
        b0 = payload[i]
        if b0 == EV_CONNECTION:
            if i + EV_CONN_LEN > len(payload):
                break
            code = payload[i + EV_CONN_CODE]
            ets = int.from_bytes(payload[i + EV_CONN_TS:i + EV_CONN_TS + EV_CONN_TS_LEN], "little")
            print("  [%8d ms] %s" % (ets, EVENTS.get(code, "event %02x" % code)))
            i += EV_CONN_LEN
        elif get(b0, EV_WRAPPED):
            wire_len = get(b0, EV_WRAPPED_SIZE) - EV_WRAPPED_SIZE_BIAS
            if wire_len < 0 or i + EV_WRAPPED_WIRE + wire_len > len(payload):
                break
            ets = u32(payload, i + EV_WRAPPED_TS)
            wire = payload[i + EV_WRAPPED_WIRE:i + EV_WRAPPED_WIRE + wire_len]
            sop = payload[i + EV_WRAPPED_SOP]
            line, details = dec.frame(wire, sop)
            print("  [%8d ms] %s%s" % (ets, SOP_NAMES.get(sop, "sop%d " % sop), line))
            for d in details:
                print("               %s" % d)
            i += EV_WRAPPED_WIRE + wire_len
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

    dev = usb.core.find(idVendor=KM_VID, idProduct=KM_PID)
    if dev is None:
        sys.exit("KM003C (%04X:%04X) not found" % (KM_VID, KM_PID))
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (usb.core.USBError, NotImplementedError):
        pass

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
        except usb.core.USBError:
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
