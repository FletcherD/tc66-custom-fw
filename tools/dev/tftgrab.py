#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyserial", "pillow"]
# ///
"""Read the TFT's frame memory back over the `tftrd` dev verb (dev-tft
feature) and rebuild the screen as a PNG — a webcam-free view of what the
firmware actually drew.

  tftgrab.py id                          RDDID (04h): 3 ID bytes (sanity check)
  tftgrab.py reg <cmd> [--dummy D] [-n N] any read command, hex dump
  tftgrab.py grab out.png [opts]         full-screen RAMRD (2Eh) -> PNG
                                         (--screen N selects a UI screen first)

The panel's SDA is bidirectional and CS is tied low, so every read ends
with a panel reset + re-init on the device (~0.3 s blank, frame memory
kept). RAMRD returns 18-bit pixels (3 bytes, 6 bits MSB-aligned) after 9 dummy
clocks; the pixel stream is row-major in screen coordinates, R first. The plot screen's
rolling scroll is unrolled from the MADCTL/SSA the verb reports.
Wire format: docs/HARDWARE.md (`tftrd`), firmware: rust/src/tft.rs.
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../lib"))
from tc66lib import open_meter  # noqa: E402

W, H = 160, 80
MADCTL_ROT0 = 0x68  # screen x maps to a rising GRAM line (tft.rs SCROLL_RISING)
STAGES = {1: "SPI TBE timeout", 2: "SPI shift-idle timeout", 3: "DMA timeout",
          4: "DMA error", 5: "engine busy", 6: "pump budget", 0x0F: "bad args"}


def tftrd(s, cmd, dummy, n, win=None):
    """-> (madctl, ssa, data). Raises on ER or a short/failed reply."""
    pkt = b"tftrd" + struct.pack("<BBH", cmd, dummy, n)
    if win:
        pkt += bytes(win)
    s.reset_input_buffer()
    s.write(pkt)
    head = s.read(4)
    if head[:2] == b"ER":
        st = head[2] if len(head) > 2 else -1
        raise IOError("tftrd ER stage %d (%s)" % (st, STAGES.get(st, "?")))
    if head[:2] != b"OK" or len(head) != 4:
        raise IOError("tftrd bad header: %r" % head)
    data = s.read(n + 1)
    if len(data) != n + 1:
        raise IOError("tftrd short reply: %d of %d bytes" % (len(data), n + 1))
    if data[-1] != 0:
        st = data[-1]
        raise IOError("panel re-init after read failed: stage %d (%s)"
                      % (st, STAGES.get(st, "?")))
    return head[2], head[3], data[:-1]


def unscroll(px, madctl, ssa):
    """Map the written-coordinate image to what the panel shows under the
    rolling scroll (tft.rs: SSA = 1 + k shows GRAM line 1 + (p + k) mod 160
    on panel line p; screen x rises with GRAM line only for MADCTL 0x68)."""
    if ssa == 0:
        return px
    k = ssa - 1
    out = [None] * (W * H)
    for x in range(W):
        if madctl == MADCTL_ROT0:
            src = (x + k) % W
        else:
            src = W - 1 - ((W - 1 - x + k) % W)
        for y in range(H):
            out[y * W + x] = px[y * W + src]
    return out


def decode(data):
    """18-bit readback -> RGB888 of the RGB565 the firmware wrote: R/B keep
    their top 5 bits (the panel's 565->666 fill bit is not ours), G 6."""
    px = []
    for i in range(0, W * H * 3, 3):
        r, g, b = data[i] >> 3, data[i + 1] >> 2, data[i + 2] >> 3
        px.append((r << 3 | r >> 2, g << 2 | g >> 4, b << 3 | b >> 2))
    return px


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="op", required=True)
    sub.add_parser("id", help="RDDID 04h (1 dummy bit, 3 bytes)")
    rp = sub.add_parser("reg", help="raw read command",
                        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    rp.add_argument("cmd", type=lambda v: int(v, 0))
    rp.add_argument("--dummy", type=int, default=0, help="dummy clocks after the command")
    rp.add_argument("-n", type=int, default=1, help="bytes to read")
    gp = sub.add_parser("grab", help="full-screen RAMRD -> PNG",
                        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    gp.add_argument("out", help="output PNG")
    gp.add_argument("--dummy", type=int, default=9,
                    help="dummy clocks between RAMRD and the first pixel (9 bench-verified)")
    gp.add_argument("--scale", type=int, default=4, help="nearest-neighbour upscale")
    gp.add_argument("--no-unscroll", action="store_true",
                    help="keep frame-memory order (ignore the reported scroll)")
    gp.add_argument("--raw", help="also save the raw 18-bit dump here")
    gp.add_argument("--screen", type=int,
                    help="select UI screen N (uiscr) before grabbing; default: leave as is")
    gp.add_argument("--wait", type=float, default=0.5,
                    help="seconds to let the selected screen draw before grabbing")
    a = ap.parse_args()

    s = open_meter()
    s.timeout = 6
    if a.op == "id":
        _, _, d = tftrd(s, 0x04, 1, 3)
        print("RDDID:", d.hex(" "))
    elif a.op == "reg":
        _, _, d = tftrd(s, a.cmd, a.dummy, a.n)
        print("%02Xh:" % a.cmd, d.hex(" "))
    else:
        from PIL import Image
        if a.screen is not None:
            s.reset_input_buffer()
            s.write(b"uiscr" + bytes([a.screen]))
            r = s.read(3)
            if r[:2] != b"OK":
                sys.exit("uiscr %d failed: %r" % (a.screen, r))
            time.sleep(a.wait)
        madctl, ssa, d = tftrd(s, 0x2E, a.dummy, W * H * 3, (0, 0, W - 1, H - 1))
        print("MADCTL %#04x  SSA %d%s" % (madctl, ssa, "" if ssa else " (no scroll)"))
        if a.raw:
            with open(a.raw, "wb") as f:
                f.write(d)
        px = decode(d)
        if not a.no_unscroll:
            px = unscroll(px, madctl, ssa)
        img = Image.new("RGB", (W, H))
        img.putdata(px)
        if a.scale > 1:
            img = img.resize((W * a.scale, H * a.scale), Image.NEAREST)
        img.save(a.out)
        print("wrote", a.out)


if __name__ == "__main__":
    main()
