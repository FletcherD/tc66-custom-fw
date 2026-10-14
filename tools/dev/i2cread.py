#!/usr/bin/env python3
"""Host side of the firmware's `i2crd` dev verb (M1 bring-up).

  i2cread.py <addr7> <rlen> [wbyte ...]    one transaction (up to 2 write bytes)
  i2cread.py --scan                        probe addresses 0x08..0x77 (0-byte read... uses 1-byte read)

Examples:
  i2cread.py 0x4a 2 0x02        # read 2 bytes from monitor reg/channel 2
  i2cread.py 0x50 4 0x20 0x04   # EEPROM: 2-byte address 0x2004, read 4
"""
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0] + "/../lib")
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

STAGES = {
    1: "bus busy", 2: "SBSEND(w)", 3: "ADDSEND(w) (address NACK?)", 4: "TBE",
    5: "BTC(w)", 6: "SBSEND(r)", 7: "ADDSEND(r) (address NACK?)", 8: "RBNE",
    9: "BTC(r)", 0x0F: "bad args",
}


def open_meter():
    ports = meter_candidates()
    if not ports:
        sys.exit("no meter found")
    s = serial.Serial(ports[0], 115200, timeout=1.5)
    s.reset_input_buffer()
    return s


def i2crd(s, addr7, wbytes, rlen):
    w = (list(wbytes) + [0, 0])[:2]
    pkt = b"i2crd" + bytes([addr7, len(wbytes), w[0], w[1], rlen])
    s.reset_input_buffer()
    s.write(pkt)
    r = s.read(2 + max(rlen, 4))
    if r[:2] == b"OK":
        return r[2 : 2 + rlen]
    if r[:2] == b"ER":
        stage, err = r[2] & 0x7F, r[2] & 0x80
        raise IOError(
            "stage %d (%s)%s stat0=%04x stat1=%02x"
            % (
                stage,
                STAGES.get(stage, "?"),
                " [bus error/NACK]" if err else " [timeout]",
                r[3] | (r[4] << 8) if len(r) >= 5 else -1,
                r[5] if len(r) >= 6 else -1,
            )
        )
    raise IOError("garbled reply: %r" % r)


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == "--scan":
        s = open_meter()
        found = []
        for a in range(0x08, 0x78):
            try:
                i2crd(s, a, [], 1)
                found.append(a)
                print("  0x%02x ACK" % a)
            except IOError:
                pass
        print("found:", ", ".join("0x%02x" % a for a in found) or "none")
        return
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    addr7 = int(sys.argv[1], 0)
    rlen = int(sys.argv[2], 0)
    wbytes = [int(x, 0) for x in sys.argv[3:]]
    s = open_meter()
    data = i2crd(s, addr7, wbytes, rlen)
    print("read %d B from 0x%02x:" % (len(data), addr7), data.hex(" "))
    if len(data) == 2:
        v = (data[0] << 8) | data[1]
        print("  as u16 BE: %d (0x%04x)   as u16 LE: %d" % (v, v, data[0] | (data[1] << 8)))


if __name__ == "__main__":
    main()
