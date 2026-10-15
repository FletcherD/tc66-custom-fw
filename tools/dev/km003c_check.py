#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyusb", "pyserial"]
# ///
"""Bench check of the meter's KM003C emulation (rust/src/powerz.rs) over its
vendor interface: Connect, GetData ADC (every field decoded), ADC + PdPacket
chained (0x0011), PD state trace (0x0020), Enable-/DisablePdMonitor (Enable
must start the sniffer — the PD engine is switched off over CDC first),
Disconnect. Prints PASS/FAIL per step; re-attaches the kernel powerz
driver at the end. Needs the udev rule tools/udev/70-tc66.rules.
"""
import os, struct, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/../lib")
import usb.core  # noqa: E402
import usb.util  # noqa: E402
from km003c import KM_VID, KM_PID, KM_EP_OUT, KM_EP_IN, KM_PUT_DATA  # noqa: E402
from tc66lib import open_meter  # noqa: E402

CONNECT, DISCONNECT, ACCEPT, REJECT, GET_DATA = 0x02, 0x03, 0x05, 0x06, 0x0C
ENABLE_PD_MON, DISABLE_PD_MON = 0x10, 0x11
ATT_ADC, ATT_PD, ATT_PD_TRACE = 0x0001, 0x0010, 0x0020
ST_SNIFF = 7  # pdsta state (rust/src/pdsta.rs)

fails = 0


def check(ok, what):
    global fails
    print("%s  %s" % ("PASS" if ok else "FAIL", what))
    fails += 0 if ok else 1


class Km:
    def __init__(self):
        self.dev = usb.core.find(idVendor=KM_VID, idProduct=KM_PID)
        if self.dev is None:
            sys.exit("KM003C (%04X:%04X) not found" % (KM_VID, KM_PID))
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
        usb.util.claim_interface(self.dev, 0)
        self.tid = 0
        self.drain()

    def drain(self):
        # the firmware keeps an ADC reply armed at all times, so reads never
        # run dry — just flush a few stale packets
        for _ in range(4):
            try:
                self.dev.read(KM_EP_IN, 512, timeout=50)
            except usb.core.USBTimeoutError:
                return

    def xfer(self, cmd, attr=0, any_tid=False):
        """Send one command; return the reply carrying our tid (the
        pre-armed ADC packet, tid 0, may arrive first and is skipped) —
        or, with any_tid, the first reply (plain ADC: the pre-armed packet
        always carries tid 0)."""
        self.tid = self.tid % 255 + 1
        self.dev.write(KM_EP_OUT, bytes([cmd, self.tid]) + (attr << 1).to_bytes(2, "little"))
        for _ in range(4):
            try:
                r = bytes(self.dev.read(KM_EP_IN, 512, timeout=500))
            except usb.core.USBTimeoutError:
                return None
            if len(r) >= 2 and (any_tid or r[1] == self.tid):
                return r
        return None

    def close(self):
        usb.util.release_interface(self.dev, 0)
        try:
            self.dev.attach_kernel_driver(0)
        except usb.core.USBError:
            pass


def logical(r):
    """[(attr, next, payload)] of a PutData reply, walking the ext headers."""
    out, i = [], 4
    while i + 4 <= len(r):
        ext = struct.unpack_from("<I", r, i)[0]
        att, nxt, size = ext & 0x7FFF, ext >> 15 & 1, ext >> 22 & 0x3FF
        out.append((att, nxt, r[i + 4:i + 4 + size]))
        i += 4 + size
        if not nxt:
            break
    return out


def show_adc(p):
    (v, i, va, ia, vo, io, t, c1, c2, dp, dm, vdd, rate, flags, c2a, dpa, dma) = \
        struct.unpack("<6ih5H2B3H", p)
    print("      VBUS %.4f V  IBUS %+.5f A  avg %.4f V %+.5f A  ori %.4f V %+.5f A"
          % (v / 1e6, i / 1e6, va / 1e6, ia / 1e6, vo / 1e6, io / 1e6))
    print("      temp %.3f C  CC1 %.1f mV  CC2 %.1f mV  D+ %.1f mV  D- %.1f mV  Vdd %.1f mV"
          % (t / 128, c1 / 10, c2 / 10, dp / 10, dm / 10, vdd / 10))
    print("      [36] rate %d  [37] flags 0x%02x  CC2avg %d mV  D+avg %d mV  D-avg %d mV"
          % (rate, flags, c2a, dpa, dma))
    return t, dp, dm, dpa, dma


def pdctl(n):
    s = open_meter()
    s.write(b"pdctl" + bytes([n]))
    s.read(2)
    s.close()


def pd_state():
    s = open_meter()
    s.write(b"pdsta")
    r = s.read(24)
    s.close()
    return r[2] if r[:2] == b"OK" else None


def main():
    km = Km()
    try:
        r = km.xfer(CONNECT)
        check(r is not None and r[0] == ACCEPT, "Connect -> Accept")

        r = km.xfer(GET_DATA, ATT_ADC, any_tid=True)
        lp = logical(r) if r and r[0] == KM_PUT_DATA else []
        check(len(lp) == 1 and lp[0][0] == ATT_ADC and len(lp[0][2]) == 44, "GetData ADC -> 44-B ADC")
        if lp:
            t, dp, dm, dpa, dma = show_adc(lp[0][2])
            check(abs(dpa - dp / 10) <= 1 and abs(dma - dm / 10) <= 1, "D± avg fields match D± (mV)")
            check(t % 128 != 0 or t == 0, "temperature has sub-degree resolution (may rarely land on a whole degree)")

        r = km.xfer(GET_DATA, ATT_ADC | ATT_PD)
        lp = logical(r) if r and r[0] == KM_PUT_DATA else []
        print("      0x0011 reply %d B: %s" % (len(r or b""), [(hex(a), n, len(p)) for a, n, p in lp]))
        check(len(lp) == 2 and lp[0][:2] == (ATT_ADC, 1) and len(lp[0][2]) == 44
              and lp[1][:2] == (ATT_PD, 0) and len(lp[1][2]) >= 12,
              "GetData ADC+PD -> chained ADC (next=1) + PdPacket")

        r = km.xfer(GET_DATA, ATT_PD_TRACE)
        lp = logical(r) if r and r[0] == KM_PUT_DATA else []
        check(len(lp) == 1 and lp[0][0] == ATT_PD_TRACE and lp[0][2] == b"\0\0",
              "GetData PD trace -> two empty queues")

        pdctl(0)
        time.sleep(0.3)
        check(pd_state() == 0, "pdctl 0 -> PD engine off")
        r = km.xfer(ENABLE_PD_MON, 1)
        check(r is not None and r[0] == ACCEPT, "EnablePdMonitor -> Accept")
        time.sleep(0.5)
        check(pd_state() == ST_SNIFF, "EnablePdMonitor started the sniffer")
        r = km.xfer(DISABLE_PD_MON)
        check(r is not None and r[0] == ACCEPT, "DisablePdMonitor -> Accept")
        check(pd_state() == ST_SNIFF, "DisablePdMonitor leaves the sniffer running")

        r = km.xfer(0x0E, 0)  # StartGraph: not emulated
        check(r is not None and r[0] == REJECT, "StartGraph -> Reject (not emulated)")
        r = km.xfer(DISCONNECT)
        check(r is not None and r[0] == ACCEPT, "Disconnect -> Accept")
    finally:
        km.close()
    print("%d failure(s)" % fails)
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
