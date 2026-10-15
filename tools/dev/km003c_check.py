#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyusb", "pyserial", "pycryptodome"]
# ///
"""Bench check of the meter's KM003C emulation (rust/src/powerz.rs) over its
vendor interface: Connect, GetData ADC (every field decoded), ADC + PdPacket
chained (0x0011), PD state trace (0x0020), Enable-/DisablePdMonitor (Enable
must start the sniffer — the PD engine is switched off over CDC first),
MemoryRead of the HardwareID (+ NotReadable for anything else),
StreamingAuth, AdcQueue streaming at all four StartGraph rates (record
layout, sequence steps, sample rate, chained ADC + AdcQueue), StopGraph,
Disconnect. Prints PASS/FAIL per step; re-attaches the kernel powerz
driver at the end. Needs the udev rule tools/udev/70-tc66.rules.
"""
import binascii, os, struct, sys, time

from Crypto.Cipher import AES

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)) + "/../lib")
import usb.core  # noqa: E402
import usb.util  # noqa: E402
from km003c import KM_VID, KM_PID, KM_EP_OUT, KM_EP_IN, KM_PUT_DATA  # noqa: E402
from tc66lib import open_meter  # noqa: E402

CONNECT, DISCONNECT, ACCEPT, REJECT, GET_DATA = 0x02, 0x03, 0x05, 0x06, 0x0C
ENABLE_PD_MON, DISABLE_PD_MON = 0x10, 0x11
ATT_ADC, ATT_ADC_QUEUE, ATT_PD, ATT_PD_TRACE = 0x0001, 0x0002, 0x0010, 0x0020
START_GRAPH, STOP_GRAPH, MEMORY_READ, STREAMING_AUTH, NOT_READABLE = 0x0E, 0x0F, 0x44, 0x4C, 0x27
MEM_KEY = b"Lh2yfB7n6X7d9a5Z"
SPS = [2, 10, 50, 1000]  # StartGraph rate index -> samples/s
SEQ_STEP = [500, 100, 20, 1]
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
                r = bytes(self.dev.read(KM_EP_IN, 2048, timeout=500))
            except usb.core.USBTimeoutError:
                return None
            if len(r) >= 2 and (any_tid or r[1] == self.tid):
                return r
        return None

    def raw(self, pkt, n=1):
        """Send a raw packet; return the next n transfers (None on timeout)."""
        self.dev.write(KM_EP_OUT, pkt)
        out = []
        for _ in range(n):
            try:
                out.append(bytes(self.dev.read(KM_EP_IN, 2048, timeout=500)))
            except usb.core.USBTimeoutError:
                out.append(None)
        return out

    def next_tid(self):
        self.tid = self.tid % 255 + 1
        return self.tid

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


def memory_read(km, addr, size):
    """MemoryRead request -> (confirmation, data) transfers."""
    head = struct.pack("<III", addr, size, 0xFFFFFFFF)
    plain = head + struct.pack("<I", binascii.crc32(head)) + b"\xff" * 16
    tid = km.next_tid()
    pkt = bytes([MEMORY_READ, tid, 1, 1]) + AES.new(MEM_KEY, AES.MODE_ECB).encrypt(plain)
    return tid, km.raw(pkt, 2)


def aq_records(r):
    """AdcQueue records [(seq, uV, uA, cc1, cc2, dp, dm)] + the chained ADC
    payload (or None) from a PutData reply; raises on a malformed one."""
    i, adc = 4, None
    if len(r) == 4:
        return [], None  # the empty reply
    while True:
        ext = struct.unpack_from("<I", r, i)[0]
        att, nxt, chunk, size = ext & 0x7FFF, ext >> 15 & 1, ext >> 16 & 0x3F, ext >> 22
        if att == ATT_ADC:
            adc = r[i + 4:i + 4 + size]
            i += 4 + size
            if not nxt:
                return [], adc
            continue
        if att != ATT_ADC_QUEUE or size != 20 or nxt or len(r) != i + 4 + 20 * chunk:
            raise ValueError("bad AdcQueue packet: ext %08x len %d" % (ext, len(r)))
        return [struct.unpack_from("<HxxiiHHHH", r, i + 4 + 20 * k) for k in range(chunk)], adc


def stream(km, rate, secs, mask=ATT_ADC_QUEUE, every=0.04):
    """StartGraph(rate), poll for `secs`; -> (records, replies, max per reply)."""
    r = km.xfer(START_GRAPH, rate)
    check(r is not None and r[0] == ACCEPT, "StartGraph %d SPS -> Accept" % SPS[rate])
    recs, replies, biggest, adc = [], 0, 0, None
    t_end = time.time() + secs
    while time.time() < t_end:
        r = km.xfer(GET_DATA, mask)
        if r is None or r[0] != KM_PUT_DATA:
            check(False, "GetData 0x%04x -> PutData" % mask)
            break
        got, a = aq_records(r)
        adc = a or adc
        recs += got
        replies += 1
        biggest = max(biggest, len(got))
        time.sleep(every)
    return recs, replies, biggest, adc


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

        tid, (conf, data) = memory_read(km, 0x40010450, 12)
        ok = conf is not None and len(conf) == 20 and conf[:4] == bytes([0xC4, tid, 1, 1])
        ok = ok and struct.unpack_from("<II", conf, 4) == (0x40010450, 12)
        hwid = AES.new(MEM_KEY, AES.MODE_ECB).decrypt(data)[:12] if data and len(data) == 16 else b""
        check(ok and hwid[:6] == b"TC66FW", "MemoryRead HardwareID -> confirmation + 16-B block (%s)" % hwid.hex())
        tid, (r, _) = memory_read(km, 0x420, 64)
        check(r is not None and r[:2] == bytes([NOT_READABLE, tid]), "MemoryRead other address -> NotReadable")

        payload = os.urandom(32)
        (r,) = km.raw(bytes([STREAMING_AUTH, km.next_tid(), 0x00, 0x02]) + payload)
        check(r is not None and r[:4] == bytes([STREAMING_AUTH, 0, 0x03, 0x02]) and len(r) == 36,
              "StreamingAuth -> level 1 (0x0203)")

        for rate, secs in ((3, 1.5), (2, 1.5), (1, 2.0), (0, 3.0)):
            t0 = time.time()
            recs, replies, biggest, _ = stream(km, rate, secs)
            dt = time.time() - t0
            steps = {(b[0] - a[0]) & 0xFFFF for a, b in zip(recs, recs[1:])}
            print("      %4d SPS: %d records in %d replies over %.2f s (%.0f/s), max %d/reply, "
                  "seq steps %s" % (SPS[rate], len(recs), replies, dt, len(recs) / dt, biggest, sorted(steps)))
            if recs:
                print("      first: seq %d  %.4f V  %+.5f A  CC1 %d CC2 %d D+ %d D- %d" % (
                    recs[0][0], recs[0][1] / 1e6, recs[0][2] / 1e6, *recs[0][3:]))
            check(steps == {SEQ_STEP[rate]}, "%d SPS: every sequence step is %d (no drops)"
                  % (SPS[rate], SEQ_STEP[rate]))
            check(0.8 < len(recs) / dt / SPS[rate] < 1.1, "%d SPS: sample rate within -20/+10%%" % SPS[rate])
            check(biggest <= 48, "%d SPS: at most 48 records per reply" % SPS[rate])
        recs, _, _, adc = stream(km, 3, 0.3, mask=ATT_ADC | ATT_ADC_QUEUE)
        check(adc is not None and len(adc) == 44 and len(recs) > 0, "GetData ADC+AdcQueue -> chained ADC + records")
        r = km.xfer(STOP_GRAPH)
        check(r is not None and r[0] == ACCEPT, "StopGraph -> Accept")
        r = km.xfer(STOP_GRAPH)
        check(r is not None and r[0] == REJECT, "StopGraph when not streaming -> Reject")
        r = km.xfer(GET_DATA, ATT_ADC_QUEUE)
        check(r is not None and r[:4] == bytes([KM_PUT_DATA, km.tid, 0, 0]), "GetData AdcQueue, no stream -> empty PutData")
        r = km.xfer(DISCONNECT)
        check(r is not None and r[0] == ACCEPT, "Disconnect -> Accept")
    finally:
        km.close()
    print("%d failure(s)" % fails)
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
