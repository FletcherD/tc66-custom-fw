#!/usr/bin/env -S uv run --quiet --script
# /// script
# dependencies = ["bleak", "cryptography", "pyserial"]
# ///
"""Host side of the firmware's BLE link (rust/src/bt.rs, DX-BT24-M module).

The module is a transparent UART<->GATT bridge: service ffe0, notify ffe1
(meter -> host), write ffe2 (host -> meter). Lines are CRLF-terminated
b-verbs (bgetva/bquery/brawva), replies come back as <=20-B notifications.

  ble.py scan                 list ffe0 advertisers (the meter is "BT24-M")
  ble.py send <verb> [-w S]   write "<verb>\\r\\n", print bytes notified for S s
  ble.py getva                bgetva -> collect 192 B -> decrypt + print
  ble.py rxtest [text]        RX-path check without dispatch: btini over USB
                              (raw mode), write text over BLE, btget shows
                              what reached the MCU ring (leaves the link in
                              raw mode — power-cycle or `btrun` to restore)
  ble.py spy <port> <pin>     btspy over USB (1 s pin sampler) while a BLE
                              write is in flight: do edges reach that pin?
  ble.py sta                  btsta register dump (USART1 / DMA ch4 / GPIOB)
"""
import argparse
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner

SVC = "0000ffe0-0000-1000-8000-00805f9b34fb"
CH_NOTIFY = "0000ffe1-0000-1000-8000-00805f9b34fb"
CH_WRITE = "0000ffe2-0000-1000-8000-00805f9b34fb"
KEY = bytes((0x58, 0x21, 0xfa, 0x56, 0x01, 0xb2, 0xf0, 0x26,
             0x87, 0xff, 0x12, 0x04, 0x62, 0x2a, 0x4f, 0xb0,
             0x86, 0xf4, 0x02, 0x60, 0x81, 0x6f, 0x9a, 0x0b,
             0xa7, 0xf1, 0x06, 0x61, 0x9a, 0xb8, 0x72, 0x88))


async def find(timeout, address=None):
    if address:
        return address
    hit = {}

    def cb(d, adv):
        if any(u.lower() == SVC for u in adv.service_uuids):
            hit[d.address] = (d.name, adv.rssi)

    s = BleakScanner(cb)
    await s.start()
    t0 = time.time()
    while time.time() - t0 < timeout and not hit:
        await asyncio.sleep(0.2)
    await s.stop()
    return hit


async def scan(a):
    hit = await find(a.timeout)
    for addr, (name, rssi) in hit.items():
        print(addr, name, rssi, "dBm")
    if not hit:
        sys.exit("no ffe0 advertiser found")


async def session(a, verb, wait):
    hit = await find(a.timeout, a.address)
    if not hit:
        sys.exit("meter not found (is it advertising / not held by a phone?)")
    addr = hit if isinstance(hit, str) else next(iter(hit))
    got = bytearray()
    stamps = []

    def on_notify(_, data):
        got.extend(data)
        stamps.append((time.time(), len(data)))

    async with BleakClient(addr, timeout=a.timeout) as c:
        await c.start_notify(CH_NOTIFY, on_notify)
        t0 = time.time()
        await c.write_gatt_char(CH_WRITE, verb.encode() + b"\r\n", response=False)
        while time.time() - t0 < wait:
            await asyncio.sleep(0.05)
            if a.expect and len(got) >= a.expect:
                break
        await c.stop_notify(CH_NOTIFY)
    for t, n in stamps:
        print("  +%.3f s  %d B" % (t - t0, n))
    return bytes(got)


def decode(raw):
    from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
    d = Cipher(algorithms.AES(KEY), modes.ECB()).decryptor()
    p = d.update(raw) + d.finalize()
    if p[:4] != b"pac1":
        sys.exit("bad packet marker: %r" % p[:8])
    u = lambda o: int.from_bytes(p[o:o + 4], "little")
    return {
        "fw": p[8:12].split(b"\0")[0].decode(), "V": u(48) / 1e4, "A": u(52) / 1e5,
        "W": u(56) / 1e4, "mAh": u(72), "mWh": u(76),
        "temp_C": (-1 if u(88) else 1) * u(92), "D+": u(96) / 100, "D-": u(100) / 100,
    }


async def rxtest(a):
    sys.path.insert(0, __file__.rsplit("/", 1)[0] + "/lib")
    import serial
    from tc66lib import meter_candidates
    s = serial.Serial(meter_candidates()[0], 115200, timeout=2)
    s.write(b"btini f" if a.fast else b"btini")
    print("btini ->", s.read(2))
    a.expect = 0
    await session(a, a.text, 0.5)
    s.reset_input_buffer()
    s.write(b"btget")
    r = s.read(40)
    print("btget ->", r)
    if r[2:] == a.text.encode() + b"\r\n":
        print("PASS: BLE -> module -> USART1 -> DMA ring intact")
    else:
        sys.exit("FAIL: ring content differs from what was written")


def open_usb():
    sys.path.insert(0, __file__.rsplit("/", 1)[0] + "/lib")
    import serial
    from tc66lib import meter_candidates
    s = serial.Serial(meter_candidates()[0], 115200, timeout=3)
    s.reset_input_buffer()
    return s


def sta():
    s = open_usb()
    s.write(b"btsta")
    r = s.read(30)
    names = ["USART1_STAT", "USART1_CTL0", "USART1_BAUD", "DMA_CH4CTL", "DMA_CH4CNT", "DMA_CH3CTL", "DMA_CH3CNT"]
    for i, n in enumerate(names):
        print("  %-13s 0x%08x" % (n, int.from_bytes(r[2 + 4 * i:6 + 4 * i], "little")))


async def spy(a):
    hit = await find(a.timeout, a.address)
    addr = hit if isinstance(hit, str) else next(iter(hit))
    s = open_usb()
    async with BleakClient(addr, timeout=a.timeout) as c:
        s.write(b"btspy" + bytes([a.port, a.pin]))
        await asyncio.sleep(0.2)
        for _ in range(3):
            await c.write_gatt_char(CH_WRITE, b"U" * 20, response=False)
            await asyncio.sleep(0.1)
        r = s.read(14)
    u = lambda o: int.from_bytes(r[o:o + 4], "little")
    print("port %d pin %d: transitions=%d low=%d of %d samples" % (a.port, a.pin, u(2), u(6), u(10)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-a", "--address", help="skip the scan, connect to this BD address")
    ap.add_argument("-t", "--timeout", type=float, default=15.0, help="scan/connect timeout (s)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    p = sub.add_parser("send")
    p.add_argument("verb")
    p.add_argument("-w", "--wait", type=float, default=3.0, help="collect notifications for this long (s)")
    p.add_argument("-e", "--expect", type=int, default=0, help="stop early once this many bytes arrived")
    sub.add_parser("getva")
    p = sub.add_parser("rxtest")
    p.add_argument("text", nargs="?", default="hello")
    p.add_argument("-f", "--fast", action="store_true", help="init the link at 115200 instead of the module's 9600 default")
    p = sub.add_parser("spy")
    p.add_argument("port", type=int, help="0 = GPIOA, 1 = GPIOB")
    p.add_argument("pin", type=int)
    sub.add_parser("sta")
    a = ap.parse_args()
    if a.cmd == "scan":
        asyncio.run(scan(a))
    elif a.cmd == "send":
        got = asyncio.run(session(a, a.verb, a.wait))
        print("%d B: %r" % (len(got), got))
    elif a.cmd == "getva":
        a.expect = 192
        got = asyncio.run(session(a, "bgetva", 6.0))
        print("%d B" % len(got))
        if len(got) != 192:
            sys.exit("incomplete packet")
        for k, v in decode(got).items():
            print("  %-7s %s" % (k, v))
    elif a.cmd == "rxtest":
        asyncio.run(rxtest(a))
    elif a.cmd == "spy":
        asyncio.run(spy(a))
    elif a.cmd == "sta":
        sta()


if __name__ == "__main__":
    main()
