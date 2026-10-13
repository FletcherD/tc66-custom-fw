#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial", "cryptography", "tqdm"]
# ///
"""Flash, back up, and restore the RuiDeng TC66/TC66C — standalone, no Rust
toolchain needed.

  tc66flash.py flash tc66fw-vX.Y.Z.enc.bin   flash a release image (K1 bootloader)
  tc66flash.py stock                         download RuiDeng's stock firmware
                                             and flash it (back to stock)
  tc66flash.py backup-eeprom [out.bin]       save the 32 KB EEPROM (calibration
                                             etc.) — needs the custom firmware
  tc66flash.py version                       what the meter is running

Flashing goes through the meter's own bootloader, which is never modified:
hold K1 while plugging the meter into the computer and it waits for an image.
A failed or interrupted flash is always recoverable the same way.

Close every other program that might open the meter's serial port (the
vendor PC app, serial terminals) while flashing. On Linux, ModemManager can
probe new serial ports mid-flash — stop it (`sudo systemctl stop
ModemManager`) if a flash aborts.
"""
import argparse
import hashlib
import struct
import sys
import time
import urllib.request

import serial
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from serial.tools import list_ports
from tqdm import tqdm

# AES-256-ECB key of the bootloader's firmware-image format (vendor images
# and ours alike): the bootloader decrypts each 64-B chunk before writing it.
IMAGE_KEY = bytes.fromhex("381255645a6b7aaabc6d5877f5c1fbff9805d1ac3c8335661a610c7ff73a638c")
IMAGE_SIZE = 51840  # 810 x 64-B chunks = the whole app slot
APP_BASE, APP_END = 0x08002C00, 0x0800FC00
STOCK_URL = "http://www.ruidengkeji.com/rdupdate/firmware/TC66/TC66.bin"
KNOWN_STOCK = {
    "ef796b29557c91e0a469740ba1934ca0a838916c76ab55eb6d1253da32830940": "V1.18",
}
# USB ids: the stock bootloader/firmware enumerate as RuiDeng; the custom
# firmware as a POWER-Z KM003C (for the Linux powerz hwmon driver).
STOCK_IDS = {(0x28E9, 0x018A)}
CUSTOM_IDS = {(0x5FC9, 0x0063)}
EEPROM_SIZE = 0x8000
EEPROM_CHUNK = 32


def candidate_ports():
    ports = []
    for p in list_ports.comports():
        if (p.vid, p.pid) in STOCK_IDS | CUSTOM_IDS:
            ports.append(p.device)
    return ports


def open_port(dev, timeout):
    s = serial.Serial(dev, 115200, timeout=timeout)
    s.reset_input_buffer()
    return s


def expect(s, *prefixes, cap=3.0):
    """Non-blocking read until the reply starts with one of `prefixes`. A
    flash must never leave the port idle for long (a blocking read would)."""
    end = time.time() + cap
    buf = b""
    while time.time() < end:
        n = s.in_waiting
        if n:
            buf += s.read(n)
            if any(buf.startswith(p) for p in prefixes):
                return buf
        else:
            time.sleep(0.001)
    raise RuntimeError(f"expected {prefixes}, got {buf!r}")


def query(dev):
    """'boot' (bootloader), 'firm' (running firmware) or None."""
    try:
        s = open_port(dev, 1.0)
    except (OSError, serial.SerialException):
        return None
    try:
        s.write(b"query")
        r = s.read(4)
        return r.decode("latin1") if r in (b"boot", b"firm") else None
    except (OSError, serial.SerialException):
        return None
    finally:
        s.close()


def find(state):
    for dev in candidate_ports():
        if query(dev) == state:
            return dev
    return None


def check_image(enc):
    """Decrypt the vector table and sanity-check it: refuses files that are
    not TC66 firmware images (wrong key, truncated, random data)."""
    if len(enc) != IMAGE_SIZE:
        sys.exit(f"not a TC66 image: {len(enc)} bytes, expected {IMAGE_SIZE}")
    dec = Cipher(algorithms.AES(IMAGE_KEY), modes.ECB()).decryptor()
    head = dec.update(enc[:16]) + dec.finalize()
    sp, reset = struct.unpack_from("<II", head)
    if not (0x20000000 <= sp < 0x20002000 and APP_BASE < reset < APP_END and reset & 1):
        sys.exit(f"not a TC66 image (decrypted sp={sp:#x} reset={reset:#x}) — "
                 "pass the encrypted .enc.bin release file")


def wait_for_bootloader():
    dev = find("boot")
    if dev:
        return dev
    print("""
Put the meter into its bootloader:
  1. Unplug the meter from the computer (and from any charger on its input).
  2. Press and HOLD the K1 button (the left/'previous page' button).
  3. While holding K1, plug the meter into the computer
     (TC66C: its micro-USB port).
  4. Release K1 after ~2 seconds.
Waiting for the bootloader (Ctrl-C to abort)...""", flush=True)
    warned = False
    while True:
        dev = find("boot")
        if dev:
            return dev
        if not warned and find("firm"):
            print("  The meter is running its normal firmware: K1 was not held "
                  "while it powered up. Unplug it and try again.", flush=True)
            warned = True
        time.sleep(0.5)


def flash(enc):
    check_image(enc)
    dev = wait_for_bootloader()
    print(f"bootloader found on {dev}")
    s = open_port(dev, 0)
    try:
        s.write(b"update")
        expect(s, b"uprdy", cap=10.0)
        for i in tqdm(range(IMAGE_SIZE // 64), desc="flashing", unit="chunk"):
            s.write(enc[i * 64:(i + 1) * 64])
            expect(s, b"OK")
    except RuntimeError as e:
        sys.exit(f"flash aborted: {e}\nThe bootloader is intact — repeat the "
                 "K1 procedure and flash again.")
    finally:
        s.close()
    print("Done. The meter restarts into the new firmware.")


def cmd_flash(a):
    with open(a.image, "rb") as f:
        enc = f.read()
    print(f"{a.image}: sha256 {hashlib.sha256(enc).hexdigest()}")
    flash(enc)


def cmd_stock(a):
    print(f"downloading {a.url}")
    with urllib.request.urlopen(a.url, timeout=30) as r:
        enc = r.read()
    digest = hashlib.sha256(enc).hexdigest()
    print(f"sha256 {digest} ({KNOWN_STOCK.get(digest, 'unknown version')})")
    if digest not in KNOWN_STOCK and not a.any_version:
        sys.exit("unrecognized stock image — re-run with --any-version to flash it anyway")
    flash(enc)


def open_custom():
    for dev in candidate_ports():
        try:
            s = open_port(dev, 1.0)
        except (OSError, serial.SerialException):
            continue
        s.write(b"versn")
        r = s.read(32)
        if r.startswith(b"OK"):
            return s, r[2:].decode("ascii", "replace")
        s.close()
    return None, None


def cmd_backup(a):
    s, ver = open_custom()
    if s is None:
        sys.exit("no meter running the custom firmware found (the stock firmware "
                 "cannot read out its EEPROM — flash the custom firmware first; "
                 "it never writes the calibration)")
    print(f"custom firmware {ver}")
    img = bytearray()
    with tqdm(total=EEPROM_SIZE, unit="B", unit_scale=True, desc="eeprom") as bar:
        for addr in range(0, EEPROM_SIZE, EEPROM_CHUNK):
            for attempt in range(3):
                s.reset_input_buffer()
                s.write(b"eeprd" + struct.pack("<HB", addr, EEPROM_CHUNK))
                r = s.read(2 + EEPROM_CHUNK)
                if r[:2] == b"OK" and len(r) == 2 + EEPROM_CHUNK:
                    break
                if attempt == 2:
                    sys.exit(f"EEPROM read failed at {addr:#06x}: {r!r}")
            img += r[2:]
            bar.update(EEPROM_CHUNK)
    s.close()
    with open(a.out, "wb") as f:
        f.write(img)
    gain_v, = struct.unpack_from("<I", img, 0x2004)
    print(f"wrote {a.out} (sha256 {hashlib.sha256(img).hexdigest()})")
    print(f"calibration words present: gainV={gain_v} — keep this file safe; "
          "it is specific to this meter")


def cmd_version(a):
    s, ver = open_custom()
    if s is not None:
        s.close()
        print(f"custom firmware {ver}")
        return
    if find("boot"):
        print("bootloader (K1 mode) — ready to flash")
    elif find("firm"):
        print("stock firmware (or a custom build without `versn`)")
    else:
        sys.exit("no meter found")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("flash", help="flash an encrypted release image")
    f.add_argument("image", help="tc66fw-*.enc.bin")
    f.set_defaults(fn=cmd_flash)
    st = sub.add_parser("stock", help="download and flash RuiDeng's stock firmware",
                        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    st.add_argument("--url", default=STOCK_URL, help="stock image URL")
    st.add_argument("--any-version", action="store_true",
                    help="flash even if the download is not a known stock release")
    st.set_defaults(fn=cmd_stock)
    b = sub.add_parser("backup-eeprom", help="save the meter's EEPROM to a file",
                       formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    b.add_argument("out", nargs="?", default="tc66-eeprom-backup.bin", help="output file")
    b.set_defaults(fn=cmd_backup)
    v = sub.add_parser("version", help="identify what the meter is running")
    v.set_defaults(fn=cmd_version)
    a = ap.parse_args()
    try:
        a.fn(a)
    except KeyboardInterrupt:
        sys.exit("aborted")


if __name__ == "__main__":
    main()
