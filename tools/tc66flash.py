#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "pyserial", "cryptography", "tqdm",
#     "pyusb; sys_platform == 'darwin'",
#     "libusb-package; sys_platform == 'darwin'",
# ]
# ///
"""Flash, back up, and restore the RuiDeng TC66/TC66C — standalone, no Rust
toolchain needed.

  tc66flash.py flash tc66fw-vN.enc.bin   flash a release image (K1 bootloader)
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

On macOS, `flash` and `stock` talk to the bootloader over libusb instead of
the serial port (see UsbLink for why). Only root may detach macOS's serial
driver from the meter, so they re-run themselves under sudo, which asks for
your password.
"""
import argparse
import hashlib
import os
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


class SerialLink:
    """The bootloader through the OS serial port (CDC-ACM driver)."""

    def __init__(self, dev):
        self.name = dev
        self.s = open_port(dev, 0)

    def write(self, data):
        self.s.write(data)

    def recv(self, timeout):
        """Whatever has arrived, waiting up to `timeout` s for the first
        byte. Non-blocking: a flash must never leave the port idle for long
        (a blocking read would)."""
        end = time.monotonic() + timeout
        while True:
            n = self.s.in_waiting
            if n:
                return self.s.read(n)
            if time.monotonic() >= end:
                return b""
            time.sleep(0.001)

    def close(self, reattach=True):
        self.s.close()


class UsbLink:
    """The bootloader over libusb, past the OS serial driver: exactly one
    64-B bulk packet per chunk.

    The macOS CDC driver follows every write whose length is a multiple of
    64 B with a zero-length packet. The bootloader (V1.06) answers that empty
    packet with a second OK and processes it like a chunk: the image ends up
    corrupted, while the meter still shows UPDATE SUCCESS. libusb sends no
    zero-length packet unless asked to. Detaching the OS driver needs root on
    macOS (on Linux: write access to the USB device)."""

    def __init__(self, dev):
        import usb.core
        import usb.util
        self.core, self.util, self.dev = usb.core, usb.util, dev
        self.name = f"USB {dev.idVendor:04x}:{dev.idProduct:04x} (bus {dev.bus}, address {dev.address})"
        self.detached, self.claimed = [], []
        try:
            cfg = dev.get_active_configuration()
            comm = next((i for i in cfg if i.bInterfaceClass == 0x02), None)  # CDC control
            data = next((i for i in cfg if i.bInterfaceClass == 0x0A), None)  # CDC data
            if data is None:
                raise RuntimeError(f"{self.name} has no CDC data interface")
            for intf in cfg:
                n = intf.bInterfaceNumber
                try:
                    if dev.is_kernel_driver_active(n):
                        dev.detach_kernel_driver(n)
                        self.detached.append(n)
                except (NotImplementedError, usb.core.USBError):
                    pass  # if it mattered, claiming the interface fails below
            for intf in (comm, data):
                if intf is not None:
                    usb.util.claim_interface(dev, intf.bInterfaceNumber)
                    self.claimed.append(intf.bInterfaceNumber)

            def endpoint(direction):
                return usb.util.find_descriptor(data, custom_match=lambda e: (
                    usb.util.endpoint_direction(e.bEndpointAddress) == direction)).bEndpointAddress
            self.ep_out = endpoint(usb.util.ENDPOINT_OUT)
            self.ep_in = endpoint(usb.util.ENDPOINT_IN)
            if comm is not None:  # SET_CONTROL_LINE_STATE DTR+RTS, as opening the port does
                try:
                    dev.ctrl_transfer(0x21, 0x22, 0x03, comm.bInterfaceNumber, None, 1000)
                except usb.core.USBError:
                    pass
            while self.recv(0.05):  # drop stale replies
                pass
        except usb.core.USBError as e:
            self.close()
            hint = {"darwin": "run as root",
                    "win32": "--transport usb needs the WinUSB driver (e.g. via Zadig); "
                             "the default --transport serial does not"}.get(
                sys.platform, "check the USB device's permissions")
            raise PermissionError(f"cannot take {self.name} over from the OS driver ({e}): {hint}") from e
        except RuntimeError:
            self.close()
            raise

    def write(self, data):
        try:
            n = self.dev.write(self.ep_out, data, 2000)
        except self.core.USBError as e:
            raise RuntimeError(f"USB write failed: {e}") from e
        if n != len(data):
            raise RuntimeError(f"USB write sent {n} of {len(data)} bytes")

    def recv(self, timeout):
        """One USB packet, waiting up to `timeout` s for it."""
        try:  # a libusb timeout of 0 would mean forever
            return bytes(self.dev.read(self.ep_in, 64, max(1, round(timeout * 1000))))
        except self.core.USBTimeoutError:
            return b""
        except self.core.USBError as e:
            raise RuntimeError(f"USB read failed: {e}") from e

    def close(self, reattach=True):
        """Let go of the meter. With `reattach`, hand it back to the OS driver
        (not after a flash: the bootloader is starting the new image)."""
        for n in self.claimed:
            try:
                self.util.release_interface(self.dev, n)
            except self.core.USBError:
                pass
        if reattach:
            for n in self.detached:
                try:
                    self.dev.attach_kernel_driver(n)
                except (NotImplementedError, self.core.USBError):
                    pass
        self.util.dispose_resources(self.dev)


def expect(link, *prefixes, cap=3.0):
    """Read until the reply starts with one of `prefixes`."""
    end = time.monotonic() + cap
    buf = b""
    while time.monotonic() < end:
        buf += link.recv(min(0.1, end - time.monotonic()))
        if any(buf.startswith(p) for p in prefixes):
            return buf
    raise RuntimeError(f"expected {prefixes}, got {buf!r}")


# Each chunk must be answered by exactly one OK. The bootloader replies once
# per USB packet it receives, ~10 ms after it, so a packet the host did not
# mean to send (the zero-length packet macOS adds) shows up as a second OK
# ~10 ms after the first. Listening that long after every chunk would triple
# the flash time, so only the first chunks and the last one get the long
# window. The first ones catch a driver that adds such packets to every
# write, before much is written. The last one checks the total: a surplus OK
# anywhere is taken as the reply to a later chunk, which leaves the host one
# reply behind, so the last chunk's own OK arrives inside that window.
QUIET_LONG, QUIET_SHORT, LONG_CHUNKS = 0.025, 0.002, 8


def expect_ok(link, chunk, quiet):
    end = time.monotonic() + 3.0
    buf = b""
    while len(buf) < 2 and time.monotonic() < end:
        buf += link.recv(min(0.1, end - time.monotonic()))
    if buf == b"OK":
        buf += link.recv(quiet)
    if buf == b"OK":
        return
    if buf.startswith(b"OK"):  # more than one reply: collect the rest for the message
        buf += link.recv(QUIET_LONG)
        raise RuntimeError(
            f"chunk {chunk}: the bootloader answered more than once ({buf!r}), so "
            "the host is out of step with it (a serial driver that adds "
            "zero-length packets, as macOS's does)")
    raise RuntimeError(f"chunk {chunk}: expected b'OK', got {buf!r}")


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


K1_HELP = """
Put the meter into its bootloader:
  1. Unplug the meter from the computer (and from any charger on its input).
  2. Press and HOLD the K1 button (the left/'previous page' button).
  3. While holding K1, plug the meter into the computer
     (TC66C: its micro-USB port).
  4. Release K1 after ~2 seconds.
Waiting for the bootloader (Ctrl-C to abort)..."""
NOT_IN_BOOTLOADER = ("  The meter is running its normal firmware: K1 was not held "
                     "while it powered up. Unplug it and try again.")


def wait_for_bootloader():
    dev = find("boot")
    if dev:
        return dev
    print(K1_HELP, flush=True)
    warned = False
    while True:
        dev = find("boot")
        if dev:
            return dev
        if not warned and find("firm"):
            print(NOT_IN_BOOTLOADER, flush=True)
            warned = True
        time.sleep(0.5)


def usb_backend():
    try:  # installed on macOS only (see the script header)
        import libusb_package  # ships libusb itself: nothing to install system-wide
        import usb.backend.libusb1
    except ImportError:
        sys.exit("--transport usb needs pyusb and libusb-package: "
                 "uv run --with pyusb --with libusb-package tc66flash.py ...")
    be = libusb_package.get_libusb1_backend() or usb.backend.libusb1.get_backend()
    if be is None:
        sys.exit("libusb not found")
    return be


def usb_find(be, ids):
    import usb.core
    return usb.core.find(backend=be, custom_match=lambda d: (d.idVendor, d.idProduct) in ids)


def wait_for_bootloader_usb():
    """wait_for_bootloader(), then take the meter over from the OS driver.
    Returns an open UsbLink that has answered `query` with `boot`.

    The wait itself stays on the serial port: `query` is 5 bytes, so no
    zero-length packet follows it, and a meter that is not in its bootloader
    gets the usual hint without ever being taken over."""
    be = usb_backend()
    port = wait_for_bootloader()
    dev = usb_find(be, STOCK_IDS)
    if dev is None:
        sys.exit(f"bootloader on {port}, but libusb sees no matching USB device")
    try:
        link = UsbLink(dev)
    except (PermissionError, RuntimeError) as e:
        sys.exit(str(e))
    try:
        link.write(b"query")
        r = link.recv(1.0)
    except RuntimeError as e:
        link.close()
        sys.exit(f"{link.name}: {e}")
    if r != b"boot":
        link.close()
        sys.exit(f"{link.name} answered {r!r} to `query` over libusb, expected b'boot'")
    return link


def flash(enc, transport):
    check_image(enc)
    if transport == "usb":
        link = wait_for_bootloader_usb()
    else:
        link = SerialLink(wait_for_bootloader())
    print(f"bootloader found on {link.name}")
    done = False
    try:
        link.write(b"update")
        expect(link, b"uprdy", cap=10.0)
        n = IMAGE_SIZE // 64
        for i in tqdm(range(n), desc="flashing", unit="chunk"):
            link.write(enc[i * 64:(i + 1) * 64])
            expect_ok(link, i, QUIET_LONG if i < LONG_CHUNKS or i == n - 1 else QUIET_SHORT)
        done = True
    except RuntimeError as e:
        sys.exit(f"flash aborted: {e}\nThe bootloader is intact — repeat the "
                 "K1 procedure and flash again.")
    finally:
        link.close(reattach=not done)
    print("Done. The meter restarts into the new firmware.")


def default_transport():
    return "usb" if sys.platform == "darwin" else "serial"


def prepare_transport(a):
    """Refuse the serial path on macOS; get root for libusb there."""
    if sys.platform != "darwin":
        return
    if a.transport == "serial":
        sys.exit("--transport serial does not work on macOS: its serial driver "
                 "sends an empty USB packet after every 64-byte chunk, the "
                 "bootloader takes it as data, and the flashed image never "
                 "boots (although the meter shows UPDATE SUCCESS). Use the "
                 "default, --transport usb.")
    if os.geteuid() != 0:
        print("Flashing on macOS needs root: only root may detach the serial "
              "driver from the meter. Re-running under sudo.", flush=True)
        # -B: no root-owned __pycache__ in the user's uv environment
        os.execvp("sudo", ["sudo", sys.executable, "-B",
                           os.path.abspath(sys.argv[0]), *sys.argv[1:]])


def cmd_flash(a):
    with open(a.image, "rb") as f:
        enc = f.read()
    check_image(enc)  # before any password prompt
    prepare_transport(a)
    print(f"{a.image}: sha256 {hashlib.sha256(enc).hexdigest()}")
    flash(enc, a.transport)


def cmd_stock(a):
    prepare_transport(a)
    print(f"downloading {a.url}")
    with urllib.request.urlopen(a.url, timeout=30) as r:
        enc = r.read()
    digest = hashlib.sha256(enc).hexdigest()
    print(f"sha256 {digest} ({KNOWN_STOCK.get(digest, 'unknown version')})")
    if digest not in KNOWN_STOCK and not a.any_version:
        sys.exit("unrecognized stock image — re-run with --any-version to flash it anyway")
    flash(enc, a.transport)


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
    print(f"custom firmware v{ver}")
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
        print(f"custom firmware v{ver}")
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
    t = argparse.ArgumentParser(add_help=False)
    t.add_argument("--transport", choices=("usb", "serial"), default=default_transport(),
                   help="how to reach the bootloader: usb = libusb, one packet per "
                        "chunk (default on macOS, needs root there); serial = the "
                        "OS serial port (default elsewhere)")
    f = sub.add_parser("flash", parents=[t], help="flash an encrypted release image")
    f.add_argument("image", help="tc66fw-*.enc.bin")
    f.set_defaults(fn=cmd_flash)
    st = sub.add_parser("stock", parents=[t], help="download and flash RuiDeng's stock firmware",
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
