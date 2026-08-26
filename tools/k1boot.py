#!/usr/bin/env python3
"""Drive the TC66's K1 button from the host and enter the meter's bootloader
hands-free.

K1 (PF0) is wired to GP8 of the Pico debugprobe, whose (patched, see FIRMWARE.md)
firmware exposes GP8 as the CMSIS-DAP nRESET pin: open drain, so "release" is
Hi-Z (K1's pull-up takes it high) and "hold" drives it low. We talk to it with
the stock DAP_SWJ_Pins command over the CMSIS-DAP v2 bulk interface — no SWD
connect, so SWCLK/SWDIO are never driven (they'd back-power the meter while its
port is off).

  k1boot.py hold | release | state     # raw K1 control / pin readback
  k1boot.py bootloader [-n TRIES]      # hold K1, power-cycle b2, probe for
                                       # `query` -> `boot`; retry (the meter
                                       # occasionally wedges on enumeration)
"""
import argparse, fcntl, glob, os, select, subprocess, sys, termios, time

import usb.core, usb.util

DAP_SWJ_PINS = 0x10
PIN_nRESET = 1 << 7
HUB_POWER = os.environ.get(
    "HUB_POWER", os.path.expanduser("~/.claude/skills/usb-hub-power/hub_power.py"))
METER_BYID_GLOB = "/dev/serial/by-id/*RuiDeng_TC66*"


class Probe:
    """CMSIS-DAP v2 (vendor bulk) connection to the Pico debugprobe."""

    def __init__(self):
        dev = usb.core.find(idVendor=0x2E8A, idProduct=0x000C)
        if dev is None:
            raise RuntimeError("debugprobe (2e8a:000c) not on USB")
        cfg = dev.get_active_configuration()
        # CMSIS-DAP v2 iface: vendor class, protocol 0 (protocol 1 = picotool reset)
        intf = next(i for i in cfg
                    if i.bInterfaceClass == 0xFF and i.bInterfaceProtocol == 0)
        eps = sorted(intf.endpoints(), key=lambda e: usb.util.endpoint_direction(
            e.bEndpointAddress))  # OUT (0x00) before IN (0x80)
        self.ep_out, self.ep_in = eps[0], eps[-1]

    def swj_pins(self, value, select_):
        """DAP_SWJ_Pins; returns the pin-state byte (bit 7 = actual K1 level)."""
        self.ep_out.write(bytes([DAP_SWJ_PINS, value, select_, 0, 0, 0, 0]))
        resp = self.ep_in.read(64, timeout=1000)
        if resp[0] != DAP_SWJ_PINS:
            raise RuntimeError(f"bad DAP response: {bytes(resp[:2]).hex()}")
        return resp[1]

    def k1_hold(self):
        return self.swj_pins(0, PIN_nRESET)

    def k1_release(self):
        return self.swj_pins(PIN_nRESET, PIN_nRESET)

    def k1_level(self):
        return (self.swj_pins(0, 0) >> 7) & 1


def raw_query(port, timeout=5.0):
    """Send `query`, return the reply bytes. Never blocks past `timeout`.

    Hardened per ~/tc66/probe_meter.py: O_NONBLOCK + select with a monotonic
    deadline, and tcflush before close — if the meter isn't draining its USB
    bulk-OUT, written bytes stuck in the tty TX queue make a normal close()
    block forever."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, b"query")
        buf = b""
        end = time.monotonic() + timeout
        while len(buf) < 4:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            r, _, _ = select.select([fd], [], [], remaining)
            if not r:
                break
            chunk = os.read(fd, 64)
            if not chunk:
                break
            buf += chunk
        return buf
    finally:
        try:
            termios.tcflush(fd, termios.TCIOFLUSH)
        except Exception:
            pass
        os.close(fd)


def wait_enumerate(timeout=10.0):
    """Wait for the meter's by-id node after a power-cycle; return port or None."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        nodes = glob.glob(METER_BYID_GLOB)
        if nodes:
            time.sleep(0.5)  # let the tty node settle
            return os.path.realpath(nodes[0])
        time.sleep(0.3)
    return None


def enter_bootloader(tries):
    probe = Probe()
    probe.k1_hold()
    time.sleep(0.05)  # K1's debounce cap slows the edges; settle before reading
    held_low = probe.k1_level() == 0
    probe.k1_release()
    if not held_low:
        print("!! K1 won't go low — check GP8-K1 wiring")
        return 3
    for attempt in range(1, tries + 1):
        print(f"attempt {attempt}/{tries}: K1 low + power-cycle b2", flush=True)
        probe.k1_hold()
        try:
            subprocess.run([HUB_POWER, "cycle", "b2", "-t", "2"], check=True,
                           stdout=subprocess.DEVNULL)
        finally:
            probe.k1_release()
        port = wait_enumerate()
        if port is None:
            print("  no enumeration", flush=True)
            continue
        try:
            reply = raw_query(port)
        except OSError as e:
            print(f"  {port}: {e}", flush=True)
            continue
        print(f"  {port}: {reply!r}", flush=True)
        if reply.startswith(b"boot"):
            print(f"bootloader up on {port}", flush=True)
            return 0
    print("!! meter did not enter bootloader")
    return 2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["hold", "release", "state", "bootloader"])
    ap.add_argument("-n", "--tries", type=int, default=5)
    args = ap.parse_args()
    if args.cmd == "bootloader":
        return enter_bootloader(args.tries)
    probe = Probe()
    if args.cmd == "hold":
        probe.k1_hold()
    elif args.cmd == "release":
        probe.k1_release()
    time.sleep(0.05)  # K1's debounce cap makes the release edge slow
    print(f"K1 level: {probe.k1_level()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
