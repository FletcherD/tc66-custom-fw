"""Shared TC66 meter helpers.

Import these instead of re-defining `find_bootloader()` / `expect()` in every
capture/flash script. Canonical implementations live here.

    from tc66lib import find_bootloader, expect, wait_for_bootloader, V118

Subdir scripts can locate this module with:
    import sys, os
    _R = os.path.abspath(os.path.dirname(__file__))
    while _R != "/" and not os.path.exists(os.path.join(_R, "tc66lib.py")):
        _R = os.path.dirname(_R)
    sys.path.insert(0, _R)
"""
import glob, os, re, subprocess, sys, time
import serial
try:
    import termios
except ImportError:
    termios = None

HUB_TOOL     = os.path.expanduser("~/.claude/skills/usb-hub-power/hub_power.py")
# Bootloader enumerates as stock RuiDeng; the custom app wears the POWER-Z
# KM003C identity (5fc9:0063) so the mainline `powerz` hwmon driver binds its
# vendor interface. Both are "the meter".
METER_VIDPIDS = ("28e9:018a", "5fc9:0063")
METER_VIDPID  = METER_VIDPIDS[0]  # kept for old imports


def find_meter_hub_port(hub_tool=HUB_TOOL):
    """Return the bench-hub port id (e.g. 'b2') the TC66 is plugged into, parsed
    from `hub_power.py status`. Matches on the meter's USB vid:pid / name so it
    works on ANY port. Returns None if the meter isn't enumerated (powered off,
    or running a non-USB app like the SRAM copier). Detect it while the meter is
    up (bootloader/app) and cache the result across a power-cycle."""
    try:
        out = subprocess.run([sys.executable, hub_tool, "status"],
                             capture_output=True, text=True, timeout=30).stdout
    except Exception:
        return None
    for line in out.splitlines():
        # cmd_status prints: "  b2  on   5fc9:0063  ChargerLAB POWER-Z KM003C TC66"
        m = re.match(r"\s*([a-z]\d+)\s+(?:on|OFF)\s+(.*)", line)
        if m and (any(v in m.group(2) for v in METER_VIDPIDS)
                  or "RuiDeng" in m.group(2) or "POWER-Z" in m.group(2)):
            return m.group(1)
    return None


def _safe_close(s):
    """Close a serial port without hanging. A plain close() blocks forever
    draining the kernel TX queue if the CDC peer isn't reading its OUT endpoint
    (e.g. the meter mid-reboot after a query write). tcflush discards that queue
    first. See CLAUDE.md 'close() hangs on a non-draining CDC'."""
    try:
        if s is not None and termios is not None:
            termios.tcflush(s.fileno(), termios.TCIOFLUSH)
    except Exception:
        pass
    try:
        if s is not None:
            s.close()
    except Exception:
        pass

REPO_ROOT    = os.path.dirname(os.path.abspath(__file__))
FIRMWARE_DIR = os.path.join(REPO_ROOT, "firmware")
V118         = os.path.join(FIRMWARE_DIR, "V118.bin")

METER_BYIDS = (
    # custom app (CDC is interfaces 1+2 of the powerz composite)
    "/dev/serial/by-id/usb-ChargerLAB_POWER-Z_KM003C_TC66-if01",
    # stock bootloader (recovery / first flash)
    "/dev/serial/by-id/usb-RuiDeng_TC66_USB_Type-C_Meter_TC66-if00",
)
METER_BYID = METER_BYIDS[1]  # kept for old imports
PICO_BYID  = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_5303284748AA549C-if00"


def meter_candidates():
    """Serial ports that might be the meter — stable by-id first, then ttyACM*,
    skipping the Pico-trigger and ChipWhisperer CDC devices."""
    cands = []
    for byid in METER_BYIDS:
        if os.path.exists(byid):
            cands.append(os.path.realpath(byid))
    skip = {os.path.realpath(l) for l in glob.glob("/dev/serial/by-id/*")
            if "Pico" in l or "ChipWhisperer" in l}
    for p in sorted(glob.glob("/dev/ttyACM*")):
        rp = os.path.realpath(p)
        if rp not in skip and rp not in cands:
            cands.append(rp)
    return cands


def find_bootloader(baud=115200):
    """Return an OPEN, non-blocking `serial.Serial` iff the meter answers `query`
    with `boot`, else None. Robust to ports that are mid-re-enumeration after a
    post-flash reset (those raise on open/read and are simply skipped)."""
    for port in meter_candidates():
        s = None
        try:
            s = serial.Serial(port, baud, timeout=1.0)
            s.reset_input_buffer()
            s.write(b"query")
            if s.read(64).decode("latin1", "replace").startswith("boot"):
                s.timeout = 0            # non-blocking, as expect() requires
                return s
            _safe_close(s)
        except Exception:
            _safe_close(s)
    return None


def wait_for_bootloader(timeout=25.0, poll=1.0, baud=115200):
    """Poll `find_bootloader()` until it answers `boot` or `timeout` elapses.
    Needed between flash passes: a completed flash resets the meter, which (with
    K1 clipped) re-enters the bootloader only after a few seconds + USB re-enum."""
    end = time.time() + timeout
    while time.time() < end:
        s = find_bootloader(baud)
        if s is not None:
            return s
        time.sleep(poll)
    return None


def expect(ser, *prefixes, cap=3.0):
    """Poll `ser.in_waiting` and return the accumulated bytes as soon as they
    start with any of `prefixes`. Raises RuntimeError on timeout.

    Requires `ser.timeout == 0` (non-blocking) — `find_bootloader` sets that. A
    blocking `read(n)` would instead wait the full timeout for bytes that never
    come (e.g. the 2-byte `OK` behind a 64-byte read → ~6 s/chunk)."""
    end = time.time() + cap
    buf = b""
    want = tuple(p.encode() if isinstance(p, str) else p for p in prefixes)
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            if any(buf.startswith(w) for w in want):
                return buf
        else:
            time.sleep(0.001)
    raise RuntimeError(f"expected {prefixes}, got {buf!r} after {cap}s")
