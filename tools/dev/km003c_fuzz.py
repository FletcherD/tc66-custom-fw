#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyusb", "pyserial", "tqdm"]
# ///
"""Long-running robustness fuzz of the meter's KM003C emulation.

Rotates randomly through:
  - every km003c-rs CLI (adc_simple, adc_queue_simple at random rates and
    durations — must report 0 missing samples, pd_trace, test_usbpd,
    memory_scan incl. --full-scan, offline-log metadata) — github.com/
    okhsunrog/km003c-rs, built in --km003c-rs/target/release;
  - raw vendor-interface fuzz bursts over pyusb: random bytes and lengths
    (including zero-length and multi-packet OUT writes), known commands with
    random tid/attribute/payload, random GetData masks, StartGraph/StopGraph
    at random, MemoryRead/StreamingAuth with random payloads;
  - host USB resets (USBDEVFS_RESET).
A background thread meanwhile polls the CDC port with read-only verbs
(query/versn/getva/rawva/pdsta) whenever ttyACM is bound.

After every step: the device is enumerated, and a Connect + GetData(ADC) on
the vendor interface returns a well-formed 52-B reply. Every few steps the
CDC port is re-bound (km003c-rs detaches cdc_acm and never rebinds it) and
the boot run count is compared — a change means the meter rebooted.
Failures are logged and counted, the run continues. Never sends anything
that can change OUT voltage (no CDC trigger verbs; the vendor protocol has
none). Re-binding needs passwordless `sudo tee` on the sysfs bind files.
"""
import argparse
import fcntl
import glob
import json
import os
import random
import re
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../lib"))
import serial  # noqa: E402
import usb.core  # noqa: E402
import usb.util  # noqa: E402
from tqdm import tqdm  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402

VID, PID = 0x5FC9, 0x0063
EP_OUT, EP_IN = 0x01, 0x81
CMDS = [0x02, 0x03, 0x05, 0x06, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x41,
        0x44, 0x48, 0x4A, 0x4B, 0x4C, 0x4D]


def usb_path():
    for d in glob.glob("/sys/bus/usb/devices/*"):
        try:
            if (open(d + "/idVendor").read().strip() == "%04x" % VID
                    and open(d + "/idProduct").read().strip() == "%04x" % PID):
                return os.path.basename(d), d
        except OSError:
            pass
    return None, None


def bind(driver, iface):
    name, _ = usb_path()
    if not name:
        return False
    node = "/sys/bus/usb/devices/%s:1.%d/driver" % (name, iface)
    if os.path.exists(node):
        return True
    r = subprocess.run(["sudo", "-n", "tee", "/sys/bus/usb/drivers/%s/bind" % driver],
                       input="%s:1.%d" % (name, iface), capture_output=True, text=True)
    return r.returncode == 0


def usb_reset():
    _, d = usb_path()
    if not d:
        return "not enumerated"
    bus, dev = int(open(d + "/busnum").read()), int(open(d + "/devnum").read())
    fd = os.open("/dev/bus/usb/%03d/%03d" % (bus, dev), os.O_WRONLY)
    try:
        fcntl.ioctl(fd, ord("U") << 8 | 20)
        return "ok"
    except OSError as e:
        return "ioctl: %s" % e
    finally:
        os.close(fd)


class Vendor:
    """Claimed vendor interface 0 (kernel powerz driver detached)."""

    def __enter__(self):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            raise RuntimeError("not enumerated")
        try:
            if self.dev.is_kernel_driver_active(0):
                self.dev.detach_kernel_driver(0)
        except usb.core.USBError:
            pass
        usb.util.claim_interface(self.dev, 0)
        return self

    def __exit__(self, *a):
        try:
            usb.util.release_interface(self.dev, 0)
            usb.util.dispose_resources(self.dev)
        except usb.core.USBError:
            pass

    def write(self, data):
        self.dev.write(EP_OUT, data, timeout=500)

    def read(self, timeout=100):
        try:
            return bytes(self.dev.read(EP_IN, 4096, timeout=timeout))
        except usb.core.USBTimeoutError:
            return None

    def drain(self, n=20):
        got = 0
        while got < n and self.read(30) is not None:
            got += 1
        return got


def vendor_health():
    """Connect + GetData(ADC) -> a valid 52-B ADC reply. Returns error or None."""
    try:
        with Vendor() as v:
            v.drain()
            v.write(bytes([0x02, 0x7E, 0, 0]))
            ok_connect = False
            for _ in range(5):
                r = v.read(500)
                if r is None:
                    break
                if r[:2] == bytes([0x05, 0x7E]):
                    ok_connect = True
                    break
            if not ok_connect:
                return "Connect not accepted"
            v.write(bytes([0x0C, 0x7F, 0x02, 0x00]))
            for _ in range(5):
                r = v.read(500)
                if r is None:
                    return "GetData ADC: no reply"
                if r[0] == 0x41 and r[1] == 0x7F:
                    ext = int.from_bytes(r[4:8], "little")
                    if len(r) != 52 or ext & 0x7FFF != 1 or ext >> 22 != 44:
                        return "GetData ADC malformed: %s" % r[:8].hex()
                    v.write(bytes([0x0F, 0x10, 0, 0]))  # leave no stream running
                    v.read(300)
                    v.write(bytes([0x03, 0x11, 0, 0]))
                    v.read(300)
                    return None
            return "GetData ADC: reply never matched"
    except (usb.core.USBError, RuntimeError, ValueError) as e:
        return "vendor: %r" % e


def cdc_runcount():
    """Re-bind cdc_acm, then (versn, runcount) over CDC; None on failure."""
    bind("cdc_acm", 1)
    for _ in range(20):
        c = meter_candidates()
        if c:
            break
        time.sleep(0.25)
    else:
        return None
    r = subprocess.run(["uv", "run", "-q", os.path.join(HERE, "../cfg.py"), "get", "runcount"],
                       capture_output=True, text=True, timeout=30)
    try:
        s = serial.Serial(c[0], 115200, timeout=1)
        s.reset_input_buffer()
        s.write(b"versn")
        v = s.read(8)
        s.close()
    except (serial.SerialException, OSError):
        return None
    m = re.search(r"(\d+)", r.stdout)
    return v.decode(errors="replace"), int(m.group(1)) if m else None


class CdcHammer(threading.Thread):
    """Read-only CDC verbs in a loop whenever ttyACM is present."""
    # (verb, reply length); versn is "OK" + the ASCII build id, so its
    # length varies — it is read up to 8 B and checked by prefix
    VERBS = [(b"query", 4), (b"versn", 0), (b"getva", 192), (b"rawva", 46), (b"pdsta", 24)]

    def __init__(self):
        super().__init__(daemon=True)
        self.stop = False
        self.paused = False  # set while the main thread uses the port itself
        self.ok = 0
        self.short = 0
        self.lock = threading.Lock()

    def run(self):
        while not self.stop:
            c = meter_candidates()
            if self.paused or not c:
                time.sleep(0.5)
                continue
            try:
                s = serial.Serial(c[0], 115200, timeout=0.5)
                for _ in range(200):
                    if self.stop or self.paused:
                        break
                    verb, n = random.choice(self.VERBS)
                    s.reset_input_buffer()
                    s.write(verb)
                    r = s.read(n or 8)
                    with self.lock:
                        if len(r) == n or (n == 0 and r[:2] == b"OK" and len(r) > 2):
                            self.ok += 1
                        else:
                            self.short += 1
                s.close()
            except (serial.SerialException, OSError):
                time.sleep(0.5)


def run_tool(bindir, args, timeout):
    t0 = time.time()
    try:
        r = subprocess.run([os.path.join(bindir, args[0])] + args[1:], capture_output=True,
                           text=True, timeout=timeout)
        out, code = r.stdout + r.stderr, r.returncode
    except subprocess.TimeoutExpired as e:
        out, code = (e.stdout or b"").decode(errors="replace") if isinstance(e.stdout, bytes) else (e.stdout or ""), "timeout"
    return code, out, time.time() - t0


def step_tool(bindir):
    choice = random.choice(["adc_simple", "adc_queue", "adc_queue", "adc_queue", "pd_trace",
                            "test_usbpd", "memory_scan", "memory_scan_full", "offline"])
    reset = [] if random.random() < 0.7 else ["--no-reset"]
    if choice == "adc_simple":
        args, to = ["adc_simple"] + reset, 30
    elif choice == "adc_queue":
        rate = random.choice([2, 10, 50, 1000, 1000])
        dur = random.randint(2, 20)
        args, to = ["adc_queue_simple", "-r", str(rate), "-d", str(dur)] + reset, dur + 40
    elif choice == "pd_trace":
        args, to = ["pd_trace", "-p", str(random.randint(1, 15)),
                    "-i", str(random.randint(50, 500))] + reset, 60
    elif choice == "test_usbpd":
        d = random.randint(2, 10)
        args, to = ["test_usbpd", "-d", str(d)] + reset, d + 40
    elif choice == "memory_scan":
        addr = random.choice([0x40010450, 0x420, 0x4420, 0x3000C00, random.getrandbits(32)])
        args, to = ["memory_scan", "--address", hex(addr),
                    "--size", str(random.choice([1, 12, 16, 64, 100]))] + reset, 60
    elif choice == "memory_scan_full":
        args, to = ["memory_scan", "--full-scan"] + reset, 180
    else:
        args, to = ["offline-log", "metadata"] + reset, 60
    code, out, dt = run_tool(bindir, args, to)
    detail = {"tool": " ".join(args), "exit": code, "secs": round(dt, 1)}
    err = None
    if code == "timeout":
        err = "tool timed out"
    if args[0] == "adc_queue_simple":
        m = re.search(r"Missing samples: (\d+)", out)
        t = re.search(r"Total samples: (\d+)", out)
        detail["samples"] = int(t.group(1)) if t else None
        detail["missing"] = int(m.group(1)) if m else None
        if code != 0 or m is None:
            err = err or "adc_queue_simple failed (exit %s)" % code
        elif int(m.group(1)) > 0:
            err = "adc_queue_simple: %s missing samples" % m.group(1)
    elif args[0] in ("adc_simple",) and code != 0:
        err = err or "adc_simple exit %s" % code
    if err:
        detail["tail"] = out[-600:]
    return choice, detail, err


def step_raw():
    """One raw fuzz burst; the meter's replies are drained, not judged."""
    n_sent = 0
    with Vendor() as v:
        for _ in range(random.randint(20, 200)):
            kind = random.random()
            if kind < 0.25:
                pkt = os.urandom(random.randint(0, 64))
            elif kind < 0.35:
                pkt = os.urandom(random.choice([65, 100, 128, 200, 512]))
            elif kind < 0.65:
                cmd = random.choice(CMDS) | (0x80 if random.random() < 0.1 else 0)
                pkt = bytes([cmd, random.getrandbits(8)]) + random.getrandbits(16).to_bytes(2, "little")
                pkt += os.urandom(random.choice([0, 0, 0, 4, 16, 32, 60]))
            elif kind < 0.85:
                mask = 0
                for b in random.sample(range(11), random.randint(1, 4)):
                    mask |= 1 << b
                pkt = bytes([0x0C, random.getrandbits(8)]) + (mask << 1).to_bytes(2, "little")
            elif kind < 0.92:
                pkt = bytes([random.choice([0x0E, 0x0F]), random.getrandbits(8),
                             random.choice([0, 2, 4, 6, 8, 0xFE]), random.choice([0, 0, 1])])
            else:
                pkt = bytes([random.choice([0x44, 0x4C]), random.getrandbits(8)]) + \
                    random.choice([b"\x01\x01", b"\x00\x02", os.urandom(2)]) + os.urandom(32)
            try:
                v.write(pkt)
                n_sent += 1
            except usb.core.USBError:
                pass
            if random.random() < 0.5:
                v.drain(random.randint(0, 3))
            if random.random() < 0.05:
                time.sleep(random.uniform(0, 0.3))
        v.drain(50)
    return {"raw_packets": n_sent}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("minutes", type=float, nargs="?", default=60.0, help="run length")
    ap.add_argument("--km003c-rs", default=os.path.expanduser("~/km003c-rs"),
                    help="km003c-rs checkout with built release CLIs")
    ap.add_argument("--log", default="km003c_fuzz.jsonl", help="JSON-lines step log")
    ap.add_argument("--seed", type=int, default=None, help="random seed (default: time)")
    ap.add_argument("--cdc-every", type=int, default=5, help="steps between CDC/runcount checks")
    args = ap.parse_args()
    seed = args.seed if args.seed is not None else int(time.time())
    random.seed(seed)
    bindir = os.path.join(args.km003c_rs, "target/release")

    base = cdc_runcount()  # before the hammer thread starts
    if base is None:
        sys.exit("no CDC answer at start")
    print("start: %s, runcount %s, seed %d" % (base[0], base[1], seed))
    hammer = CdcHammer()
    hammer.start()
    stats = {"steps": 0, "failures": 0, "reboots": 0}
    kinds = {}
    end = time.time() + args.minutes * 60
    log = open(args.log, "a")
    with tqdm(total=int(args.minutes * 60), unit="s", bar_format="{l_bar}{bar}| {n}/{total} s {postfix}") as bar:
        t_start = time.time()
        while time.time() < end:
            r = random.random()
            err = None
            try:
                if r < 0.55:
                    kind, detail, err = step_tool(bindir)
                elif r < 0.9:
                    kind, detail = "raw", step_raw()
                else:
                    kind, detail = "usb_reset", {"result": usb_reset()}
                    time.sleep(1)
            except Exception as e:  # keep going: a failed step is data
                kind, detail, err = "error", {}, "step raised %r" % e
            time.sleep(0.3)
            h = vendor_health()
            if h:
                time.sleep(3)
                h2 = vendor_health()
                detail["health"] = h
                if h2:
                    usb_reset()
                    time.sleep(3)
                    h3 = vendor_health()
                    detail["health_after_reset"] = h3
                    err = err or ("vendor health: %s" % h if h3 else "vendor health needed a USB reset: %s" % h)
            stats["steps"] += 1
            if stats["steps"] % args.cdc_every == 0:
                hammer.paused = True
                time.sleep(0.6)
                rc = cdc_runcount()
                hammer.paused = False
                bind("powerz", 0)
                if rc is None:
                    err = err or "CDC check failed"
                else:
                    detail["versn"], detail["runcount"] = rc
                    if rc[1] is not None and base[1] is not None and rc[1] != base[1]:
                        stats["reboots"] += 1
                        err = err or "meter rebooted (runcount %s -> %s)" % (base[1], rc[1])
                        base = rc
            kinds[kind] = kinds.get(kind, 0) + 1
            if err:
                stats["failures"] += 1
                tqdm.write("FAIL step %d %s: %s" % (stats["steps"], kind, err))
            log.write(json.dumps({"t": round(time.time() - t_start, 1), "step": stats["steps"],
                                  "kind": kind, "err": err, **detail}) + "\n")
            log.flush()
            bar.n = min(int(time.time() - t_start), bar.total)
            bar.set_postfix_str("steps %d fail %d cdc %d/%d" % (
                stats["steps"], stats["failures"], hammer.ok, hammer.short))
            bar.refresh()
    hammer.stop = True
    hammer.join(timeout=5)
    bind("cdc_acm", 1)
    bind("powerz", 0)
    print("done: %d steps (%s), %d failures, %d reboots; CDC hammer %d ok / %d short replies"
          % (stats["steps"], ", ".join("%s %d" % kv for kv in sorted(kinds.items())),
             stats["failures"], stats["reboots"], hammer.ok, hammer.short))
    sys.exit(1 if stats["failures"] else 0)


if __name__ == "__main__":
    main()
