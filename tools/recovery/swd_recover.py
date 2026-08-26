#!/usr/bin/env python3
"""SWD recovery for a crashed/locked-up TC66 app WITHOUT a hub power-cycle.

Halt escapes lockup (DHCSR C_HALT, no reset -> no QSPI cache desync). An SRAM
stub then CALLS the bootloader's flash-resident routines (RDP2 only permits
flash erase/program from flash-resident code) to erase+program the app region.
Per-step retry rides out intermittent fetch faults on a marginal cache; on a
healthy (good-cache) meter it completes first try.

Only the app region 0x08002c00..0x0800fc00 is touched.

This version holds ONE persistent openocd over its Tcl-RPC socket (fast, no
per-batch spawn) and prints live progress. Retry counts are visible, so a noisy
run immediately tells you the cache is marginal.

Usage: swd_recover.py <image.bin> [--run]
"""
import socket, subprocess, sys, struct, re, time, os

ERASE_STUB, PROG_STUB = 0x20001000, 0x20001080
ERASE_MARK, PROG_MARK = 0x20001100, 0x20001108
BUF, MSP, APP_BASE, BATCH = 0x20001200, 0x20001e00, 0x08002c00, 256
ERASE_OK, PROG_OK = 0xE5A5EDED, 0x0D09EEEE
HALT_V, RESUME_V = 0xA05F0003, 0xA05F0001
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))


class OCD:
    """Persistent openocd + Tcl-RPC (port 6666). Raw mww/mdw only — openocd's
    halt/reset/load_image lock up this RDP core."""
    def __init__(self):
        self.p = subprocess.Popen(
            ["openocd", "-f", "openocd.cfg"], cwd=ROOT,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(80):
            try:
                self.s = socket.create_connection(("127.0.0.1", 6666), timeout=2)
                break
            except OSError:
                if self.p.poll() is not None:
                    raise RuntimeError("openocd exited before Tcl-RPC came up")
                time.sleep(0.1)
        else:
            raise RuntimeError("openocd Tcl-RPC (6666) never came up")
        self.s.settimeout(15)

    def cmd(self, c):
        self.s.sendall(c.encode() + b"\x1a")
        buf = b""
        while not buf.endswith(b"\x1a"):
            d = self.s.recv(8192)
            if not d:
                break
            buf += d
        return buf[:-1].decode(errors="replace")

    def mww(self, addr, val):
        self.cmd(f"mww 0x{addr:08x} 0x{val & 0xffffffff:08x}")

    def load(self, addr, words):
        # one RPC round-trip for the whole block (Tcl ';' separates statements)
        self.cmd(";".join(f"mww 0x{addr + i*4:08x} 0x{w & 0xffffffff:08x}"
                          for i, w in enumerate(words)))

    def mdw(self, addr, n=1):
        out = self.cmd(f"mdw 0x{addr:08x} {n}")
        vals, want = [], addr
        for a, chunk in re.findall(r"0x([0-9a-f]{8}):\s+([0-9a-f ]+)", out):
            vals += [int(x, 16) for x in chunk.split()]
        return vals

    def close(self):
        try:
            self.cmd("shutdown")
        except Exception:
            pass
        try:
            self.p.wait(timeout=5)
        except Exception:
            self.p.kill()


def binwords(path):
    d = open(path, "rb").read()
    if len(d) % 4:
        d += b"\x00" * (4 - len(d) % 4)
    return list(struct.unpack("<%dI" % (len(d) // 4), d))


def halt(o):
    o.mww(0xE000EDF0, HALT_V)
    o.mww(0xE000EDF0, HALT_V)


def setreg(o, rn, v):
    o.mww(0xE000EDF8, v)          # DCRDR = value
    o.mww(0xE000EDF4, 0x10000 | rn)  # DCRSR = write | regnum


def run_stub(o, pc, regs, mark, ok_val, name, tries=25, timeout_s=6.0):
    """Halt, arm stub at pc with regs, resume, poll mark until the stub writes
    (mark, status). Success = (ok_val, 0). Returns True/False; prints retries."""
    for t in range(tries):
        halt(o)
        o.mww(mark, 0)
        setreg(o, 17, MSP)          # MSP
        for rn, v in regs:
            setreg(o, rn, v)
        setreg(o, 15, pc)           # PC
        setreg(o, 16, 0x01000000)   # xPSR (thumb)
        o.mww(0xE000EDF0, RESUME_V)
        end = time.time() + timeout_s
        v = []
        while time.time() < end:
            v = o.mdw(mark, 2)
            if len(v) >= 2 and v[0] != 0:
                break
            time.sleep(0.01)
        halt(o)
        v = o.mdw(mark, 2)
        if len(v) >= 2 and v[0] == ok_val and v[1] == 0:
            if t:
                print(f"    {name}: ok after {t+1} tries", flush=True)
            return True
        print(f"    {name}: try {t+1} -> {[hex(x) for x in v]} (retry)", flush=True)
    print(f"    {name}: FAILED after {tries}", flush=True)
    return False


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    img = sys.argv[1]
    do_run = "--run" in sys.argv
    iw = binwords(img)
    while len(iw) % BATCH:
        iw.append(0xFFFFFFFF)
    nb = len(iw) // BATCH
    estub = binwords(os.path.join(HERE, "erase_stub.bin"))
    pstub = binwords(os.path.join(HERE, "prog_stub.bin"))
    print(f"{img}: {len(iw)} words, {nb} batches of {BATCH}", flush=True)

    t0 = time.time()
    o = OCD()
    try:
        dh = o.mdw(0xE000EDF0)
        print(f"  DHCSR={dh[0]:#010x} "
              f"({'LOCKUP' if dh and dh[0] & 0x80000 else 'running/halted'})", flush=True)

        # erase (stub loaded fresh each attempt inside run_stub? no — load once here)
        halt(o)
        o.load(ERASE_STUB, estub)
        print("  erasing app region...", flush=True)
        if not run_stub(o, ERASE_STUB, [], ERASE_MARK, ERASE_OK, "erase",
                        timeout_s=8.0):
            sys.exit(1)
        print(f"  erase OK ({time.time()-t0:.1f}s)", flush=True)

        # program stub persists in SRAM; reload once
        halt(o)
        o.load(PROG_STUB, pstub)
        for b in range(nb):
            batch = iw[b*BATCH:(b+1)*BATCH]
            fa = APP_BASE + b*BATCH*4
            halt(o)
            o.load(BUF, batch)
            o.load(PROG_STUB, pstub)
            regs = [(0, fa), (1, BUF), (2, BATCH)]
            if not run_stub(o, PROG_STUB, regs, PROG_MARK, PROG_OK,
                            f"batch {b}", timeout_s=4.0):
                print(f"  batch {b} @0x{fa:08x} FAILED", flush=True)
                sys.exit(1)
            if b % 8 == 0 or b == nb-1:
                print(f"  batch {b+1}/{nb} @0x{fa:08x} OK ({time.time()-t0:.1f}s)",
                      flush=True)
        print(f"REFLASH COMPLETE — all {nb} batches STAT=0 ({time.time()-t0:.1f}s)",
              flush=True)

        if do_run:
            sp, rst = struct.unpack("<II", open(img, "rb").read(8))
            halt(o)
            setreg(o, 17, sp)   # MSP
            setreg(o, 13, sp)   # SP
            setreg(o, 15, rst & ~1)
            setreg(o, 16, 0x01000000)
            o.mww(0xE000ED08, APP_BASE)   # VTOR
            o.mww(0xE000EDF0, RESUME_V)
            print(f"redirected to new app (SP=0x{sp:08x} PC=0x{rst:08x})", flush=True)
    finally:
        o.close()


if __name__ == "__main__":
    main()
