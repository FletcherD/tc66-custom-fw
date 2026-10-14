#!/usr/bin/env python3
"""Flash a TC66 app image over the RUNNING APP's new 'updat' USB command — no K1,
no bootloader entry. Requires an app built with the 'updat' self-update command support (this
path). Sends a DECRYPTED (plaintext) image; the SRAM stub programs
the bytes directly, so pass a plaintext image (e.g. app.bin).

  python3 app_update.py app.bin

Protocol (mirrors the bootloader 'update' path):
  PC: query            -> app replies "firm"     (confirm we're on the app)
  PC: updat            -> stub erases 52 pages, replies "uprdy"
  PC: <64-B chunk> ... -> stub programs 16 words, replies "OK" per chunk
  (after the last chunk the stub triggers a system reset -> new app)

Uses the same fast non-blocking expect() as flash_image.py so ModemManager can't
grab an idle window. Do NOT open the meter port from anything else while this runs.
If it fails mid-stream the app region is partially written; recover by holding K1,
cold-booting into the bootloader, and re-flashing a known-good image over the bootloader.
"""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../lib"))
from tc66lib import meter_candidates, expect, _safe_close
import serial


def find_app(baud=115200):
    """Open the meter iff it answers `query` with `firm` (running app), else None."""
    for port in meter_candidates():
        s = None
        try:
            s = serial.Serial(port, baud, timeout=1.0)
            s.reset_input_buffer()
            s.write(b"query")
            if s.read(64).decode("latin1", "replace").startswith("firm"):
                s.timeout = 0
                return s
            _safe_close(s)
        except Exception:
            _safe_close(s)
    return None


def main():
    if len(sys.argv) < 2:
        print(__doc__); return 2
    img = sys.argv[1]
    data = open(img, "rb").read()
    total = (len(data) + 63) // 64
    print(f"{img}: {len(data)} B, {total} chunks", flush=True)

    ser = find_app()
    if ser is None:
        print("!! running app not found (query->firm). Is the meter booted into the app?")
        return 2
    print(f"app on {ser.port}", flush=True)

    ser.reset_input_buffer()
    ser.write(b"updat")
    # wait-for-uprdy covers the on-device 52-page erase (~1-2 s)
    r = expect(ser, "uprdy", cap=8.0)
    print(f"stub armed ({r!r}); erased, streaming...", flush=True)

    t0 = time.time()
    for i in range(total):
        chunk = data[i * 64:i * 64 + 64]
        if len(chunk) < 64:
            chunk = chunk + b"\x00" * (64 - len(chunk))
        tc = time.time()
        ser.write(chunk)
        expect(ser, "OK", cap=3.0)
        if i < 5 or i % 100 == 0 or i == total - 1:
            print(f"  {i+1}/{total}  chunk={(time.time()-tc)*1000:.0f}ms  total={time.time()-t0:.1f}s",
                  flush=True)
    print(f"UPDATE COMPLETE in {time.time()-t0:.1f}s. Meter will reset into the new app.",
          flush=True)
    _safe_close(ser)
    return 0


if __name__ == "__main__":
    sys.exit(main())
