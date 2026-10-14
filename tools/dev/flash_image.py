#!/usr/bin/env python3
"""Flash an arbitrary TC66 image over the bootloader, using the robust fast-read
path from flash_recover.py (non-blocking expect, ~ms/chunk) so ModemManager can't
slip into an idle window. Meter must already be in the bootloader (hold K1, cold-boot).

  python3 flash_image.py app.bin

Do NOT open the meter port from anything else while this runs.
"""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../lib"))
from tc66lib import find_bootloader, expect

def main():
    img = sys.argv[1]
    data = open(img, "rb").read()
    total = (len(data) + 63) // 64
    print(f"{img}: {len(data)} B, {total} chunks", flush=True)
    ser = find_bootloader()
    if ser is None:
        print("!! bootloader not found (hold K1, cold-boot, retry)"); return 2
    print(f"bootloader on {ser.port}", flush=True)
    ser.reset_input_buffer()
    ser.write(b"update"); expect(ser, "uprdy")
    print("flashing...", flush=True)
    t0 = time.time()
    for i in range(total):
        tc = time.time()
        ser.write(data[i*64:i*64+64])
        expect(ser, "OK")
        if i < 5 or i % 100 == 0 or i == total-1:
            print(f"  {i+1}/{total}  chunk={(time.time()-tc)*1000:.0f}ms  total={time.time()-t0:.1f}s", flush=True)
    print(f"FLASH COMPLETE in {time.time()-t0:.1f}s. Meter will restart.", flush=True)
    ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
