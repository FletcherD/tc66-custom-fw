#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Stream samples from the custom firmware's plaintext `rawva` verb.

rawva reply (46 B, layout in rust/src/getva.rs): "raw1" magic, then LE u32
seq, v_1e4, i_1e5, p_1e4, r_1e1, mah, mwh, temp_c (i32), dp_1e2, dm_1e2,
CRC16/MODBUS of bytes 0..44 (LE u16 at 44).

Poll at >=2x the ~10.4 Hz sampler window rate and keep replies only when
`seq` advances: every window exactly once, a seq jump = a missed window.

    tools/rawva.py            # stream to stdout until ^C
    tools/rawva.py 10         # stream for 10 seconds
"""
import sys, time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)) + "/lib")
import serial  # noqa: E402
from tc66lib import meter_candidates  # noqa: E402
from rawva_proto import rawva  # noqa: E402


def open_meter():
    """Return an open Serial on the meter in app mode, or None."""
    for c in meter_candidates():
        try:
            s = serial.Serial(c, 115200, timeout=1.0)
            s.reset_input_buffer()
            s.write(b"query")
            if s.read(4) == b"firm":
                return s
            s.close()
        except Exception:
            pass
    return None


def stream(s, duration=None, poll_hz=25):
    """Yield each sampler window exactly once (deduped by seq)."""
    last = None
    end = None if duration is None else time.time() + duration
    while end is None or time.time() < end:
        t0 = time.time()
        smp = rawva(s)
        if smp["seq"] != last:
            if last is not None and smp["seq"] - last > 1:
                print(f"# missed {smp['seq'] - last - 1} window(s)",
                      file=sys.stderr)
            last = smp["seq"]
            yield smp
        time.sleep(max(0.0, 1 / poll_hz - (time.time() - t0)))


if __name__ == "__main__":
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else None
    s = open_meter()
    if s is None:
        sys.exit("no meter in app mode")
    try:
        for smp in stream(s, dur):
            print(f"{smp['seq']:8d}  {smp['V']:8.4f} V  {smp['A']:8.5f} A  "
                  f"{smp['W']:8.4f} W  {smp['mAh']:6d} mAh  {smp['degC']:3d} C")
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
