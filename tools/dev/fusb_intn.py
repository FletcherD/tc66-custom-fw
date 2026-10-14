#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = ["pyserial"]
# ///
"""Locate/verify the FUSB302 INT_N pin: unmask the chip's interrupts, force
I_COMP_CHNG by moving the CC1 MDAC threshold across the CC1 voltage, and
print the GPIOA/B/F inputs (gpird) after each step. INT_N = the pin that goes
low on each forced interrupt and back high after the read-clear (PA15).
Needs a dev build with `dev-board` and a Type-C source on IN (Rp on CC1);
ends with pdctl 0.
"""
import serial, glob, struct, time
s=serial.Serial(glob.glob('/dev/ttyACM*')[0],115200,timeout=1)
def x(c,n): s.reset_input_buffer(); s.write(c); return s.read(n)
def wr(reg,val): r=x(b'i2crd'+bytes([0x22,2,reg,val,0]),2); assert r==b'OK',r
def rd(reg): r=x(b'i2crd'+bytes([0x22,1,reg,0,1]),3); assert r[:2]==b'OK',r; return r[2]
prev=None
def g(label):
    global prev
    r=x(b'gpird',8); a,b,f=struct.unpack('<3H',r[2:8])
    diff='' if prev is None else f"  changed A:{a^prev[0]:#06x} B:{b^prev[1]:#06x} F:{f^prev[2]:#06x}"
    print(f"{label:34s} PA15={a>>15&1}  A={a:#06x} B={b:#06x} F={f:#06x}{diff}")
    prev=(a,b,f)
print(x(b'pdctl\x00',2))
print('status0', hex(rd(0x40)))
wr(0x0B,0x0F)                       # Power: all blocks on
wr(0x02,0x07)                       # Switches0: Rd both + MEAS_CC1
g('reset (Control0 INT_MASK=1)')
for m in (0x0A,0x0E,0x0F): wr(m,0x00)
wr(0x06,0x04)                       # Control0: INT_MASK=0
g('unmasked, pending?')
ints=[rd(r) for r in (0x3E,0x3F,0x42)]; g(f'read-cleared ints {[hex(i) for i in ints]}')
for k in range(3):
    wr(0x04,0x00); time.sleep(0.01); wr(0x04,0x3F); time.sleep(0.01)  # CC1 MDAC thresholds 42 mV / 2.7 V
    g(f'after COMP toggle #{k+1}')
    ints=[rd(r) for r in (0x3E,0x3F,0x42)]; g(f'read-cleared ints {[hex(i) for i in ints]}')
wr(0x04,0x00); time.sleep(0.01); wr(0x04,0x3F); time.sleep(0.01)
g('toggled again (pending)')
wr(0x06,0x24); g('Control0 INT_MASK=1 again')
print(x(b'pdctl\x00',2))
