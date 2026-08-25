# TC66 Rust firmware — build + flash. Mirrors reference-c/Makefile's flow.
# Normal loop:  make flash      (hands-free via the running app's `updat`)
# Recovery:     hold K1 while plugging in, then  make recover

ELF = target/thumbv7m-none-eabi/release/tc66-fw

all: tc66fw.pad.bin

build:
	cargo build --release
	arm-none-eabi-size $(ELF)

# cortex-m-rt's Reset copies __sidata -> __sdata..__edata and zeroes
# __sbss..__ebss assuming contiguous VMAs inside RAM; a linker-script mistake
# (e.g. INSERT AFTER .data pushing __edata to a disjoint address) turns that
# into a boot-time RAM smear. Pin all four inside the real RAM region.
init-gate: build
	@arm-none-eabi-nm $(ELF) | python3 -c "import sys; \
syms = {n: int(a,16) for a,_,n in (l.split() for l in sys.stdin) if n in ('__sdata','__edata','__sbss','__ebss')}; \
bad = {n: hex(v) for n,v in syms.items() if not 0x20000000 <= v <= 0x20000E40}; \
assert len(syms)==4 and not bad, 'init symbols outside RAM: %s' % (bad or syms); \
print('init-gate ok: data/bss init bounded to %s..%s' % (hex(min(syms.values())), hex(max(syms.values()))))"

tc66fw.bin: build flasher-gate init-gate
	arm-none-eabi-objcopy -O binary $(ELF) tc66fw.bin
	@python3 -c "import struct; d = open('tc66fw.bin','rb').read(); \
sp, rst = struct.unpack('<II', d[:8]); \
assert sp == 0x20000e40, 'bad initial SP: ' + hex(sp); \
assert 0x08002c00 < rst < 0x0800fc00 and rst & 1, 'bad reset vector: ' + hex(rst); \
assert len(d) <= 51840, 'image too big: %d' % len(d); \
print('header ok: sp=%#x reset=%#x size=%d' % (sp, rst, len(d)))"

# 51840 B = 810 x 64-B chunks: the updat flasher streams exactly this much.
tc66fw.pad.bin: tc66fw.bin
	python3 -c "d = open('tc66fw.bin','rb').read(); \
open('tc66fw.pad.bin','wb').write(d + b'\x00' * (51840 - len(d)))"

flash: tc66fw.pad.bin
	python3 ../tools/app_update.py tc66fw.pad.bin

# Bootloader recovery path (AES image; hold K1 while plugging in first)
tc66fw.enc.bin: tc66fw.bin
	python3 ../tools/encrypt.py tc66fw.bin tc66fw.enc.bin
recover: tc66fw.enc.bin
	python3 ../tools/flash_image.py tc66fw.enc.bin

# The .flasher section erases app flash while running from SRAM: prove it never
# references app flash (literal pools) and makes no PC-relative calls out of the
# section (bootloader routines must be blx-reg only).
flasher-gate: build
	@arm-none-eabi-objdump -d -j .flasher $(ELF) | python3 -c "import re,sys; \
dis = sys.stdin.read(); \
bad  = [hex(v) for v in (int(m,16) for m in re.findall(r'\.word\s+0x([0-9a-f]{8})', dis)) if 0x08002c00 <= v < 0x08010000]; \
bad += [t for t in re.findall(r'\b(?:bl|b\.w|blx)\s+([0-9a-f]+)\b', dis) if not 0x20001400 <= int(t,16) < 0x20001c00]; \
assert not bad, 'flasher references app flash: %s' % bad; \
print('flasher-gate ok: .flasher is self-contained')"

# Non-halting SRAM marker read (see src/markers.rs for the map)
markers:
	openocd -f ../reference-c/openocd.cfg -c "mdw 0x20001000 26" -c shutdown

clean:
	cargo clean
	rm -f tc66fw.bin tc66fw.pad.bin tc66fw.enc.bin

.PHONY: all build flasher-gate init-gate flash recover markers clean
