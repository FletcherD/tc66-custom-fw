# TC66 Rust firmware — build + flash. Mirrors reference-c/Makefile's flow.
# Normal loop:  make flash      (hands-free via the running app's `updat`)
# Recovery:     hold K1 while plugging in, then  make recover

ELF = target/thumbv7m-none-eabi/release/tc66-fw

all: tc66fw.pad.bin

build:
	cargo build --release
	arm-none-eabi-size $(ELF)

tc66fw.bin: build
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

# Non-halting SRAM marker read (see src/markers.rs for the map)
markers:
	openocd -f ../reference-c/openocd.cfg -c "mdw 0x20001000 26" -c shutdown

clean:
	cargo clean
	rm -f tc66fw.bin tc66fw.pad.bin tc66fw.enc.bin

.PHONY: all build flash recover markers clean
