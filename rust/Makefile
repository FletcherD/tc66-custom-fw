# TC66 Rust firmware — build + flash.
# Dev loop:   make flash      (dev build, hands-free via the running app's `updat`)
# Release:    make release    (product build: no dev verbs -> tc66fw-release.enc.bin)
# Recovery:   hold K1 while plugging in, then  make recover
# CI:         make gates      (build + every gate, dev and release feature sets)

ELF = target/thumbv7m-none-eabi/release/tc66-fw

# The dev loop adds the dev groups in current use + the PC profiler to the
# default (release) feature set; RELEASE=1 builds the defaults only. Flash is
# tight — swap groups here (Cargo.toml lists them: fcp dev-board dev-bt
# dev-tft, or dev-verbs for all). Extra features stack on either:
# make flash FEATURES=grace
DEV_FEATURES = pcprf
CARGO_FEATURES = $(strip $(if $(RELEASE),,$(DEV_FEATURES)) $(FEATURES))

all: tc66fw.pad.bin

build:
	cargo build --release $(if $(CARGO_FEATURES),--features "$(CARGO_FEATURES)")
	arm-none-eabi-size $(ELF)

init-gate: build
	@arm-none-eabi-nm $(ELF) | python3 -c "import sys; \
syms = {n: int(a,16) for a,_,n in (l.split() for l in sys.stdin) if n in ('__sdata','__edata','__sbss','__ebss')}; \
bad = {n: hex(v) for n,v in syms.items() if not 0x20000000 <= v <= 0x20000E40}; \
assert len(syms)==4 and not bad, 'init symbols outside RAM: %s' % (bad or syms); \
print('init-gate ok: data/bss init bounded to %s..%s' % (hex(min(syms.values())), hex(max(syms.values()))))"

# Static worst-case stack depth vs the fenced RAM budget. Build 35 shipped a
# silent boot-path stack overflow (hardfault before the D+ pull-up = invisible
# dead board, K1 recovery); this catches that class at build time.
stack-gate: build
	@python3 ../tools/stack_gate.py $(ELF)

# Hot path must sit below 0x08008000 (zero-wait flash — QUIRKS §4); fast.x
# places it, this proves it and reports the region budget.
fast-gate: build
	@python3 ../tools/fast_gate.py $(ELF)

tc66fw.bin: build flasher-gate init-gate stack-gate fast-gate
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

# Release image: the product feature set, AES-encrypted for the stock
# bootloader (the format the vendor's own firmware images use). Recursive so
# the build/gates run with RELEASE=1.
release:
	$(MAKE) RELEASE=1 tc66fw.enc.bin
	cp tc66fw.enc.bin tc66fw-release.enc.bin
	@echo "release image: rust/tc66fw-release.enc.bin"

# Every gate on both feature sets (CI runs this on each push), plus a
# compile check of every dev group (all together may not fit the image)
gates: host-check
	$(MAKE) tc66fw.bin
	$(MAKE) RELEASE=1 tc66fw.bin
	cargo check --release --features "dev-verbs pcprf grace"

# Host-side proofs of the pure modules: src/ntc.rs bit-identical to the
# stock f32 temperature model over all inputs, src/aes256.rs vs FIPS-197 and
# the RustCrypto aes crate, src/pdinfo.rs replaying a PD negotiation, every
# pdinfo/detect screen line vs tools/host_check/text.golden (own dir:
# rust/.cargo's thumb target must not apply)
host-check:
	cd ../tools/host_check && cargo run -q --release

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

# Non-halting SRAM marker read (see src/markers.rs for the map,
# incl. the per-subsystem diagnostic blocks at 0x1070/0x10a0/0x10d0)
markers:
	openocd -f ../openocd.cfg -c "mdw 0x20001000 64" -c shutdown

clean:
	cargo clean
	rm -f tc66fw.bin tc66fw.pad.bin tc66fw.enc.bin tc66fw-release.enc.bin

.PHONY: host-check all release gates build flasher-gate init-gate stack-gate fast-gate flash recover markers clean
