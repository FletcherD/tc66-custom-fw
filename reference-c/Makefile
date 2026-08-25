# Custom TC66 app build (Milestone 0). Produces m0.bin (raw) for flashing.
CROSS   = arm-none-eabi-
CC      = $(CROSS)gcc
OBJCOPY = $(CROSS)objcopy
CFLAGS  = -mcpu=cortex-m3 -mthumb -Os -ffreestanding -nostdlib -Wall -Wextra \
          -fno-common -ffunction-sections -fdata-sections -std=gnu11
LDFLAGS = -T linker.ld -nostdlib -Wl,--gc-sections -Wl,-Map=m0.map

OBJS = startup.o main.o usb.o flasher_blob.o

.PHONY: all clean
all: m0.bin

# --- SRAM flasher: assembled and linked at 0x20001400, emitted as a raw blob ---
flasher.o: flasher.s
	$(CC) $(CFLAGS) -c $< -o $@
flasher.elf: flasher.o flasher.ld
	$(CC) -mcpu=cortex-m3 -mthumb -nostdlib -T flasher.ld flasher.o -o $@
flasher.bin: flasher.elf
	$(OBJCOPY) -O binary $< $@
	@echo "flasher.bin: $$(wc -c < flasher.bin) bytes"

# flasher_blob embeds flasher.bin (must exist first)
flasher_blob.o: flasher_blob.s flasher.bin
	$(CC) $(CFLAGS) -c flasher_blob.s -o $@

startup.o: startup.s
	$(CC) $(CFLAGS) -c $< -o $@
main.o: main.c usb.h
	$(CC) $(CFLAGS) -c $< -o $@
# usb.c built at -O0: the -Os build inlined the whole USB path into one giant
# usb_poll whose return locked up (INVSTATE) post-enumeration; -O0 gives clean
# per-function frames (fixes the codegen artifact / localizes any real fault).
usb.o: usb.c usb.h getva_reply.inc
	$(CC) $(filter-out -Os,$(CFLAGS)) -O0 -c $< -o $@

m0.elf: $(OBJS) linker.ld
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@

m0.bin: m0.elf
	$(OBJCOPY) -O binary $< $@
	@echo "m0.bin: $$(wc -c < m0.bin) bytes"

# m0_pad.bin: zero-padded to the full 51840-byte app image (810 x 64-B chunks) so the
# `updat` SRAM flasher (which streams a fixed 810 chunks) completes and auto-resets.
# Flash over USB with NO K1:  python3 ../firmware/app_update.py m0_pad.bin
m0_pad.bin: m0.bin
	python3 -c "d=open('m0.bin','rb').read(); open('m0_pad.bin','wb').write(d+b'\x00'*(51840-len(d)))"
	@echo "m0_pad.bin: $$(wc -c < m0_pad.bin) bytes (810 chunks)"

.PHONY: pad
pad: m0_pad.bin

clean:
	rm -f *.o *.elf *.bin *.map
