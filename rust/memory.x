MEMORY
{
  /* App slot is 52K (0x08002c00..0x0800fc00), but the updat flasher streams exactly
     51840 B (810 x 64) — using that as LENGTH makes the linker fail before the pad
     step would. */
  FLASH : ORIGIN = 0x08002C00, LENGTH = 51840
  /* RAM deliberately ends at 0x20000E40: the bootloader handoff faults if the
     initial SP is top-of-RAM 0x20002000, and this fences statics + stack below the
     SRAM marker window (0x20001000+) and the flasher (0x20001400+). */
  RAM : ORIGIN = 0x20000000, LENGTH = 0xE40
  /* The self-flasher's execution home: .flasher is linked to RUN here (VMA) but
     is stored in FLASH (LMA) and copied over by flasher::launch() at updat time.
     Sits above the marker window (0x20001000..0x2000106x). */
  FLASHER_RAM : ORIGIN = 0x20001400, LENGTH = 0x800
}

/* = 0x20000e40, the known-good initial SP (cortex-m-rt "set-sp" loads it). */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
