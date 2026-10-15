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
  /* Top 1 KB of SRAM: .hibss home (see below). Nothing else lives up here —
     bootloader handoff is done by the time the app runs, and the updat
     flasher never returns, so stale contents there are harmless. */
  HIRAM : ORIGIN = 0x20001C00, LENGTH = 0x400
}

/* The KM003C AdcQueue ring (adcq.rs, 256 x 8 B) reuses FLASHER_RAM: idle
   except while updat runs the flasher there, and flasher::launch() masks
   interrupts before copying it in. Not a section — nothing is linked or
   zeroed here; adcq.rs only touches it between its own head/tail. */
__adcq_ring = ORIGIN(FLASHER_RAM);
ASSERT(LENGTH(FLASHER_RAM) >= 256 * 8, "FLASHER_RAM too small for the AdcQueue ring");
/* = 0x20000e40, the known-good initial SP (cortex-m-rt "set-sp" loads it). */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* Large statics parked in the 1 KB above FLASHER_RAM, out of the fenced
   stack region: every byte here raises the stack budget (stack_gate.py's
   STACK_TOP - __ebss) byte-for-byte. cortex-m-rt's Reset does NOT touch this
   region — main() zeroes __shibss..__ehibss as its first act, before
   getva::init() and long before either interrupt is unmasked. Statics placed
   here (#[link_section = ".hibss"]) must be all-zero/MaybeUninit at rest and
   must not be touched before that zero loop.
   INSERT AFTER .uninit, NOT .bss: link.x's `__ebss = .` assignment trails
   the .bss section, so inserting there drags __ebss up to HIRAM and Reset
   smear-zeroes everything in between (init-gate catches it). */
SECTIONS
{
  .hibss (NOLOAD) : ALIGN(4)
  {
    __shibss = .;
    *(.hibss .hibss.*);
    . = ALIGN(4);
    __ehibss = .;
  } > HIRAM
} INSERT AFTER .uninit;

/* link.x PROVIDEs these from `.` after .uninit — which flasher.x's inserted
   section pushes to 0x2000_17xx, tripping the stack-overlap assert. Defining
   them here (before the PROVIDEs) pins them to the real end of statics. */
_stack_end = __euninit;
__sheap = __euninit;
