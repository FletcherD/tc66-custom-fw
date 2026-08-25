/* The self-flasher section: runs at 0x20001400 (VMA in FLASHER_RAM), stored in
   FLASH (LMA). flasher::launch() copies __siflasher -> __sflasher..__eflasher
   before jumping. Linking it at its run address means all absolute addressing is
   correct — no position-independence requirements on the Rust code. */
SECTIONS
{
  .flasher : ALIGN(4)
  {
    . = ALIGN(4);
    __sflasher = .;
    *(.flasher .flasher.*);
    . = ALIGN(4);
    __eflasher = .;
  } > FLASHER_RAM AT>FLASH
  __siflasher = LOADADDR(.flasher);
}
/* INSERT AFTER .uninit, NOT .data: cortex-m-rt deliberately extends its
   __sdata..__edata Reset copy range around sections inserted after .data, which
   assumes a contiguous VMA. Our VMA is disjoint (0x20001400), so that would make
   Reset smear flash over all of low RAM (it trampled DEVICE_PERIPHERALS and
   panicked take() — found the hard way). launch() does its own copy instead. */
INSERT AFTER .uninit;

/* NOTE: link.x PROVIDEs __sheap/_stack_end from `.` after .uninit, which the
   inserted section pushes to 0x2000_17xx — memory.x pins both back (a definition
   included BEFORE the PROVIDE suppresses it; a later one here does not, at least
   under rust-lld). See memory.x. */
