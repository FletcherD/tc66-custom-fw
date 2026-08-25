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
} INSERT AFTER .data;
