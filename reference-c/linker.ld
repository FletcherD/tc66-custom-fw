/* linker.ld — custom TC66 app: flashed into the 52 KB app slot after the bootloader.
 * SP at top of 8 KB SRAM (must be in 0x2000xxxx for the bootloader to jump to us). */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08002c00, LENGTH = 52K   /* 0x08002c00 .. 0x0800fc00 */
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}
/* SP: use the stock app's exact value (0x20000e40), a known-good SP the bootloader
 * hands off to. The from-scratch top-of-RAM SP (0x20002000) coincided with the
 * fault's BFAR, so rule it out first. */
_estack = 0x20000e40;

ENTRY(Reset_Handler)

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

  .text : {
    *(.text*)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  _sidata = LOADADDR(.data);
  .data : {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT> FLASH

  .bss : {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /DISCARD/ : { *(.ARM.exidx*) *(.comment) *(.ARM.attributes) }
}
