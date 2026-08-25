/* flasher.ld — link the SRAM streaming flasher at its run address 0x20001400.
 * It uses absolute addresses only; objcopy'd to a raw blob and embedded in the app. */
MEMORY { SRAM (rwx) : ORIGIN = 0x20001400, LENGTH = 0x800 }
ENTRY(_start)
SECTIONS
{
  . = 0x20001400;
  .text : { KEEP(*(.text.start)) *(.text*) *(.rodata*) . = ALIGN(4); } > SRAM
  /DISCARD/ : { *(.ARM.exidx*) *(.comment) *(.ARM.attributes) }
}
