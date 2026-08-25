/* flasher_blob.s — embed the assembled SRAM flasher (flasher.bin) into the app
 * image as a rodata blob so main.c can copy it to SRAM on 'updat'. */
    .section .rodata.flasher, "a", %progbits
    .align 2
    .global _flasher_blob_start
    .global _flasher_blob_end
_flasher_blob_start:
    .incbin "flasher.bin"
_flasher_blob_end:
    .align 2
