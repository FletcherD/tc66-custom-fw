/* flasher.s — SRAM-resident streaming flasher for the custom-fw app self-update.
 *
 * Copied to 0x20001400 by main.c launch_flasher() on the "updat" command and
 * entered with IRQs masked. Adapted from the hardware-validated stub in
 * firmware/updater/stub.s. Runs entirely from SRAM (it erases/overwrites the app
 * flash that holds its own USB driver) and polls the USB device peripheral
 * directly. Reuses the BOOTLOADER's FMC routines (never erased) for flash writes.
 *
 * Protocol (host tool: firmware/app_update.py):
 *   erase 52 pages [0x08002c00,0x0800fc00)  -> send "uprdy"
 *   810x: recv 64-B PLAINTEXT chunk; program 16 words; send "OK"
 *   SCB_AIRCR system reset -> bootloader -> (K1 released) -> new app
 *
 * SRAM map (must stay clear of the 0x20001000..0x2000103c progress markers and of
 * this code at 0x20001400): RXBUF 0x20001c00, stack top 0x20002000.
 */
    .syntax unified
    .cpu cortex-m3
    .thumb

    .equ FLASH_ERASE_PAGES, 0x0800048f
    .equ FMC_UNLOCK,        0x08000ed5
    .equ FMC_PROGRAM_WORD,  0x08000ef5
    .equ FMC_LOCK,          0x08000e49

    .equ APP_BASE,    0x08002c00
    .equ PAGE_SIZE,   0x400
    .equ NUM_PAGES,   52
    .equ IMAGE_BYTES, 51840
    .equ CHUNK,       64

    .equ RXBUF,       0x20001c00
    .equ STACK_TOP,   0x20002000

    /* USB device peripheral — must match usb.c PMA layout (EP1 TX 0xC0, EP3 RX 0x100) */
    .equ EP1R,        0x40005c04
    .equ EP3R,        0x40005c0c
    .equ EP1_TX_PMA,  0x40006180   /* PMA 0xC0 doubled  */
    .equ EP1_COUNT_TX,0x40006014
    .equ EP3_RX_PMA,  0x40006200   /* PMA 0x100 doubled */
    .equ EP3_COUNT_RX,0x4000603c
    .equ SCB_AIRCR,   0xe000ed0c
    .equ AIRCR_RESET, 0x05fa0004

    .section .text.start, "ax"
    .global _start
    .thumb_func
_start:
    cpsid   i
    ldr     r0, =STACK_TOP
    mov     sp, r0

    /* erase whole app region first (bootloader unlock/erase/lock) */
    ldr     r0, =APP_BASE
    movw    r1, #PAGE_SIZE
    movs    r2, #NUM_PAGES
    ldr     r3, =FLASH_ERASE_PAGES
    blx     r3

    /* arm EP3 RX=VALID once (main.c left it NAK after delivering "updat") */
    ldr     r4, =EP3R
    ldr     r0, [r4]
    ldr     r1, =0xBF8F
    ands    r0, r0, r1
    ldr     r1, =0x3000
    eors    r0, r0, r1
    str     r0, [r4]

    ldr     r0, =msg_uprdy
    movs    r1, #5
    bl      usb_send

    movs    r4, #0                      /* running byte offset */
.Lchunk:
    bl      usb_recv64

    ldr     r3, =FMC_UNLOCK
    blx     r3
    ldr     r5, =RXBUF
    ldr     r6, =APP_BASE
    add     r6, r6, r4
    movs    r7, #0
.Lword:
    ldr.w   r1, [r5, r7, lsl #2]
    add.w   r0, r6, r7, lsl #2
    ldr     r3, =FMC_PROGRAM_WORD
    blx     r3
    adds    r7, r7, #1
    cmp     r7, #16
    bcc     .Lword
    ldr     r3, =FMC_LOCK
    blx     r3

    ldr     r0, =msg_ok
    movs    r1, #2
    bl      usb_send

    adds    r4, r4, #CHUNK
    ldr     r0, =IMAGE_BYTES
    cmp     r4, r0
    bcc     .Lchunk

    ldr     r0, =SCB_AIRCR
    ldr     r1, =AIRCR_RESET
    str     r1, [r0]
    dsb
.Lhang:
    b       .Lhang

/* usb_recv64: block until EP3 OUT delivers a packet, copy 64 B PMA->RXBUF, re-arm */
    .thumb_func
usb_recv64:
    push    {r4-r7, lr}
    ldr     r4, =EP3R
1:  ldr     r0, [r4]
    tst     r0, #0x8000
    beq     1b
    ldr     r5, =EP3_RX_PMA
    ldr     r6, =RXBUF
    movs    r7, #0
2:  ldrh    r0, [r5]
    strh    r0, [r6]
    adds    r5, r5, #4
    adds    r6, r6, #2
    adds    r7, r7, #1
    cmp     r7, #32
    bcc     2b
    ldr     r0, [r4]                    /* clear CTR_RX */
    ldr     r1, =0x0F8F
    ands    r0, r0, r1
    str     r0, [r4]
    ldr     r0, [r4]                    /* re-arm STAT_RX=VALID */
    ldr     r1, =0xBF8F
    ands    r0, r0, r1
    ldr     r1, =0x3000
    eors    r0, r0, r1
    str     r0, [r4]
    pop     {r4-r7, pc}

/* usb_send(r0=buf, r1=len<=64): copy to EP1 TX PMA, arm STAT_TX=VALID, wait CTR_TX */
    .thumb_func
usb_send:
    push    {r4-r7, lr}
    mov     r4, r0
    mov     r5, r1
    ldr     r6, =EP1_TX_PMA
    movs    r7, #0
1:  cmp     r7, r5
    bhs     2f
    ldrh    r0, [r4]
    strh    r0, [r6]
    adds    r4, r4, #2
    adds    r6, r6, #4
    adds    r7, r7, #2
    b       1b
2:  ldr     r6, =EP1_COUNT_TX
    strh    r5, [r6]
    ldr     r4, =EP1R
    ldr     r0, [r4]                    /* clear stale CTR_TX */
    ldr     r1, =0x8F0F
    ands    r0, r0, r1
    str     r0, [r4]
    ldr     r0, [r4]                    /* arm STAT_TX=VALID */
    ldr     r1, =0x8FBF
    ands    r0, r0, r1
    movs    r1, #0x30
    eors    r0, r0, r1
    str     r0, [r4]
3:  ldr     r0, [r4]
    tst     r0, #0x80
    beq     3b
    pop     {r4-r7, pc}

    .ltorg
    .align 2
msg_uprdy:
    .ascii "uprdy"
    .align 2
msg_ok:
    .ascii "OK"
    .align 2
