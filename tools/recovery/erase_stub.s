.syntax unified
.thumb
.thumb_func
_start:
    push {lr}
    ldr r0, =0x08002c00
    ldr r1, =0x400
    movs r2, #52
    ldr r7, =0x0800048f      @ BL_ERASE_PAGES
    blx r7
    ldr r0, =0x20001100
    ldr r1, =0xE5A5EDED
    str r1, [r0]
    ldr r1, =0x40022000
    ldr r1, [r1, #0x0C]
    str r1, [r0, #4]
1:  b 1b
