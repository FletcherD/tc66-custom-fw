.syntax unified
.thumb
.thumb_func
_start:
    ldr r0, =0x40022000      @ FMC base
    @ unlock
    ldr r1, =0x45670123
    str r1, [r0, #4]
    ldr r1, =0xCDEF89AB
    str r1, [r0, #4]
    @ page erase 0x08002c00
    movs r1, #2              @ PER
    str r1, [r0, #0x10]
    ldr r2, =0x08002c00
    str r2, [r0, #0x14]      @ FMC_ADDR
    movs r1, #0x42           @ PER|START
    str r1, [r0, #0x10]
1:  ldr r1, [r0, #0x0C]      @ wait BSY
    lsrs r1, r1, #1
    bcs 1b
    movs r1, #0
    str r1, [r0, #0x10]
    @ program word
    movs r1, #1              @ PG
    str r1, [r0, #0x10]
    ldr r2, =0x08002c00
    ldr r3, =0x12345678
    str r3, [r2]             @ CPU write -> FMC programs
2:  ldr r1, [r0, #0x0C]
    lsrs r1, r1, #1
    bcs 2b
    movs r1, #0
    str r1, [r0, #0x10]
    @ marker = FMC_STAT, and done-flag
    ldr r1, [r0, #0x0C]
    ldr r2, =0x20000500
    str r1, [r2]
    ldr r1, =0x0D09EEEE
    str r1, [r2, #4]
3:  b 3b
