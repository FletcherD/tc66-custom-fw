.syntax unified
.thumb
.thumb_func
_start:
    ldr r5, =0x20001200        @ marker base
    ldr r6, =0x08000ed5        @ BL_FMC_UNLOCK
    blx r6
    ldr r0, =0x08002c00        @ erase 1 page (base,pagesize,count)
    ldr r1, =0x400
    movs r2, #1
    ldr r6, =0x0800048f        @ BL_ERASE_PAGES
    blx r6
    ldr r0, =0x40022000
    ldr r1, [r0, #0x0C]        @ STAT after erase
    str r1, [r5, #0]
    ldr r0, =0x08002c00        @ program word (addr,word)
    ldr r1, =0x12345678
    ldr r6, =0x08000ef5        @ BL_FMC_PROGRAM_WORD
    blx r6
    ldr r0, =0x40022000
    ldr r1, [r0, #0x0C]        @ STAT after program
    str r1, [r5, #4]
    ldr r6, =0x08000e49        @ BL_FMC_LOCK
    blx r6
    ldr r1, =0x0D09EEEE
    str r1, [r5, #8]
1:  b 1b
