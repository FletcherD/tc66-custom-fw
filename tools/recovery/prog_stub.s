.syntax unified
.thumb
.thumb_func
_start:
    push {r4,r5,r6,r7,lr}
    mov r4, r0
    mov r5, r1
    mov r6, r2
    ldr r7, =0x08000ed5      @ unlock
    blx r7
1:  cmp r6, #0
    beq 2f
    mov r0, r4
    ldr r1, [r5]
    ldr r7, =0x08000ef5      @ program(addr,word)
    blx r7
    adds r4, #4
    adds r5, #4
    subs r6, #1
    b 1b
2:  ldr r7, =0x08000e49      @ lock
    blx r7
    ldr r0, =0x20001108
    ldr r1, =0x0D09EEEE
    str r1, [r0]
    ldr r1, =0x40022000
    ldr r1, [r1, #0x0C]
    str r1, [r0, #4]
3:  b 3b
