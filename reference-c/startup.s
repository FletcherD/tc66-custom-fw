/* startup.s — Cortex-M3 startup for the custom TC66 app (M0).
 * Vector table at 0x08002c00 with a COMPLETE set of IRQ vectors (GD32F1x0 has up
 * to ~IRQ42; we provide 48). Sets VTOR, clears/masks NVIC, inits .data/.bss, main().
 * All fault/unexpected-exception handlers capture context into the SRAM marker
 * window so a non-halting SWD read reveals what happened. */
    .syntax unified
    .cpu cortex-m3
    .thumb

    .section .isr_vector, "a", %progbits
    .global _vectors
_vectors:
    .word _estack            /* 0  initial SP (MSP) */
    .word Reset_Handler       /* 1  reset */
    .word NMI_Handler         /* 2  NMI */
    .word HardFault_Handler   /* 3  HardFault */
    .word HardFault_Handler   /* 4  MemManage */
    .word HardFault_Handler   /* 5  BusFault */
    .word HardFault_Handler   /* 6  UsageFault */
    .word 0, 0, 0, 0          /* 7-10 reserved */
    .word IRQ_Default         /* 11 SVC */
    .word IRQ_Default         /* 12 DebugMon */
    .word 0                   /* 13 reserved */
    .word IRQ_Default         /* 14 PendSV */
    .word IRQ_Default         /* 15 SysTick */
    /* External IRQ 0..47 -> all captured (we run polled; none should fire) */
    .rept 48
    .word IRQ_Default
    .endr

    .section .text
    .thumb_func
    .global Reset_Handler
Reset_Handler:
    cpsid   i
    /* VTOR = _vectors */
    ldr     r0, =_vectors
    ldr     r1, =0xE000ED08
    str     r0, [r1]
    /* MSP = _estack */
    ldr     r0, =_estack
    mov     sp, r0
    /* Mask+clear ALL NVIC IRQs (bootloader may have enabled USBD IRQ37).
       ICER0/1 @0xE000E180/184 (disable), ICPR0/1 @0xE000E280/284 (clear pending) */
    ldr     r1, =0xE000E180
    mvn     r0, #0                    /* 0xFFFFFFFF */
    str     r0, [r1, #0]              /* ICER0 */
    str     r0, [r1, #4]              /* ICER1 */
    str     r0, [r1, #0x100]          /* ICPR0 (0xE000E280) */
    str     r0, [r1, #0x104]          /* ICPR1 (0xE000E284) */
    /* .data init (flash LMA -> RAM VMA) */
    ldr     r0, =_sidata
    ldr     r1, =_sdata
    ldr     r2, =_edata
1:  cmp     r1, r2
    bcs     2f
    ldr     r3, [r0], #4
    str     r3, [r1], #4
    b       1b
2:  ldr     r0, =_sbss
    ldr     r1, =_ebss
    movs    r3, #0
3:  cmp     r0, r1
    bcs     4f
    str     r3, [r0], #4
    b       3b
4:  dsb
    isb
    bl      main
5:  b       5b

/* HardFault / MemManage / BusFault / UsageFault capture:
 *   0x20001008 = 0xFA017FA0 sentinel
 *   0x2000100c = stacked PC (faulting instruction)
 *   0x20001010 = HFSR (0xE000ED2C)
 *   0x20001014 = CFSR (0xE000ED28)
 *   0x20001018 = stacked LR
 *   0x2000101c = IPSR (which fault) */
    .thumb_func
    .global HardFault_Handler
HardFault_Handler:
    mrs     r0, msp
    ldr     r1, =0x20001008
    ldr     r2, =0xFA017FA0
    str     r2, [r1, #0]
    ldr     r2, [r0, #0x18]          /* stacked PC */
    str     r2, [r1, #4]
    ldr     r3, =0xE000ED2C
    ldr     r2, [r3]
    str     r2, [r1, #8]
    ldr     r3, =0xE000ED28
    ldr     r2, [r3]
    str     r2, [r1, #12]
    ldr     r2, [r0, #0x14]          /* stacked LR */
    str     r2, [r1, #16]
    mrs     r2, ipsr
    str     r2, [r1, #20]
6:  b       6b

/* NMI + any unexpected IRQ/system exception: record IPSR + stacked PC, then spin.
 *   0x2000104c = 0xEXC<ipsr> sentinel
 *   0x20001050 = IPSR (exception number)
 *   0x20001054 = stacked PC */
    .thumb_func
    .global NMI_Handler
NMI_Handler:
    b       IRQ_Default

    .thumb_func
    .global IRQ_Default
IRQ_Default:
    mrs     r0, msp
    ldr     r1, =0x2000104c
    ldr     r2, =0xE7C00000
    str     r2, [r1, #0]
    mrs     r2, ipsr
    str     r2, [r1, #4]
    ldr     r2, [r0, #0x18]          /* stacked PC */
    str     r2, [r1, #8]
7:  b       7b

    .thumb_func
    .global Default_Handler
Default_Handler:
    b       IRQ_Default
