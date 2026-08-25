//! stm32-usbd glue: the GD32F150's USBD is the STM32F0/F1-style USB_FS device IP
//! (same register block at 0x40005C00, 512-B doubled-addressed PMA at 0x40006000 —
//! Word16x1 in stm32-usbd's terms). Modeled on stm32f1xx-hal's impl for the F103.

use stm32_usbd::UsbPeripheral;

use crate::regs::{modify, RCU_APB1EN};

pub struct Gd32Usb;

unsafe impl Sync for Gd32Usb {}

unsafe impl UsbPeripheral for Gd32Usb {
    const REGISTERS: *const () = 0x4000_5C00 as *const ();
    /* No internal D+ pull-up: PA9 drives D+ through a resistor (board.rs). */
    const DP_PULL_UP_FEATURE: bool = false;
    const EP_MEMORY: *const () = 0x4000_6000 as *const ();
    const EP_MEMORY_SIZE: usize = 512;
    const EP_MEMORY_ACCESS: stm32_usbd::MemoryAccess = stm32_usbd::MemoryAccess::Word16x1;

    fn enable() {
        modify(RCU_APB1EN, |v| v | 1 << 23); /* USBD clock enable */
    }

    fn startup_delay() {
        /* tSTARTUP ~1 us; 100 cycles at 48 MHz is generous */
        cortex_m::asm::delay(100);
    }
}

pub type UsbBus = stm32_usbd::UsbBus<Gd32Usb>;
