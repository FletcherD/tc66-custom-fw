//! Polled USB CDC-ACM device — line-for-line port of reference-c/usb.c.
//!
//! The GD32F1x0 USBD is the STM32F0/F1-style "USB_FS device" IP: EPnR/CNTR/ISTR/
//! DADDR/BTABLE at 0x40005C00 and a doubled-addressed 16-bit PMA at 0x40006000.
//! Everything host-visible matches the meter's own USB identity (VID/PID/strings/
//! endpoints), and the PMA layout is a contract with the SRAM flasher blob.
//!
//! Every function is #[inline(never)]: the C reference must be compiled -O0 (its
//! Makefile documents an -Os inlining lockup, root cause unknown), so we keep
//! per-function frames — which also keeps MK_PROBE pointing at a real frame.

use crate::markers::*;
use crate::regs::*;

/* CNTR bits */
const CNTR_FRES: u32 = 1 << 0;
/* ISTR bits */
const ISTR_CTR: u32 = 1 << 15;
const ISTR_RESET: u32 = 1 << 10;
const ISTR_EPID: u32 = 0x0F;
/* EPnR fields */
const EP_CTR_RX: u32 = 1 << 15;
const EP_SETUP: u32 = 1 << 11;
const EP_CTR_TX: u32 = 1 << 7;
const EP_TYPE_BULK: u32 = 0 << 9;
const EP_TYPE_CONTROL: u32 = 1 << 9;
const EP_TYPE_INTR: u32 = 3 << 9;
/* STAT values */
const STAT_DISABLED: u32 = 0;
const STAT_STALL: u32 = 1;
const STAT_NAK: u32 = 2;
const STAT_VALID: u32 = 3;

/* ---- PMA buffer layout (pma byte offsets) — MUST match flasher.s ---- */
const PMA_EP0_TX: u32 = 0x20;
const PMA_EP0_RX: u32 = 0x60;
const PMA_EP1_TX: u32 = 0xC0; /* flasher contract: CPU 0x40006180 */
const PMA_EP3_RX: u32 = 0x100; /* flasher contract: CPU 0x40006200 */
const PMA_EP2_TX: u32 = 0x140;
const COUNT_RX_64: u16 = 0x8400; /* BL_SIZE=1 (32-B blocks), NUM_BLOCK=1 -> 64 B */

/* ---------------- EPnR toggle-safe helpers ----------------
 * STAT/DTOG bits are toggle-type (written 1 = flip), CTR bits are rc_w0. These
 * masks preserve the other fields while forcing the target STAT field to `s`.
 * The masks are load-bearing — clear_ctr_tx MUST be 0x8F0F, not 0x8F7F. */
#[inline(never)]
fn ep_set_stat_rx(n: u32, s: u32) {
    wr(usb_epr(n), (rd(usb_epr(n)) & 0xBF8F) ^ (s << 12));
}
#[inline(never)]
fn ep_set_stat_tx(n: u32, s: u32) {
    wr(usb_epr(n), (rd(usb_epr(n)) & 0x8FBF) ^ (s << 4));
}
#[inline(never)]
fn ep_clear_ctr_rx(n: u32) {
    wr(usb_epr(n), rd(usb_epr(n)) & 0x0F8F); /* clr CTR_RX, keep CTR_TX */
}
#[inline(never)]
fn ep_clear_ctr_tx(n: u32) {
    wr(usb_epr(n), rd(usb_epr(n)) & 0x8F0F); /* clr CTR_TX, keep CTR_RX */
}
/* Set EP_TYPE + EA on a freshly-reset (0) register without toggling STAT bits. */
#[inline(never)]
fn ep_set_type_ea(n: u32, ty: u32, ea: u32) {
    wr(usb_epr(n), ty | ea);
}

/* ---------------- PMA copy (doubled addressing) ---------------- */
#[inline(never)]
fn pma_write(po: u32, src: *const u8, n: u32) {
    let mut d = pma_u16(po);
    let n = n as usize;
    let mut i = 0usize;
    unsafe {
        while i + 1 < n {
            let h = *src.add(i) as u16 | ((*src.add(i + 1) as u16) << 8);
            core::ptr::write_volatile(d, h);
            d = d.add(2);
            i += 2;
        }
        if i < n {
            core::ptr::write_volatile(d, *src.add(i) as u16);
        }
    }
}
#[inline(never)]
fn pma_read(po: u32, dst: *mut u8, n: u32) {
    let mut s = pma_u16(po);
    let n = n as usize;
    let mut i = 0usize;
    unsafe {
        while i + 1 < n {
            let h = core::ptr::read_volatile(s);
            *dst.add(i) = h as u8;
            *dst.add(i + 1) = (h >> 8) as u8;
            s = s.add(2);
            i += 2;
        }
        if i < n {
            *dst.add(i) = core::ptr::read_volatile(s) as u8;
        }
    }
}

/* ================= descriptors (the meter USB identity) ================= */
static DESC_DEVICE: [u8; 18] = [
    0x12, 0x01, 0x00, 0x02, 0x02, 0x00, 0x00, 0x40, //
    0xe9, 0x28, 0x8a, 0x01, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
];
static DESC_CONFIG: [u8; 67] = [
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32, //
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00, //
    0x05, 0x24, 0x00, 0x10, 0x01, //
    0x05, 0x24, 0x01, 0x00, 0x01, //
    0x04, 0x24, 0x02, 0x02, //
    0x05, 0x24, 0x06, 0x00, 0x01, //
    0x07, 0x05, 0x82, 0x03, 0x08, 0x00, 0x0a, //
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00, //
    0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00, //
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
];
static STR_LANGID: [u8; 4] = [0x04, 0x03, 0x09, 0x04];
/* UTF-16LE strings (bLength, 0x03, then chars) */
#[rustfmt::skip]
static STR_MFR: [u8; 16] = [16, 3, b'R',0, b'u',0, b'i',0, b'D',0, b'e',0, b'n',0, b'g',0];
#[rustfmt::skip]
static STR_PROD: [u8; 44] = [44, 3, b'T',0, b'C',0, b'6',0, b'6',0, b' ',0, b'U',0, b'S',0, b'B',0,
                             b' ',0, b'T',0, b'y',0, b'p',0, b'e',0, b'-',0, b'C',0, b' ',0,
                             b'M',0, b'e',0, b't',0, b'e',0, b'r',0];
#[rustfmt::skip]
static STR_SERIAL: [u8; 10] = [10, 3, b'T',0, b'C',0, b'6',0, b'6',0];

static ONE: [u8; 1] = [1]; /* GET_CONFIGURATION reply */
static STATUS_ZERO: [u8; 2] = [0, 0]; /* GET_STATUS reply */

/* ================= EP0 control state ================= */
struct UsbState {
    setup: [u8; 8],
    ep0_in_ptr: *const u8, /* remaining IN data to send */
    ep0_in_len: u32,       /* bytes remaining */
    ep0_zlp_after: bool,   /* terminating ZLP when in_len hits 0 and last==64 */
    pending_addr: u8,      /* SET_ADDRESS value to apply after status */
    addr_pending: bool,
    ep0_out_data: bool, /* next EP0 OUT is a real data stage (SET_LINE_CODING) */
    configured: bool,   /* set on SET_CONFIGURATION; gates the CDC dispatcher */
    line_coding: [u8; 7],
    rxbuf: [u8; 64],
}

/* Single core, IRQs permanently masked, poll() is not reentrant — a raw-pointer
 * cell is sound and avoids `static mut` reference aliasing. Never bind a `&mut`
 * to the state; go through `(*s())` per access. */
struct RacyCell(core::cell::UnsafeCell<UsbState>);
unsafe impl Sync for RacyCell {}
static STATE: RacyCell = RacyCell(core::cell::UnsafeCell::new(UsbState {
    setup: [0; 8],
    ep0_in_ptr: core::ptr::null(),
    ep0_in_len: 0,
    ep0_zlp_after: false,
    pending_addr: 0,
    addr_pending: false,
    ep0_out_data: false,
    configured: false,
    line_coding: [0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08], /* 115200 8N1 */
    rxbuf: [0; 64],
}));
#[inline(always)]
fn s() -> *mut UsbState {
    STATE.0.get()
}

#[inline(never)]
fn ep0_arm_tx() {
    mk(MK_PROBE, 0x630);
    unsafe {
        let chunk = if (*s()).ep0_in_len > 64 { 64 } else { (*s()).ep0_in_len };
        pma_write(PMA_EP0_TX, (*s()).ep0_in_ptr, chunk);
        core::ptr::write_volatile(bt(0, 1), chunk as u16); /* COUNT_TX */
        (*s()).ep0_in_ptr = (*s()).ep0_in_ptr.add(chunk as usize);
        (*s()).ep0_in_len -= chunk;
    }
    ep_set_stat_tx(0, STAT_VALID);
}

#[inline(never)]
fn ep0_start_in(p: *const u8, len: u32, w_length: u32) {
    mk(MK_PROBE, 0x610);
    let len = if len > w_length { w_length } else { len }; /* never send more than asked */
    unsafe {
        (*s()).ep0_in_ptr = p;
        (*s()).ep0_in_len = len;
        (*s()).ep0_zlp_after = len != 0 && len % 64 == 0 && len < w_length;
    }
    ep0_arm_tx();
    ep_set_stat_rx(0, STAT_VALID); /* allow status-OUT */
}

#[inline(never)]
fn ep0_send_zlp_status() {
    mk(MK_PROBE, 0x620);
    unsafe {
        (*s()).ep0_in_ptr = core::ptr::null();
        (*s()).ep0_in_len = 0;
        (*s()).ep0_zlp_after = false;
        core::ptr::write_volatile(bt(0, 1), 0); /* COUNT_TX = 0 */
    }
    ep_set_stat_tx(0, STAT_VALID);
}

#[inline(never)]
fn ep0_stall() {
    mk(MK_PROBE, 0x6f0);
    ep_set_stat_tx(0, STAT_STALL);
    ep_set_stat_rx(0, STAT_STALL);
}

#[inline(never)]
fn handle_setup() {
    let (bm_req, b_req, w_val, w_len) = unsafe {
        let st = s();
        (
            (*st).setup[0] as u32,
            (*st).setup[1] as u32,
            (*st).setup[2] as u32 | (((*st).setup[3] as u32) << 8),
            (*st).setup[6] as u32 | (((*st).setup[7] as u32) << 8),
        )
    };
    mk(MK_PROBE, 0x400 | b_req);
    mk(MK_SETUP, (bm_req << 8) | b_req);

    if (bm_req & 0x60) == 0x00 {
        /* ---- standard requests ---- */
        match b_req {
            0x06 => {
                /* GET_DESCRIPTOR */
                let ty = w_val >> 8;
                let idx = w_val & 0xff;
                let (p, l): (*const u8, u32) = if ty == 1 {
                    (DESC_DEVICE.as_ptr(), DESC_DEVICE.len() as u32)
                } else if ty == 2 {
                    (DESC_CONFIG.as_ptr(), DESC_CONFIG.len() as u32)
                } else if ty == 3 {
                    match idx {
                        0 => (STR_LANGID.as_ptr(), STR_LANGID.len() as u32),
                        1 => (STR_MFR.as_ptr(), STR_MFR.len() as u32),
                        2 => (STR_PROD.as_ptr(), STR_PROD.len() as u32),
                        3 => (STR_SERIAL.as_ptr(), STR_SERIAL.len() as u32),
                        _ => (core::ptr::null(), 0),
                    }
                } else {
                    (core::ptr::null(), 0)
                };
                if !p.is_null() {
                    ep0_start_in(p, l, w_len);
                } else {
                    ep0_stall(); /* devqual etc -> STALL */
                }
            }
            0x05 => {
                /* SET_ADDRESS (apply after status IN) */
                unsafe {
                    (*s()).pending_addr = (w_val & 0x7f) as u8;
                    (*s()).addr_pending = true;
                }
                mk(MK_STAGE, STAGE_ADDRESS);
                ep0_send_zlp_status();
            }
            0x09 => {
                /* SET_CONFIGURATION */
                let cfg = (w_val & 0xff) != 0;
                unsafe { (*s()).configured = cfg };
                if cfg {
                    mk(MK_STAGE, STAGE_CONFIG);
                }
                ep0_send_zlp_status();
            }
            0x08 => {
                /* GET_CONFIGURATION */
                ep0_start_in(ONE.as_ptr(), 1, w_len);
            }
            0x00 => {
                /* GET_STATUS */
                ep0_start_in(STATUS_ZERO.as_ptr(), 2, w_len);
            }
            0x01 | 0x03 => {
                /* CLEAR_FEATURE / SET_FEATURE */
                ep0_send_zlp_status();
            }
            _ => ep0_stall(),
        }
    } else if (bm_req & 0x60) == 0x20 {
        /* ---- CDC class requests ---- */
        match b_req {
            0x20 => {
                /* SET_LINE_CODING (7-byte OUT data stage) */
                unsafe { (*s()).ep0_out_data = true }; /* next EP0 OUT is real data */
                ep_set_stat_rx(0, STAT_VALID);
            }
            0x21 => {
                /* GET_LINE_CODING */
                let p = unsafe { (*s()).line_coding.as_ptr() };
                ep0_start_in(p, 7, w_len);
            }
            0x22 => {
                /* SET_CONTROL_LINE_STATE (no data) */
                ep0_send_zlp_status();
            }
            _ => ep0_stall(),
        }
    } else {
        ep0_stall();
    }
}

#[inline(never)]
fn handle_ep0() {
    mk(MK_PROBE, 0x300);
    let epr = rd(usb_epr(0));
    if epr & EP_CTR_RX != 0 {
        let is_setup = epr & EP_SETUP != 0;
        let cnt = unsafe { core::ptr::read_volatile(bt(0, 3)) } as u32 & 0x3FF; /* COUNT_RX */
        ep_clear_ctr_rx(0);
        if is_setup {
            mk(MK_PROBE, 0x310);
            unsafe {
                (*s()).ep0_out_data = false;
                pma_read(PMA_EP0_RX, (*s()).setup.as_mut_ptr(), 8);
            }
            handle_setup();
        } else if unsafe { (*s()).ep0_out_data } {
            /* real OUT data stage (SET_LINE_CODING payload) -> ack with status IN */
            mk(MK_PROBE, 0x320);
            unsafe {
                (*s()).ep0_out_data = false;
                if cnt >= 7 {
                    pma_read(PMA_EP0_RX, (*s()).line_coding.as_mut_ptr(), 7);
                }
            }
            ep0_send_zlp_status();
        } else {
            /* status-OUT ZLP after a control-read: accept + re-arm for next SETUP */
            mk(MK_PROBE, 0x330);
            ep_set_stat_rx(0, STAT_VALID);
        }
    }
    if rd(usb_epr(0)) & EP_CTR_TX != 0 {
        mk(MK_PROBE, 0x380);
        ep_clear_ctr_tx(0);
        unsafe {
            if (*s()).addr_pending {
                /* SET_ADDRESS status done */
                wr(USB_DADDR, 0x80 | (*s()).pending_addr as u32);
                (*s()).addr_pending = false;
                mk(MK_ENUM, (*s()).pending_addr as u32);
            }
            if (*s()).ep0_in_len != 0 {
                ep0_arm_tx(); /* next IN chunk */
            } else if (*s()).ep0_zlp_after {
                (*s()).ep0_zlp_after = false;
                core::ptr::write_volatile(bt(0, 1), 0);
                ep_set_stat_tx(0, STAT_VALID); /* terminating ZLP */
            }
        }
        /* re-arm RX for the next SETUP */
        ep_set_stat_rx(0, STAT_VALID);
    }
}

/* ---------------- bus reset: (re)configure all endpoints ---------------- */
#[inline(never)]
fn usb_reset() {
    mk(MK_PROBE, 0x200);
    mk(MK_RESET, mk_get(MK_RESET) + 1);
    wr(USB_BTABLE, 0);
    unsafe {
        (*s()).addr_pending = false;
        (*s()).ep0_in_len = 0;

        /* EP0: control, EA0, buffers */
        core::ptr::write_volatile(bt(0, 0), PMA_EP0_TX as u16);
        core::ptr::write_volatile(bt(0, 1), 0);
        core::ptr::write_volatile(bt(0, 2), PMA_EP0_RX as u16);
        core::ptr::write_volatile(bt(0, 3), COUNT_RX_64);
        ep_set_type_ea(0, EP_TYPE_CONTROL, 0);
        ep_set_stat_rx(0, STAT_VALID);
        ep_set_stat_tx(0, STAT_NAK);

        /* EP1: bulk IN (0x81) */
        core::ptr::write_volatile(bt(1, 0), PMA_EP1_TX as u16);
        core::ptr::write_volatile(bt(1, 1), 0);
        ep_set_type_ea(1, EP_TYPE_BULK, 1);
        ep_set_stat_tx(1, STAT_NAK);
        ep_set_stat_rx(1, STAT_DISABLED);

        /* EP2: interrupt IN (0x82) */
        core::ptr::write_volatile(bt(2, 0), PMA_EP2_TX as u16);
        core::ptr::write_volatile(bt(2, 1), 0);
        ep_set_type_ea(2, EP_TYPE_INTR, 2);
        ep_set_stat_tx(2, STAT_NAK);
        ep_set_stat_rx(2, STAT_DISABLED);

        /* EP3: bulk OUT (0x03) */
        core::ptr::write_volatile(bt(3, 2), PMA_EP3_RX as u16);
        core::ptr::write_volatile(bt(3, 3), COUNT_RX_64);
        ep_set_type_ea(3, EP_TYPE_BULK, 3);
        ep_set_stat_rx(3, STAT_VALID);
        ep_set_stat_tx(3, STAT_DISABLED);
    }
    wr(USB_DADDR, 0x80); /* EF=1, address 0 */
    if mk_get(MK_STAGE) < STAGE_RESET {
        mk(MK_STAGE, STAGE_RESET);
    }
    mk(MK_PROBE, 0x2ff);
}

/* ---------------- CDC bulk OUT (EP3) ---------------- */
#[inline(never)]
fn handle_ep3() {
    if rd(usb_epr(3)) & EP_CTR_RX == 0 {
        return;
    }
    mk(MK_PROBE, 0x500);
    let mut cnt = unsafe { core::ptr::read_volatile(bt(3, 3)) } as u32 & 0x3FF;
    if cnt > 64 {
        cnt = 64;
    }
    unsafe { pma_read(PMA_EP3_RX, (*s()).rxbuf.as_mut_ptr(), cnt) };
    ep_clear_ctr_rx(3);
    let b = unsafe { &(*s()).rxbuf };
    mk(
        MK_CMD,
        b[0] as u32 | (b[1] as u32) << 8 | (b[2] as u32) << 16 | (b[3] as u32) << 24,
    );
    mk(MK_CMDCNT, mk_get(MK_CMDCNT) + 1);
    if mk_get(MK_STAGE) < STAGE_CMD {
        mk(MK_STAGE, STAGE_CMD);
    }

    /* Only dispatch once configured, so stray/foreign packets before enumeration
     * completes can't trigger a CDC reply (and its bounded TX wait). */
    let is_updat = if unsafe { (*s()).configured } {
        crate::on_packet(unsafe { &(&(*s()).rxbuf)[..cnt as usize] })
    } else {
        false
    };
    /* On updat we deliberately DO NOT re-arm EP3 (leave it NAK) so the SRAM
     * flasher can arm it once, exactly as the flasher expects. */
    if !is_updat {
        ep_set_stat_rx(3, STAT_VALID);
    }
}

/* ---------------- public API ---------------- */
#[inline(never)]
pub fn cdc_send(buf: &[u8]) {
    let len = if buf.len() > 64 { 64 } else { buf.len() } as u32;
    pma_write(PMA_EP1_TX, buf.as_ptr(), len);
    unsafe { core::ptr::write_volatile(bt(1, 1), len as u16) }; /* COUNT_TX */
    ep_clear_ctr_tx(1);
    ep_set_stat_tx(1, STAT_VALID);
    /* wait (bounded) until the host fetches it (CTR_TX) so back-to-back sends
     * don't clobber. Bounded so an absent host can never wedge the main loop. */
    for _ in 0..2_000_000u32 {
        if rd(usb_epr(1)) & EP_CTR_TX != 0 {
            break;
        }
    }
    ep_clear_ctr_tx(1);
}

#[inline(never)]
fn clock_init() {
    /* Target: PLL x12 from HSI/2 => 48 MHz, USBDPSC=/1 (48 MHz USB clock).
     * The bootloader hands off with its OWN clock running (PLL as system clock),
     * so switch back to HSI and disable PLL before touching PLLMF — a running
     * PLL ignores config writes. Robust from any state. */
    modify(RCU_CTL0, |v| v | 1 << 0); /* HSIEN */
    while rd(RCU_CTL0) & (1 << 1) == 0 {} /* wait HSISTB */
    modify(RCU_CFG0, |v| v & !0x3); /* SCS = HSI */
    while rd(RCU_CFG0) & 0xC != 0x0 {} /* wait SCSS = HSI */
    modify(RCU_CTL0, |v| v & !(1 << 24)); /* PLLEN = 0 (now allowed) */
    while rd(RCU_CTL0) & (1 << 25) != 0 {} /* wait PLLSTB clear */
    modify(RCU_CFG0, |v| v & !0x003C_0000); /* clear PLLMF[3:0] (bits 21:18) */
    modify(RCU_CFG0, |v| v & !(1 << 27)); /* clear PLLMF[4] */
    modify(RCU_CFG0, |v| v & !(1 << 16)); /* PLLSEL = HSI/2 */
    modify(RCU_CFG0, |v| v | 10 << 18); /* PLLMF=10 -> x12 (=48MHz) */
    modify(RCU_CFG0, |v| (v & !0x00C0_0000) | 0x0040_0000); /* USBDPSC = 01 (CK_PLL/1) */
    modify(RCU_CTL0, |v| v | 1 << 24); /* PLLEN */
    while rd(RCU_CTL0) & (1 << 25) == 0 {} /* wait PLLSTB */
    modify(RCU_CFG0, |v| (v & !0x3) | 0x2); /* SCS = PLL */
    while rd(RCU_CFG0) & 0xC != 0x8 {} /* wait SCSS = PLL */
    modify(RCU_APB1EN, |v| v | 1 << 23); /* USBD clock enable */
    mk(MK_STAGE, STAGE_CLOCK);
}

/* busy-wait milliseconds at 48 MHz (asm::delay counts >= n cycles) */
#[inline(never)]
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(ms * 48_000);
}

#[inline(never)]
fn pa9_config_output() {
    modify(RCU_AHBEN, |v| v | 1 << 17); /* GPIOA clock (AHBEN bit17) */
    modify(GPIOA_MODER, |v| (v & !(3 << 18)) | 1 << 18); /* PA9 = output */
    modify(GPIOA_OTYPER, |v| v & !(1 << 9)); /* push-pull */
    modify(GPIOA_OSPEED, |v| (v & !(3 << 18)) | 1 << 18); /* medium speed */
    modify(GPIOA_PUPDR, |v| v & !(3 << 18)); /* no pull */
}
#[inline(never)]
fn pa9_low() {
    wr(GPIOA_BC, PA9_MASK); /* D+ low -> disconnect */
}
#[inline(never)]
fn pa9_high() {
    wr(GPIOA_BOP, PA9_MASK); /* D+ pull-up -> connect */
}

#[inline(never)]
pub fn init() {
    /* init the runtime markers so we can read absolute counts over SWD */
    mk(MK_HEART, 0);
    mk(MK_CTR, 0);
    mk(MK_RESET, 0);
    mk(MK_CMDCNT, 0);
    mk(MK_ENUM, 0);
    unsafe { (*s()).configured = false };

    clock_init();

    /* Force a fresh USB enumeration of THIS app. The bootloader jumped to us with
     * D+ still pulled up and the host bound to the *bootloader*; a bus reset would
     * otherwise never arrive. Pull PA9 low (electrical disconnect), hold it, bring
     * the USB core up, then reconnect. */
    pa9_config_output();
    pa9_low(); /* USB disconnect */
    delay_ms(120); /* let the host notice */

    /* power-on / core reset: clear PDWN but hold FRES, wait tSTARTUP, release. */
    wr(USB_CNTR, CNTR_FRES); /* PDWN=0, FRES=1 */
    cortex_m::asm::delay(10_000); /* tSTARTUP settle (C: ~2000-iter loop) */
    wr(USB_CNTR, 0); /* release reset */
    wr(USB_ISTR, 0); /* clear pending */
    wr(USB_BTABLE, 0);
    wr(USB_DADDR, 0x80); /* EF set, address 0 */
    mk(MK_STAGE, STAGE_USB_INIT);

    pa9_high(); /* reconnect -> host enumerates us */
}

#[inline(never)]
pub fn poll() {
    mk(MK_HEART, mk_get(MK_HEART) + 1);
    mk(MK_PROBE, 0x100);
    let istr = rd(USB_ISTR);
    mk(MK_ISTR, istr);
    mk(MK_PROBE, 0x110);
    if istr & ISTR_RESET != 0 {
        wr(USB_ISTR, !ISTR_RESET); /* clear (rc_w0) */
        usb_reset();
        return;
    }
    /* Acknowledge the non-CTR status flags so they can't accumulate or wedge the
     * peripheral: SUSP(11) WKUP(12) SOF(9) ESOF(8) ERR(13) PMAOVR(14). No power
     * management action — just clear (rc_w0). */
    if istr & 0x7B00 != 0 {
        wr(USB_ISTR, !0x7B00);
    }
    if istr & ISTR_CTR != 0 {
        mk(MK_CTR, mk_get(MK_CTR) + 1);
        let ep = istr & ISTR_EPID;
        if ep == 0 {
            handle_ep0();
        } else if ep == 3 {
            handle_ep3();
        } else {
            /* EP1/EP2 IN complete — just clear CTR bits */
            if rd(usb_epr(ep)) & EP_CTR_TX != 0 {
                ep_clear_ctr_tx(ep);
            }
            if rd(usb_epr(ep)) & EP_CTR_RX != 0 {
                ep_clear_ctr_rx(ep);
            }
        }
    }
    /* corruption detector: SP at poll exit (C also recorded the return address;
     * dropped here — no stable-Rust equivalent) */
    let sp: u32;
    unsafe { core::arch::asm!("mov {}, sp", out(reg) sp) };
    mk(MK_SPEXIT, sp);
}
