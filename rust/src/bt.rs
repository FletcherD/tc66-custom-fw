//! DX-BT24-M BLE module link — the stock BLE transport, reimplemented.
//!
//! The module is a transparent UART<->GATT bridge on **USART1** (TX = PA2
//! AF1, RX = PB0 AF4 — recovered from the stock firmware's init) that owns
//! the whole BLE side (service 0xFFE0, notify 0xFFE1 / write 0xFFE2). The
//! stock firmware provisioned it persistently to **115200 baud** and the
//! advertised name `TC66C` (`TTM:BPS-115200` / `TTM:REN-TC66C`), so normal
//! operation is just "speak 115200"; `init(true)` selects the module's
//! 9600 power-on default for TTM re-provisioning via the dev verbs.
//!
//! No interrupts (the two-IRQ rule stands): RX is a **circular DMA ring on
//! channel 4** — the part's fixed USART1_RX request, free of the TFT engine
//! on ch2 — that the main loop drains via `service()`; TX is polled with
//! bounded waits (TBE only depends on our own shifter, never on the module).
//! At 115200 a 192-B `bgetva` reply blocks the loop ~17 ms, which the
//! tick-delta cadences absorb; USB and the sampler live in ISRs.
//!
//! Concurrency: everything here is thread-mode only (main loop + dev
//! verbs — mutually exclusive by construction, single context). The DMA
//! ring is hardware-written; the consumer computes the write index from
//! CH4CNT and only reads bytes behind it, so a torn in-flight byte is
//! never observed. `service()` copies a completed line out of the
//! assembler before dispatching, so the callback may freely call back
//! into `send`/`read_raw`.
//!
//! Modes (MK_BT_STATE): 0 = uninitialized, 1 = raw (bring-up: `service`
//! leaves bytes for the `btrd` verb), 2 = dispatch (lines are matched as
//! `b`-verbs). Dev flow: `btini` -> raw probing (`btwr`/`btrd`) -> `btgo`;
//! the always-on path enters dispatch mode directly.

use crate::markers::*;
use gd32f1::gd32f150 as pac;

const RING: usize = 64;
/// Longest meaningful line ("bgetva" + CRLF is 8; TTM replies fit too).
const LINE_MAX: usize = 24;

pub const MODE_RAW: u32 = 1;
pub const MODE_DISPATCH: u32 = 2;

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}

/// DMA-written RX ring. The CPU only reads it (behind the write index).
static RX_RING: Racy<[u8; RING]> = Racy(core::cell::UnsafeCell::new([0; RING]));

struct RxState {
    /// next ring index the consumer reads (thread-only)
    rd: usize,
    line: [u8; LINE_MAX],
    llen: usize,
    /// an overlong line is being discarded until its terminator
    over: bool,
}
/// Thread-mode only (see module doc); no ISR ever touches this.
static RX: Racy<RxState> = Racy(core::cell::UnsafeCell::new(RxState {
    rd: 0,
    line: [0; LINE_MAX],
    llen: 0,
    over: false,
}));

fn mode() -> u32 {
    mk_get(MK_BT_STATE)
}

pub fn set_dispatch() {
    if mode() != 0 {
        mk(MK_BT_STATE, MODE_DISPATCH);
    }
}

/// Bring up USART1 + the RX DMA ring. Pure register configuration — no
/// waits, nothing can block. Idempotent: re-init tears the old session
/// down first (dev verb may be re-issued, e.g. to switch baud).
/// `low_baud` selects the module's 9600 power-on default instead of the
/// provisioned 115200.
#[inline(never)]
pub fn init(low_baud: bool) {
    let rcu = unsafe { &*pac::Rcu::ptr() };
    let gpioa = unsafe { &*pac::Gpioa::ptr() };
    let gpiob = unsafe { &*pac::Gpiob::ptr() };
    let usart = unsafe { &*pac::Usart1::ptr() };
    let dma = unsafe { &*pac::Dma::ptr() };

    rcu.ahben()
        .modify(|_, w| w.paen().enabled().pben().enabled().dmaen().enabled());
    rcu.apb1en().modify(|_, w| w.usart1en().enabled());

    /* stop any previous session before reconfiguring */
    usart.ctl0().modify(|_, w| w.uen().disabled());
    dma.ch4ctl().modify(|_, w| w.chen().disabled());

    /* pins as the stock firmware configures them: AF, push-pull, pull-up */
    gpioa.afsel0().modify(|_, w| w.sel2().af1());
    gpioa.omode().modify(|_, w| w.om2().push_pull());
    gpioa.ospd().modify(|_, w| w.ospd2().speed10m());
    gpioa.pud().modify(|_, w| w.pud2().pull_up());
    gpioa.ctl().modify(|_, w| w.ctl2().alternate());
    gpiob.afsel0().modify(|_, w| w.sel0().af4());
    gpiob.omode().modify(|_, w| w.om0().push_pull());
    gpiob.ospd().modify(|_, w| w.ospd0().speed10m());
    gpiob.pud().modify(|_, w| w.pud0().pull_up());
    gpiob.ctl().modify(|_, w| w.ctl0().alternate());

    /* clean peripheral state (stock does a full deinit too) */
    rcu.apb1rst().modify(|_, w| w.usart1rst().set_bit());
    rcu.apb1rst().modify(|_, w| w.usart1rst().clear_bit());

    /* 8N1, oversampling 16: BAUD = PCLK1 (48 MHz) / rate */
    let div: u32 = if low_baud { 5000 } else { 417 };
    usart.baud().write(|w| unsafe { w.bits(div) });
    /* RX by DMA only — no USART interrupts, ever */
    usart.ctl2().write(|w| w.denr().enabled());

    /* circular RX ring on channel 4 (the fixed USART1_RX request) */
    dma.intc().write(|w| {
        w.gifc4()
            .clear()
            .ftfifc4()
            .clear()
            .htfifc4()
            .clear()
            .errifc4()
            .clear()
    });
    dma.ch4paddr()
        .write(|w| unsafe { w.paddr().bits(usart.rdata().as_ptr() as u32) });
    dma.ch4maddr()
        .write(|w| unsafe { w.maddr().bits(RX_RING.0.get() as u32) });
    dma.ch4cnt().write(|w| w.cnt().bits(RING as u16));
    dma.ch4ctl().write(|w| {
        w.dir()
            .from_peripheral()
            .cmen()
            .enabled()
            .pnaga()
            .fixed()
            .mnaga()
            .increment()
            .pwidth()
            .bits8()
            .mwidth()
            .bits8()
            .prio()
            .low()
    });
    dma.ch4ctl().modify(|_, w| w.chen().enabled());

    usart
        .ctl0()
        .write(|w| w.ren().enabled().ten().enabled().uen().enabled());

    let st = unsafe { &mut *RX.0.get() };
    st.rd = 0;
    st.llen = 0;
    st.over = false;
    mk(MK_BT_STATE, MODE_RAW);
    mk(MK_BT_RX, 0);
    mk(MK_BT_TX, 0);
    mk(MK_BT_LINES, 0);
}

/// Ring index one past the newest DMA-written byte.
fn write_index(dma: &pac::dma::RegisterBlock) -> usize {
    (RING - dma.ch4cnt().read().cnt().bits() as usize) % RING
}

/// Pop one received byte, if any. Thread mode only.
fn pop_byte() -> Option<u8> {
    let dma = unsafe { &*pac::Dma::ptr() };
    let wr = write_index(dma);
    let st = unsafe { &mut *RX.0.get() };
    if st.rd == wr {
        return None;
    }
    let b = unsafe { (*RX_RING.0.get())[st.rd] };
    st.rd = (st.rd + 1) % RING;
    mk(MK_BT_RX, mk_get(MK_BT_RX) + 1);
    Some(b)
}

/// Drain raw bytes (bring-up visibility for the `btrd` verb; bypasses the
/// line assembler). Returns the number of bytes written into `buf`.
#[cfg(feature = "dev-verbs")]
#[inline(never)]
pub fn read_raw(buf: &mut [u8]) -> usize {
    let mut n = 0;
    while n < buf.len() {
        match pop_byte() {
            Some(b) => {
                buf[n] = b;
                n += 1;
            }
            None => break,
        }
    }
    n
}

/// Main-loop pump: clear a stuck overrun (can't normally happen — DMA
/// drains RBNE — but ORE would otherwise latch forever), then, in
/// dispatch mode, assemble CR/LF-terminated lines and hand each complete
/// one (terminator stripped, module chatter included — the dispatcher
/// ignores what it doesn't know) to `dispatch`. Bounded per pass.
pub fn service(mut dispatch: impl FnMut(&[u8])) {
    if mode() != MODE_DISPATCH {
        return;
    }
    let usart = unsafe { &*pac::Usart1::ptr() };
    if usart.stat().read().orerr().bit_is_set() {
        usart.intc().write(|w| w.orec().clear());
    }
    /* at most one ring's worth per pass keeps this bounded */
    for _ in 0..RING {
        let b = match pop_byte() {
            Some(b) => b,
            None => return,
        };
        /* borrow RX only inside this block; the line is copied out before
         * dispatch runs so the callback may call back into this module */
        let mut fire: Option<([u8; LINE_MAX], usize)> = None;
        {
            let st = unsafe { &mut *RX.0.get() };
            if b == b'\n' || b == b'\r' {
                if !st.over && st.llen > 0 {
                    fire = Some((st.line, st.llen));
                }
                st.llen = 0;
                st.over = false;
            } else if st.over {
                /* discarding an overlong line */
            } else if st.llen < LINE_MAX {
                st.line[st.llen] = b;
                st.llen += 1;
            } else {
                st.over = true;
                st.llen = 0;
            }
        }
        if let Some((line, n)) = fire {
            mk(MK_BT_LINES, mk_get(MK_BT_LINES) + 1);
            dispatch(&line[..n]);
        }
    }
}

/// Polled bounded TX. TBE/TC depend only on our own shift register (the
/// module has no flow control), so the budget is pure paranoia. Err stages:
/// 1 = TBE timeout, 2 = TC timeout, 3 = not initialized.
#[inline(never)]
pub fn send(data: &[u8]) -> Result<(), u8> {
    if mode() == 0 {
        return Err(3);
    }
    let usart = unsafe { &*pac::Usart1::ptr() };
    for &b in data {
        let mut budget = 50_000u32;
        while usart.stat().read().tbe().bit_is_clear() {
            budget -= 1;
            if budget == 0 {
                return Err(1);
            }
        }
        usart.tdata().write(|w| unsafe { w.bits(b as u32) });
    }
    let mut budget = 50_000u32;
    while usart.stat().read().tc().bit_is_clear() {
        budget -= 1;
        if budget == 0 {
            return Err(2);
        }
    }
    mk(MK_BT_TX, mk_get(MK_BT_TX) + data.len() as u32);
    Ok(())
}
