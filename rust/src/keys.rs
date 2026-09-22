//! K1/K2 front buttons (PF0/PF1, active-low with the internal pull-up).
//!
//! Polled from the main loop at the sampler tick cadence (250 Hz): a key
//! changes state only after `DEBOUNCE` consecutive identical samples, and
//! `poll` reports the press *edges* since the last call as a bitmask
//! (bit 0 = K1, bit 1 = K2). Holding a key yields one event. Nothing here
//! touches anything but GPIOF; no interrupts are involved.

use gd32f1::gd32f150 as pac;

const DEBOUNCE: u8 = 5; /* 5 ticks = 20 ms of agreement */

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}
/* [stable pressed-mask, candidate mask, agreement count] */
static ST: Racy<[u8; 3]> = Racy(core::cell::UnsafeCell::new([0; 3]));

/// GPIOF clock on, PF0/PF1 as pulled-up inputs.
#[inline(never)]
pub fn init(rcu: &pac::Rcu, gpiof: &pac::Gpiof) {
    rcu.ahben().modify(|_, w| w.pfen().enabled());
    gpiof.ctl().modify(|_, w| w.ctl0().input().ctl1().input());
    gpiof.pud().modify(|_, w| w.pud0().pull_up().pud1().pull_up());
}

/// Raw pin levels: bit 0 = PF0, bit 1 = PF1 (1 = high = released).
#[inline(never)]
pub fn raw() -> u8 {
    let g = unsafe { &*pac::Gpiof::ptr() };
    (g.istat().read().bits() & 0b11) as u8
}

/// Debounced pressed-state mask (bit set = key held down).
pub fn held() -> u8 {
    unsafe { (*ST.0.get())[0] }
}

/// One debounce sample; returns the mask of keys that just became pressed.
#[inline(never)]
pub fn poll() -> u8 {
    let st = unsafe { &mut *ST.0.get() };
    let now = !raw() & 0b11; /* active-low -> pressed mask */
    if now != st[1] {
        st[1] = now;
        st[2] = 1;
        return 0;
    }
    if st[2] < DEBOUNCE {
        st[2] += 1;
        return 0;
    }
    let pressed = now & !st[0];
    st[0] = now;
    pressed
}
