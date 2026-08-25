//! INA226 measurement + per-unit calibration (M1), sampled from the TIMER5
//! interrupt at 250 Hz (4 ms tick).
//!
//! The monitor is a TI INA226 at 7-bit 0x4A (verified: manuf 0x5449, die
//! 0x2260); registers read 16-bit big-endian after a 1-byte pointer write.
//! Calibration model (HARDWARE.md, constants live in the EEPROM at 0x50):
//!   V[1e-4 V] = (sum of 8 bus-voltage raws * gainV + offV) / 20000
//!   biased    = shunt raw XOR 0x8000            (two's-complement -> offset-binary)
//!   I[1e-5 A] = (((sum of 16 biased) - ref) * gainC / 10000) / 4
//!               forward (sum >= ref) uses gainCp; reverse uses gainCn, negated
//! All divisions truncate. Verified plausible on this unit:
//! ref = 524268 ~= 16 * 0x8000, gainV/20000 ~= 1.25 mV * 1e4 / 8.
//!
//! # Sampling architecture (IRQ-driven)
//!
//! `start()` reprograms TIMER5 to a 4 ms update interrupt and unmasks its
//! NVIC line (the only IRQ this firmware enables — everything else stays
//! masked and polled). Each tick the ISR performs ONE I2C read (~0.5 ms at
//! 100 kHz, ~12% of a tick) in the interleaved repeating pattern
//!   shunt, shunt, bus  (x8  ->  24 ticks = 16 shunt + 8 bus per window)
//! which preserves the EXACT sum depths the EEPROM calibration bakes in
//! (16 shunt / 8 bus) while keeping both values evenly fresh: the newest
//! shunt sample is at most 2 ticks (8 ms) old, the newest bus sample at most
//! 3 ticks (12 ms) — vs up to 9/17 ticks (36/68 ms) with the old
//! 8-bus-then-16-shunt phasing. At tick 24 the window's V/I are calibrated
//! and published (~10.4 Hz).
//!
//! The window-end on-chip-ADC burst (NTC + D+ + D-, 48 conversions ~1 ms)
//! is NOT run in the ISR: it would make the worst-case handler ~1.5 ms and
//! sit in front of any future interrupt. The ISR sets `ADC_REQ`; the main
//! loop runs the burst in `service()`. Temp/D± therefore lag the window
//! boundary by one main-loop pass — normally <1 ms, worst ~0.5 s when the
//! loop is stuck in write_all's absent-host budget (acceptable: these are
//! slow-moving values, and under the old polled scheme the whole sampler
//! stalled in that case).
//!
//! # Shared state & soundness (single-core Cortex-M3, one IRQ)
//!
//! Every `static` here is one of four sound patterns:
//!  - **ISR-exclusive** (`SAMPLER`): touched only from the TIMER5 ISR after
//!    `start()`; before that only const-init. No concurrent access, ever.
//!  - **Atomics** (`TICKS`, `CAL_READY`, `ADC_REQ`): single-word, tear-free.
//!  - **Write-once publication** (`CAL`): the thread writes the struct, then
//!    Release-stores `CAL_READY`; the ISR Acquire-loads the flag and only
//!    then reads `CAL`, which is never written again. No temporal overlap.
//!  - **Split-field + critical-section copy** (`PUBLISHED`): the ISR writes
//!    only {v, i, inst_*, avg_*, ori_*}; the thread writes only
//!    {mah, mwh, dp, dm, temp} — disjoint field sets, so no store races.
//!    EVERY thread-mode access (the whole-struct `snapshot()` copy AND the
//!    thread-field writes) runs inside `cortex_m::interrupt::free`, so a
//!    thread-held `&mut`/`&` never coexists with the ISR's — no aliasing,
//!    and a snapshot can never mix fields across windows (the multi-word
//!    copy cannot be torn by the ISR). The masked section is a ~52-byte
//!    copy, ~1-2 us: negligible tick jitter.
//!  - `LAST_CAL_TRY` is thread-only scratch.
//!
//! **Bus rule:** the ISR owns I2C0 while running. ALL thread-mode I2C
//! (`i2crd` verb, the cal-load retry) MUST run inside `pause()`, which
//! masks the TIMER5 NVIC line (dsb+isb) for the duration — otherwise a tick
//! firing mid-transaction SRESETs the peripheral under the thread's feet.
//! A pause longer than 4 ms loses at most the pending tick(s); the window
//! just stretches (sums are per-read, not per-time). The on-chip ADC is
//! thread-only (deferred burst + `adcrd`), so it needs no pause.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::i2c;
use crate::markers::{
    mk, mk_get, MK_I2CERR, MK_SMP_ADCB, MK_SMP_ADCE, MK_SMP_BASE, MK_SMP_CAL, MK_SMP_I2CE,
    MK_SMP_IRQS, MK_SMP_SECS, MK_SMP_STATE, MK_WINDOWS,
};
use gd32f1::gd32f150 as pac;
use pac::interrupt;

const INA_ADDR: u8 = 0x4A;
const EEPROM_ADDR: u8 = 0x50;

const REG_CONFIG: u8 = 0x00;
const REG_SHUNT: u8 = 0x01;
const REG_BUS: u8 = 0x02;

pub struct Cal {
    pub gain_v: u32,  // EEPROM 0x2004
    pub off_v: u32,   // EEPROM 0x2044
    pub gain_cp: u32, // EEPROM 0x2008
    pub gain_cn: u32, // EEPROM 0x200c
    pub refz: u32,    // EEPROM 0x2058 (sum of 16 offset-binary shunt raws at 0 A)
}

/* ADC channels, identified empirically via `adcrd` scans (see HARDWARE.md):
 * ch8/ch9 = PB0/PB1 = D+/D- (direct, USB idle J-state signature);
 * ch1 = PA1 = NTC divider on this unit (rock-stable, reads room temp on a
 * cool board with the stock constants; ch0 and ch3 drift = floating, even
 * though the stock firmware reads ch3 on the hardware rev it targets). */
const CH_NTC: u8 = 1;
const CH_DP: u8 = 8;
const CH_DM: u8 = 9;

/// D± wire value per HARDWARE.md: sum-of-16 * 330 / 65520 -> 1e-2 V.
fn d_line_1e2(sum16: u32) -> u32 {
    sum16 * 330 / 65520
}

/// NTC temperature — the STOCK model, recovered exactly from the stock
/// firmware (HARDWARE.md "Temperature (NTC) & D±"): 10k @ 25 C, beta = 3450,
/// on the bottom of an 11.5k divider:
///   R = avg * 11500 / (4095 - avg);  T = 1/(1/298.15 + ln(R/10000)/3450)
/// (Stock reads ch3 on the rev it targets; on THIS unit the divider is
/// measurably on ch1 — ch3 floats. Verified: reads room temp on a cool board.)
fn ntc_temp_c(sum16: u32) -> i32 {
    let avg = sum16 as f32 / 16.0;
    if avg < 1.0 || avg > 4094.0 {
        return 0; /* open/shorted divider — don't feed ln() garbage */
    }
    let r = avg * 11500.0 / (4095.0 - avg);
    let inv_t = 1.0 / 298.15 + libm::logf(r / 10_000.0) / 3450.0;
    (1.0 / inv_t - 273.15) as i32
}

fn eeprom_u32(off: u16) -> Option<u32> {
    let mut b = [0u8; 4];
    i2c::transact(EEPROM_ADDR, &[(off >> 8) as u8, off as u8], &mut b).ok()?;
    Some(u32::from_le_bytes(b))
}

fn ina_read(reg: u8) -> Result<u16, i2c::I2cErr> {
    let mut b = [0u8; 2];
    i2c::transact(INA_ADDR, &[reg], &mut b)?;
    Ok(u16::from_be_bytes(b))
}

fn load_cal() -> Option<Cal> {
    Some(Cal {
        gain_v: eeprom_u32(0x2004)?,
        off_v: eeprom_u32(0x2044)?,
        gain_cp: eeprom_u32(0x2008)?,
        gain_cn: eeprom_u32(0x200c)?,
        refz: eeprom_u32(0x2058)?,
    })
}

/// Put the INA226 in a known state: the power-on default 0x4127 (continuous
/// shunt+bus, 1.1 ms conversions, no on-chip averaging — we average in firmware
/// per the calibration model). Load-bearing: the chip returns 0 for bus
/// voltage until (re)configured.
fn configure() -> bool {
    i2c::transact(INA_ADDR, &[REG_CONFIG, 0x41, 0x27], &mut []).is_ok()
}

/// Calibrated V from a sum of `n` bus raws. The EEPROM constants bake in the
/// stock sum depth of 8, so the sum is rescaled to that domain (multiply
/// before divide: exact for n = 1/2/4/8, <1 LSB/16 truncation otherwise).
pub fn cal_v(cal: &Cal, bus_sum: u64, n: u64) -> u32 {
    ((bus_sum * 8 * cal.gain_v as u64 / n + cal.off_v as u64) / 20000) as u32
}

/// Calibrated signed I (1e-5 A) from a sum of `n` biased shunt raws
/// (constants bake in sum depth 16; same rescaling as cal_v).
pub fn cal_i(cal: &Cal, shunt_sum: u64, n: u64) -> i32 {
    let scaled = shunt_sum * 16 / n;
    let refz = cal.refz as u64;
    if scaled >= refz {
        ((scaled - refz) * cal.gain_cp as u64 / 40000) as i32
    } else {
        -(((refz - scaled) * cal.gain_cn as u64 / 40000) as i32)
    }
}

/// Uncalibrated ("original") values from raw sums using the nominal design
/// constants instead of the per-unit EEPROM trim: bus 1.25 mV/LSB; shunt
/// 2.5 uV/LSB across the 15 mOhm design shunt = 500/3 uA/LSB. (This unit's
/// trims sit within 0.3% of these nominals — HARDWARE.md.)
pub fn ori_uv(bus_sum: u64, n: u64) -> u32 {
    (bus_sum * 1250 / n) as u32
}
pub fn ori_ua(shunt_sum: u64, n: u64) -> i32 {
    let scaled = (shunt_sum * 16 / n) as i64 - 16 * 0x8000;
    (scaled * 500 / (3 * 16)) as i32
}

/// Wire-unit derived values: p = 1e-4 W, r = 1e-1 ohm (0 when i == 0).
pub fn derive(v_1e4: u32, i_1e5: u32) -> (u32, u32) {
    let p = (v_1e4 as u64 * i_1e5 as u64 / 100_000) as u32;
    let r = if i_1e5 == 0 {
        0
    } else {
        (v_1e4 as u64 * 100 / i_1e5 as u64) as u32
    };
    (p, r)
}

/* ---------------- shared state (soundness argued in the module doc) ------ */

struct Racy<T>(core::cell::UnsafeCell<T>);
unsafe impl<T> Sync for Racy<T> {}

/// Latest published measurement set. Field ownership (who writes):
/// ISR: v_1e4, i_1e5, inst_*, avg_*, ori_*.  Thread: mah, mwh, dp_1e2,
/// dm_1e2, temp_c.  Readers use `snapshot()` (whole-struct copy under
/// `interrupt::free`).
#[derive(Clone, Copy)]
pub struct Meas {
    pub v_1e4: u32,
    pub i_1e5: u32,
    pub dp_1e2: u32,
    pub dm_1e2: u32,
    pub temp_c: i32,
    pub mah: u32,
    pub mwh: u32,
    /* powerz fields: latest single conversion, 1 s calibrated average,
     * 1 s nominal-scale ("original") average */
    pub inst_v_1e4: u32,
    pub inst_i_1e5: i32,
    pub avg_v_1e4: u32,
    pub avg_i_1e5: i32,
    pub ori_uv: u32,
    pub ori_ua: i32,
}

static PUBLISHED: Racy<Meas> = Racy(core::cell::UnsafeCell::new(Meas {
    v_1e4: 0,
    i_1e5: 0,
    dp_1e2: 0,
    dm_1e2: 0,
    temp_c: 0,
    mah: 0,
    mwh: 0,
    inst_v_1e4: 0,
    inst_i_1e5: 0,
    avg_v_1e4: 0,
    avg_i_1e5: 0,
    ori_uv: 0,
    ori_ua: 0,
}));

/// 4 ms ticks since `start()` (ISR increments; thread derives every cadence
/// — 1 Hz energy, 0.5 s UI, cal retries — from deltas, so ticks are never
/// lost to main-loop blocking).
static TICKS: AtomicU32 = AtomicU32::new(0);

/// Write-once cal publication: thread fills CAL, then Release-stores
/// CAL_READY = true; the ISR Acquire-loads the flag before touching CAL.
/// After the flag is set, CAL is never written again.
static CAL: Racy<Cal> = Racy(core::cell::UnsafeCell::new(Cal {
    gain_v: 0,
    off_v: 0,
    gain_cp: 0,
    gain_cn: 0,
    refz: 0,
}));
static CAL_READY: AtomicBool = AtomicBool::new(false);

/// ISR -> thread: a window just completed, run the deferred ADC burst.
static ADC_REQ: AtomicBool = AtomicBool::new(false);

/// Window/second accumulators — ISR-exclusive after `start()`.
static SAMPLER: Racy<Sampler> = Racy(core::cell::UnsafeCell::new(Sampler::new()));

/// Thread-only: tick of the last cal-load attempt.
static LAST_CAL_TRY: Racy<u32> = Racy(core::cell::UnsafeCell::new(0));

/// Consistent copy of the published measurements (safe against ISR tearing).
#[inline(never)]
pub fn snapshot() -> Meas {
    cortex_m::interrupt::free(|_| unsafe { *PUBLISHED.0.get() })
}

/// Thread-owned energy fields (1 Hz integration lives in the main loop).
/// interrupt::free so a thread `&mut PUBLISHED` never coexists with the ISR's.
pub fn set_energy(mah: u32, mwh: u32) {
    cortex_m::interrupt::free(|_| unsafe {
        let m = &mut *PUBLISHED.0.get();
        m.mah = mah;
        m.mwh = mwh;
    });
}

pub fn ticks() -> u32 {
    TICKS.load(Ordering::Relaxed)
}

/// Run `f` with the sampler IRQ masked (NVIC line only — dsb+isb ensure no
/// late entry). Mandatory wrapper for ALL thread-mode I2C. A missed update
/// event just stretches the current window; nothing is corrupted.
#[inline(never)]
pub fn pause<R>(f: impl FnOnce() -> R) -> R {
    let was = cortex_m::peripheral::NVIC::is_enabled(pac::Interrupt::TIMER5_DAC);
    cortex_m::peripheral::NVIC::mask(pac::Interrupt::TIMER5_DAC);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    let r = f();
    if was {
        unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIMER5_DAC) };
    }
    r
}

/// One attempt to bring up the monitor + load calibration (~6 ms of I2C).
/// Caller must hold exclusive I2C (pre-`start()`, or inside `pause()` —
/// which also makes the MK_I2CERR read-modify-write exclusive with the ISR's).
fn try_cal() {
    let attempts = (mk_get(MK_SMP_CAL) >> 8).wrapping_add(1);
    if configure() {
        if let Some(c) = load_cal() {
            unsafe { *CAL.0.get() = c };
            CAL_READY.store(true, Ordering::Release);
            mk(MK_SMP_CAL, (attempts << 8) | 1);
            return;
        }
    }
    mk(MK_SMP_CAL, attempts << 8);
    mk(MK_I2CERR, mk_get(MK_I2CERR) + 1);
}

/// Reprogram TIMER5 from scratch as the 250 Hz sampler tick and enable its
/// interrupt — the ONLY unmasked IRQ (startup.rs masks everything at boot;
/// this is the single deliberate, late enable). Call once, just before the
/// main loop, after all other init.
pub fn start(rcu: &pac::Rcu, tim: &pac::Timer5) {
    /* SRAM persists across reset: zero our diagnostic block so a marker
     * read never shows a previous boot's values */
    for w in 0..7 {
        mk(MK_SMP_BASE + 4 * w, 0);
    }

    /* first cal attempt: IRQ not yet enabled, so I2C is exclusively ours */
    try_cal();

    rcu.apb1en().modify(|_, w| w.timer5en().enabled());
    tim.psc().write(|w| w.psc().bits(4799)); /* 48 MHz -> 10 kHz */
    tim.car().write(|w| w.car().bits(39)); /* 40 counts = 4 ms update */
    tim.swevg().write(|w| w.upg().set_bit()); /* latch PSC/CAR now */
    tim.intf().write(|w| w); /* the UPG above set UPIF: write-0-to-clear */
    tim.dmainten().modify(|_, w| w.upie().set_bit());
    tim.ctl0().modify(|_, w| w.cen().enabled());

    cortex_m::peripheral::NVIC::unpend(pac::Interrupt::TIMER5_DAC);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIMER5_DAC);
        /* PRIMASK has been set since __pre_init; every other NVIC line is
         * still masked, so this exposes exactly one interrupt source. */
        cortex_m::interrupt::enable();
    }
}

/// Main-loop half of the sampler: cal-load retries (every 64 ticks ~0.25 s
/// until loaded) and the deferred window-end ADC burst. Call every pass.
pub fn service() {
    if !CAL_READY.load(Ordering::Acquire) {
        let now = ticks();
        let last = unsafe { *LAST_CAL_TRY.0.get() };
        if now.wrapping_sub(last) >= 64 {
            unsafe { *LAST_CAL_TRY.0.get() = now };
            pause(try_cal);
        }
    }
    if ADC_REQ.swap(false, Ordering::Acquire) {
        adc_burst();
    }
}

/// The deferred window-end ADC burst (NTC + D+ + D-, ~1 ms) — thread mode
/// only; failures fall back to 0 exactly like the old in-window behavior,
/// counted in MK_SMP_ADCE (not MK_I2CERR: that word now belongs to the
/// ISR/paused-cal contexts).
fn adc_burst() {
    let ntc = crate::adc::read_sum16(CH_NTC);
    let dp = crate::adc::read_sum16(CH_DP);
    let dm = crate::adc::read_sum16(CH_DM);
    if let Err(stage) = ntc.and(dp).and(dm) {
        let n = (mk_get(MK_SMP_ADCE) >> 8).wrapping_add(1);
        mk(MK_SMP_ADCE, (n << 8) | stage as u32);
    }
    cortex_m::interrupt::free(|_| unsafe {
        let m = &mut *PUBLISHED.0.get();
        m.dp_1e2 = dp.map(d_line_1e2).unwrap_or(0);
        m.dm_1e2 = dm.map(d_line_1e2).unwrap_or(0);
        m.temp_c = ntc.map(ntc_temp_c).unwrap_or(0);
    });
    mk(MK_SMP_ADCB, mk_get(MK_SMP_ADCB) + 1);
}

/* ---------------- the ISR ---------------- */

/// 250 Hz sampler tick. Budget: one I2C read ~0.5 ms (~12% CPU); the
/// window-publish tick adds a few us of integer math; worst case the
/// 1 s drain coincides (+ ~2 us). No ADC, no USB, no floats in here.
#[interrupt]
fn TIMER5_DAC() {
    let tim = unsafe { &*pac::Timer5::ptr() };
    tim.intf().write(|w| w); /* clear UPIF (write-0-to-clear) */
    let n = TICKS.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
    mk(MK_SMP_IRQS, n);
    if !CAL_READY.load(Ordering::Acquire) {
        mk(MK_SMP_STATE, (n << 8) | 0x03); /* idle: waiting for cal */
        return;
    }
    /* CAL is write-once-published (Acquire above) and never written again;
     * SAMPLER is ISR-exclusive after start(). */
    let cal = unsafe { &*CAL.0.get() };
    let s = unsafe { &mut *SAMPLER.0.get() };
    s.tick(cal, n);
    mk(MK_SMP_STATE, (n << 8) | 0x02); /* exit */
}

/// Window + 1 s accumulators, advanced once per tick from the ISR.
struct Sampler {
    step: u8,      /* 0..24 within the current window */
    sec_ticks: u8, /* 0..250 within the current second */
    bus_sum: u32,
    shunt_sum: u32,
    /* latest single conversions (biased for shunt) for the instantaneous
     * powerz values, and 1 s raw accumulators for the averaged ones */
    last_bus: u16,
    last_shunt: u16,
    have_bus: bool,
    have_shunt: bool,
    sec_bus_sum: u32,
    sec_bus_n: u32,
    sec_shunt_sum: u32,
    sec_shunt_n: u32,
}

impl Sampler {
    const fn new() -> Self {
        Sampler {
            step: 0,
            sec_ticks: 0,
            bus_sum: 0,
            shunt_sum: 0,
            last_bus: 0,
            last_shunt: 0,
            have_bus: false,
            have_shunt: false,
            sec_bus_sum: 0,
            sec_bus_n: 0,
            sec_shunt_sum: 0,
            sec_shunt_n: 0,
        }
    }

    fn reset_window(&mut self) {
        self.step = 0;
        self.bus_sum = 0;
        self.shunt_sum = 0;
    }

    /// One tick: one interleaved I2C read; publish V/I + request the ADC
    /// burst at window end; drain the 1 s averages every 250 ticks.
    /// I2C failure: count it, restart the window, keep ticking.
    fn tick(&mut self, cal: &Cal, n: u32) {
        mk(MK_SMP_STATE, (n << 8) | (0x10 + self.step as u32));
        /* interleave: step % 3 -> shunt, shunt, bus (x8 = 16 shunt + 8 bus) */
        let is_bus = self.step % 3 == 2;
        match ina_read(if is_bus { REG_BUS } else { REG_SHUNT }) {
            Err(e) => {
                mk(
                    MK_SMP_I2CE,
                    (e.stage as u32) << 24 | (e.stat0 as u32) << 8 | self.step as u32,
                );
                mk(MK_I2CERR, mk_get(MK_I2CERR) + 1);
                self.reset_window();
            }
            Ok(r) => {
                if is_bus {
                    self.bus_sum += r as u32;
                    self.last_bus = r;
                    self.have_bus = true;
                    self.sec_bus_sum += r as u32;
                    self.sec_bus_n += 1;
                } else {
                    let biased = r ^ 0x8000;
                    self.shunt_sum += biased as u32;
                    self.last_shunt = biased;
                    self.have_shunt = true;
                    self.sec_shunt_sum += biased as u32;
                    self.sec_shunt_n += 1;
                }
                if self.have_bus && self.have_shunt {
                    /* instantaneous values: freshest single conversions */
                    let iv = cal_v(cal, self.last_bus as u64, 1);
                    let ii = cal_i(cal, self.last_shunt as u64, 1);
                    let m = unsafe { &mut *PUBLISHED.0.get() };
                    m.inst_v_1e4 = iv;
                    m.inst_i_1e5 = ii;
                }
                self.step += 1;
                if self.step == 24 {
                    mk(MK_SMP_STATE, (n << 8) | 0x40); /* window publish */
                    let v = cal_v(cal, self.bus_sum as u64, 8);
                    let i = cal_i(cal, self.shunt_sum as u64, 16);
                    let m = unsafe { &mut *PUBLISHED.0.get() };
                    m.v_1e4 = v;
                    m.i_1e5 = i.unsigned_abs();
                    self.reset_window();
                    mk(MK_WINDOWS, mk_get(MK_WINDOWS) + 1);
                    ADC_REQ.store(true, Ordering::Release);
                }
            }
        }
        self.sec_ticks += 1;
        if self.sec_ticks as u32 >= 250 {
            mk(MK_SMP_STATE, (n << 8) | 0x50); /* 1 s drain */
            self.sec_ticks = 0;
            self.drain_second(cal);
            mk(MK_SMP_SECS, mk_get(MK_SMP_SECS) + 1);
        }
    }

    /// Fold the 1 s raw accumulators into the powerz avg/ori fields; skipped
    /// (fields keep their last values) if the second had no successful reads.
    fn drain_second(&mut self, cal: &Cal) {
        let (bs, bn) = (self.sec_bus_sum as u64, self.sec_bus_n as u64);
        let (ss, sn) = (self.sec_shunt_sum as u64, self.sec_shunt_n as u64);
        self.sec_bus_sum = 0;
        self.sec_bus_n = 0;
        self.sec_shunt_sum = 0;
        self.sec_shunt_n = 0;
        if bn == 0 || sn == 0 {
            return;
        }
        let av = cal_v(cal, bs, bn);
        let ai = cal_i(cal, ss, sn);
        let ouv = ori_uv(bs, bn);
        let oua = ori_ua(ss, sn);
        let m = unsafe { &mut *PUBLISHED.0.get() };
        m.avg_v_1e4 = av;
        m.avg_i_1e5 = ai;
        m.ori_uv = ouv;
        m.ori_ua = oua;
    }
}
