//! INA226 measurement + per-unit calibration (M1), sampled from the TIMER5
//! interrupt every 2.24 ms (~446 Hz).
//!
//! The monitor is a TI INA226 at 7-bit 0x4A (verified: manuf 0x5449, die
//! 0x2260); registers read 16-bit big-endian after a 1-byte pointer write.
//! Calibration model (docs/HARDWARE.md, constants live in the EEPROM at 0x50):
//!   V[1e-4 V] = (sum of 8 bus-voltage raws * gainV + offV) / 20000
//!   biased    = shunt raw XOR 0x8000            (two's-complement -> offset-binary)
//!   I[1e-5 A] = (((sum of 16 biased) - ref) * gainC / 10000) / 4
//!               forward (sum >= ref) uses gainCp; reverse uses gainCn, negated
//! All divisions truncate. Verified plausible on this unit:
//! ref = 524268 ~= 16 * 0x8000, gainV/20000 ~= 1.25 mV * 1e4 / 8.
//!
//! # Sampling architecture (IRQ-driven)
//!
//! The INA226 runs continuous shunt+bus at 1.1 ms each (CONFIG 0x4127): a
//! fresh shunt and bus result every 2.2 ms cycle (measured 2200.6-2202 us on
//! this unit, tools/dev/inaexp.py). `start()` reprograms TIMER5 to a
//! `TICK_US` update interrupt — 1.8% longer than that cycle, so each read
//! gets a new conversion (never one twice; a conversion is skipped about
//! every 55 ticks as the phase slips) — and unmasks its NVIC line (one of
//! the two IRQs this firmware enables — the other is USBD_LP, same
//! priority, so neither ever preempts the other; everything else stays
//! masked and polled). Each tick the ISR performs TWO I2C reads, shunt then
//! bus (~0.13 ms each at 400 kHz), so ~98% of the chip's conversions are
//! used instead of a sparse subset. ALERT is not wired, so the reads are
//! not synchronized to conversion-ready; ISR entry jitter (a USBD_LP
//! handler in progress) can rarely make one read repeat a conversion.
//! After `window_ms` worth of ticks (setting, default 96 ms = 43 ticks;
//! `set_window_ms`) the window's V/I are calibrated and published;
//! `cal_v`/`cal_i` rescale the sums to the depths the EEPROM constants bake
//! in (8 bus / 16 shunt). The window-end ADC burst (temp/D±) runs on its own
//! fixed ~96 ms cadence, whatever the window.
//!
//! Once per second (a us accumulator, counted even before cal loads) the
//! ISR folds every sample of that second into the powerz `avg_*` fields and
//! bumps `seconds()`; the main loop integrates energy from those full-second
//! averages, one per `seconds()` step.
//!
//! `ticks()` stays a 4 ms time base for the rest of the firmware (PD/detect
//! timeouts, energy, keys, UI cadences): the ISR adds `TICK_US` to a
//! remainder and advances `TICKS` once per 4000 us.
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
//! # Shared state & soundness (single-core Cortex-M3; the USBD_LP ISR
//! # never touches any static in this module, so "the ISR" below is
//! # unambiguously the TIMER5 tick)
//!
//! Every `static` here is one of four sound patterns:
//!  - **ISR-exclusive** (`SAMPLER`): touched only from the TIMER5 ISR after
//!    `start()`; before that only const-init. No concurrent access, ever.
//!  - **Atomics** (`TICKS`, `TICK_REM`, `CAL_READY`, `ADC_REQ`): single-word,
//!    tear-free (`TICK_REM` is ISR-only).
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
//! A pause longer than one tick loses the extra tick(s) (the time base
//! runs slow by as much); the window just stretches (sums are per-read,
//! not per-time). The on-chip ADC is
//! thread-only (deferred burst + `adcrd`), so it needs no pause.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::i2c;
use crate::markers::{
    mk, mk_get, MK_I2CERR, MK_SMP_ADCB, MK_SMP_ADCE, MK_SMP_BASE, MK_SMP_CAL, MK_SMP_I2CE,
    MK_SMP_IRQS, MK_SMP_SECS, MK_SMP_STATE, MK_WINDOWS,
};
use gd32f1::gd32f150 as pac;

const INA_ADDR: u8 = 0x4A;
const EEPROM_ADDR: u8 = 0x50;

const REG_CONFIG: u8 = 0x00;
const REG_SHUNT: u8 = 0x01;
const REG_BUS: u8 = 0x02;

/// Sampler tick period, us: the 2.2 ms shunt+bus cycle at CONFIG 0x4127
/// + 1.8% so the timer never outpaces the chip's oscillator.
const TICK_US: u32 = 2240;
/// Default shunt+bus pairs per published window (43 x 2.24 ms = 96.3 ms);
/// the `window_ms` setting overrides it via `set_window_ms`.
const WINDOW_DEFAULT: u32 = 43;
/// Ticks between window-independent ADC bursts (~96 ms).
const ADC_TICKS: u8 = 43;
/// Ticks to sit out I2C after a stuck-bus failure (~200 ms).
const BACKOFF_TICKS: u8 = 90;

pub struct Cal {
    pub gain_v: u32,  // EEPROM 0x2004
    pub off_v: u32,   // EEPROM 0x2044
    pub gain_cp: u32, // EEPROM 0x2008
    pub gain_cn: u32, // EEPROM 0x200c
    pub refz: u32,    // EEPROM 0x2058 (sum of 16 offset-binary shunt raws at 0 A)
}

/* ADC channels — the stock map, recovered from its ADC init (docs/HARDWARE.md
 * "On-chip ADC"): PA0 = NTC, PA1 = D+, PB1 = D−. */
const CH_NTC: u8 = 0;
const CH_DP: u8 = 1;
const CH_DM: u8 = 9;
const CH_VREF: u8 = 17; /* internal 1.2 V reference -> Vdd for powerz */

/// D± wire value per docs/HARDWARE.md: sum-of-16 * 330 / 65520 -> 1e-2 V.
fn d_line_1e2(sum16: u32) -> u32 {
    sum16 * 330 / 65520
}

/// D± line voltage in 0.1 mV from a sum of 16 12-bit samples (3.3 V ref).
fn d_line_tmv(sum16: u32) -> u16 {
    (sum16 * 33000 / 65520) as u16
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
#[inline(never)]
pub fn cal_v(cal: &Cal, bus_sum: u64, n: u64) -> u32 {
    ((bus_sum * 8 * cal.gain_v as u64 / n + cal.off_v as u64) / 20000) as u32
}

/// Calibrated signed I (1e-5 A) from a sum of `n` biased shunt raws
/// (constants bake in sum depth 16; same rescaling as cal_v).
#[inline(never)]
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
/// trims sit within 0.3% of these nominals — docs/HARDWARE.md.)
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
    /// logic-rail voltage from the Vrefint ratio (thread-written, powerz Vdd)
    pub vdd_mv: u32,
    /* powerz resolution: 1/128 °C, D± in 0.1 mV (thread-written) */
    pub temp_c128: i32,
    pub dp_tmv: u16,
    pub dm_tmv: u16,
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
    vdd_mv: 0,
    temp_c128: 0,
    dp_tmv: 0,
    dm_tmv: 0,
}));

/// 4 ms ticks since `start()` (ISR advances it from `TICK_REM`; thread
/// derives every cadence — 1 Hz energy, 0.5 s UI, cal retries — from
/// deltas, so ticks are never lost to main-loop blocking).
static TICKS: AtomicU32 = AtomicU32::new(0);
/// ISR-only: us accumulated toward the next 4 ms `TICKS` step.
static TICK_REM: AtomicU32 = AtomicU32::new(0);
/// ISR-only: us into the current second.
static SEC_US: AtomicU32 = AtomicU32::new(0);
/// Seconds completed by the ISR (the `avg_*` fields hold the latest one's
/// averages once cal is loaded); the main loop's energy cadence.
static SECS: AtomicU32 = AtomicU32::new(0);
/// Shunt+bus pairs per window (thread writes via `set_window_ms`, ISR
/// reads); 0 = `WINDOW_DEFAULT` (zero-init keeps it in .bss).
static WINDOW_PAIRS: AtomicU32 = AtomicU32::new(0);

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

/// Published V/I only — the main loop uses this instead of `snapshot()`
/// because whole-Meas copies cost ~56 B of its frame each at opt-level 1
/// (stack-gate budget).
#[inline(never)]
pub fn snapshot_vi() -> (u32, u32) {
    cortex_m::interrupt::free(|_| unsafe {
        let m = &*PUBLISHED.0.get();
        (m.v_1e4, m.i_1e5)
    })
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

#[inline(never)]
pub fn ticks() -> u32 {
    TICKS.load(Ordering::Relaxed)
}

/// Seconds the sampler has completed (see `SECS`).
#[inline(never)]
pub fn seconds() -> u32 {
    SECS.load(Ordering::Acquire)
}

/// The last full second's average V (1e-4 V) and |I| (1e-5 A) — every
/// sample of that second, for energy integration.
#[inline(never)]
pub fn snapshot_avg_vi() -> (u32, u32) {
    cortex_m::interrupt::free(|_| unsafe {
        let m = &*PUBLISHED.0.get();
        (m.avg_v_1e4, m.avg_i_1e5.unsigned_abs())
    })
}

/// `window_ms` setting: published-window length, rounded to whole ticks
/// (at least one). Takes effect from the window in progress.
pub fn set_window_ms(ms: u32) {
    let pairs = ((ms * 1000 + TICK_US / 2) / TICK_US).max(1);
    WINDOW_PAIRS.store(pairs, Ordering::Relaxed);
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

/// Reprogram TIMER5 from scratch as the `TICK_US` sampler tick and unmask its
/// NVIC line. Called once from run(), after the grace window ends (so a
/// sampler fault stays escapable) — PRIMASK was already cleared by the
/// USBD_LP unmask block, so this only opens the one extra NVIC line.
pub fn start(rcu: &pac::Rcu, tim: &pac::Timer5) {
    /* SRAM persists across reset: zero our diagnostic block so a marker
     * read never shows a previous boot's values */
    for w in 0..7 {
        mk(MK_SMP_BASE + 4 * w, 0);
    }
    #[cfg(feature = "pcprf")]
    mk(crate::markers::MK_PCPRF_BASE, 0); /* PC-profiler sample counter */

    /* first cal attempt: IRQ not yet enabled, so I2C is exclusively ours */
    try_cal();

    rcu.apb1en().modify(|_, w| w.timer5en().enabled());
    tim.psc().write(|w| w.psc().bits(47)); /* 48 MHz -> 1 MHz */
    tim.car().write(|w| w.car().bits((TICK_US - 1) as u16));
    tim.swevg().write(|w| w.upg().set_bit()); /* latch PSC/CAR now */
    tim.intf().write(|w| w); /* the UPG above set UPIF: write-0-to-clear */
    tim.dmainten().modify(|_, w| w.upie().set_bit());
    tim.ctl0().modify(|_, w| w.cen().enabled());

    cortex_m::peripheral::NVIC::unpend(pac::Interrupt::TIMER5_DAC);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIMER5_DAC);
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
    let vref = crate::adc::read_sum16(CH_VREF);
    if let Err(stage) = ntc.and(dp).and(dm).and(vref) {
        let n = (mk_get(MK_SMP_ADCE) >> 8).wrapping_add(1);
        mk(MK_SMP_ADCE, (n << 8) | stage as u32);
    }
    /* Vdd from the internal reference ratio: Vdd = 4095·1200 mV / avg(vref)
     * (avg = sum/16; 1.2 V nominal, uncalibrated per part — ~2%, fine for
     * the powerz Vdd display field) */
    let vdd = match vref {
        Ok(s) if s > 0 => (16u64 * 4095 * 1200 / s as u64) as u32,
        _ => 0,
    };
    let t128 = ntc.map(crate::ntc::temp_c128).unwrap_or(0);
    cortex_m::interrupt::free(|_| unsafe {
        let m = &mut *PUBLISHED.0.get();
        m.dp_1e2 = dp.map(d_line_1e2).unwrap_or(0);
        m.dm_1e2 = dm.map(d_line_1e2).unwrap_or(0);
        m.dp_tmv = dp.map(d_line_tmv).unwrap_or(0);
        m.dm_tmv = dm.map(d_line_tmv).unwrap_or(0);
        m.temp_c128 = t128;
        m.temp_c = t128 / 128;
        m.vdd_mv = vdd;
    });
    mk(MK_SMP_ADCB, mk_get(MK_SMP_ADCB) + 1);
}

/* ---------------- the ISR ---------------- */

/* TIMER5_DAC entry veneer: a statistical PC profiler for free. Before the
 * tick body runs, grab the interrupted context's PC from the hardware
 * exception frame (sp[6]: r0,r1,r2,r3,r12,lr,PC,xPSR) into a 128-entry
 * SRAM ring at PC_RING (markers.rs: counter word + ring, 0x2000_1100 —
 * read out over CDC via the `pcprf` verb, tools/dev/pcprofile.py). The chip's
 * DWT is a stub (PCSR reads 0xFFFFFFFF, CYCCNT never counts), so this is
 * the only PC sampling this hardware gets. Samples thread mode (and
 * attributes USBD_LP time to the interrupted thread PC on tail-chains);
 * tick-cadence thread work is under-sampled (it runs just after ticks).
 * r0-r3 are stacked = free; LR (EXC_RETURN) is untouched, so the body
 * returns from the exception like any plain handler. Feature-gated:
 * `pcprf` (default on) — without it the vector branches straight to the
 * body. */
#[cfg(feature = "pcprf")]
core::arch::global_asm!(
    ".section .fasttext.TIMER5_DAC, \"ax\", %progbits",
    ".global TIMER5_DAC",
    ".type TIMER5_DAC, %function",
    ".thumb_func",
    "TIMER5_DAC:",
    "ldr r0, [sp, #24]",     // stacked PC
    "ldr r1, =0x20001100",   // PC_RING: [0] = counter, [1..129] = samples
    "ldr r2, [r1]",
    "and r3, r2, #127",
    "adds r2, #1",
    "str r2, [r1]",
    "add r3, r1, r3, lsl #2",
    "str r0, [r3, #4]",
    "b {body}",
    body = sym timer5_body,
);

#[cfg(not(feature = "pcprf"))]
core::arch::global_asm!(
    ".section .fasttext.TIMER5_DAC, \"ax\", %progbits",
    ".global TIMER5_DAC",
    ".type TIMER5_DAC, %function",
    ".thumb_func",
    "TIMER5_DAC:",
    "b {body}",
    body = sym timer5_body,
);

/// Sampler tick (body — entered via the profiling veneer above).
/// Budget: two I2C reads ~0.26 ms (~12% CPU); the window-publish tick adds
/// a few us of integer math; worst case the 1 s drain coincides (+ ~2 us).
/// No ADC, no USB, no floats in here.
extern "C" fn timer5_body() {
    let tim = unsafe { &*pac::Timer5::ptr() };
    tim.intf().write(|w| w); /* clear UPIF (write-0-to-clear) */
    let rem = TICK_REM.load(Ordering::Relaxed) + TICK_US;
    if rem >= 4000 {
        TICK_REM.store(rem - 4000, Ordering::Relaxed);
        TICKS.fetch_add(1, Ordering::Relaxed);
    } else {
        TICK_REM.store(rem, Ordering::Relaxed);
    }
    let n = mk_get(MK_SMP_IRQS).wrapping_add(1);
    mk(MK_SMP_IRQS, n);
    let mut sec = SEC_US.load(Ordering::Relaxed) + TICK_US;
    let new_second = sec >= 1_000_000;
    if new_second {
        sec -= 1_000_000;
    }
    SEC_US.store(sec, Ordering::Relaxed);
    if CAL_READY.load(Ordering::Acquire) {
        /* CAL is write-once-published (Acquire above) and never written
         * again; SAMPLER is ISR-exclusive after start(). */
        let cal = unsafe { &*CAL.0.get() };
        let s = unsafe { &mut *SAMPLER.0.get() };
        s.tick(cal, n);
        if new_second {
            mk(MK_SMP_STATE, (n << 8) | 0x50); /* 1 s drain */
            s.drain_second(cal);
            mk(MK_SMP_SECS, mk_get(MK_SMP_SECS) + 1);
        }
        mk(MK_SMP_STATE, (n << 8) | 0x02); /* exit */
    } else {
        mk(MK_SMP_STATE, (n << 8) | 0x03); /* idle: waiting for cal */
    }
    if new_second {
        SECS.fetch_add(1, Ordering::Release);
    }
}

/// Window + 1 s accumulators, advanced once per tick from the ISR.
struct Sampler {
    step: u32,   /* pairs summed in the current window */
    backoff: u8, /* ticks left to skip I2C after a bus-busy failure */
    adc_step: u8, /* ticks since the last ADC-burst request */
    bus_sum: u32,
    shunt_sum: u32,
    /* 1 s raw accumulators for the averaged powerz values */
    sec_bus_sum: u32,
    sec_bus_n: u32,
    sec_shunt_sum: u32,
    sec_shunt_n: u32,
}

impl Sampler {
    const fn new() -> Self {
        Sampler {
            step: 0,
            backoff: 0,
            adc_step: 0,
            bus_sum: 0,
            shunt_sum: 0,
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

    /// One tick: read shunt + bus; publish V/I at window end; request the
    /// ADC burst every `ADC_TICKS`.
    /// I2C failure: count it, restart the window, keep ticking.
    fn tick(&mut self, cal: &Cal, n: u32) {
        self.adc_step += 1;
        if self.adc_step >= ADC_TICKS {
            self.adc_step = 0;
            ADC_REQ.store(true, Ordering::Release);
        }
        if self.backoff > 0 {
            /* bus recently stuck busy: sit out — even the fast-fail busy
             * wait costs ~1.5 ms/tick, which must not become the ISR's
             * steady state (the build-65 freeze) */
            self.backoff -= 1;
            mk(MK_SMP_STATE, (n << 8) | 0x04);
            return;
        }
        mk(MK_SMP_STATE, (n << 8) | 0x10);
        let r = ina_read(REG_SHUNT)
            .map_err(|e| (e, 0))
            .and_then(|s| Ok((s, ina_read(REG_BUS).map_err(|e| (e, 0x80))?)));
        match r {
            Err((e, which)) => {
                mk(
                    MK_SMP_I2CE,
                    (e.stage as u32) << 24 | (e.stat0 as u32) << 8 | which | (self.step & 0x7F),
                );
                mk(MK_I2CERR, mk_get(MK_I2CERR) + 1);
                if e.stage & 0x7F == 1 {
                    self.backoff = BACKOFF_TICKS;
                }
                self.reset_window();
            }
            Ok((shunt, bus)) => {
                let biased = shunt ^ 0x8000;
                self.bus_sum += bus as u32;
                self.shunt_sum += biased as u32;
                self.sec_bus_sum += bus as u32;
                self.sec_bus_n += 1;
                self.sec_shunt_sum += biased as u32;
                self.sec_shunt_n += 1;
                /* instantaneous values: this tick's single conversions */
                let iv = cal_v(cal, bus as u64, 1);
                let ii = cal_i(cal, biased as u64, 1);
                let m = unsafe { &mut *PUBLISHED.0.get() };
                m.inst_v_1e4 = iv;
                m.inst_i_1e5 = ii;
                self.step += 1;
                /* >= : a shortened window_ms ends the one in progress now */
                let window = match WINDOW_PAIRS.load(Ordering::Relaxed) {
                    0 => WINDOW_DEFAULT,
                    w => w,
                };
                if self.step >= window {
                    mk(MK_SMP_STATE, (n << 8) | 0x40); /* window publish */
                    let v = cal_v(cal, self.bus_sum as u64, self.step as u64);
                    let i = cal_i(cal, self.shunt_sum as u64, self.step as u64);
                    let m = unsafe { &mut *PUBLISHED.0.get() };
                    m.v_1e4 = v;
                    m.i_1e5 = i.unsigned_abs();
                    self.reset_window();
                    mk(MK_WINDOWS, mk_get(MK_WINDOWS) + 1);
                }
            }
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

/// `inaxp` dev verb backend: INA226 conversion timing + raw noise under an
/// arbitrary CONFIG. Writes `cfg`, waits out the first conversion-ready
/// (Mask/Enable CVRF, polled over I2C — ALERT is not wired), then records `n`
/// more: SysTick counts since the previous one + the raw shunt and bus
/// registers. Triggered modes (MODE 1..3) rewrite CONFIG before each sample
/// and time from that trigger. Restores the sampler's CONFIG afterwards.
/// Holds `pause` throughout (sampler ticks skipped, ≤ n × 100 ms).
/// `out`: SYST CSR u32, SYST RVR u32, then n × (dt u32, shunt u16, bus u16),
/// LE. Err: I2C stage, or 0x20 = no CVRF within 100 ms.
#[cfg(feature = "dev-ina")]
pub fn experiment(cfg: u16, n: usize, out: &mut [u8]) -> Result<usize, u8> {
    const REG_MASK: u8 = 0x06;
    const CVRF: u16 = 1 << 3;
    const TIMEOUT: u32 = 4_800_000; /* SysTick counts: 100 ms at 48 MHz */
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    out[0..4].copy_from_slice(&syst.csr.read().to_le_bytes());
    out[4..8].copy_from_slice(&syst.rvr.read().to_le_bytes());
    let triggered = matches!(cfg & 7, 1..=3);
    let write_cfg = || {
        i2c::transact(INA_ADDR, &[REG_CONFIG, (cfg >> 8) as u8, cfg as u8], &mut [])
            .map_err(|e| e.stage)
    };
    let wait_ready = |since: u32| -> Result<u32, u8> {
        loop {
            let m = ina_read(REG_MASK).map_err(|e| e.stage)?;
            let t = cortex_m::peripheral::SYST::get_current();
            if m & CVRF != 0 {
                return Ok(t);
            }
            if since.wrapping_sub(t) & 0x00FF_FFFF > TIMEOUT {
                return Err(0x20);
            }
        }
    };
    let r = pause(|| {
        write_cfg()?;
        let mut prev = wait_ready(cortex_m::peripheral::SYST::get_current())?;
        let mut len = 8;
        for _ in 0..n {
            if triggered {
                write_cfg()?;
                prev = cortex_m::peripheral::SYST::get_current();
            }
            let t = wait_ready(prev)?;
            let dt = prev.wrapping_sub(t) & 0x00FF_FFFF;
            prev = t;
            let shunt = ina_read(REG_SHUNT).map_err(|e| e.stage)?;
            let bus = ina_read(REG_BUS).map_err(|e| e.stage)?;
            let Some(rec) = out.get_mut(len..len + 8) else {
                break;
            };
            rec[0..4].copy_from_slice(&dt.to_le_bytes());
            rec[4..6].copy_from_slice(&shunt.to_le_bytes());
            rec[6..8].copy_from_slice(&bus.to_le_bytes());
            len += 8;
        }
        Ok(len)
    });
    pause(configure);
    r
}
