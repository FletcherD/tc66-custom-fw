//! INA226 measurement + per-unit calibration (M1).
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

use crate::i2c;
use crate::markers::{mk, mk_get, MK_I2CERR, MK_WINDOWS};

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

pub struct Reading {
    pub v_1e4: u32,
    pub i_1e5: u32,
    /// current direction (getva publishes magnitude; kept for the display/M3)
    #[allow(dead_code)]
    pub reverse: bool,
    pub dp_1e2: u32, // D+ in 1e-2 V
    pub dm_1e2: u32, // D- in 1e-2 V
    pub temp_c: i32, // NTC temperature, whole deg C
}

/* ADC channels, identified empirically via `adcrd` scans (see HARDWARE.md):
 * ch8/ch9 = PB0/PB1 = D+/D- (direct, USB idle J-state signature);
 * ch1 = PA1 = NTC divider (stable across scans; ch0 drifts = floating).
 * Cross-checked against the internal die sensor (ch16 ~41 C). */
const CH_NTC: u8 = 1;
const CH_DP: u8 = 8;
const CH_DM: u8 = 9;

/// D± wire value per HARDWARE.md: sum-of-16 * 330 / 65520 -> 1e-2 V.
fn d_line_1e2(sum16: u32) -> u32 {
    sum16 * 330 / 65520
}

/// NTC temperature — EXPERIMENTAL model: 10k B=3950 NTC low-side with a 22k
/// pull-up to 3.3 V. Chosen because it lands ~2 C below the die sensor on a
/// warm board (consistent); divider values are a fit, not traced — refine by
/// comparing against the stock display at a couple of temperatures. Whole C.
fn ntc_temp_c(sum16: u32) -> i32 {
    let v = sum16 as f32 * 3.3 / 65520.0;
    if v <= 0.01 || v >= 3.29 {
        return 0; /* open/shorted divider — don't feed ln() garbage */
    }
    let r = 22_000.0 * v / (3.3 - v);
    let inv_t = 1.0 / 298.15 + libm::logf(r / 10_000.0) / 3950.0;
    (1.0 / inv_t - 273.15) as i32
}

fn eeprom_u32(off: u16) -> Option<u32> {
    let mut b = [0u8; 4];
    i2c::transact(EEPROM_ADDR, &[(off >> 8) as u8, off as u8], &mut b).ok()?;
    Some(u32::from_le_bytes(b))
}

fn ina_read(reg: u8) -> Option<u16> {
    let mut b = [0u8; 2];
    i2c::transact(INA_ADDR, &[reg], &mut b).ok()?;
    Some(u16::from_be_bytes(b))
}

pub fn load_cal() -> Option<Cal> {
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
/// per the calibration model).
pub fn configure() -> bool {
    i2c::transact(INA_ADDR, &[REG_CONFIG, 0x41, 0x27], &mut []).is_ok()
}

/// Apply the calibration formulas to raw sums.
pub fn calibrate(cal: &Cal, bus_sum8: u32, shunt_sum16: u32) -> Reading {
    let v = ((bus_sum8 as u64 * cal.gain_v as u64 + cal.off_v as u64) / 20000) as u32;
    let (mag, reverse) = if shunt_sum16 >= cal.refz {
        (shunt_sum16 - cal.refz, false)
    } else {
        (cal.refz - shunt_sum16, true)
    };
    let gain = if reverse { cal.gain_cn } else { cal.gain_cp };
    let i = ((mag as u64 * gain as u64 / 10000) / 4) as u32;
    Reading {
        v_1e4: v,
        i_1e5: i,
        reverse,
        dp_1e2: 0,
        dm_1e2: 0,
        temp_c: 0,
    }
}

/// Timer-paced sampler: one I2C read per tick (8 bus + 16 shunt per window),
/// so measurement pace is set by the timer, not by host polling. All failures
/// are non-fatal: counted in MK_I2CERR, window restarted, cal load retried.
pub struct Sampler {
    cal: Option<Cal>,
    step: u8,
    bus_sum: u32,
    shunt_sum: u32,
    retry: u8,
}

impl Sampler {
    pub const fn new() -> Self {
        Sampler {
            cal: None,
            step: 0,
            bus_sum: 0,
            shunt_sum: 0,
            retry: 0,
        }
    }

    fn reset_window(&mut self) {
        self.step = 0;
        self.bus_sum = 0;
        self.shunt_sum = 0;
    }

    fn bump_err(&self) {
        mk(MK_I2CERR, mk_get(MK_I2CERR) + 1);
    }

    /// Call once per timer tick. Returns a Reading when a window completes.
    #[inline(never)]
    pub fn tick(&mut self) -> Option<Reading> {
        if self.cal.is_none() {
            /* first tick, then every 64 ticks (~0.25 s at 250 Hz): try to bring
             * up the monitor + load calibration; failures just retry later */
            let due = self.retry == 0 || self.retry >= 64;
            self.retry = if due { 1 } else { self.retry + 1 };
            if due {
                if configure() {
                    self.cal = load_cal();
                }
                if self.cal.is_none() {
                    self.bump_err();
                }
            }
            return None;
        }
        let ok = if self.step < 8 {
            match ina_read(REG_BUS) {
                Some(r) => {
                    self.bus_sum += r as u32;
                    true
                }
                None => false,
            }
        } else {
            match ina_read(REG_SHUNT) {
                Some(r) => {
                    self.shunt_sum += (r ^ 0x8000) as u32;
                    true
                }
                None => false,
            }
        };
        if !ok {
            self.bump_err();
            self.reset_window();
            return None;
        }
        self.step += 1;
        if self.step == 24 {
            /* final tick of the window: ADC block (3 x 16 fast conversions,
             * well under a tick) + calibrate + publish */
            let ntc = crate::adc::read_sum16(CH_NTC);
            let dp = crate::adc::read_sum16(CH_DP);
            let dm = crate::adc::read_sum16(CH_DM);
            if ntc.is_err() || dp.is_err() || dm.is_err() {
                self.bump_err();
            }
            let rd = self.cal.as_ref().map(|c| {
                let mut rd = calibrate(c, self.bus_sum, self.shunt_sum);
                rd.dp_1e2 = dp.map(d_line_1e2).unwrap_or(0);
                rd.dm_1e2 = dm.map(d_line_1e2).unwrap_or(0);
                rd.temp_c = ntc.map(ntc_temp_c).unwrap_or(0);
                rd
            });
            self.reset_window();
            mk(MK_WINDOWS, mk_get(MK_WINDOWS) + 1);
            return rd;
        }
        None
    }
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
