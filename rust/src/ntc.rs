//! NTC temperature — the STOCK model, recovered exactly from the stock
//! firmware (docs/HARDWARE.md "Temperature (NTC) & D±"): 10k @ 25 C,
//! beta = 3450, on the bottom of an 11.5k divider:
//!   R = avg * 11500 / (4095 - avg);  T = 1/(1/298.15 + ln(R/10000)/3450)
//! Evaluated in fixed point (ln via a Q24 log2). Pure integer code with no
//! crate deps: tools/dev/host_check compiles this file on the host and proves it
//! bit-identical to the stock f32 form over every sum16.

const LN2_Q30: i64 = 744_261_118; /* ln 2 * 2^30 */
const LN_1_15_Q24: i64 = 2_344_816; /* ln(11500/10000) * 2^24 */
const B0_Q24: i64 = 194_135_151; /* 3450/298.15 * 2^24 */

/// Whole °C (truncated toward zero) from a sum of 16 12-bit ADC samples.
pub fn temp_c(sum16: u32) -> i32 {
    if !(16..=65504).contains(&sum16) {
        return 0; /* open/shorted divider — don't feed ln() garbage */
    }
    /* ln(R/10k) = ln(1.15) + ln(sum16) - ln(65520 - sum16), Q24 */
    let d = log2_q24(sum16) as i64 - log2_q24(65520 - sum16) as i64;
    let ln_r = ((d * LN2_Q30) >> 30) + LN_1_15_Q24;
    /* T[µK] = 3450 / (3450/298.15 + ln R); the divisor is >= 3.3 over the
     * guarded range — max(1) only lets the compiler drop the /0 check */
    let t_uk = (3_450_000_000u64 << 24) / (B0_Q24 + ln_r).max(1) as u64;
    ((t_uk as i64 - 273_150_000) / 1_000_000) as i32
}

/// log2(n) in Q24, n >= 1: integer part from the MSB, fraction by repeated
/// squaring of the Q31 mantissa.
fn log2_q24(n: u32) -> u32 {
    let k = 31 - n.leading_zeros();
    let mut x = (n << (31 - k)) as u64;
    let mut f = 0u32;
    for _ in 0..24 {
        x = (x * x) >> 31;
        f <<= 1;
        if x >= 1 << 32 {
            x >>= 1;
            f |= 1;
        }
    }
    (k << 24) | f
}
