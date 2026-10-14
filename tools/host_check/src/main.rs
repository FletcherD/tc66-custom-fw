//! Host-side proofs for the firmware's pure-math modules, compiled here
//! verbatim from rust/src:
//! - ntc.rs: fixed-point NTC conversion bit-identical to the stock f32 model
//!   over every possible ADC sum16 (also reported against exact f64).
//! - aes256.rs: S-box = the GF(2^8) definition, FIPS-197 C.3 vector, and
//!   random blocks under the meter's getva key vs the RustCrypto `aes` crate.

#[path = "../../../rust/src/ntc.rs"]
mod ntc;
#[allow(dead_code)]
#[path = "../../../rust/src/aes256.rs"]
mod aes256;

use aes::cipher::{BlockEncrypt, KeyInit};

/// The stock model in f32 + libm logf, as the firmware computed it before
/// the fixed-point port.
fn stock_f32(sum16: u32) -> i32 {
    let avg = sum16 as f32 / 16.0;
    if avg < 1.0 || avg > 4094.0 {
        return 0;
    }
    let r = avg * 11500.0 / (4095.0 - avg);
    let inv_t = 1.0 / 298.15 + libm::logf(r / 10_000.0) / 3450.0;
    (1.0 / inv_t - 273.15) as i32
}

fn exact(sum16: u32) -> f64 {
    let avg = sum16 as f64 / 16.0;
    if !(1.0..=4094.0).contains(&avg) {
        return 0.0;
    }
    let r = avg * 11500.0 / (4095.0 - avg);
    1.0 / (1.0 / 298.15 + (r / 10_000.0).ln() / 3450.0) - 273.15
}

fn check_ntc() -> bool {
    let (mut vs_stock, mut vs_exact) = (0u32, 0u32);
    for s in 0..=65535u32 {
        let v = ntc::temp_c(s);
        if v != stock_f32(s) {
            vs_stock += 1;
            println!("  sum16={s}: fixed={v} stock_f32={} exact={:.6}", stock_f32(s), exact(s));
        }
        if v as f64 != exact(s).trunc() {
            vs_exact += 1;
        }
    }
    println!("ntc: sum16 0..=65535 ({}..={} C): {vs_stock} differ from stock f32, {vs_exact} from trunc(exact)",
             ntc::temp_c(65504), ntc::temp_c(16));
    vs_stock == 0
}

fn gmul(mut a: u8, mut b: u8) -> u8 {
    let mut p = 0;
    while b != 0 {
        if b & 1 != 0 { p ^= a; }
        a = (a << 1) ^ if a & 0x80 != 0 { 0x1b } else { 0 };
        b >>= 1;
    }
    p
}

fn sbox_ref(x: u8) -> u8 {
    let inv = if x == 0 { 0 } else { (1..=255u8).find(|&y| gmul(x, y) == 1).unwrap() };
    inv ^ inv.rotate_left(1) ^ inv.rotate_left(2) ^ inv.rotate_left(3) ^ inv.rotate_left(4) ^ 0x63
}

/// The meter's getva key (rust/src/getva.rs KEY).
const GETVA_KEY: [u8; 32] = [
    0x58, 0x21, 0xfa, 0x56, 0x01, 0xb2, 0xf0, 0x26, 0x87, 0xff, 0x12, 0x04,
    0x62, 0x2a, 0x4f, 0xb0, 0x86, 0xf4, 0x02, 0x60, 0x81, 0x6f, 0x9a, 0x0b,
    0xa7, 0xf1, 0x06, 0x61, 0x9a, 0xb8, 0x72, 0x88,
];

fn check_aes() -> bool {
    let mut ok = true;
    let bad_sbox = (0..=255u8).filter(|&x| aes256::sbox(x) != sbox_ref(x)).count();
    println!("aes: S-box entries wrong: {bad_sbox}");
    ok &= bad_sbox == 0;

    /* FIPS-197 Appendix C.3 */
    let key: [u8; 32] = core::array::from_fn(|i| i as u8);
    let mut blk: [u8; 16] = core::array::from_fn(|i| (i as u8) * 0x11);
    aes256::encrypt_block(&aes256::expand_key(&key), &mut blk);
    let want = [0x8e, 0xa2, 0xb7, 0xca, 0x51, 0x67, 0x45, 0xbf, 0xea, 0xfc, 0x49, 0x90, 0x4b, 0x49, 0x60, 0x89];
    println!("aes: FIPS-197 C.3 vector {}", if blk == want { "ok" } else { "WRONG" });
    ok &= blk == want;

    let rk = aes256::expand_key(&GETVA_KEY);
    let reference = aes::Aes256Enc::new(&GETVA_KEY.into());
    let mut x = 0x2545_f491_4f6c_dd1du64; /* xorshift, fixed seed */
    let mut bad = 0;
    for _ in 0..100_000 {
        let mut b = [0u8; 16];
        for chunk in b.chunks_mut(8) {
            x ^= x << 13; x ^= x >> 7; x ^= x << 17;
            chunk.copy_from_slice(&x.to_le_bytes());
        }
        let mut r = b.into();
        reference.encrypt_block(&mut r);
        aes256::encrypt_block(&rk, &mut b);
        bad += (b[..] != r[..]) as u32;
    }
    println!("aes: 100000 random blocks under the getva key: {bad} differ from the aes crate");
    ok & (bad == 0)
}

fn main() {
    let ok = check_ntc() & check_aes();
    std::process::exit(!ok as i32);
}
